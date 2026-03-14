#include "Planning/SIPPPlanner.h"

#include "Algo/Reverse.h"
#include "HAL/PlatformTime.h"
#include "Planning/GridMap3D.h"

namespace
{
    constexpr int32 SIPP_GUARD_LIMIT = 250000;
    constexpr int32 SIPP_MAX_RECONSTRUCTED_PATH_POINTS = 20000;
    constexpr int32 SIPP_MAX_SEARCH_SLACK = 8192;
    constexpr double SIPP_TIME_BUDGET_MS = 100.0;

    FIntVector NormalizeMinCellSipp(const FIntVector& A, const FIntVector& B)
    {
        return FIntVector(
            FMath::Min(A.X, B.X),
            FMath::Min(A.Y, B.Y),
            FMath::Min(A.Z, B.Z));
    }

    FIntVector NormalizeMaxCellSipp(const FIntVector& A, const FIntVector& B)
    {
        return FIntVector(
            FMath::Max(A.X, B.X),
            FMath::Max(A.Y, B.Y),
            FMath::Max(A.Z, B.Z));
    }
}

FSIPPPlanner::FSIPPPlanner(const TArray<FTemporalNoFlyZoneConfig>& InNoFlyZoneConfigs)
    : NoFlyZoneConfigs(InNoFlyZoneConfigs)
{
}

float FSIPPPlanner::Heuristic(const FIntVector& A, const FIntVector& B) const
{
    return
        FMath::Abs(A.X - B.X) +
        FMath::Abs(A.Y - B.Y) +
        FMath::Abs(A.Z - B.Z);
}

void FSIPPPlanner::BuildBlockedIntervals(
    const FGridMap3D& GridMap,
    TMap<FIntVector, TArray<FSippInterval>>& OutBlockedIntervalsByCell,
    int32& OutMaxBlockedTime) const
{
    OutBlockedIntervalsByCell.Reset();
    OutMaxBlockedTime = 0;

    for (const FTemporalNoFlyZoneConfig& Zone : NoFlyZoneConfigs)
    {
        if (!Zone.bEnabled)
        {
            continue;
        }

        const int32 StartTime = FMath::Max(0, Zone.StartTimeStep);
        const int32 EndTime = FMath::Max(StartTime, Zone.EndTimeStep);

        FIntVector MinCell = NormalizeMinCellSipp(Zone.MinCell, Zone.MaxCell);
        FIntVector MaxCell = NormalizeMaxCellSipp(Zone.MinCell, Zone.MaxCell);

        MinCell.X = FMath::Clamp(MinCell.X, 0, FMath::Max(0, GridMap.GridDim.X - 1));
        MinCell.Y = FMath::Clamp(MinCell.Y, 0, FMath::Max(0, GridMap.GridDim.Y - 1));
        MinCell.Z = FMath::Clamp(MinCell.Z, 0, FMath::Max(0, GridMap.GridDim.Z - 1));
        MaxCell.X = FMath::Clamp(MaxCell.X, 0, FMath::Max(0, GridMap.GridDim.X - 1));
        MaxCell.Y = FMath::Clamp(MaxCell.Y, 0, FMath::Max(0, GridMap.GridDim.Y - 1));
        MaxCell.Z = FMath::Clamp(MaxCell.Z, 0, FMath::Max(0, GridMap.GridDim.Z - 1));

        if (MinCell.X > MaxCell.X || MinCell.Y > MaxCell.Y || MinCell.Z > MaxCell.Z)
        {
            continue;
        }

        OutMaxBlockedTime = FMath::Max(OutMaxBlockedTime, EndTime);

        for (int32 X = MinCell.X; X <= MaxCell.X; ++X)
        {
            for (int32 Y = MinCell.Y; Y <= MaxCell.Y; ++Y)
            {
                for (int32 Z = MinCell.Z; Z <= MaxCell.Z; ++Z)
                {
                    FSippInterval Interval;
                    Interval.Start = StartTime;
                    Interval.End = EndTime;
                    OutBlockedIntervalsByCell.FindOrAdd(FIntVector(X, Y, Z)).Add(Interval);
                }
            }
        }
    }

    for (auto& KVP : OutBlockedIntervalsByCell)
    {
        MergeIntervals(KVP.Value);
    }
}

void FSIPPPlanner::MergeIntervals(TArray<FSippInterval>& InOutIntervals) const
{
    if (InOutIntervals.Num() <= 1)
    {
        return;
    }

    InOutIntervals.Sort([](const FSippInterval& A, const FSippInterval& B)
        {
            if (A.Start != B.Start)
            {
                return A.Start < B.Start;
            }

            return A.End < B.End;
        });

    TArray<FSippInterval> Merged;
    Merged.Reserve(InOutIntervals.Num());
    Merged.Add(InOutIntervals[0]);

    for (int32 Index = 1; Index < InOutIntervals.Num(); ++Index)
    {
        const FSippInterval& Candidate = InOutIntervals[Index];
        FSippInterval& Tail = Merged.Last();

        if (Candidate.Start <= Tail.End + 1)
        {
            Tail.End = FMath::Max(Tail.End, Candidate.End);
            continue;
        }

        Merged.Add(Candidate);
    }

    InOutIntervals = MoveTemp(Merged);
}

const TArray<FSIPPPlanner::FSippInterval>& FSIPPPlanner::GetSafeIntervalsForCell(
    const FIntVector& Cell,
    const TMap<FIntVector, TArray<FSippInterval>>& BlockedIntervalsByCell,
    TMap<FIntVector, TArray<FSippInterval>>& InOutSafeIntervalCache) const
{
    if (const TArray<FSippInterval>* CachedIntervals = InOutSafeIntervalCache.Find(Cell))
    {
        return *CachedIntervals;
    }

    TArray<FSippInterval>& SafeIntervals = InOutSafeIntervalCache.FindOrAdd(Cell);
    const TArray<FSippInterval>* BlockedIntervals = BlockedIntervalsByCell.Find(Cell);

    if (!BlockedIntervals || BlockedIntervals->Num() <= 0)
    {
        FSippInterval FullInterval;
        FullInterval.Start = 0;
        FullInterval.End = InfiniteIntervalEnd;
        SafeIntervals.Add(FullInterval);
        return SafeIntervals;
    }

    int32 CurrentStart = 0;
    for (const FSippInterval& Blocked : *BlockedIntervals)
    {
        if (CurrentStart < Blocked.Start)
        {
            FSippInterval Safe;
            Safe.Start = CurrentStart;
            Safe.End = Blocked.Start - 1;
            SafeIntervals.Add(Safe);
        }

        CurrentStart = FMath::Max(CurrentStart, Blocked.End + 1);
    }

    if (CurrentStart <= InfiniteIntervalEnd)
    {
        FSippInterval Safe;
        Safe.Start = CurrentStart;
        Safe.End = InfiniteIntervalEnd;
        SafeIntervals.Add(Safe);
    }

    return SafeIntervals;
}

bool FSIPPPlanner::ReconstructCellPath(
    const TMap<FSippStateKey, FSippParentInfo>& ParentByState,
    const TMap<FSippStateKey, int32>& ArrivalByState,
    const FSippStateKey& GoalState,
    TArray<FIntVector>& OutCellPath) const
{
    OutCellPath.Reset();

    struct FStateVisit
    {
        FSippStateKey State;
        int32 ArrivalTime = 0;
    };

    TArray<FStateVisit> ReverseVisits;

    FSippStateKey WalkState = GoalState;
    while (true)
    {
        const int32* ArrivalTimePtr = ArrivalByState.Find(WalkState);
        if (!ArrivalTimePtr)
        {
            return false;
        }

        FStateVisit Visit;
        Visit.State = WalkState;
        Visit.ArrivalTime = *ArrivalTimePtr;
        ReverseVisits.Add(Visit);

        const FSippParentInfo* ParentInfo = ParentByState.Find(WalkState);
        if (!ParentInfo || !ParentInfo->bHasParent)
        {
            break;
        }

        WalkState = ParentInfo->ParentState;
    }

    Algo::Reverse(ReverseVisits);
    if (ReverseVisits.Num() <= 0)
    {
        return false;
    }

    int64 EstimatedPointCount = 1;
    for (int32 Index = 1; Index < ReverseVisits.Num(); ++Index)
    {
        const int64 Delta = static_cast<int64>(ReverseVisits[Index].ArrivalTime) - ReverseVisits[Index - 1].ArrivalTime;
        EstimatedPointCount += FMath::Max<int64>(1, Delta);
        if (EstimatedPointCount > SIPP_MAX_RECONSTRUCTED_PATH_POINTS)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("SIPP: reconstructed path exceeds point budget. Points=%lld Budget=%d"),
                EstimatedPointCount,
                SIPP_MAX_RECONSTRUCTED_PATH_POINTS);
            return false;
        }
    }

    OutCellPath.Reserve(static_cast<int32>(EstimatedPointCount));
    OutCellPath.Add(ReverseVisits[0].State.Cell);
    int32 CurrentTime = ReverseVisits[0].ArrivalTime;

    for (int32 Index = 1; Index < ReverseVisits.Num(); ++Index)
    {
        const FStateVisit& Previous = ReverseVisits[Index - 1];
        const FStateVisit& Current = ReverseVisits[Index];

        for (int32 TimeStep = CurrentTime + 1; TimeStep < Current.ArrivalTime; ++TimeStep)
        {
            OutCellPath.Add(Previous.State.Cell);
        }

        OutCellPath.Add(Current.State.Cell);
        CurrentTime = Current.ArrivalTime;
    }

    return OutCellPath.Num() > 0;
}

TArray<FVector> FSIPPPlanner::CellPathToWorldPath(
    const FGridMap3D& GridMap,
    const TArray<FIntVector>& CellPath) const
{
    TArray<FVector> WorldPath;
    WorldPath.Reserve(CellPath.Num());

    for (const FIntVector& Cell : CellPath)
    {
        WorldPath.Add(GridMap.CellToWorld(Cell));
    }

    return WorldPath;
}

bool FSIPPPlanner::Plan(
    const FGridMap3D& GridMap,
    const FVector& StartWorld,
    const FVector& GoalWorld,
    TArray<FVector>& OutPath)
{
    OutPath.Reset();

    const FIntVector StartCell = GridMap.WorldToCell(StartWorld);
    const FIntVector GoalCell = GridMap.WorldToCell(GoalWorld);

    if (!GridMap.IsInside(StartCell.X, StartCell.Y, StartCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("SIPP: StartCell is outside grid"));
        return false;
    }

    if (!GridMap.IsInside(GoalCell.X, GoalCell.Y, GoalCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("SIPP: GoalCell is outside grid"));
        return false;
    }

    if (GridMap.IsBlocked(StartCell.X, StartCell.Y, StartCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("SIPP: StartCell is statically blocked"));
        return false;
    }

    if (GridMap.IsBlocked(GoalCell.X, GoalCell.Y, GoalCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("SIPP: GoalCell is statically blocked"));
        return false;
    }

    TMap<FIntVector, TArray<FSippInterval>> BlockedIntervalsByCell;
    int32 MaxBlockedTime = 0;
    BuildBlockedIntervals(GridMap, BlockedIntervalsByCell, MaxBlockedTime);

    TMap<FIntVector, TArray<FSippInterval>> SafeIntervalCache;

    const TArray<FSippInterval>& StartIntervals = GetSafeIntervalsForCell(StartCell, BlockedIntervalsByCell, SafeIntervalCache);

    const FSippInterval* StartInterval = nullptr;
    for (const FSippInterval& Interval : StartIntervals)
    {
        if (0 >= Interval.Start && 0 <= Interval.End)
        {
            StartInterval = &Interval;
            break;
        }
    }

    if (!StartInterval)
    {
        UE_LOG(LogTemp, Error, TEXT("SIPP: StartCell is blocked at t=0"));
        return false;
    }

    const int32 SpatialLowerBound = static_cast<int32>(Heuristic(StartCell, GoalCell));
    const int32 TotalCells = GridMap.GridDim.X * GridMap.GridDim.Y * GridMap.GridDim.Z;
    const int32 SearchSlack = FMath::Clamp(TotalCells / 8, 64, SIPP_MAX_SEARCH_SLACK);
    const int32 SearchDeadline =
        FMath::Max(MaxBlockedTime + 1, SpatialLowerBound) +
        SearchSlack;

    auto OpenPredicate = [](const FSippOpenEntry& A, const FSippOpenEntry& B)
        {
            if (A.FScore != B.FScore)
            {
                return A.FScore < B.FScore;
            }

            return A.ArrivalTime < B.ArrivalTime;
        };

    TArray<FSippOpenEntry> OpenHeap;
    TMap<FSippStateKey, int32> ArrivalByState;
    TMap<FSippStateKey, FSippParentInfo> ParentByState;
    const double SearchStartTime = FPlatformTime::Seconds();
    int32 ExpandedStates = 0;
    int32 GeneratedStates = 1;
    int32 MaxOpenSize = 1;

    FSippStateKey StartState;
    StartState.Cell = StartCell;
    StartState.IntervalStart = StartInterval->Start;
    StartState.IntervalEnd = StartInterval->End;

    ArrivalByState.Add(StartState, 0);

    FSippOpenEntry StartEntry;
    StartEntry.State = StartState;
    StartEntry.ArrivalTime = 0;
    StartEntry.FScore = Heuristic(StartCell, GoalCell);
    OpenHeap.HeapPush(StartEntry, OpenPredicate);

    static const FIntVector Directions[6] =
    {
        FIntVector(1, 0, 0),
        FIntVector(-1, 0, 0),
        FIntVector(0, 1, 0),
        FIntVector(0, -1, 0),
        FIntVector(0, 0, 1),
        FIntVector(0, 0, -1)
    };

    int32 GuardCounter = 0;
    while (OpenHeap.Num() > 0)
    {
        if (++GuardCounter > SIPP_GUARD_LIMIT)
        {
            const double ElapsedMs = (FPlatformTime::Seconds() - SearchStartTime) * 1000.0;
            UE_LOG(
                LogTemp,
                Error,
                TEXT("SIPP: exceeded guard limit. Expanded=%d Generated=%d Open=%d ElapsedMs=%.2f"),
                ExpandedStates,
                GeneratedStates,
                OpenHeap.Num(),
                ElapsedMs);
            return false;
        }

        if ((GuardCounter & 0xFF) == 0)
        {
            const double ElapsedMs = (FPlatformTime::Seconds() - SearchStartTime) * 1000.0;
            if (ElapsedMs > SIPP_TIME_BUDGET_MS)
            {
                UE_LOG(
                    LogTemp,
                    Error,
                    TEXT("SIPP: aborted by time budget. Expanded=%d Generated=%d Open=%d MaxOpen=%d Deadline=%d ElapsedMs=%.2f BudgetMs=%.2f"),
                    ExpandedStates,
                    GeneratedStates,
                    OpenHeap.Num(),
                    MaxOpenSize,
                    SearchDeadline,
                    ElapsedMs,
                    SIPP_TIME_BUDGET_MS);
                return false;
            }
        }

        while (OpenHeap.Num() > 0)
        {
            const FSippOpenEntry& Top = OpenHeap[0];
            const int32* BestArrivalPtr = ArrivalByState.Find(Top.State);
            if (!BestArrivalPtr || *BestArrivalPtr != Top.ArrivalTime)
            {
                FSippOpenEntry Discarded;
                OpenHeap.HeapPop(Discarded, OpenPredicate);
                continue;
            }

            break;
        }

        if (OpenHeap.Num() <= 0)
        {
            break;
        }

        FSippOpenEntry CurrentEntry;
        OpenHeap.HeapPop(CurrentEntry, OpenPredicate);
        ExpandedStates++;

        if (CurrentEntry.State.Cell == GoalCell && CurrentEntry.State.IntervalEnd >= MaxBlockedTime)
        {
            TArray<FIntVector> CellPath;
            if (!ReconstructCellPath(ParentByState, ArrivalByState, CurrentEntry.State, CellPath))
            {
                return false;
            }

            OutPath = CellPathToWorldPath(GridMap, CellPath);
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("SIPP: path found. Arrival=%d LowerBound=%d MaxBlockedTime=%d PathPoints=%d Expanded=%d"),
                CurrentEntry.ArrivalTime,
                SpatialLowerBound,
                MaxBlockedTime,
                OutPath.Num(),
                ExpandedStates);
            return OutPath.Num() > 0;
        }

        for (const FIntVector& Direction : Directions)
        {
            const FIntVector NextCell = CurrentEntry.State.Cell + Direction;
            if (!GridMap.IsInside(NextCell.X, NextCell.Y, NextCell.Z))
            {
                continue;
            }

            if (GridMap.IsBlocked(NextCell.X, NextCell.Y, NextCell.Z))
            {
                continue;
            }

            const TArray<FSippInterval>& NextIntervals = GetSafeIntervalsForCell(NextCell, BlockedIntervalsByCell, SafeIntervalCache);
            for (const FSippInterval& Interval : NextIntervals)
            {
                const int32 EarliestArrival = FMath::Max(CurrentEntry.ArrivalTime + 1, Interval.Start);
                const int32 DepartureTime = EarliestArrival - 1;

                if (DepartureTime > CurrentEntry.State.IntervalEnd)
                {
                    break;
                }

                if (EarliestArrival > Interval.End)
                {
                    continue;
                }

                if (EarliestArrival > SearchDeadline)
                {
                    continue;
                }

                FSippStateKey NextState;
                NextState.Cell = NextCell;
                NextState.IntervalStart = Interval.Start;
                NextState.IntervalEnd = Interval.End;

                const int32* BestArrivalPtr = ArrivalByState.Find(NextState);
                if (BestArrivalPtr && *BestArrivalPtr <= EarliestArrival)
                {
                    continue;
                }

                ArrivalByState.Add(NextState, EarliestArrival);

                FSippParentInfo ParentInfo;
                ParentInfo.ParentState = CurrentEntry.State;
                ParentInfo.ParentArrivalTime = CurrentEntry.ArrivalTime;
                ParentInfo.bHasParent = true;
                ParentByState.Add(NextState, ParentInfo);

                FSippOpenEntry NextEntry;
                NextEntry.State = NextState;
                NextEntry.ArrivalTime = EarliestArrival;
                NextEntry.FScore = static_cast<float>(EarliestArrival) + Heuristic(NextCell, GoalCell);
                OpenHeap.HeapPush(NextEntry, OpenPredicate);
                GeneratedStates++;
                MaxOpenSize = FMath::Max(MaxOpenSize, OpenHeap.Num());
            }
        }
    }

    const double ElapsedMs = (FPlatformTime::Seconds() - SearchStartTime) * 1000.0;
    UE_LOG(
        LogTemp,
        Warning,
        TEXT("SIPP: no path found. Expanded=%d Generated=%d MaxOpen=%d Deadline=%d ElapsedMs=%.2f"),
        ExpandedStates,
        GeneratedStates,
        MaxOpenSize,
        SearchDeadline,
        ElapsedMs);
    return false;
}
