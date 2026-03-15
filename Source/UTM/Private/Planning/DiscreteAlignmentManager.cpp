#include "Planning/DiscreteAlignmentManager.h"

#include "Planning/GridMap3D.h"

namespace
{
    int32 SignInt(int32 Value)
    {
        return Value > 0 ? 1 : (Value < 0 ? -1 : 0);
    }
}

FDiscreteAlignmentManager::FDiscreteAlignmentManager(const FDiscreteAlignmentSettings& InSettings)
    : Settings(InSettings)
{
}

void FDiscreteAlignmentManager::SetSettings(const FDiscreteAlignmentSettings& InSettings)
{
    Settings = InSettings;
}

FDiscreteAlignmentResult FDiscreteAlignmentManager::AlignStep(
    const FGridMap3D& GridMap,
    const TArray<FIntVector>& PlannedCells,
    int32 PreviousPlanIndex,
    int32 TimeStep,
    const FIntVector& ObservedCell,
    bool bDelayRequested) const
{
    FDiscreteAlignmentResult Result;
    Result.ObservedCell = ObservedCell;

    if (PlannedCells.Num() <= 0)
    {
        Result.Reason = TEXT("planned path is empty");
        return Result;
    }

    const int32 LastPlanIndex = PlannedCells.Num() - 1;
    const int32 ExpectedPlanIndex = FMath::Clamp(TimeStep, 0, LastPlanIndex);

    if (!Settings.bEnabled)
    {
        Result.bValid = true;
        Result.ReferencePlanIndex = FMath::Clamp(PreviousPlanIndex, 0, LastPlanIndex);
        Result.NextPlanIndex = bDelayRequested
            ? Result.ReferencePlanIndex
            : FMath::Min(Result.ReferencePlanIndex + 1, LastPlanIndex);
        Result.ReferenceCell = PlannedCells[Result.ReferencePlanIndex];
        Result.NextCell = PlannedCells[Result.NextPlanIndex];
        Result.SpatialErrorCells = ManhattanDistance(ObservedCell, Result.ReferenceCell);
        Result.TemporalErrorSteps = Result.ReferencePlanIndex - ExpectedPlanIndex;
        Result.Action = bDelayRequested ? EDiscreteAlignmentAction::HoldForDelay : EDiscreteAlignmentAction::FollowPlan;
        return Result;
    }

    Result.ReferencePlanIndex = FindBestPlanIndex(PlannedCells, PreviousPlanIndex, TimeStep, ObservedCell);
    Result.ReferenceCell = PlannedCells[Result.ReferencePlanIndex];
    Result.SpatialErrorCells = ManhattanDistance(ObservedCell, Result.ReferenceCell);
    Result.TemporalErrorSteps = Result.ReferencePlanIndex - ExpectedPlanIndex;

    if (bDelayRequested)
    {
        Result.bValid = true;
        Result.Action = EDiscreteAlignmentAction::HoldForDelay;
        Result.NextCell = ObservedCell;
        Result.NextPlanIndex = Result.ReferencePlanIndex;
        return Result;
    }

    if (Result.SpatialErrorCells == 0)
    {
        Result.bValid = true;
        Result.NextPlanIndex = FMath::Min(Result.ReferencePlanIndex + 1, LastPlanIndex);
        Result.NextCell = PlannedCells[Result.NextPlanIndex];

        if (Result.ReferencePlanIndex >= LastPlanIndex)
        {
            Result.Action = EDiscreteAlignmentAction::GoalHold;
            Result.NextCell = ObservedCell;
            Result.NextPlanIndex = LastPlanIndex;
        }
        else if (Result.ReferencePlanIndex > PreviousPlanIndex)
        {
            Result.Action = EDiscreteAlignmentAction::SnapToPlanIndex;
        }
        else
        {
            Result.Action = EDiscreteAlignmentAction::FollowPlan;
        }

        return Result;
    }

    if (Result.SpatialErrorCells <= Settings.MaxSpatialErrorCells && Settings.bAllowRecoveryMoves)
    {
        FIntVector RecoveryCell = ObservedCell;
        if (ComputeRecoveryCell(GridMap, ObservedCell, Result.ReferenceCell, RecoveryCell))
        {
            Result.bValid = true;
            Result.Action = EDiscreteAlignmentAction::RecoverTowardPlan;
            Result.NextCell = RecoveryCell;
            Result.NextPlanIndex = Result.ReferencePlanIndex;
            Result.bRequiresReplan = false;
            Result.Reason = TEXT("recover toward nearest planned cell");
            return Result;
        }
    }

    Result.bRequiresReplan = true;
    Result.Reason = FString::Printf(
        TEXT("spatial error %d exceeds alignment tolerance %d"),
        Result.SpatialErrorCells,
        Settings.MaxSpatialErrorCells);

    if (Settings.bHoldPositionOnFailure)
    {
        Result.bValid = true;
        Result.Action = EDiscreteAlignmentAction::HoldForAlignment;
        Result.NextCell = ObservedCell;
        Result.NextPlanIndex = Result.ReferencePlanIndex;
    }

    return Result;
}

const TCHAR* FDiscreteAlignmentManager::LexToString(EDiscreteAlignmentAction Action)
{
    switch (Action)
    {
    case EDiscreteAlignmentAction::FollowPlan:
        return TEXT("FollowPlan");
    case EDiscreteAlignmentAction::HoldForDelay:
        return TEXT("HoldForDelay");
    case EDiscreteAlignmentAction::SnapToPlanIndex:
        return TEXT("SnapToPlanIndex");
    case EDiscreteAlignmentAction::RecoverTowardPlan:
        return TEXT("RecoverTowardPlan");
    case EDiscreteAlignmentAction::HoldForAlignment:
        return TEXT("HoldForAlignment");
    case EDiscreteAlignmentAction::GoalHold:
        return TEXT("GoalHold");
    default:
        return TEXT("Unknown");
    }
}

int32 FDiscreteAlignmentManager::FindBestPlanIndex(
    const TArray<FIntVector>& PlannedCells,
    int32 PreviousPlanIndex,
    int32 TimeStep,
    const FIntVector& ObservedCell) const
{
    const int32 LastPlanIndex = PlannedCells.Num() - 1;
    const int32 ClampedPrevious = FMath::Clamp(PreviousPlanIndex, 0, LastPlanIndex);
    const int32 ExpectedPlanIndex = FMath::Clamp(TimeStep, 0, LastPlanIndex);

    const int32 SearchStart = FMath::Max(0, FMath::Min(ClampedPrevious, ExpectedPlanIndex) - Settings.SearchRadiusSteps);
    const int32 SearchEnd = FMath::Min(
        LastPlanIndex,
        FMath::Max(ClampedPrevious + Settings.MaxSnapAheadSteps, ExpectedPlanIndex) + Settings.SearchRadiusSteps);

    int32 BestIndex = ClampedPrevious;
    int32 BestScore = MAX_int32;

    for (int32 CandidateIndex = SearchStart; CandidateIndex <= SearchEnd; ++CandidateIndex)
    {
        const int32 SpatialCost = ManhattanDistance(ObservedCell, PlannedCells[CandidateIndex]);
        const int32 TemporalCost = FMath::Abs(CandidateIndex - ExpectedPlanIndex);
        const int32 DriftCost = FMath::Abs(CandidateIndex - ClampedPrevious);
        const int32 SnapAheadPenalty =
            CandidateIndex > ClampedPrevious + Settings.MaxSnapAheadSteps
            ? (CandidateIndex - (ClampedPrevious + Settings.MaxSnapAheadSteps)) * 1000
            : 0;

        const int32 Score = SpatialCost * 100 + TemporalCost * 10 + DriftCost + SnapAheadPenalty;
        if (Score < BestScore)
        {
            BestScore = Score;
            BestIndex = CandidateIndex;
        }
    }

    return BestIndex;
}

bool FDiscreteAlignmentManager::ComputeRecoveryCell(
    const FGridMap3D& GridMap,
    const FIntVector& FromCell,
    const FIntVector& TargetCell,
    FIntVector& OutRecoveryCell) const
{
    TArray<TPair<int32, FIntVector>> Candidates;
    Candidates.Reserve(3);

    const FIntVector Delta = TargetCell - FromCell;
    if (Delta.X != 0)
    {
        Candidates.Emplace(FMath::Abs(Delta.X), FIntVector(FromCell.X + SignInt(Delta.X), FromCell.Y, FromCell.Z));
    }
    if (Delta.Y != 0)
    {
        Candidates.Emplace(FMath::Abs(Delta.Y), FIntVector(FromCell.X, FromCell.Y + SignInt(Delta.Y), FromCell.Z));
    }
    if (Delta.Z != 0)
    {
        Candidates.Emplace(FMath::Abs(Delta.Z), FIntVector(FromCell.X, FromCell.Y, FromCell.Z + SignInt(Delta.Z)));
    }

    Candidates.Sort(
        [](const TPair<int32, FIntVector>& A, const TPair<int32, FIntVector>& B)
        {
            return A.Key > B.Key;
        });

    for (const TPair<int32, FIntVector>& Candidate : Candidates)
    {
        const FIntVector& Cell = Candidate.Value;
        if (!GridMap.IsInside(Cell.X, Cell.Y, Cell.Z))
        {
            continue;
        }

        if (GridMap.IsBlocked(Cell.X, Cell.Y, Cell.Z))
        {
            continue;
        }

        OutRecoveryCell = Cell;
        return true;
    }

    return false;
}

int32 FDiscreteAlignmentManager::ManhattanDistance(const FIntVector& A, const FIntVector& B)
{
    return FMath::Abs(A.X - B.X)
        + FMath::Abs(A.Y - B.Y)
        + FMath::Abs(A.Z - B.Z);
}
