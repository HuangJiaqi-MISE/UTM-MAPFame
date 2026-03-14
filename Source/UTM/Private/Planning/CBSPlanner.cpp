#include "Planning/CBSPlanner.h"
#include "Planning/GridMap3D.h"
#include "Planning/AStarPlanner.h"
#include "Algo/Reverse.h"

namespace
{
    constexpr float INF_COST = 1e9f;

    bool ValidateMissionIdsUnique(const TArray<FDroneMissionConfig>& Missions)
    {
        TSet<int32> SeenMissionIds;

        for (const FDroneMissionConfig& Mission : Missions)
        {
            if (Mission.MissionId <= 0)
            {
                UE_LOG(LogTemp, Error, TEXT("CBS: MissionId must be positive. Invalid MissionId=%d"), Mission.MissionId);
                return false;
            }

            if (SeenMissionIds.Contains(Mission.MissionId))
            {
                UE_LOG(LogTemp, Error, TEXT("CBS: duplicate MissionId detected: %d"), Mission.MissionId);
                return false;
            }

            SeenMissionIds.Add(Mission.MissionId);
        }

        return true;
    }
}

bool FCBSPlanner::PlanMissions(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    TMap<int32, TArray<FVector>>& OutPaths)
{
    OutPaths.Reset();

    if (Missions.Num() <= 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("CBS: no missions to plan"));
        return false;
    }

    if (!ValidateMissionIdsUnique(Missions))
    {
        return false;
    }

    FCBSNode RootNode;
    if (!BuildRootNode(GridMap, Missions, RootNode))
    {
        UE_LOG(LogTemp, Error, TEXT("CBS: failed to build root node"));
        return false;
    }

    TArray<FCBSNode> OpenNodes;
    OpenNodes.Add(RootNode);

    int32 IterationGuard = 0;
    const int32 MaxIterations = 512;

    while (OpenNodes.Num() > 0)
    {
        if (++IterationGuard > MaxIterations)
        {
            UE_LOG(LogTemp, Error, TEXT("CBS: exceeded iteration guard"));
            return false;
        }

        int32 BestNodeIdx = 0;
        for (int32 i = 1; i < OpenNodes.Num(); ++i)
        {
            if (OpenNodes[i].Cost < OpenNodes[BestNodeIdx].Cost)
            {
                BestNodeIdx = i;
            }
        }

        FCBSNode Current = OpenNodes[BestNodeIdx];
        OpenNodes.RemoveAtSwap(BestNodeIdx);

        const FCBSConflict Conflict = FindFirstConflict(Current.PathsByAgent);

        if (!Conflict.bValid)
        {
            UE_LOG(LogTemp, Warning, TEXT("CBS: conflict-free solution found. Cost=%d"), Current.Cost);

            for (const auto& KVP : Current.PathsByAgent)
            {
                OutPaths.Add(KVP.Key, CellPathToWorldPath(GridMap, KVP.Value));
            }

            return true;
        }

        if (!Conflict.bIsEdgeConflict)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("CBS: branching on vertex conflict. AgentA=%d AgentB=%d Time=%d Cell=(%d,%d,%d)"),
                Conflict.AgentA,
                Conflict.AgentB,
                Conflict.TimeStep,
                Conflict.Cell.X,
                Conflict.Cell.Y,
                Conflict.Cell.Z
            );

            FCBSNode ChildA = Current;
            {
                FCBSConstraint Constraint;
                Constraint.AgentId = Conflict.AgentA;
                Constraint.Cell = Conflict.Cell;
                Constraint.TimeStep = Conflict.TimeStep;
                Constraint.bIsEdgeConstraint = false;

                ChildA.Constraints.Add(Constraint);

                if (ReplanSingleAgentForNode(GridMap, Missions, Conflict.AgentA, ChildA))
                {
                    ChildA.Cost = ComputeSolutionCost(ChildA.PathsByAgent);
                    OpenNodes.Add(ChildA);
                }
                else
                {
                    UE_LOG(LogTemp, Warning, TEXT("CBS: failed to replan child for AgentA=%d"), Conflict.AgentA);
                }
            }

            FCBSNode ChildB = Current;
            {
                FCBSConstraint Constraint;
                Constraint.AgentId = Conflict.AgentB;
                Constraint.Cell = Conflict.Cell;
                Constraint.TimeStep = Conflict.TimeStep;
                Constraint.bIsEdgeConstraint = false;

                ChildB.Constraints.Add(Constraint);

                if (ReplanSingleAgentForNode(GridMap, Missions, Conflict.AgentB, ChildB))
                {
                    ChildB.Cost = ComputeSolutionCost(ChildB.PathsByAgent);
                    OpenNodes.Add(ChildB);
                }
                else
                {
                    UE_LOG(LogTemp, Warning, TEXT("CBS: failed to replan child for AgentB=%d"), Conflict.AgentB);
                }
            }
        }
        else
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("CBS: branching on edge conflict. AgentA=%d AgentB=%d Time=%d A:(%d,%d,%d)->(%d,%d,%d) B:(%d,%d,%d)->(%d,%d,%d)"),
                Conflict.AgentA,
                Conflict.AgentB,
                Conflict.TimeStep,
                Conflict.FromA.X, Conflict.FromA.Y, Conflict.FromA.Z,
                Conflict.ToA.X, Conflict.ToA.Y, Conflict.ToA.Z,
                Conflict.FromB.X, Conflict.FromB.Y, Conflict.FromB.Z,
                Conflict.ToB.X, Conflict.ToB.Y, Conflict.ToB.Z
            );

            FCBSNode ChildA = Current;
            {
                FCBSConstraint Constraint;
                Constraint.AgentId = Conflict.AgentA;
                Constraint.TimeStep = Conflict.TimeStep;
                Constraint.bIsEdgeConstraint = true;
                Constraint.FromCell = Conflict.FromA;
                Constraint.ToCell = Conflict.ToA;

                ChildA.Constraints.Add(Constraint);

                if (ReplanSingleAgentForNode(GridMap, Missions, Conflict.AgentA, ChildA))
                {
                    ChildA.Cost = ComputeSolutionCost(ChildA.PathsByAgent);
                    OpenNodes.Add(ChildA);
                }
                else
                {
                    UE_LOG(LogTemp, Warning, TEXT("CBS: failed to replan edge-constrained child for AgentA=%d"), Conflict.AgentA);
                }
            }

            FCBSNode ChildB = Current;
            {
                FCBSConstraint Constraint;
                Constraint.AgentId = Conflict.AgentB;
                Constraint.TimeStep = Conflict.TimeStep;
                Constraint.bIsEdgeConstraint = true;
                Constraint.FromCell = Conflict.FromB;
                Constraint.ToCell = Conflict.ToB;

                ChildB.Constraints.Add(Constraint);

                if (ReplanSingleAgentForNode(GridMap, Missions, Conflict.AgentB, ChildB))
                {
                    ChildB.Cost = ComputeSolutionCost(ChildB.PathsByAgent);
                    OpenNodes.Add(ChildB);
                }
                else
                {
                    UE_LOG(LogTemp, Warning, TEXT("CBS: failed to replan edge-constrained child for AgentB=%d"), Conflict.AgentB);
                }
            }
        }


    }

    UE_LOG(LogTemp, Error, TEXT("CBS: open list exhausted without solution"));
    return false;
}

bool FCBSPlanner::BuildRootNode(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    FCBSNode& OutRootNode) const
{
    OutRootNode.Constraints.Reset();
    OutRootNode.PathsByAgent.Reset();
    OutRootNode.Cost = 0;

    FAStarPlanner LowLevelPlanner;

    for (const FDroneMissionConfig& Mission : Missions)
    {
        TArray<FVector> WorldPath;

        const bool bFound = LowLevelPlanner.Plan(
            GridMap,
            Mission.StartWorld,
            Mission.GoalWorld,
            WorldPath
        );

        if (!bFound)
        {
            UE_LOG(LogTemp, Error, TEXT("CBS: root planning failed for MissionId=%d"), Mission.MissionId);
            return false;
        }

        const TArray<FIntVector> CellPath = WorldPathToCellPath(GridMap, WorldPath);
        if (CellPath.Num() <= 0)
        {
            UE_LOG(LogTemp, Error, TEXT("CBS: empty root path for MissionId=%d"), Mission.MissionId);
            return false;
        }

        OutRootNode.PathsByAgent.Add(Mission.MissionId, CellPath);

        UE_LOG(LogTemp, Warning, TEXT("CBS root path built for MissionId=%d, Steps=%d"),
            Mission.MissionId,
            CellPath.Num());
    }

    OutRootNode.Cost = ComputeSolutionCost(OutRootNode.PathsByAgent);
    return true;
}

bool FCBSPlanner::ReplanSingleAgentForNode(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    int32 AgentId,
    FCBSNode& InOutNode) const
{
    const FDroneMissionConfig* MissionPtr = nullptr;

    for (const FDroneMissionConfig& Mission : Missions)
    {
        if (Mission.MissionId == AgentId)
        {
            MissionPtr = &Mission;
            break;
        }
    }

    if (!MissionPtr)
    {
        UE_LOG(LogTemp, Error, TEXT("CBS: mission not found for AgentId=%d"), AgentId);
        return false;
    }

    const TArray<FIntVector>* ExistingPath = InOutNode.PathsByAgent.Find(AgentId);

    TArray<FIntVector> NewPath;
    if (!LowLevelPlanForAgent(GridMap, *MissionPtr, InOutNode.Constraints, ExistingPath, NewPath))
    {
        return false;
    }

    InOutNode.PathsByAgent.Add(AgentId, NewPath);
    return true;
}

FCBSPlanner::FCBSConflict FCBSPlanner::FindFirstConflict(
    const TMap<int32, TArray<FIntVector>>& PathsByAgent) const
{
    FCBSConflict Conflict;

    TArray<int32> AgentIds;
    PathsByAgent.GetKeys(AgentIds);
    AgentIds.Sort();

    int32 MaxTime = 0;
    for (const auto& KVP : PathsByAgent)
    {
        MaxTime = FMath::Max(MaxTime, KVP.Value.Num());
    }

    for (int32 TimeStep = 0; TimeStep < MaxTime; ++TimeStep)
    {
        for (int32 i = 0; i < AgentIds.Num(); ++i)
        {
            for (int32 j = i + 1; j < AgentIds.Num(); ++j)
            {
                const int32 AgentA = AgentIds[i];
                const int32 AgentB = AgentIds[j];

                const TArray<FIntVector>* PathA = PathsByAgent.Find(AgentA);
                const TArray<FIntVector>* PathB = PathsByAgent.Find(AgentB);

                if (!PathA || !PathB)
                {
                    continue;
                }

                const FIntVector CellA = GetCellAtTime(*PathA, TimeStep);
                const FIntVector CellB = GetCellAtTime(*PathB, TimeStep);

                if (CellA == CellB)
                {
                    Conflict.bValid = true;
                    Conflict.AgentA = AgentA;
                    Conflict.AgentB = AgentB;
                    Conflict.TimeStep = TimeStep;
                    Conflict.bIsEdgeConflict = false;
                    Conflict.Cell = CellA;
                    return Conflict;
                }

                if (TimeStep > 0)
                {
                    const FIntVector PrevA = GetCellAtTime(*PathA, TimeStep - 1);
                    const FIntVector PrevB = GetCellAtTime(*PathB, TimeStep - 1);

                    if (PrevA == CellB && PrevB == CellA)
                    {
                        Conflict.bValid = true;
                        Conflict.AgentA = AgentA;
                        Conflict.AgentB = AgentB;
                        Conflict.TimeStep = TimeStep;
                        Conflict.bIsEdgeConflict = true;

                        Conflict.FromA = PrevA;
                        Conflict.ToA = CellA;
                        Conflict.FromB = PrevB;
                        Conflict.ToB = CellB;
                        return Conflict;
                    }
                }
            }
        }
    }

    return Conflict;
}

int32 FCBSPlanner::ComputeSolutionCost(
    const TMap<int32, TArray<FIntVector>>& PathsByAgent) const
{
    int32 TotalCost = 0;

    for (const auto& KVP : PathsByAgent)
    {
        TotalCost += KVP.Value.Num();
    }

    return TotalCost;
}

TArray<FIntVector> FCBSPlanner::WorldPathToCellPath(
    const FGridMap3D& GridMap,
    const TArray<FVector>& WorldPath) const
{
    TArray<FIntVector> CellPath;
    CellPath.Reserve(WorldPath.Num());

    for (const FVector& P : WorldPath)
    {
        CellPath.Add(GridMap.WorldToCell(P));
    }

    return CellPath;
}

TArray<FVector> FCBSPlanner::CellPathToWorldPath(
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

FIntVector FCBSPlanner::GetCellAtTime(
    const TArray<FIntVector>& Path,
    int32 TimeStep) const
{
    if (Path.Num() <= 0)
    {
        return FIntVector::ZeroValue;
    }

    if (TimeStep < Path.Num())
    {
        return Path[TimeStep];
    }

    return Path.Last();
}

bool FCBSPlanner::IsVertexConstrained(
    int32 AgentId,
    const FIntVector& Cell,
    int32 TimeStep,
    const TArray<FCBSConstraint>& Constraints) const
{
    for (const FCBSConstraint& Constraint : Constraints)
    {
        if (Constraint.AgentId != AgentId)
        {
            continue;
        }

        if (Constraint.bIsEdgeConstraint)
        {
            continue;
        }

        if (Constraint.TimeStep == TimeStep && Constraint.Cell == Cell)
        {
            return true;
        }
    }

    return false;
}

bool FCBSPlanner::IsEdgeConstrained(
    int32 AgentId,
    const FIntVector& FromCell,
    const FIntVector& ToCell,
    int32 TimeStep,
    const TArray<FCBSConstraint>& Constraints) const
{
    for (const FCBSConstraint& Constraint : Constraints)
    {
        if (Constraint.AgentId != AgentId)
        {
            continue;
        }

        if (!Constraint.bIsEdgeConstraint)
        {
            continue;
        }

        if (Constraint.TimeStep != TimeStep)
        {
            continue;
        }

        if (Constraint.FromCell == FromCell && Constraint.ToCell == ToCell)
        {
            return true;
        }
    }

    return false;
}


bool FCBSPlanner::LowLevelPlanForAgent(
    const FGridMap3D& GridMap,
    const FDroneMissionConfig& Mission,
    const TArray<FCBSConstraint>& Constraints,
    const TArray<FIntVector>* ExistingPath,
    TArray<FIntVector>& OutPath) const
{
    OutPath.Reset();

    const FIntVector StartCell = GridMap.WorldToCell(Mission.StartWorld);
    const FIntVector GoalCell = GridMap.WorldToCell(Mission.GoalWorld);

    if (!GridMap.IsInside(StartCell.X, StartCell.Y, StartCell.Z) ||
        !GridMap.IsInside(GoalCell.X, GoalCell.Y, GoalCell.Z))
    {
        return false;
    }

    if (GridMap.IsBlocked(StartCell.X, StartCell.Y, StartCell.Z) ||
        GridMap.IsBlocked(GoalCell.X, GoalCell.Y, GoalCell.Z))
    {
        return false;
    }

    const FCompiledConstraints Compiled = CompileConstraintsForAgent(Mission.MissionId, Constraints);
    const int32 TotalCells = GridMap.GridDim.X * GridMap.GridDim.Y * GridMap.GridDim.Z;

    if (TotalCells <= 0)
    {
        UE_LOG(LogTemp, Error, TEXT("CBS low-level: invalid grid size for AgentId=%d"), Mission.MissionId);
        return false;
    }

    if (Compiled.IsVertexForbidden(StartCell, 0))
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("CBS low-level: start cell is constrained at t=0 for AgentId=%d"),
            Mission.MissionId);
        return false;
    }

    auto Heuristic = [](const FIntVector& A, const FIntVector& B) -> int32
        {
            return
                FMath::Abs(A.X - B.X) +
                FMath::Abs(A.Y - B.Y) +
                FMath::Abs(A.Z - B.Z);
        };

    const int32 SpatialLowerBound = Heuristic(StartCell, GoalCell);
    const int32 HoldingTime = Compiled.GetHoldingTime(GoalCell, SpatialLowerBound);

    // Static layer must not start before the goal holding time.
    // Otherwise time folding can happen before the goal is legally occupiable,
    // causing the low-level search to spin and hit the guard limit.
    const int32 StaticTimestep = Compiled.GetStaticTimestep(HoldingTime);

    const int32 IncumbentCost =
        (ExistingPath && ExistingPath->Num() > 0)
        ? FMath::Max(0, ExistingPath->Num() - 1)
        : SpatialLowerBound;

    int32 HardUpperBound = FMath::Max(HoldingTime, IncumbentCost + 64);
    HardUpperBound = FMath::Max(HardUpperBound, SpatialLowerBound + FMath::Clamp(GridMap.GridDim.X + GridMap.GridDim.Y + GridMap.GridDim.Z, 32, 256));
    HardUpperBound = FMath::Min(HardUpperBound, SpatialLowerBound + FMath::Clamp(TotalCells / 32, 128, 2048));
    HardUpperBound = FMath::Max(HardUpperBound, HoldingTime);

    TArray<int32> CostLimits;
    CostLimits.Reserve(5);

    auto AddCostLimit = [&CostLimits, HoldingTime, HardUpperBound](int32 Candidate)
        {
            const int32 Clamped = FMath::Clamp(FMath::Max(Candidate, HoldingTime), HoldingTime, HardUpperBound);
            if (!CostLimits.Contains(Clamped))
            {
                CostLimits.Add(Clamped);
            }
        };

    AddCostLimit(IncumbentCost + 8);
    AddCostLimit(IncumbentCost + 16);
    AddCostLimit(IncumbentCost + 32);
    AddCostLimit(IncumbentCost + 64);
    AddCostLimit(HardUpperBound);
    CostLimits.Sort();

    struct FSearchNode
    {
        int32 G = MAX_int32;
        int32 H = 0;
        FCBSPlanner::FStateKey Parent;
        bool bHasParent = false;
    };

    struct FOpenEntry
    {
        FCBSPlanner::FStateKey Key;
        int32 G = 0;
        int32 H = 0;

        int32 GetF() const
        {
            return G + H;
        }

        bool operator<(const FOpenEntry& Other) const
        {
            if (GetF() != Other.GetF())
            {
                return GetF() > Other.GetF();
            }

            if (H != Other.H)
            {
                return H > Other.H;
            }

            if (G != Other.G)
            {
                return G < Other.G;
            }

            if (Key.TimeStep != Other.Key.TimeStep)
            {
                return Key.TimeStep > Other.Key.TimeStep;
            }

            if (Key.Cell != Other.Key.Cell)
            {
                if (Key.Cell.X != Other.Key.Cell.X)
                {
                    return Key.Cell.X > Other.Key.Cell.X;
                }
                if (Key.Cell.Y != Other.Key.Cell.Y)
                {
                    return Key.Cell.Y > Other.Key.Cell.Y;
                }
                return Key.Cell.Z > Other.Key.Cell.Z;
            }

            return false;
        }
    };

    auto TrySearchWithCostLimit =
        [&](int32 LengthMax, TArray<FIntVector>& CandidatePath) -> bool
        {
            CandidatePath.Reset();

            if (SpatialLowerBound > LengthMax || HoldingTime > LengthMax)
            {
                return false;
            }

            TArray<FOpenEntry> OpenHeap;
            TMap<FStateKey, FSearchNode> NodeTable;

            const FStateKey StartKey{ StartCell, 0 };
            FSearchNode& StartNode = NodeTable.Add(StartKey);
            StartNode.G = 0;
            StartNode.H = Heuristic(StartCell, GoalCell);
            StartNode.bHasParent = false;

            OpenHeap.HeapPush(FOpenEntry{ StartKey, StartNode.G, StartNode.H });

            int32 GuardCounter = 0;
            const int32 GuardLimit = 100000;

            while (OpenHeap.Num() > 0)
            {
                FOpenEntry CurrentEntry;
                OpenHeap.HeapPop(CurrentEntry);

                FSearchNode* CurrentNode = NodeTable.Find(CurrentEntry.Key);
                if (!CurrentNode)
                {
                    continue;
                }

                if (CurrentNode->G != CurrentEntry.G)
                {
                    continue;
                }

                if (++GuardCounter > GuardLimit)
                {
                    UE_LOG(
                        LogTemp,
                        Error,
                        TEXT("CBS low-level exceeded guard limit for AgentId=%d LengthMax=%d Holding=%d StaticT=%d Constraints=%d Open=%d"),
                        Mission.MissionId,
                        LengthMax,
                        HoldingTime,
                        StaticTimestep,
                        Constraints.Num(),
                        OpenHeap.Num());
                    return false;
                }

                if (CurrentEntry.Key.Cell == GoalCell && CurrentEntry.Key.TimeStep >= HoldingTime)
                {
                    TArray<FIntVector> ReversePath;
                    FStateKey WalkKey = CurrentEntry.Key;

                    while (true)
                    {
                        const FSearchNode* WalkNode = NodeTable.Find(WalkKey);
                        if (!WalkNode)
                        {
                            break;
                        }

                        ReversePath.Add(WalkKey.Cell);
                        if (!WalkNode->bHasParent)
                        {
                            break;
                        }

                        WalkKey = WalkNode->Parent;
                    }

                    Algo::Reverse(ReversePath);
                    CandidatePath = MoveTemp(ReversePath);
                    return CandidatePath.Num() > 0;
                }

                TArray<FIntVector, TInlineAllocator<7>> CandidateCells;
                CandidateCells.Add(CurrentEntry.Key.Cell); // wait

                static const FIntVector Dirs[6] =
                {
                    FIntVector(1, 0, 0),
                    FIntVector(-1, 0, 0),
                    FIntVector(0, 1, 0),
                    FIntVector(0, -1, 0),
                    FIntVector(0, 0, 1),
                    FIntVector(0, 0, -1)
                };

                for (const FIntVector& Dir : Dirs)
                {
                    const FIntVector Next = CurrentEntry.Key.Cell + Dir;
                    if (!GridMap.IsInside(Next.X, Next.Y, Next.Z))
                    {
                        continue;
                    }

                    if (GridMap.IsBlocked(Next.X, Next.Y, Next.Z))
                    {
                        continue;
                    }

                    CandidateCells.Add(Next);
                }

                for (const FIntVector& NextCell : CandidateCells)
                {
                    int32 NextTime = CurrentEntry.Key.TimeStep + 1;
                    const bool bIsWaitAction = (NextCell == CurrentEntry.Key.Cell);

                    if (StaticTimestep < NextTime)
                    {
                        if (bIsWaitAction)
                        {
                            continue;
                        }

                        NextTime--;
                    }

                    if (Compiled.IsVertexForbidden(NextCell, NextTime))
                    {
                        continue;
                    }

                    if (Compiled.IsEdgeForbidden(CurrentEntry.Key.Cell, NextCell, NextTime))
                    {
                        continue;
                    }

                    const int32 TentativeG = CurrentNode->G + 1;
                    const int32 NextH = Heuristic(NextCell, GoalCell);
                    if (TentativeG + NextH > LengthMax)
                    {
                        continue;
                    }

                    const FStateKey NextKey{ NextCell, NextTime };
                    FSearchNode* ExistingNode = NodeTable.Find(NextKey);
                    if (ExistingNode && TentativeG >= ExistingNode->G)
                    {
                        continue;
                    }

                    FSearchNode* NextNode = ExistingNode;
                    if (!NextNode)
                    {
                        NextNode = &NodeTable.Add(NextKey);
                    }

                    NextNode->G = TentativeG;
                    NextNode->H = NextH;
                    NextNode->Parent = CurrentEntry.Key;
                    NextNode->bHasParent = true;

                    OpenHeap.HeapPush(FOpenEntry{ NextKey, TentativeG, NextH });
                }
            }

            return false;
        };

    for (const int32 CostLimit : CostLimits)
    {
        TArray<FIntVector> CandidatePath;
        if (TrySearchWithCostLimit(CostLimit, CandidatePath))
        {
            OutPath = MoveTemp(CandidatePath);
            return true;
        }
    }

    return false;
}



FCBSPlanner::FCompiledConstraints FCBSPlanner::CompileConstraintsForAgent(
    int32 AgentId,
    const TArray<FCBSConstraint>& Constraints) const
{
    FCompiledConstraints Result;

    for (const FCBSConstraint& C : Constraints)
    {
        if (C.AgentId != AgentId)
        {
            continue;
        }

        Result.MaxConstraintTime = FMath::Max(Result.MaxConstraintTime, C.TimeStep);

        if (!C.bIsEdgeConstraint)
        {
            Result.VertexByTime.FindOrAdd(C.TimeStep).Add(C.Cell);
        }
        else
        {
            FEdgeConstraintKey Key;
            Key.From = C.FromCell;
            Key.To = C.ToCell;
            Result.EdgeByTime.FindOrAdd(C.TimeStep).Add(Key);
        }
    }

    return Result;
}
