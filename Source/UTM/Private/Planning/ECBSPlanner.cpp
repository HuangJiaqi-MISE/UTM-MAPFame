#include "Planning/ECBSPlanner.h"

#include "Algo/Reverse.h"
#include "Planning/GridMap3D.h"

#include "Planning/AStarPlanner.h"

namespace
{
    constexpr bool GECBSVerboseLogs = false;

    bool ValidateMissionIdsUniqueECBS(const TArray<FDroneMissionConfig>& Missions)
    {
        TSet<int32> SeenMissionIds;

        for (const FDroneMissionConfig& Mission : Missions)
        {
            if (Mission.MissionId <= 0)
            {
                UE_LOG(LogTemp, Error, TEXT("ECBS: MissionId must be positive. Invalid MissionId=%d"), Mission.MissionId);
                return false;
            }

            if (SeenMissionIds.Contains(Mission.MissionId))
            {
                UE_LOG(LogTemp, Error, TEXT("ECBS: duplicate MissionId detected: %d"), Mission.MissionId);
                return false;
            }

            SeenMissionIds.Add(Mission.MissionId);
        }

        return true;
    }
}

FECBSPlanner::FECBSPlanner(float InSuboptimalityBound)
    : SuboptimalityBound(FMath::Max(1.0f, InSuboptimalityBound))
{
}

bool FECBSPlanner::PlanMissions(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    TMap<int32, TArray<FVector>>& OutPaths)
{
    OutPaths.Reset();

    if (Missions.Num() <= 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("ECBS: no missions to plan"));
        return false;
    }

    if (!ValidateMissionIdsUniqueECBS(Missions))
    {
        return false;
    }

    FECBSNode RootNode;
    if (!BuildRootNode(GridMap, Missions, RootNode))
    {
        UE_LOG(LogTemp, Error, TEXT("ECBS: failed to build root node"));
        return false;
    }

    struct FHighLevelOpenEntry
    {
        int32 NodeId = INDEX_NONE;
        int32 Cost = 0;
    };

    struct FHighLevelFocalEntry
    {
        int32 NodeId = INDEX_NONE;
        int32 ConflictCount = 0;
        int32 Cost = 0;
    };

    auto OpenPredicate = [](const FHighLevelOpenEntry& A, const FHighLevelOpenEntry& B)
        {
            return A.Cost > B.Cost;
        };

    auto FocalPredicate = [](const FHighLevelFocalEntry& A, const FHighLevelFocalEntry& B)
        {
            if (A.ConflictCount != B.ConflictCount)
            {
                return A.ConflictCount > B.ConflictCount;
            }

            return A.Cost > B.Cost;
        };

    TArray<FECBSNode> NodeStore;
    NodeStore.Add(RootNode);

    TSet<int32> ActiveNodeIds;
    ActiveNodeIds.Add(0);

    auto BuildPathSignature = [](const FECBSNode& Node) -> FString
        {
            TArray<int32> AgentIds;
            Node.PathsByAgent.GetKeys(AgentIds);
            AgentIds.Sort();

            FString Signature;
            for (const int32 AgentId : AgentIds)
            {
                Signature += FString::Printf(TEXT("A%d:"), AgentId);

                const TArray<FIntVector>* PathPtr = Node.PathsByAgent.Find(AgentId);
                if (!PathPtr)
                {
                    Signature += TEXT("|");
                    continue;
                }

                for (const FIntVector& Cell : *PathPtr)
                {
                    Signature += FString::Printf(TEXT("%d,%d,%d;"), Cell.X, Cell.Y, Cell.Z);
                }

                Signature += TEXT("|");
            }

            return Signature;
        };

    TSet<FString> VisitedPathSignatures;
    VisitedPathSignatures.Add(BuildPathSignature(RootNode));

    TArray<FHighLevelOpenEntry> OpenHeap;
    TArray<FHighLevelFocalEntry> FocalHeap;

    FHighLevelOpenEntry RootOpenEntry;
    RootOpenEntry.NodeId = 0;
    RootOpenEntry.Cost = RootNode.Cost;
    OpenHeap.HeapPush(RootOpenEntry, OpenPredicate);

    float CurrentFocalBound = SuboptimalityBound * static_cast<float>(RootNode.Cost);

    FHighLevelFocalEntry RootFocalEntry;
    RootFocalEntry.NodeId = 0;
    RootFocalEntry.ConflictCount = RootNode.ConflictCount;
    RootFocalEntry.Cost = RootNode.Cost;
    FocalHeap.HeapPush(RootFocalEntry, FocalPredicate);

    auto RefreshFocalHeap = [this, &NodeStore, &ActiveNodeIds, &FocalHeap, &FocalPredicate](float FocalBound)
        {
            FocalHeap.Reset();

            for (int32 NodeId : ActiveNodeIds)
            {
                FECBSNode& Node = NodeStore[NodeId];
                if (static_cast<float>(Node.Cost) > FocalBound)
                {
                    continue;
                }

                if (Node.ConflictCount == INDEX_NONE)
                {
                    Node.ConflictCount = ComputeTotalConflictCount(Node.PathsByAgent);
                }

                FHighLevelFocalEntry Entry;
                Entry.NodeId = NodeId;
                Entry.ConflictCount = Node.ConflictCount;
                Entry.Cost = Node.Cost;
                FocalHeap.HeapPush(Entry, FocalPredicate);
            }
        };

    auto TryPopValidFocalNode = [&FocalHeap, &FocalPredicate, &ActiveNodeIds, &NodeStore](float FocalBound) -> int32
        {
            while (FocalHeap.Num() > 0)
            {
                FHighLevelFocalEntry CandidateEntry;
                FocalHeap.HeapPop(CandidateEntry, FocalPredicate);

                if (!ActiveNodeIds.Contains(CandidateEntry.NodeId))
                {
                    continue;
                }

                const FECBSNode& CandidateNode = NodeStore[CandidateEntry.NodeId];
                if (static_cast<float>(CandidateNode.Cost) > FocalBound)
                {
                    continue;
                }

                if (CandidateNode.ConflictCount != CandidateEntry.ConflictCount || CandidateNode.Cost != CandidateEntry.Cost)
                {
                    continue;
                }

                return CandidateEntry.NodeId;
            }

            return INDEX_NONE;
        };

    int32 IterationGuard = 0;
    const int32 MaxIterations = FMath::Clamp(Missions.Num() * Missions.Num() * 4, 2048, 20000);
    const double SearchStartTimeSec = FPlatformTime::Seconds();
	// ECBS 的搜索时间预算可以根据任务数量进行调整。每个任务增加都会增加搜索空间的复杂度，因此时间预算应该适当增加。
    // 但同时也需要设置一个上限，防止在极端情况下搜索时间过长。
    // 这里上限为30秒
    const double MaxWallTimeSec = FMath::Clamp(static_cast<double>(Missions.Num()) * 0.05, 1.5, 30.0);

    while (ActiveNodeIds.Num() > 0)
    {
        const double ElapsedTimeSec = FPlatformTime::Seconds() - SearchStartTimeSec;
        if (ElapsedTimeSec > MaxWallTimeSec)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("ECBS: aborted by time budget. Expanded=%d Active=%d Open=%d UniquePaths=%d Elapsed=%.2fs Budget=%.2fs"),
                IterationGuard,
                ActiveNodeIds.Num(),
                OpenHeap.Num(),
                VisitedPathSignatures.Num(),
                ElapsedTimeSec,
                MaxWallTimeSec);
            return false;
        }

        if (++IterationGuard > MaxIterations)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("ECBS: exceeded iteration guard. Expanded=%d MaxIterations=%d Active=%d Open=%d UniquePaths=%d"),
                IterationGuard - 1,
                MaxIterations,
                ActiveNodeIds.Num(),
                OpenHeap.Num(),
                VisitedPathSignatures.Num());
            return false;
        }

        if (GECBSVerboseLogs && (IterationGuard % 256) == 0)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("ECBS progress: Expanded=%d Active=%d Open=%d UniquePaths=%d Elapsed=%.2fs"),
                IterationGuard,
                ActiveNodeIds.Num(),
                OpenHeap.Num(),
                VisitedPathSignatures.Num(),
                ElapsedTimeSec);
        }

        while (OpenHeap.Num() > 0)
        {
            const FHighLevelOpenEntry& Top = OpenHeap[0];
            if (!ActiveNodeIds.Contains(Top.NodeId) || NodeStore[Top.NodeId].Cost != Top.Cost)
            {
                FHighLevelOpenEntry Discarded;
                OpenHeap.HeapPop(Discarded, OpenPredicate);
                continue;
            }

            break;
        }

        if (OpenHeap.Num() <= 0)
        {
            break;
        }

        const int32 BestCost = OpenHeap[0].Cost;
        const float NewFocalBound = SuboptimalityBound * static_cast<float>(BestCost);
        if (NewFocalBound > CurrentFocalBound)
        {
            CurrentFocalBound = NewFocalBound;
            RefreshFocalHeap(CurrentFocalBound);
        }

        int32 SelectedNodeId = TryPopValidFocalNode(CurrentFocalBound);

























        if (SelectedNodeId == INDEX_NONE)
        {
            RefreshFocalHeap(CurrentFocalBound);
            SelectedNodeId = TryPopValidFocalNode(CurrentFocalBound);
























        }

        if (SelectedNodeId == INDEX_NONE)
        {
            UE_LOG(LogTemp, Error, TEXT("ECBS: focal list is empty"));
            return false;
        }

        FECBSNode Current = NodeStore[SelectedNodeId];
        ActiveNodeIds.Remove(SelectedNodeId);

        struct FPendingChildNode
        {
            FECBSNode Node;
            int32 AgentId = INDEX_NONE;
        };

        auto BuildChildNode =
            [this, &GridMap, &Missions](const FECBSNode& ParentNode, int32 AgentId, const FECBSConstraint& Constraint, FECBSNode& OutChild)
            {
                OutChild = ParentNode;
                OutChild.Constraints.Add(Constraint);

                if (!ReplanSingleAgentForNode(GridMap, Missions, AgentId, OutChild))
                {
                    UE_LOG(LogTemp, Warning, TEXT("ECBS: failed to replan child for AgentId=%d"), AgentId);
                    return false;
                }

                OutChild.Cost = ComputeSolutionCost(OutChild.PathsByAgent);
                OutChild.ConflictCount = ComputeTotalConflictCount(OutChild.PathsByAgent);
                return true;
            };

        auto EnqueueChildNode =
            [&NodeStore, &ActiveNodeIds, &OpenHeap, &OpenPredicate, &FocalHeap, &FocalPredicate, &BuildPathSignature, &VisitedPathSignatures, CurrentFocalBound](FECBSNode&& Child)
            {
                const FString PathSignature = BuildPathSignature(Child);
                if (VisitedPathSignatures.Contains(PathSignature))
                {
                    return false;
                }

                VisitedPathSignatures.Add(PathSignature);

                const int32 ChildNodeId = NodeStore.Add(MoveTemp(Child));
                ActiveNodeIds.Add(ChildNodeId);

                FHighLevelOpenEntry OpenEntry;
                OpenEntry.NodeId = ChildNodeId;
                OpenEntry.Cost = NodeStore[ChildNodeId].Cost;
                OpenHeap.HeapPush(OpenEntry, OpenPredicate);

                if (static_cast<float>(NodeStore[ChildNodeId].Cost) <= CurrentFocalBound)
                {
                    FHighLevelFocalEntry FocalEntry;
                    FocalEntry.NodeId = ChildNodeId;
                    FocalEntry.ConflictCount = NodeStore[ChildNodeId].ConflictCount;
                    FocalEntry.Cost = NodeStore[ChildNodeId].Cost;
                    FocalHeap.HeapPush(FocalEntry, FocalPredicate);
                }

                return true;
            };

        TSet<FString> LocalBypassSignatures;
        LocalBypassSignatures.Add(BuildPathSignature(Current));

        while (true)
        {
            const FECBSConflict Conflict = FindFirstConflict(Current.PathsByAgent);
            if (!Conflict.bValid)
            {
                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("ECBS: conflict-free solution found. Cost=%d ConflictCount=%d Bound=%.2f"),
                    Current.Cost,
                    Current.ConflictCount,
                    SuboptimalityBound);

                for (const auto& KVP : Current.PathsByAgent)
                {
                    OutPaths.Add(KVP.Key, CellPathToWorldPath(GridMap, KVP.Value));
                }

                return true;
            }

            const int32 CurrentConflictCount =
                (Current.ConflictCount != INDEX_NONE)
                ? Current.ConflictCount
                : ComputeTotalConflictCount(Current.PathsByAgent);

            TArray<FPendingChildNode, TInlineAllocator<2>> PendingChildren;

            if (!Conflict.bIsEdgeConflict)
            {
                if (GECBSVerboseLogs)
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("ECBS: branching on vertex conflict. AgentA=%d AgentB=%d Time=%d Cell=(%d,%d,%d)"),
                        Conflict.AgentA,
                        Conflict.AgentB,
                        Conflict.TimeStep,
                        Conflict.Cell.X,
                        Conflict.Cell.Y,
                        Conflict.Cell.Z);
                }

                FECBSConstraint ConstraintA;
                ConstraintA.AgentId = Conflict.AgentA;
                ConstraintA.Cell = Conflict.Cell;
                ConstraintA.TimeStep = Conflict.TimeStep;
                FECBSNode ChildA;
                if (BuildChildNode(Current, Conflict.AgentA, ConstraintA, ChildA))
                {
                    FPendingChildNode& PendingChild = PendingChildren.AddDefaulted_GetRef();
                    PendingChild.Node = MoveTemp(ChildA);
                    PendingChild.AgentId = Conflict.AgentA;
                }

                FECBSConstraint ConstraintB;
                ConstraintB.AgentId = Conflict.AgentB;
                ConstraintB.Cell = Conflict.Cell;
                ConstraintB.TimeStep = Conflict.TimeStep;
                FECBSNode ChildB;
                if (BuildChildNode(Current, Conflict.AgentB, ConstraintB, ChildB))
                {
                    FPendingChildNode& PendingChild = PendingChildren.AddDefaulted_GetRef();
                    PendingChild.Node = MoveTemp(ChildB);
                    PendingChild.AgentId = Conflict.AgentB;
                }
            }
            else
            {
                if (GECBSVerboseLogs)
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("ECBS: branching on edge conflict. AgentA=%d AgentB=%d Time=%d A:(%d,%d,%d)->(%d,%d,%d) B:(%d,%d,%d)->(%d,%d,%d)"),
                        Conflict.AgentA,
                        Conflict.AgentB,
                        Conflict.TimeStep,
                        Conflict.FromA.X, Conflict.FromA.Y, Conflict.FromA.Z,
                        Conflict.ToA.X, Conflict.ToA.Y, Conflict.ToA.Z,
                        Conflict.FromB.X, Conflict.FromB.Y, Conflict.FromB.Z,
                        Conflict.ToB.X, Conflict.ToB.Y, Conflict.ToB.Z);
                }

                FECBSConstraint ConstraintA;
                ConstraintA.AgentId = Conflict.AgentA;
                ConstraintA.TimeStep = Conflict.TimeStep;
                ConstraintA.bIsEdgeConstraint = true;
                ConstraintA.FromCell = Conflict.FromA;
                ConstraintA.ToCell = Conflict.ToA;
                FECBSNode ChildA;
                if (BuildChildNode(Current, Conflict.AgentA, ConstraintA, ChildA))
                {
                    FPendingChildNode& PendingChild = PendingChildren.AddDefaulted_GetRef();
                    PendingChild.Node = MoveTemp(ChildA);
                    PendingChild.AgentId = Conflict.AgentA;
                }

                FECBSConstraint ConstraintB;
                ConstraintB.AgentId = Conflict.AgentB;
                ConstraintB.TimeStep = Conflict.TimeStep;
                ConstraintB.bIsEdgeConstraint = true;
                ConstraintB.FromCell = Conflict.FromB;
                ConstraintB.ToCell = Conflict.ToB;
                FECBSNode ChildB;
                if (BuildChildNode(Current, Conflict.AgentB, ConstraintB, ChildB))
                {
                    FPendingChildNode& PendingChild = PendingChildren.AddDefaulted_GetRef();
                    PendingChild.Node = MoveTemp(ChildB);
                    PendingChild.AgentId = Conflict.AgentB;
                }
            }

            int32 BestBypassIdx = INDEX_NONE;
            for (int32 ChildIdx = 0; ChildIdx < PendingChildren.Num(); ++ChildIdx)
            {
                const FECBSNode& CandidateChild = PendingChildren[ChildIdx].Node;
                if (static_cast<float>(CandidateChild.Cost) > CurrentFocalBound ||
                    CandidateChild.ConflictCount >= CurrentConflictCount)
                {
                    continue;
                }

                const FString CandidateSignature = BuildPathSignature(CandidateChild);
                if (LocalBypassSignatures.Contains(CandidateSignature))
                {
                    continue;
                }

                if (BestBypassIdx == INDEX_NONE ||
                    CandidateChild.ConflictCount < PendingChildren[BestBypassIdx].Node.ConflictCount ||
                    (CandidateChild.ConflictCount == PendingChildren[BestBypassIdx].Node.ConflictCount &&
                        CandidateChild.Cost < PendingChildren[BestBypassIdx].Node.Cost))
                {
                    BestBypassIdx = ChildIdx;
                }
            }

            if (BestBypassIdx != INDEX_NONE)
            {
                const int32 PreviousCost = Current.Cost;
                const int32 BypassAgentId = PendingChildren[BestBypassIdx].AgentId;
                const int32 BypassCost = PendingChildren[BestBypassIdx].Node.Cost;
                const int32 BypassConflictCount = PendingChildren[BestBypassIdx].Node.ConflictCount;
                const FString BypassSignature = BuildPathSignature(PendingChildren[BestBypassIdx].Node);
                LocalBypassSignatures.Add(BypassSignature);
                Current = MoveTemp(PendingChildren[BestBypassIdx].Node);

                if (GECBSVerboseLogs)
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("ECBS: bypass applied. AgentId=%d Cost=%d->%d ConflictCount=%d->%d"),
                        BypassAgentId,
                        PreviousCost,
                        BypassCost,
                        CurrentConflictCount,
                        BypassConflictCount);
                }
                continue;
            }

            for (FPendingChildNode& PendingChild : PendingChildren)
            {
                EnqueueChildNode(MoveTemp(PendingChild.Node));
            }
            break;
        }
    }

    UE_LOG(LogTemp, Error, TEXT("ECBS: open list exhausted without solution"));
    return false;
}

bool FECBSPlanner::BuildRootNode(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    FECBSNode& OutRootNode) const
{
    OutRootNode.Constraints.Reset();
    OutRootNode.PathsByAgent.Reset();
    OutRootNode.Cost = 0;
    OutRootNode.ConflictCount = 0;

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
            UE_LOG(LogTemp, Error, TEXT("ECBS: root planning failed for MissionId=%d"), Mission.MissionId);
            return false;
        }

        const TArray<FIntVector> CellPath = WorldPathToCellPath(GridMap, WorldPath);
        if (CellPath.Num() <= 0)
        {
            UE_LOG(LogTemp, Error, TEXT("ECBS: empty root path for MissionId=%d"), Mission.MissionId);
            return false;
        }

        OutRootNode.PathsByAgent.Add(Mission.MissionId, CellPath);

        if (GECBSVerboseLogs)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("ECBS root independent path built for MissionId=%d, Steps=%d"),
                Mission.MissionId,
                CellPath.Num());
        }
    }

    OutRootNode.Cost = ComputeSolutionCost(OutRootNode.PathsByAgent);
    OutRootNode.ConflictCount = ComputeTotalConflictCount(OutRootNode.PathsByAgent);
    return true;
}

bool FECBSPlanner::ReplanSingleAgentForNode(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    int32 AgentId,
    FECBSNode& InOutNode) const
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
        UE_LOG(LogTemp, Error, TEXT("ECBS: mission not found for AgentId=%d"), AgentId);
        return false;
    }

    TArray<FIntVector> NewPath;
    if (!LowLevelPlanForAgent(GridMap, *MissionPtr, InOutNode.Constraints, InOutNode.PathsByAgent, NewPath))
    {
        return false;
    }

    InOutNode.PathsByAgent.Add(AgentId, NewPath);
    return true;
}

FECBSPlanner::FECBSConflict FECBSPlanner::FindFirstConflict(
    const TMap<int32, TArray<FIntVector>>& PathsByAgent) const
{
    TArray<int32> AgentIds;
    PathsByAgent.GetKeys(AgentIds);
    AgentIds.Sort();

    TArray<FECBSConflict> Conflicts;
    Conflicts.Reserve(FMath::Max(0, AgentIds.Num() * (AgentIds.Num() - 1) / 2));

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

            const int32 MaxTime = FMath::Max(PathA->Num(), PathB->Num());
            for (int32 TimeStep = 0; TimeStep < MaxTime; ++TimeStep)
            {
                const FIntVector CellA = GetCellAtTime(*PathA, TimeStep);
                const FIntVector CellB = GetCellAtTime(*PathB, TimeStep);

                if (CellA == CellB)
                {
                    FECBSConflict Conflict;
                    Conflict.bValid = true;
                    Conflict.AgentA = AgentA;
                    Conflict.AgentB = AgentB;
                    Conflict.TimeStep = TimeStep;
                    Conflict.Cell = CellA;

                    Conflicts.Add(Conflict);
                    break;
                }

                if (TimeStep > 0)
                {
                    const FIntVector PrevA = GetCellAtTime(*PathA, TimeStep - 1);
                    const FIntVector PrevB = GetCellAtTime(*PathB, TimeStep - 1);

                    if (PrevA == CellB && PrevB == CellA)
                    {
                        FECBSConflict Conflict;
                        Conflict.bValid = true;
                        Conflict.AgentA = AgentA;
                        Conflict.AgentB = AgentB;
                        Conflict.TimeStep = TimeStep;
                        Conflict.bIsEdgeConflict = true;
                        Conflict.FromA = PrevA;
                        Conflict.ToA = CellA;
                        Conflict.FromB = PrevB;
                        Conflict.ToB = CellB;
                        Conflicts.Add(Conflict);
                        break;
                    }
                }
            }
        }
    }

    FECBSConflict SelectedConflict;
    if (Conflicts.Num() <= 0)
    {
        return SelectedConflict;
    }

    TMap<int32, int32> ConflictDegreeByAgent;
    TMap<FIntVector, int32> VertexHotspotCount;

    for (const FECBSConflict& Conflict : Conflicts)
    {
        ConflictDegreeByAgent.FindOrAdd(Conflict.AgentA)++;
        ConflictDegreeByAgent.FindOrAdd(Conflict.AgentB)++;

        if (!Conflict.bIsEdgeConflict)
        {
            VertexHotspotCount.FindOrAdd(Conflict.Cell)++;
        }
    }

    auto GetConflictDegree = [&ConflictDegreeByAgent](int32 AgentId) -> int32
        {
            const int32* DegreePtr = ConflictDegreeByAgent.Find(AgentId);
            return DegreePtr ? *DegreePtr : 0;
        };

    auto GetVertexHotspot = [&VertexHotspotCount](const FIntVector& Cell) -> int32
        {
            const int32* CountPtr = VertexHotspotCount.Find(Cell);
            return CountPtr ? *CountPtr : 0;
        };

    auto Heuristic = [](const FIntVector& A, const FIntVector& B) -> int32
        {
            return
                FMath::Abs(A.X - B.X) +
                FMath::Abs(A.Y - B.Y) +
                FMath::Abs(A.Z - B.Z);
        };

    auto EstimateVertexWindow = [&Heuristic](const TArray<FIntVector>& Path, const FIntVector& Cell, int32 TimeStep) -> int32
        {
            if (Path.Num() <= 0)
            {
                return MAX_int32 / 4;
            }

            const int32 PathCost = FMath::Max(0, Path.Num() - 1);
            if (TimeStep >= PathCost && Cell == Path.Last())
            {
                return 0;
            }

            const int32 Earliest = Heuristic(Path[0], Cell);
            const int32 Latest = PathCost - Heuristic(Cell, Path.Last());
            if (Latest < Earliest)
            {
                return (MAX_int32 / 8) + (Earliest - Latest);
            }

            if (TimeStep < Earliest || TimeStep > Latest)
            {
                const int32 ClampedTime = FMath::Clamp(TimeStep, Earliest, Latest);
                return (MAX_int32 / 8) + FMath::Abs(TimeStep - ClampedTime);
            }

            return Latest - Earliest;
        };

    auto EstimateEdgeWindow = [&Heuristic](const TArray<FIntVector>& Path, const FIntVector& FromCell, const FIntVector& ToCell, int32 TimeStep) -> int32
        {
            if (Path.Num() <= 0)
            {
                return MAX_int32 / 4;
            }

            const int32 PathCost = FMath::Max(0, Path.Num() - 1);
            const int32 Earliest = Heuristic(Path[0], FromCell) + 1;
            const int32 Latest = PathCost - Heuristic(ToCell, Path.Last());
            if (Latest < Earliest)
            {
                return (MAX_int32 / 8) + (Earliest - Latest);
            }

            if (TimeStep < Earliest || TimeStep > Latest)
            {
                const int32 ClampedTime = FMath::Clamp(TimeStep, Earliest, Latest);
                return (MAX_int32 / 8) + FMath::Abs(TimeStep - ClampedTime);
            }

            return Latest - Earliest;
        };

    int32 BestConflictIdx = 0;
    for (int32 CandidateIdx = 1; CandidateIdx < Conflicts.Num(); ++CandidateIdx)
    {
        const FECBSConflict& Candidate = Conflicts[CandidateIdx];
        const FECBSConflict& Best = Conflicts[BestConflictIdx];

        const TArray<FIntVector>* CandidatePathA = PathsByAgent.Find(Candidate.AgentA);
        const TArray<FIntVector>* CandidatePathB = PathsByAgent.Find(Candidate.AgentB);
        const TArray<FIntVector>* BestPathA = PathsByAgent.Find(Best.AgentA);
        const TArray<FIntVector>* BestPathB = PathsByAgent.Find(Best.AgentB);

        const int32 CandidateWindowA =
            (CandidatePathA != nullptr)
            ? (Candidate.bIsEdgeConflict
                ? EstimateEdgeWindow(*CandidatePathA, Candidate.FromA, Candidate.ToA, Candidate.TimeStep)
                : EstimateVertexWindow(*CandidatePathA, Candidate.Cell, Candidate.TimeStep))
            : MAX_int32 / 4;
        const int32 CandidateWindowB =
            (CandidatePathB != nullptr)
            ? (Candidate.bIsEdgeConflict
                ? EstimateEdgeWindow(*CandidatePathB, Candidate.FromB, Candidate.ToB, Candidate.TimeStep)
                : EstimateVertexWindow(*CandidatePathB, Candidate.Cell, Candidate.TimeStep))
            : MAX_int32 / 4;
        const int32 BestWindowA =
            (BestPathA != nullptr)
            ? (Best.bIsEdgeConflict
                ? EstimateEdgeWindow(*BestPathA, Best.FromA, Best.ToA, Best.TimeStep)
                : EstimateVertexWindow(*BestPathA, Best.Cell, Best.TimeStep))
            : MAX_int32 / 4;
        const int32 BestWindowB =
            (BestPathB != nullptr)
            ? (Best.bIsEdgeConflict
                ? EstimateEdgeWindow(*BestPathB, Best.FromB, Best.ToB, Best.TimeStep)
                : EstimateVertexWindow(*BestPathB, Best.Cell, Best.TimeStep))
            : MAX_int32 / 4;

        const int32 CandidateCardinality = (CandidateWindowA <= 0 ? 1 : 0) + (CandidateWindowB <= 0 ? 1 : 0);
        const int32 BestCardinality = (BestWindowA <= 0 ? 1 : 0) + (BestWindowB <= 0 ? 1 : 0);
        if (CandidateCardinality != BestCardinality)
        {
            if (CandidateCardinality > BestCardinality)
            {
                BestConflictIdx = CandidateIdx;
            }
            continue;
        }

        const int32 CandidateWindowSum = CandidateWindowA + CandidateWindowB;
        const int32 BestWindowSum = BestWindowA + BestWindowB;
        if (CandidateWindowSum != BestWindowSum)
        {
            if (CandidateWindowSum < BestWindowSum)
            {
                BestConflictIdx = CandidateIdx;
            }
            continue;
        }

        const int32 CandidateDegree = GetConflictDegree(Candidate.AgentA) + GetConflictDegree(Candidate.AgentB);
        const int32 BestDegree = GetConflictDegree(Best.AgentA) + GetConflictDegree(Best.AgentB);
        if (CandidateDegree != BestDegree)
        {
            if (CandidateDegree > BestDegree)
            {
                BestConflictIdx = CandidateIdx;
            }
            continue;
        }

        const int32 CandidateHotspot = Candidate.bIsEdgeConflict ? 0 : GetVertexHotspot(Candidate.Cell);
        const int32 BestHotspot = Best.bIsEdgeConflict ? 0 : GetVertexHotspot(Best.Cell);
        if (CandidateHotspot != BestHotspot)
        {
            if (CandidateHotspot > BestHotspot)
            {
                BestConflictIdx = CandidateIdx;
            }
            continue;
        }

        if (Candidate.bIsEdgeConflict != Best.bIsEdgeConflict)
        {
            if (!Candidate.bIsEdgeConflict && Best.bIsEdgeConflict)
            {
                BestConflictIdx = CandidateIdx;
            }
            continue;
        }

        if (Candidate.TimeStep != Best.TimeStep)
        {
            if (Candidate.TimeStep < Best.TimeStep)
            {
                BestConflictIdx = CandidateIdx;
            }
            continue;
        }

        const int32 CandidateFirstAgent = FMath::Min(Candidate.AgentA, Candidate.AgentB);
        const int32 BestFirstAgent = FMath::Min(Best.AgentA, Best.AgentB);
        if (CandidateFirstAgent != BestFirstAgent)
        {
            if (CandidateFirstAgent < BestFirstAgent)
            {
                BestConflictIdx = CandidateIdx;
            }
            continue;
        }

        const int32 CandidateSecondAgent = FMath::Max(Candidate.AgentA, Candidate.AgentB);
        const int32 BestSecondAgent = FMath::Max(Best.AgentA, Best.AgentB);
        if (CandidateSecondAgent < BestSecondAgent)
        {
            BestConflictIdx = CandidateIdx;
        }
    }

    return Conflicts[BestConflictIdx];
}

int32 FECBSPlanner::ComputeTotalConflictCount(
    const TMap<int32, TArray<FIntVector>>& PathsByAgent) const
{
    int32 TotalConflicts = 0;

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
                const TArray<FIntVector>* PathA = PathsByAgent.Find(AgentIds[i]);
                const TArray<FIntVector>* PathB = PathsByAgent.Find(AgentIds[j]);
                if (!PathA || !PathB)
                {
                    continue;
                }

                const FIntVector CellA = GetCellAtTime(*PathA, TimeStep);
                const FIntVector CellB = GetCellAtTime(*PathB, TimeStep);

                if (CellA == CellB)
                {
                    ++TotalConflicts;
                    continue;
                }

                if (TimeStep > 0)
                {
                    const FIntVector PrevA = GetCellAtTime(*PathA, TimeStep - 1);
                    const FIntVector PrevB = GetCellAtTime(*PathB, TimeStep - 1);

                    if (PrevA == CellB && PrevB == CellA)
                    {
                        ++TotalConflicts;
                    }
                }
            }
        }
    }

    return TotalConflicts;
}

int32 FECBSPlanner::ComputeSolutionCost(
    const TMap<int32, TArray<FIntVector>>& PathsByAgent) const
{
    int32 TotalCost = 0;

    for (const auto& KVP : PathsByAgent)
    {
        TotalCost += KVP.Value.Num();
    }

    return TotalCost;
}

TArray<FIntVector> FECBSPlanner::WorldPathToCellPath(
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

TArray<FVector> FECBSPlanner::CellPathToWorldPath(
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

FIntVector FECBSPlanner::GetCellAtTime(
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

FECBSPlanner::FCompiledConstraints FECBSPlanner::CompileConstraintsForAgent(
    int32 AgentId,
    const TArray<FECBSConstraint>& Constraints) const
{
    FCompiledConstraints Result;

    for (const FECBSConstraint& Constraint : Constraints)
    {
        if (Constraint.AgentId != AgentId)
        {
            continue;
        }

        Result.MaxConstraintTime = FMath::Max(Result.MaxConstraintTime, Constraint.TimeStep);

        if (!Constraint.bIsEdgeConstraint)
        {
            Result.VertexByTime.FindOrAdd(Constraint.TimeStep).Add(Constraint.Cell);
        }
        else
        {
            FEdgeConstraintKey EdgeKey;
            EdgeKey.From = Constraint.FromCell;
            EdgeKey.To = Constraint.ToCell;
            Result.EdgeByTime.FindOrAdd(Constraint.TimeStep).Add(EdgeKey);
        }
    }

    return Result;
}

bool FECBSPlanner::LowLevelPlanForAgent(
    const FGridMap3D& GridMap,
    const FDroneMissionConfig& Mission,
    const TArray<FECBSConstraint>& Constraints,
    const TMap<int32, TArray<FIntVector>>& CurrentPathsByAgent,
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
        UE_LOG(LogTemp, Error, TEXT("ECBS low-level: invalid grid size for AgentId=%d"), Mission.MissionId);
        return false;
    }

    if (Compiled.IsVertexForbidden(StartCell, 0))
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("ECBS low-level: start cell is constrained at t=0 for AgentId=%d"),
            Mission.MissionId);
        return false;
    }

    struct FSearchNode
    {
        int32 G = MAX_int32;
        int32 H = 0;
        FStateKey Parent;
        bool bHasParent = false;
    };

    auto Heuristic = [](const FIntVector& A, const FIntVector& B) -> int32
        {
            return
                FMath::Abs(A.X - B.X) +
                FMath::Abs(A.Y - B.Y) +
                FMath::Abs(A.Z - B.Z);
        };

    struct FOpenEntry
    {
        FStateKey Key;
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

    const TArray<FIntVector>* ExistingPath = CurrentPathsByAgent.Find(Mission.MissionId);
    const int32 SpatialLowerBound = Heuristic(StartCell, GoalCell);
    const int32 HoldingTime = Compiled.GetHoldingTime(GoalCell, SpatialLowerBound);

    // Reuse the CBS low-level time folding semantics.
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
                        TEXT("ECBS low-level exceeded guard limit for AgentId=%d LengthMax=%d Holding=%d StaticT=%d Constraints=%d Open=%d"),
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
                    const FIntVector NextCell = CurrentEntry.Key.Cell + Dir;
                    if (!GridMap.IsInside(NextCell.X, NextCell.Y, NextCell.Z))
                    {
                        continue;
                    }

                    if (GridMap.IsBlocked(NextCell.X, NextCell.Y, NextCell.Z))
                    {
                        continue;
                    }

                    CandidateCells.Add(NextCell);
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
