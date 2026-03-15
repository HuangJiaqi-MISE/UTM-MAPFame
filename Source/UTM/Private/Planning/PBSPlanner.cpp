#include "Planning/PBSPlanner.h"

#include "Algo/Reverse.h"
#include "Planning/AStarPlanner.h"
#include "Planning/GridMap3D.h"

namespace
{
    constexpr float INF_COST = 1e9f;
    constexpr int32 PBS_MAX_HIGH_LEVEL_ITERATIONS = 1024;
    constexpr int32 PBS_LOW_LEVEL_GUARD_LIMIT = 1000000;

    bool ValidateMissionIdsUniquePBS(const TArray<FDroneMissionConfig>& Missions)
    {
        TSet<int32> SeenMissionIds;

        for (const FDroneMissionConfig& Mission : Missions)
        {
            if (Mission.MissionId <= 0)
            {
                UE_LOG(LogTemp, Error, TEXT("PBS: MissionId must be positive. Invalid MissionId=%d"), Mission.MissionId);
                return false;
            }

            if (SeenMissionIds.Contains(Mission.MissionId))
            {
                UE_LOG(LogTemp, Error, TEXT("PBS: duplicate MissionId detected: %d"), Mission.MissionId);
                return false;
            }

            SeenMissionIds.Add(Mission.MissionId);
        }

        return true;
    }

    FString BuildPriorityGraphSignature(const TMap<int32, TSet<int32>>& HigherThan)
    {
        TArray<int32> HigherAgents;
        HigherThan.GetKeys(HigherAgents);
        HigherAgents.Sort();

        FString Result;

        for (int32 HigherAgent : HigherAgents)
        {
            TSet<int32> Reachable;
            TArray<int32> Stack;
            Stack.Add(HigherAgent);

            while (Stack.Num() > 0)
            {
                const int32 Current = Stack.Pop(EAllowShrinking::No);
                const TSet<int32>* LowerSetPtr = HigherThan.Find(Current);
                if (!LowerSetPtr)
                {
                    continue;
                }

                for (int32 LowerAgent : *LowerSetPtr)
                {
                    if (LowerAgent == HigherAgent || Reachable.Contains(LowerAgent))
                    {
                        continue;
                    }

                    Reachable.Add(LowerAgent);
                    Stack.Add(LowerAgent);
                }
            }

            TArray<int32> Lowers = Reachable.Array();
            Lowers.Sort();

            for (int32 LowerAgent : Lowers)
            {
                Result += FString::Printf(TEXT("%d>%d;"), HigherAgent, LowerAgent);
            }
        }

        return Result;
    }

    FString BuildPathSetSignature(const TMap<int32, TArray<FIntVector>>& PathsByAgent)
    {
        TArray<int32> AgentIds;
        PathsByAgent.GetKeys(AgentIds);
        AgentIds.Sort();

        FString Result;

        for (int32 AgentId : AgentIds)
        {
            Result += FString::Printf(TEXT("%d:"), AgentId);

            const TArray<FIntVector>* PathPtr = PathsByAgent.Find(AgentId);
            if (!PathPtr)
            {
                Result += TEXT("; ");
                continue;
            }

            for (const FIntVector& Cell : *PathPtr)
            {
                Result += FString::Printf(TEXT("(%d,%d,%d)"), Cell.X, Cell.Y, Cell.Z);
            }

            Result += TEXT(";");
        }

        return Result;
    }

    FString BuildPBSNodeSignature(
        const TMap<int32, TSet<int32>>& HigherThan,
        const TMap<int32, TArray<FIntVector>>& PathsByAgent)
    {
        return BuildPriorityGraphSignature(HigherThan) + TEXT("#") + BuildPathSetSignature(PathsByAgent);
    }

    bool BuildTopologicalOrderPBS(
        const TArray<FDroneMissionConfig>& Missions,
        const TMap<int32, TSet<int32>>& HigherThan,
        TArray<int32>& OutOrder)
    {
        OutOrder.Reset();

        TMap<int32, int32> InDegree;
        TMap<int32, TSet<int32>> Adjacency;

        for (const FDroneMissionConfig& Mission : Missions)
        {
            InDegree.Add(Mission.MissionId, 0);
            Adjacency.FindOrAdd(Mission.MissionId);
        }

        for (const auto& KVP : HigherThan)
        {
            const int32 HigherAgent = KVP.Key;

            if (!InDegree.Contains(HigherAgent))
            {
                continue;
            }

            for (int32 LowerAgent : KVP.Value)
            {
                if (!InDegree.Contains(LowerAgent))
                {
                    continue;
                }

                Adjacency.FindOrAdd(HigherAgent).Add(LowerAgent);
            }
        }

        for (const auto& KVP : Adjacency)
        {
            for (int32 LowerAgent : KVP.Value)
            {
                if (int32* DegreePtr = InDegree.Find(LowerAgent))
                {
                    *DegreePtr += 1;
                }
            }
        }

        TArray<int32> ZeroInDegree;
        for (const auto& KVP : InDegree)
        {
            if (KVP.Value == 0)
            {
                ZeroInDegree.Add(KVP.Key);
            }
        }

        ZeroInDegree.Sort();

        while (ZeroInDegree.Num() > 0)
        {
            const int32 Current = ZeroInDegree[0];
            ZeroInDegree.RemoveAt(0, 1, false);

            OutOrder.Add(Current);

            const TSet<int32>* LowerSetPtr = Adjacency.Find(Current);
            if (!LowerSetPtr)
            {
                continue;
            }

            TArray<int32> Lowers = LowerSetPtr->Array();
            Lowers.Sort();

            for (int32 LowerAgent : Lowers)
            {
                int32* DegreePtr = InDegree.Find(LowerAgent);
                if (!DegreePtr)
                {
                    continue;
                }

                *DegreePtr -= 1;
                if (*DegreePtr == 0)
                {
                    ZeroInDegree.Add(LowerAgent);
                    ZeroInDegree.Sort();
                }
            }
        }

        return OutOrder.Num() == InDegree.Num();
    }

    void CollectDescendantsPBS(
        int32 StartAgent,
        const TMap<int32, TSet<int32>>& HigherThan,
        TSet<int32>& OutAffected)
    {
        OutAffected.Reset();

        TArray<int32> Stack;
        Stack.Add(StartAgent);

        while (Stack.Num() > 0)
        {
            const int32 Current = Stack.Pop(EAllowShrinking::No);
            if (OutAffected.Contains(Current))
            {
                continue;
            }

            OutAffected.Add(Current);

            const TSet<int32>* LowerSetPtr = HigherThan.Find(Current);
            if (!LowerSetPtr)
            {
                continue;
            }

            for (int32 LowerAgent : *LowerSetPtr)
            {
                if (!OutAffected.Contains(LowerAgent))
                {
                    Stack.Add(LowerAgent);
                }
            }
        }
    }

    bool HasPriorityRelationPBS(
        int32 HigherAgent,
        int32 LowerAgent,
        const TMap<int32, TSet<int32>>& HigherThan)
    {
        if (HigherAgent == LowerAgent)
        {
            return false;
        }

        TSet<int32> Descendants;
        CollectDescendantsPBS(HigherAgent, HigherThan, Descendants);
        return Descendants.Contains(LowerAgent);
    }
    bool IsAgentSettledAtGoalPBS(const TArray<FIntVector>& Path, int32 TimeStep)
    {
        return Path.Num() > 0 && TimeStep >= (Path.Num() - 1);
    }

    template<typename TConflict>
    bool IsTargetConflictPBS(
        const TConflict& Conflict,
        const TMap<int32, TArray<FIntVector>>& PathsByAgent,
        int32& OutGoalAgent,
        int32& OutOtherAgent)
    {
        OutGoalAgent = INDEX_NONE;
        OutOtherAgent = INDEX_NONE;

        if (!Conflict.bValid || Conflict.bIsEdgeConflict)
        {
            return false;
        }

        const TArray<FIntVector>* PathAPtr = PathsByAgent.Find(Conflict.AgentA);
        const TArray<FIntVector>* PathBPtr = PathsByAgent.Find(Conflict.AgentB);
        if (!PathAPtr || !PathBPtr || PathAPtr->Num() <= 0 || PathBPtr->Num() <= 0)
        {
            return false;
        }

        const bool bASettledAtGoal = IsAgentSettledAtGoalPBS(*PathAPtr, Conflict.TimeStep) && PathAPtr->Last() == Conflict.Cell;
        const bool bBSettledAtGoal = IsAgentSettledAtGoalPBS(*PathBPtr, Conflict.TimeStep) && PathBPtr->Last() == Conflict.Cell;
        if (!bASettledAtGoal && !bBSettledAtGoal)
        {
            return false;
        }

        if (bASettledAtGoal && !bBSettledAtGoal)
        {
            OutGoalAgent = Conflict.AgentA;
            OutOtherAgent = Conflict.AgentB;
            return true;
        }

        if (bBSettledAtGoal && !bASettledAtGoal)
        {
            OutGoalAgent = Conflict.AgentB;
            OutOtherAgent = Conflict.AgentA;
            return true;
        }

        const int32 CostA = PathAPtr->Num();
        const int32 CostB = PathBPtr->Num();
        if (CostA <= CostB)
        {
            OutGoalAgent = Conflict.AgentA;
            OutOtherAgent = Conflict.AgentB;
        }
        else
        {
            OutGoalAgent = Conflict.AgentB;
            OutOtherAgent = Conflict.AgentA;
        }

        return true;
    }

    int32 CountAffectedAgentsPBS(
        int32 StartAgent,
        const TMap<int32, TSet<int32>>& HigherThan)
    {
        TSet<int32> AffectedAgents;
        CollectDescendantsPBS(StartAgent, HigherThan, AffectedAgents);
        return AffectedAgents.Num();
    }

    template<typename TConflict>
    void ChooseFirstBranchPBS(
        const TConflict& Conflict,
        const TMap<int32, TArray<FIntVector>>& PathsByAgent,
        const TMap<int32, TSet<int32>>& HigherThan,
        int32& OutFirstHigher,
        int32& OutFirstLower,
        bool& bOutUsesTargetReasoning)
    {
        bOutUsesTargetReasoning = false;

        int32 GoalAgent = INDEX_NONE;
        int32 OtherAgent = INDEX_NONE;
        if (IsTargetConflictPBS(Conflict, PathsByAgent, GoalAgent, OtherAgent))
        {
            OutFirstHigher = OtherAgent;
            OutFirstLower = GoalAgent;
            bOutUsesTargetReasoning = true;
            return;
        }

        const int32 AffectsIfAOverB = CountAffectedAgentsPBS(Conflict.AgentB, HigherThan);
        const int32 AffectsIfBOverA = CountAffectedAgentsPBS(Conflict.AgentA, HigherThan);
        if (AffectsIfAOverB < AffectsIfBOverA)
        {
            OutFirstHigher = Conflict.AgentA;
            OutFirstLower = Conflict.AgentB;
            return;
        }

        if (AffectsIfBOverA < AffectsIfAOverB)
        {
            OutFirstHigher = Conflict.AgentB;
            OutFirstLower = Conflict.AgentA;
            return;
        }

        if (Conflict.AgentA < Conflict.AgentB)
        {
            OutFirstHigher = Conflict.AgentA;
            OutFirstLower = Conflict.AgentB;
        }
        else
        {
            OutFirstHigher = Conflict.AgentB;
            OutFirstLower = Conflict.AgentA;
        }
    }
}
bool FPBSPlanner::PlanMissions(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    TMap<int32, TArray<FVector>>& OutPaths)
{
    OutPaths.Reset();

    if (Missions.Num() <= 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("PBS: no missions to plan"));
        return false;
    }

    if (!ValidateMissionIdsUniquePBS(Missions))
    {
        return false;
    }

    FPBSNode RootNode;
    if (!BuildRootNode(GridMap, Missions, RootNode))
    {
        UE_LOG(LogTemp, Error, TEXT("PBS: failed to build root node"));
        return false;
    }

    TArray<FPBSNode> NodeStack;
    TSet<FString> VisitedNodeSignatures;

    NodeStack.Add(RootNode);
    VisitedNodeSignatures.Add(BuildPBSNodeSignature(RootNode.HigherThan, RootNode.PathsByAgent));

    UE_LOG(LogTemp, Warning, TEXT("PBS diagnostics build marker: 2026-03-13-v7"));

    int32 IterationGuard = 0;
    int32 ExpandedNodeCount = 0;
    int32 GeneratedChildCount = 0;
    int32 DuplicateNodeCount = 0;
    int32 FailedChildCount = 0;

    while (NodeStack.Num() > 0)
    {
        if (++IterationGuard > PBS_MAX_HIGH_LEVEL_ITERATIONS)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("PBS: exceeded iteration guard. Expanded=%d Generated=%d Duplicate=%d Failed=%d Open=%d Visited=%d"),
                ExpandedNodeCount,
                GeneratedChildCount,
                DuplicateNodeCount,
                FailedChildCount,
                NodeStack.Num(),
                VisitedNodeSignatures.Num());
            return false;
        }

        FPBSNode& Current = NodeStack.Last();
        if (Current.Conflicts.Num() <= 0)
        {
            UE_LOG(LogTemp, Warning, TEXT("PBS: conflict-free solution found. Cost=%d"), Current.Cost);

            for (const auto& KVP : Current.PathsByAgent)
            {
                OutPaths.Add(KVP.Key, CellPathToWorldPath(GridMap, KVP.Value));
            }

            return true;
        }

        if (Current.ExpansionStage == 0)
        {
            ++ExpandedNodeCount;

            const FPBSConflict Conflict = SelectConflictForBranching(Current.Conflicts, Current.PathsByAgent);
            if (!Conflict.bIsEdgeConflict)
            {
                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("PBS: branching on vertex conflict. AgentA=%d AgentB=%d Time=%d Cell=(%d,%d,%d)"),
                    Conflict.AgentA,
                    Conflict.AgentB,
                    Conflict.TimeStep,
                    Conflict.Cell.X,
                    Conflict.Cell.Y,
                    Conflict.Cell.Z);
            }
            else
            {
                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("PBS: branching on edge conflict. AgentA=%d AgentB=%d Time=%d A:(%d,%d,%d)->(%d,%d,%d) B:(%d,%d,%d)->(%d,%d,%d)"),
                    Conflict.AgentA,
                    Conflict.AgentB,
                    Conflict.TimeStep,
                    Conflict.FromA.X, Conflict.FromA.Y, Conflict.FromA.Z,
                    Conflict.ToA.X, Conflict.ToA.Y, Conflict.ToA.Z,
                    Conflict.FromB.X, Conflict.FromB.Y, Conflict.FromB.Z,
                    Conflict.ToB.X, Conflict.ToB.Y, Conflict.ToB.Z);
            }

            const bool bAHasPriorityOverB = HasPriorityRelationPBS(Conflict.AgentA, Conflict.AgentB, Current.HigherThan);
            const bool bBHasPriorityOverA = HasPriorityRelationPBS(Conflict.AgentB, Conflict.AgentA, Current.HigherThan);
            if (bAHasPriorityOverB || bBHasPriorityOverA)
            {
                const FString CurrentSignature = BuildPBSNodeSignature(Current.HigherThan, Current.PathsByAgent);
                FPBSNode RepairedNode = Current;
                const int32 HigherPriorityAgent = bAHasPriorityOverB ? Conflict.AgentA : Conflict.AgentB;
                const int32 LowerPriorityAgent = bAHasPriorityOverB ? Conflict.AgentB : Conflict.AgentA;

                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("PBS: ordered conflict detected. Higher=%d Lower=%d. Replanning lower-priority subtree."),
                    HigherPriorityAgent,
                    LowerPriorityAgent);

                if (!ReplanAffectedAgents(GridMap, Missions, LowerPriorityAgent, RepairedNode))
                {
                    ++FailedChildCount;
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("PBS: ordered conflict replanning failed. Higher=%d Lower=%d"),
                        HigherPriorityAgent,
                        LowerPriorityAgent);
                    NodeStack.Pop(EAllowShrinking::No);
                    continue;
                }

                RefreshNodeState(RepairedNode);
                const FString RepairedSignature = BuildPBSNodeSignature(RepairedNode.HigherThan, RepairedNode.PathsByAgent);
                if (RepairedSignature == CurrentSignature)
                {
                    ++DuplicateNodeCount;
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("PBS: ordered conflict replanning made no progress. Higher=%d Lower=%d"),
                        HigherPriorityAgent,
                        LowerPriorityAgent);
                    NodeStack.Pop(EAllowShrinking::No);
                    continue;
                }

                if (VisitedNodeSignatures.Contains(RepairedSignature))
                {
                    ++DuplicateNodeCount;
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("PBS: ordered conflict replanning produced an already-visited node. Higher=%d Lower=%d"),
                        HigherPriorityAgent,
                        LowerPriorityAgent);
                    NodeStack.Pop(EAllowShrinking::No);
                    continue;
                }

                VisitedNodeSignatures.Add(RepairedSignature);
                ++GeneratedChildCount;
                Current = MoveTemp(RepairedNode);
                continue;
            }

            bool bUsedTargetReasoning = false;
            int32 FirstHigherAgent = INDEX_NONE;
            int32 FirstLowerAgent = INDEX_NONE;
            ChooseFirstBranchPBS(
                Conflict,
                Current.PathsByAgent,
                Current.HigherThan,
                FirstHigherAgent,
                FirstLowerAgent,
                bUsedTargetReasoning);

            Current.ExpansionStage = 1;
            Current.PreferredHigherAgent = FirstHigherAgent;
            Current.PreferredLowerAgent = FirstLowerAgent;
            Current.AlternateHigherAgent = (FirstHigherAgent == Conflict.AgentA) ? Conflict.AgentB : Conflict.AgentA;
            Current.AlternateLowerAgent = (FirstLowerAgent == Conflict.AgentA) ? Conflict.AgentB : Conflict.AgentA;

            if (bUsedTargetReasoning)
            {
                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("PBS: target reasoning selected first branch %d > %d"),
                    Current.PreferredHigherAgent,
                    Current.PreferredLowerAgent);
            }

            FPBSNode PreferredChild = Current;
            if (AddPriorityAndReplan(GridMap, Missions, Current.PreferredHigherAgent, Current.PreferredLowerAgent, PreferredChild))
            {
                RefreshNodeState(PreferredChild);
                const FString PreferredSignature = BuildPBSNodeSignature(PreferredChild.HigherThan, PreferredChild.PathsByAgent);
                if (!VisitedNodeSignatures.Contains(PreferredSignature))
                {
                    NodeStack.Add(PreferredChild);
                    VisitedNodeSignatures.Add(PreferredSignature);
                    ++GeneratedChildCount;
                }
                else
                {
                    ++DuplicateNodeCount;
                }
            }
            else
            {
                ++FailedChildCount;
                UE_LOG(LogTemp, Warning, TEXT("PBS: failed preferred branch %d > %d"), Current.PreferredHigherAgent, Current.PreferredLowerAgent);
            }

            continue;
        }

        FPBSNode DeferredParent = Current;
        NodeStack.Pop(EAllowShrinking::No);

        FPBSNode AlternateChild = DeferredParent;
        if (AddPriorityAndReplan(GridMap, Missions, DeferredParent.AlternateHigherAgent, DeferredParent.AlternateLowerAgent, AlternateChild))
        {
            RefreshNodeState(AlternateChild);
            const FString AlternateSignature = BuildPBSNodeSignature(AlternateChild.HigherThan, AlternateChild.PathsByAgent);
            if (!VisitedNodeSignatures.Contains(AlternateSignature))
            {
                NodeStack.Add(AlternateChild);
                VisitedNodeSignatures.Add(AlternateSignature);
                ++GeneratedChildCount;
            }
            else
            {
                ++DuplicateNodeCount;
            }
        }
        else
        {
            ++FailedChildCount;
            UE_LOG(LogTemp, Warning, TEXT("PBS: failed deferred branch %d > %d"), DeferredParent.AlternateHigherAgent, DeferredParent.AlternateLowerAgent);
        }
    }

    UE_LOG(LogTemp, Error, TEXT("PBS: open list exhausted without solution"));
    return false;
}

bool FPBSPlanner::BuildRootNode(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    FPBSNode& OutRootNode) const
{
    OutRootNode.HigherThan.Reset();
    OutRootNode.PathsByAgent.Reset();
    OutRootNode.Cost = 0;

    FReservationTable EmptyReservation;
    for (const FDroneMissionConfig& Mission : Missions)
    {
        TArray<FIntVector> InitialPath;
        if (!LowLevelPlanForAgent(GridMap, Mission, EmptyReservation, nullptr, InitialPath))
        {
            UE_LOG(LogTemp, Error, TEXT("PBS: root initial planning failed for MissionId=%d"), Mission.MissionId);
            return false;
        }

        if (InitialPath.Num() <= 0)
        {
            UE_LOG(LogTemp, Error, TEXT("PBS: empty root initial path for MissionId=%d"), Mission.MissionId);
            return false;
        }

        OutRootNode.PathsByAgent.Add(Mission.MissionId, InitialPath);

        UE_LOG(
            LogTemp,
            Warning,
            TEXT("PBS root path built for MissionId=%d, Steps=%d"),
            Mission.MissionId,
            InitialPath.Num());
    }

    RefreshNodeState(OutRootNode);
    return true;
}

bool FPBSPlanner::AddPriorityAndReplan(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    int32 HigherAgent,
    int32 LowerAgent,
    FPBSNode& InOutNode) const
{
    if (HigherAgent == LowerAgent)
    {
        UE_LOG(LogTemp, Warning, TEXT("PBS: invalid self-priority %d > %d"), HigherAgent, LowerAgent);
        return false;
    }

    if (CreatesCycle(HigherAgent, LowerAgent, InOutNode.HigherThan))
    {
        UE_LOG(LogTemp, Warning, TEXT("PBS: adding %d > %d creates cycle"), HigherAgent, LowerAgent);
        return false;
    }

    TSet<int32> ExistingDescendants;
    CollectDescendantsPBS(HigherAgent, InOutNode.HigherThan, ExistingDescendants);
    if (ExistingDescendants.Contains(LowerAgent))
    {
        UE_LOG(LogTemp, Warning, TEXT("PBS: redundant priority relation %d > %d skipped"), HigherAgent, LowerAgent);
        return false;
    }

    TSet<int32>& LowerSet = InOutNode.HigherThan.FindOrAdd(HigherAgent);
    LowerSet.Add(LowerAgent);

    return ReplanAffectedAgents(GridMap, Missions, LowerAgent, InOutNode);
}

bool FPBSPlanner::ReplanAffectedAgents(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    int32 StartAgent,
    FPBSNode& InOutNode) const
{
    TMap<int32, const FDroneMissionConfig*> MissionById;
    for (const FDroneMissionConfig& Mission : Missions)
    {
        MissionById.Add(Mission.MissionId, &Mission);
    }

    TArray<int32> TopologicalOrder;
    if (!BuildTopologicalOrderPBS(Missions, InOutNode.HigherThan, TopologicalOrder))
    {
        UE_LOG(LogTemp, Error, TEXT("PBS: failed to build topological order"));
        return false;
    }

    TSet<int32> AffectedAgents;
    CollectDescendantsPBS(StartAgent, InOutNode.HigherThan, AffectedAgents);

    for (int32 AgentId : TopologicalOrder)
    {
        if (!AffectedAgents.Contains(AgentId))
        {
            continue;
        }

        const FDroneMissionConfig* MissionPtr = MissionById.FindRef(AgentId);
        if (!MissionPtr)
        {
            UE_LOG(LogTemp, Error, TEXT("PBS: mission not found for AgentId=%d"), AgentId);
            return false;
        }

        const FReservationTable Reservation = BuildReservationTableForAgent(AgentId, InOutNode);
        const TArray<FIntVector>* ExistingPathPtr = InOutNode.PathsByAgent.Find(AgentId);

        TArray<FIntVector> NewPath;
        if (!LowLevelPlanForAgent(GridMap, *MissionPtr, Reservation, ExistingPathPtr, NewPath))
        {
            UE_LOG(LogTemp, Warning, TEXT("PBS: replanning failed for AgentId=%d"), AgentId);
            return false;
        }

        InOutNode.PathsByAgent.Add(AgentId, NewPath);
    }

    return true;
}

bool FPBSPlanner::LowLevelPlanForAgent(
    const FGridMap3D& GridMap,
    const FDroneMissionConfig& Mission,
    const FReservationTable& Reservation,
    const TArray<FIntVector>* ExistingPath,
    TArray<FIntVector>& OutPath) const
{
    OutPath.Reset();

    const FIntVector StartCell = GridMap.WorldToCell(Mission.StartWorld);
    const FIntVector GoalCell = GridMap.WorldToCell(Mission.GoalWorld);

    if (!GridMap.IsInside(StartCell.X, StartCell.Y, StartCell.Z) ||
        !GridMap.IsInside(GoalCell.X, GoalCell.Y, GoalCell.Z))
    {
        UE_LOG(LogTemp, Warning, TEXT("PBS low-level: start or goal is outside grid for AgentId=%d"), Mission.MissionId);
        return false;
    }

    if (GridMap.IsBlocked(StartCell.X, StartCell.Y, StartCell.Z) ||
        GridMap.IsBlocked(GoalCell.X, GoalCell.Y, GoalCell.Z))
    {
        UE_LOG(LogTemp, Warning, TEXT("PBS low-level: start or goal is blocked for AgentId=%d"), Mission.MissionId);
        return false;
    }

    const bool bHasAnyReservation =
        Reservation.VertexByTime.Num() > 0 ||
        Reservation.EdgeByTime.Num() > 0 ||
        Reservation.PermanentVertexStartTime.Num() > 0 ||
        Reservation.CatVertexByTime.Num() > 0 ||
        Reservation.CatEdgeByTime.Num() > 0;

    if (!bHasAnyReservation)
    {
        FAStarPlanner StaticPlanner;
        TArray<FVector> WorldPath;
        if (!StaticPlanner.Plan(GridMap, Mission.StartWorld, Mission.GoalWorld, WorldPath))
        {
            UE_LOG(LogTemp, Warning, TEXT("PBS low-level: unconstrained A* failed for AgentId=%d"), Mission.MissionId);
            return false;
        }

        OutPath.Reserve(WorldPath.Num());
        for (const FVector& Point : WorldPath)
        {
            OutPath.Add(GridMap.WorldToCell(Point));
        }

        return OutPath.Num() > 0;
    }

    auto Heuristic = [](const FIntVector& A, const FIntVector& B) -> float
        {
            return
                FMath::Abs(A.X - B.X) +
                FMath::Abs(A.Y - B.Y) +
                FMath::Abs(A.Z - B.Z);
        };

    auto IsHardVertexReserved = [&Reservation](const FIntVector& Cell, int32 TimeStep) -> bool
        {
            if (const int32* PermanentStartPtr = Reservation.PermanentVertexStartTime.Find(Cell))
            {
                if (TimeStep >= *PermanentStartPtr)
                {
                    return true;
                }
            }

            const TSet<FIntVector>* Cells = Reservation.VertexByTime.Find(TimeStep);
            return Cells && Cells->Contains(Cell);
        };

    auto IsHardEdgeReserved = [&Reservation](const FIntVector& From, const FIntVector& To, int32 TimeStep) -> bool
        {
            const TSet<FEdgeKey>* Edges = Reservation.EdgeByTime.Find(TimeStep);
            if (!Edges)
            {
                return false;
            }

            FEdgeKey ReverseEdge;
            ReverseEdge.From = To;
            ReverseEdge.To = From;
            return Edges->Contains(ReverseEdge);
        };

    auto HasCatVertexConflict = [&Reservation](const FIntVector& Cell, int32 TimeStep) -> bool
        {
            const TSet<FIntVector>* Cells = Reservation.CatVertexByTime.Find(TimeStep);
            return Cells && Cells->Contains(Cell);
        };

    auto HasCatEdgeConflict = [&Reservation](const FIntVector& From, const FIntVector& To, int32 TimeStep) -> bool
        {
            const TSet<FEdgeKey>* Edges = Reservation.CatEdgeByTime.Find(TimeStep);
            if (!Edges)
            {
                return false;
            }

            FEdgeKey ReverseEdge;
            ReverseEdge.From = To;
            ReverseEdge.To = From;
            return Edges->Contains(ReverseEdge);
        };

    auto CountSoftVertexConflicts = [&HasCatVertexConflict](const FIntVector& Cell, int32 StartTime, int32 EndTime) -> int32
        {
            if (EndTime < StartTime)
            {
                return 0;
            }

            int32 Count = 0;
            for (int32 TimeStep = StartTime; TimeStep <= EndTime; ++TimeStep)
            {
                Count += HasCatVertexConflict(Cell, TimeStep) ? 1 : 0;
            }

            return Count;
        };

    auto CountSoftMoveConflicts = [&HasCatVertexConflict, &HasCatEdgeConflict](const FIntVector& From, const FIntVector& To, int32 ArrivalTime) -> int32
        {
            int32 Count = HasCatVertexConflict(To, ArrivalTime) ? 1 : 0;
            Count += HasCatEdgeConflict(From, To, ArrivalTime) ? 1 : 0;
            return Count;
        };

    if (IsHardVertexReserved(StartCell, 0))
    {
        UE_LOG(LogTemp, Warning, TEXT("PBS low-level: start cell reserved at t=0 for AgentId=%d"), Mission.MissionId);
        return false;
    }

    if (const int32* PermanentGoalBlockPtr = Reservation.PermanentVertexStartTime.Find(GoalCell))
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("PBS low-level: goal cell permanently reserved from t=%d for AgentId=%d"),
            *PermanentGoalBlockPtr,
            Mission.MissionId);
        return false;
    }

    int32 GoalReadyTime = 0;
    for (const auto& KVP : Reservation.VertexByTime)
    {
        if (KVP.Value.Contains(GoalCell))
        {
            GoalReadyTime = FMath::Max(GoalReadyTime, KVP.Key + 1);
        }
    }

    const int32 SpatialLowerBound = Heuristic(StartCell, GoalCell);
    const int32 TotalCells = GridMap.GridDim.X * GridMap.GridDim.Y * GridMap.GridDim.Z;
    const int32 HardConstraintHorizon = FMath::Max3(GoalReadyTime, Reservation.MaxReservedTime, SpatialLowerBound);
    const int32 IncumbentCost =
        (ExistingPath && ExistingPath->Num() > 0)
        ? FMath::Max(0, ExistingPath->Num() - 1)
        : SpatialLowerBound;
    // CAT is only a soft tie-breaker. Letting its longest path define the time horizon
    // makes replanning explode once a few agents accumulate very long paths.
    const int32 EffectiveCatTime = FMath::Min(Reservation.MaxCatTime, HardConstraintHorizon);
    int32 HardUpperBound = FMath::Max(HardConstraintHorizon, IncumbentCost + 64);
    HardUpperBound = FMath::Max(HardUpperBound, SpatialLowerBound + FMath::Clamp(GridMap.GridDim.X + GridMap.GridDim.Y + GridMap.GridDim.Z, 32, 256));
    HardUpperBound = FMath::Min(HardUpperBound, SpatialLowerBound + FMath::Clamp(TotalCells / 32, 128, 2048));
    HardUpperBound = FMath::Max(HardUpperBound, HardConstraintHorizon);
    const int32 MaxTime = HardUpperBound;

    struct FQueueEntry
    {
        FTimedState State;
        int32 ConflictScore = 0;
        float F = 0.f;
    };

    auto OpenPredicate = [](const FQueueEntry& A, const FQueueEntry& B)
        {
            if (A.F != B.F)
            {
                return A.F > B.F;
            }

            if (A.ConflictScore != B.ConflictScore)
            {
                return A.ConflictScore > B.ConflictScore;
            }

            // Prefer deeper states on the same f-layer to avoid broad wavefront expansion.
            return A.State.TimeStep < B.State.TimeStep;
        };

    TArray<FQueueEntry> OpenHeap;
    TMap<FTimedState, int32> BestConflictScore;
    TMap<FTimedState, FTimedState> Parent;
    int32 ExpandedStateCount = 0;
    int32 GeneratedStateCount = 0;
    int32 RejectedDuplicateStateCount = 0;
    int32 RejectedDeadlineStateCount = 0;

    FTimedState StartState;
    StartState.Cell = StartCell;
    StartState.TimeStep = 0;

    BestConflictScore.Add(StartState, HasCatVertexConflict(StartCell, 0) ? 1 : 0);

    FQueueEntry StartEntry;
    StartEntry.State = StartState;
    StartEntry.ConflictScore = BestConflictScore[StartState];
    StartEntry.F = Heuristic(StartCell, GoalCell);
    OpenHeap.HeapPush(StartEntry, OpenPredicate);
    GeneratedStateCount = 1;

    static const FIntVector Directions[7] =
    {
        FIntVector(0, 0, 0),
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
        if (++GuardCounter > PBS_LOW_LEVEL_GUARD_LIMIT)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("PBS low-level exceeded guard limit for AgentId=%d MaxTime=%d GoalReadyTime=%d MaxReservedTime=%d MaxCatTime=%d HardHorizon=%d EffectiveCatTime=%d Incumbent=%d Expanded=%d Generated=%d RejectedDup=%d RejectedDeadline=%d Open=%d Start=(%d,%d,%d) Goal=(%d,%d,%d) CatTop=%s"),
                Mission.MissionId,
                MaxTime,
                GoalReadyTime,
                Reservation.MaxReservedTime,
                Reservation.MaxCatTime,
                HardConstraintHorizon,
                EffectiveCatTime,
                IncumbentCost,
                ExpandedStateCount,
                GeneratedStateCount,
                RejectedDuplicateStateCount,
                RejectedDeadlineStateCount,
                OpenHeap.Num(),
                StartCell.X,
                StartCell.Y,
                StartCell.Z,
                GoalCell.X,
                GoalCell.Y,
                GoalCell.Z,
                *Reservation.CatTopContributors);
            return false;
        }

        while (OpenHeap.Num() > 0)
        {
            const FQueueEntry& Top = OpenHeap[0];
            const int32 CurrentBestStateConflict = BestConflictScore.FindRef(Top.State);
            if (CurrentBestStateConflict != Top.ConflictScore)
            {
                FQueueEntry Discarded;
                OpenHeap.HeapPop(Discarded, OpenPredicate);
                continue;
            }

            break;
        }

        if (OpenHeap.Num() <= 0)
        {
            break;
        }

        FQueueEntry CurrentEntry;
        OpenHeap.HeapPop(CurrentEntry, OpenPredicate);
        ++ExpandedStateCount;

        if (CurrentEntry.State.Cell == GoalCell && CurrentEntry.State.TimeStep >= GoalReadyTime)
        {
            TArray<FIntVector> ReversePath;
            FTimedState WalkState = CurrentEntry.State;
            while (true)
            {
                ReversePath.Add(WalkState.Cell);

                const FTimedState* ParentPtr = Parent.Find(WalkState);
                if (!ParentPtr)
                {
                    break;
                }

                WalkState = *ParentPtr;
            }

            Algo::Reverse(ReversePath);
            OutPath = MoveTemp(ReversePath);
            return OutPath.Num() > 0;
        }

        if (CurrentEntry.State.TimeStep >= MaxTime)
        {
            continue;
        }

        for (const FIntVector& Dir : Directions)
        {
            const FIntVector NextCell = CurrentEntry.State.Cell + Dir;
            if (!GridMap.IsInside(NextCell.X, NextCell.Y, NextCell.Z))
            {
                continue;
            }

            if (GridMap.IsBlocked(NextCell.X, NextCell.Y, NextCell.Z))
            {
                continue;
            }

            const int32 NextTime = CurrentEntry.State.TimeStep + 1;
            if (IsHardVertexReserved(NextCell, NextTime))
            {
                continue;
            }

            if (IsHardEdgeReserved(CurrentEntry.State.Cell, NextCell, NextTime))
            {
                continue;
            }

            FTimedState NextState;
            NextState.Cell = NextCell;
            NextState.TimeStep = NextTime;
            const int32 RemainingDistance = static_cast<int32>(Heuristic(NextCell, GoalCell));
            if (NextTime + RemainingDistance > MaxTime)
            {
                ++RejectedDeadlineStateCount;
                continue;
            }

            const int32 WaitConflictCount = (Dir == FIntVector::ZeroValue) ?
                CountSoftVertexConflicts(CurrentEntry.State.Cell, NextTime, NextTime) :
                0;
            const int32 TentativeConflictScore =
                CurrentEntry.ConflictScore +
                WaitConflictCount +
                CountSoftMoveConflicts(CurrentEntry.State.Cell, NextCell, NextTime);

            if (BestConflictScore.Contains(NextState))
            {
                ++RejectedDuplicateStateCount;
                continue;
            }

            BestConflictScore.Add(NextState, TentativeConflictScore);
            Parent.Add(NextState, CurrentEntry.State);

            FQueueEntry NextEntry;
            NextEntry.State = NextState;
            NextEntry.ConflictScore = TentativeConflictScore;
            NextEntry.F = (float)NextTime + Heuristic(NextCell, GoalCell);
            OpenHeap.HeapPush(NextEntry, OpenPredicate);
            ++GeneratedStateCount;
        }
    }

    return false;
}

FPBSPlanner::FReservationTable FPBSPlanner::BuildReservationTableForAgent(
    int32 AgentId,
    const FPBSNode& Node) const
{
    FReservationTable Result;
    TArray<TPair<int32, int32>> CatPathLengths;

    TSet<int32> HigherAgents;
    CollectHigherPriorityAgents(AgentId, Node.HigherThan, HigherAgents);

    for (const auto& KVP : Node.PathsByAgent)
    {
        const int32 OtherAgentId = KVP.Key;
        if (OtherAgentId == AgentId)
        {
            continue;
        }

        const TArray<FIntVector>& Path = KVP.Value;
        if (Path.Num() <= 0)
        {
            continue;
        }

        CatPathLengths.Emplace(OtherAgentId, Path.Num());
        const bool bIsHigherPriorityAgent = HigherAgents.Contains(OtherAgentId);
        for (int32 t = 0; t < Path.Num(); ++t)
        {
            Result.CatVertexByTime.FindOrAdd(t).Add(Path[t]);
            Result.MaxCatTime = FMath::Max(Result.MaxCatTime, t);

            if (t > 0)
            {
                FEdgeKey CatEdge;
                CatEdge.From = Path[t - 1];
                CatEdge.To = Path[t];
                Result.CatEdgeByTime.FindOrAdd(t).Add(CatEdge);
            }

            if (bIsHigherPriorityAgent)
            {
                Result.VertexByTime.FindOrAdd(t).Add(Path[t]);
                Result.MaxReservedTime = FMath::Max(Result.MaxReservedTime, t);

                if (t > 0)
                {
                    FEdgeKey Edge;
                    Edge.From = Path[t - 1];
                    Edge.To = Path[t];
                    Result.EdgeByTime.FindOrAdd(t).Add(Edge);
                }
            }
        }

        const FIntVector GoalCell = Path.Last();
        const int32 GoalArrivalTime = Path.Num() - 1;

        if (bIsHigherPriorityAgent)
        {
            if (int32* ExistingPermanentStartPtr = Result.PermanentVertexStartTime.Find(GoalCell))
            {
                *ExistingPermanentStartPtr = FMath::Min(*ExistingPermanentStartPtr, GoalArrivalTime);
            }
            else
            {
                Result.PermanentVertexStartTime.Add(GoalCell, GoalArrivalTime);
            }

            Result.MaxReservedTime = FMath::Max(Result.MaxReservedTime, GoalArrivalTime);
        }
    }

    CatPathLengths.Sort(
        [](const TPair<int32, int32>& A, const TPair<int32, int32>& B)
        {
            if (A.Value != B.Value)
            {
                return A.Value > B.Value;
            }

            return A.Key < B.Key;
        });

    const int32 ContributorsToLog = FMath::Min(3, CatPathLengths.Num());
    for (int32 Index = 0; Index < ContributorsToLog; ++Index)
    {
        if (!Result.CatTopContributors.IsEmpty())
        {
            Result.CatTopContributors += TEXT(",");
        }

        Result.CatTopContributors += FString::Printf(
            TEXT("%d:%d"),
            CatPathLengths[Index].Key,
            CatPathLengths[Index].Value);
    }

    if (ContributorsToLog < CatPathLengths.Num())
    {
        if (!Result.CatTopContributors.IsEmpty())
        {
            Result.CatTopContributors += TEXT(",");
        }

        Result.CatTopContributors += FString::Printf(
            TEXT("+%d more"),
            CatPathLengths.Num() - ContributorsToLog);
    }

    if (Result.CatTopContributors.IsEmpty())
    {
        Result.CatTopContributors = TEXT("none");
    }

    return Result;
}

void FPBSPlanner::CollectHigherPriorityAgents(
    int32 AgentId,
    const TMap<int32, TSet<int32>>& HigherThan,
    TSet<int32>& OutHigherAgents) const
{
    OutHigherAgents.Reset();

    TMap<int32, TSet<int32>> ReverseGraph;

    for (const auto& KVP : HigherThan)
    {
        const int32 HigherAgent = KVP.Key;
        for (int32 LowerAgent : KVP.Value)
        {
            ReverseGraph.FindOrAdd(LowerAgent).Add(HigherAgent);
        }
    }

    TArray<int32> Stack;
    Stack.Add(AgentId);

    while (Stack.Num() > 0)
    {
        const int32 Current = Stack.Pop(EAllowShrinking::No);

        const TSet<int32>* DirectHigherPtr = ReverseGraph.Find(Current);
        if (!DirectHigherPtr)
        {
            continue;
        }

        for (int32 HigherAgent : *DirectHigherPtr)
        {
            if (!OutHigherAgents.Contains(HigherAgent))
            {
                OutHigherAgents.Add(HigherAgent);
                Stack.Add(HigherAgent);
            }
        }
    }
}

bool FPBSPlanner::CreatesCycle(
    int32 HigherAgent,
    int32 LowerAgent,
    const TMap<int32, TSet<int32>>& HigherThan) const
{
    const TSet<int32>* ExistingLowerSet = HigherThan.Find(HigherAgent);
    if (ExistingLowerSet && ExistingLowerSet->Contains(LowerAgent))
    {
        return false;
    }

    TSet<int32> Visited;
    TArray<int32> Stack;
    Stack.Add(LowerAgent);

    while (Stack.Num() > 0)
    {
        const int32 Current = Stack.Pop(EAllowShrinking::No);

        if (Current == HigherAgent)
        {
            return true;
        }

        if (Visited.Contains(Current))
        {
            continue;
        }

        Visited.Add(Current);

        const TSet<int32>* LowerSetPtr = HigherThan.Find(Current);
        if (!LowerSetPtr)
        {
            continue;
        }

        for (int32 Next : *LowerSetPtr)
        {
            if (!Visited.Contains(Next))
            {
                Stack.Add(Next);
            }
        }
    }

    return false;
}





FPBSPlanner::FPBSConflict FPBSPlanner::FindFirstConflict(
    const TMap<int32, TArray<FIntVector>>& PathsByAgent) const
{
    TArray<FPBSConflict> Conflicts;
    CollectAllConflicts(PathsByAgent, Conflicts);
    return SelectConflictForBranching(Conflicts, PathsByAgent);
}

void FPBSPlanner::CollectAllConflicts(
    const TMap<int32, TArray<FIntVector>>& PathsByAgent,
    TArray<FPBSConflict>& OutConflicts) const
{
    OutConflicts.Reset();

    TArray<int32> AgentIds;
    PathsByAgent.GetKeys(AgentIds);
    AgentIds.Sort();

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

            const FPBSConflict Conflict = FindConflictBetweenTwoAgents(AgentA, *PathA, AgentB, *PathB);
            if (Conflict.bValid)
            {
                OutConflicts.Add(Conflict);
            }
        }
    }
}

int32 FPBSPlanner::ComputeSolutionCost(
    const TMap<int32, TArray<FIntVector>>& PathsByAgent) const
{
    int32 TotalCost = 0;

    for (const auto& KVP : PathsByAgent)
    {
        TotalCost += KVP.Value.Num();
    }

    return TotalCost;
}

int32 FPBSPlanner::ComputeMakespan(
    const TMap<int32, TArray<FIntVector>>& PathsByAgent) const
{
    int32 Makespan = 0;

    for (const auto& KVP : PathsByAgent)
    {
        Makespan = FMath::Max(Makespan, KVP.Value.Num());
    }

    return Makespan;
}

int32 FPBSPlanner::ComputePriorityRelationCount(
    const TMap<int32, TSet<int32>>& HigherThan) const
{
    int32 RelationCount = 0;

    for (const auto& KVP : HigherThan)
    {
        RelationCount += KVP.Value.Num();
    }

    return RelationCount;
}

FPBSPlanner::FPBSConflict FPBSPlanner::SelectConflictForBranching(
    const TArray<FPBSConflict>& Conflicts,
    const TMap<int32, TArray<FIntVector>>& PathsByAgent) const
{
    FPBSConflict SelectedConflict;
    if (Conflicts.Num() <= 0)
    {
        return SelectedConflict;
    }

    TArray<int32> CandidateConflictIndices;
    for (int32 i = 0; i < Conflicts.Num(); ++i)
    {
        int32 GoalAgent = INDEX_NONE;
        int32 OtherAgent = INDEX_NONE;
        if (IsTargetConflictPBS(Conflicts[i], PathsByAgent, GoalAgent, OtherAgent))
        {
            CandidateConflictIndices.Add(i);
        }
    }

    if (CandidateConflictIndices.Num() <= 0)
    {
        CandidateConflictIndices.Reserve(Conflicts.Num());
        for (int32 i = 0; i < Conflicts.Num(); ++i)
        {
            CandidateConflictIndices.Add(i);
        }
    }

    TMap<int32, int32> ConflictDegreeByAgent;
    TMap<FIntVector, int32> VertexHotspotCount;

    for (int32 ConflictIdx : CandidateConflictIndices)
    {
        const FPBSConflict& Conflict = Conflicts[ConflictIdx];
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

    int32 BestConflictIdx = CandidateConflictIndices[0];
    for (int32 CandidateListIdx = 1; CandidateListIdx < CandidateConflictIndices.Num(); ++CandidateListIdx)
    {
        const FPBSConflict& Candidate = Conflicts[CandidateConflictIndices[CandidateListIdx]];
        const FPBSConflict& Best = Conflicts[BestConflictIdx];

        const int32 CandidateDegree = GetConflictDegree(Candidate.AgentA) + GetConflictDegree(Candidate.AgentB);
        const int32 BestDegree = GetConflictDegree(Best.AgentA) + GetConflictDegree(Best.AgentB);
        if (CandidateDegree != BestDegree)
        {
            if (CandidateDegree > BestDegree)
            {
                BestConflictIdx = CandidateConflictIndices[CandidateListIdx];
            }
            continue;
        }

        const int32 CandidateHotspot = Candidate.bIsEdgeConflict ? 0 : GetVertexHotspot(Candidate.Cell);
        const int32 BestHotspot = Best.bIsEdgeConflict ? 0 : GetVertexHotspot(Best.Cell);
        if (CandidateHotspot != BestHotspot)
        {
            if (CandidateHotspot > BestHotspot)
            {
                BestConflictIdx = CandidateConflictIndices[CandidateListIdx];
            }
            continue;
        }

        if (Candidate.bIsEdgeConflict != Best.bIsEdgeConflict)
        {
            if (!Candidate.bIsEdgeConflict && Best.bIsEdgeConflict)
            {
                BestConflictIdx = CandidateConflictIndices[CandidateListIdx];
            }
            continue;
        }

        if (Candidate.TimeStep != Best.TimeStep)
        {
            if (Candidate.TimeStep < Best.TimeStep)
            {
                BestConflictIdx = CandidateConflictIndices[CandidateListIdx];
            }
            continue;
        }

        const int32 CandidateFirstAgent = FMath::Min(Candidate.AgentA, Candidate.AgentB);
        const int32 BestFirstAgent = FMath::Min(Best.AgentA, Best.AgentB);
        if (CandidateFirstAgent != BestFirstAgent)
        {
            if (CandidateFirstAgent < BestFirstAgent)
            {
                BestConflictIdx = CandidateConflictIndices[CandidateListIdx];
            }
            continue;
        }

        const int32 CandidateSecondAgent = FMath::Max(Candidate.AgentA, Candidate.AgentB);
        const int32 BestSecondAgent = FMath::Max(Best.AgentA, Best.AgentB);
        if (CandidateSecondAgent < BestSecondAgent)
        {
            BestConflictIdx = CandidateConflictIndices[CandidateListIdx];
        }
    }

    return Conflicts[BestConflictIdx];
}

void FPBSPlanner::RefreshNodeState(FPBSNode& Node) const
{
    Node.Cost = ComputeSolutionCost(Node.PathsByAgent);
    Node.Makespan = ComputeMakespan(Node.PathsByAgent);
    CollectAllConflicts(Node.PathsByAgent, Node.Conflicts);
    Node.ExpansionStage = 0;
    Node.PreferredHigherAgent = INDEX_NONE;
    Node.PreferredLowerAgent = INDEX_NONE;
    Node.AlternateHigherAgent = INDEX_NONE;
    Node.AlternateLowerAgent = INDEX_NONE;
}

FPBSPlanner::FPBSConflict FPBSPlanner::FindConflictBetweenTwoAgents(
    int32 AgentA,
    const TArray<FIntVector>& PathA,
    int32 AgentB,
    const TArray<FIntVector>& PathB) const
{
    FPBSConflict Conflict;
    const int32 MaxTime = FMath::Max(PathA.Num(), PathB.Num());

    for (int32 TimeStep = 0; TimeStep < MaxTime; ++TimeStep)
    {
        const FIntVector CellA = GetCellAtTime(PathA, TimeStep);
        const FIntVector CellB = GetCellAtTime(PathB, TimeStep);

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
            const FIntVector PrevA = GetCellAtTime(PathA, TimeStep - 1);
            const FIntVector PrevB = GetCellAtTime(PathB, TimeStep - 1);

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

    return Conflict;
}

FIntVector FPBSPlanner::GetCellAtTime(
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

TArray<FVector> FPBSPlanner::CellPathToWorldPath(
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
