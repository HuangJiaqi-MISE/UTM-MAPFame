#pragma once

#include "CoreMinimal.h"
#include "Planning/MultiAgentPlannerBase.h"

class FGridMap3D;

class FPBSPlanner : public IMultiAgentPlannerBase
{
public:
    virtual bool PlanMissions(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        TMap<int32, TArray<FVector>>& OutPaths) override;

private:
    struct FPBSConflict
    {
        bool bValid = false;

        int32 AgentA = INDEX_NONE;
        int32 AgentB = INDEX_NONE;
        int32 TimeStep = 0;

        bool bIsEdgeConflict = false;
        FIntVector Cell = FIntVector::ZeroValue;

        FIntVector FromA = FIntVector::ZeroValue;
        FIntVector ToA = FIntVector::ZeroValue;
        FIntVector FromB = FIntVector::ZeroValue;
        FIntVector ToB = FIntVector::ZeroValue;
    };

    struct FEdgeKey
    {
        FIntVector From = FIntVector::ZeroValue;
        FIntVector To = FIntVector::ZeroValue;

        bool operator==(const FEdgeKey& Other) const
        {
            return From == Other.From && To == Other.To;
        }
    };

    FORCEINLINE friend uint32 GetTypeHash(const FEdgeKey& Edge)
    {
        uint32 Hash = GetTypeHash(Edge.From);
        Hash = HashCombine(Hash, GetTypeHash(Edge.To));
        return Hash;
    }

    struct FTimedState
    {
        FIntVector Cell = FIntVector::ZeroValue;
        int32 TimeStep = 0;

        bool operator==(const FTimedState& Other) const
        {
            return Cell == Other.Cell && TimeStep == Other.TimeStep;
        }

        FORCEINLINE friend uint32 GetTypeHash(const FTimedState& State)
        {
            uint32 Hash = GetTypeHash(State.Cell);
            Hash = HashCombine(Hash, GetTypeHash(State.TimeStep));
            return Hash;
        }
    };
    struct FSippStateKey
    {
        FIntVector Cell = FIntVector::ZeroValue;
        int32 IntervalStart = 0;
        int32 IntervalEnd = 0;

        bool operator==(const FSippStateKey& Other) const
        {
            return Cell == Other.Cell && IntervalStart == Other.IntervalStart && IntervalEnd == Other.IntervalEnd;
        }

        FORCEINLINE friend uint32 GetTypeHash(const FSippStateKey& State)
        {
            uint32 Hash = GetTypeHash(State.Cell);
            Hash = HashCombine(Hash, GetTypeHash(State.IntervalStart));
            Hash = HashCombine(Hash, GetTypeHash(State.IntervalEnd));
            return Hash;
        }
    };

    struct FSippLabelKey
    {
        FSippStateKey State;
        int32 ArrivalTime = 0;
        int32 ConflictScore = 0;

        bool operator==(const FSippLabelKey& Other) const
        {
            return State == Other.State && ArrivalTime == Other.ArrivalTime && ConflictScore == Other.ConflictScore;
        }

        FORCEINLINE friend uint32 GetTypeHash(const FSippLabelKey& Label)
        {
            uint32 Hash = GetTypeHash(Label.State);
            Hash = HashCombine(Hash, GetTypeHash(Label.ArrivalTime));
            Hash = HashCombine(Hash, GetTypeHash(Label.ConflictScore));
            return Hash;
        }
    };

    struct FSippParentInfo
    {
        FSippLabelKey ParentLabel;
        bool bHasParent = false;
    };

    struct FSippOpenEntry
    {
        FSippLabelKey Label;
        float FScore = 0.f;

        bool operator<(const FSippOpenEntry& Other) const
        {
            return FScore < Other.FScore;
        }
    };

    struct FReservationTable
    {
        TMap<int32, TSet<FIntVector>> VertexByTime;
        TMap<int32, TSet<FEdgeKey>> EdgeByTime;
        TMap<FIntVector, int32> PermanentVertexStartTime;
        TMap<int32, TSet<FIntVector>> CatVertexByTime;
        TMap<int32, TSet<FEdgeKey>> CatEdgeByTime;
        int32 MaxReservedTime = 0;
        int32 MaxCatTime = 0;
        FString CatTopContributors;
    };

    struct FPBSNode
    {
        TMap<int32, TSet<int32>> HigherThan;
        TMap<int32, TArray<FIntVector>> PathsByAgent;
        TArray<FPBSConflict> Conflicts;
        int32 Cost = 0;
        int32 Makespan = 0;
        int32 ExpansionStage = 0;
        int32 PreferredHigherAgent = INDEX_NONE;
        int32 PreferredLowerAgent = INDEX_NONE;
        int32 AlternateHigherAgent = INDEX_NONE;
        int32 AlternateLowerAgent = INDEX_NONE;
    };

    struct FOpenListEntry
    {
        FTimedState State;
        float FScore = 0.f;

        bool operator<(const FOpenListEntry& Other) const
        {
            return FScore < Other.FScore;
        }
    };

private:
    bool BuildRootNode(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        FPBSNode& OutRootNode) const;

    bool AddPriorityAndReplan(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        int32 HigherAgent,
        int32 LowerAgent,
        FPBSNode& InOutNode) const;

    bool ReplanAffectedAgents(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        int32 StartAgent,
        FPBSNode& InOutNode) const;

    bool LowLevelPlanForAgent(
        const FGridMap3D& GridMap,
        const FDroneMissionConfig& Mission,
        const FReservationTable& Reservation,
        const TArray<FIntVector>* ExistingPath,
        TArray<FIntVector>& OutPath) const;

    FReservationTable BuildReservationTableForAgent(
        int32 AgentId,
        const FPBSNode& Node) const;

    void CollectHigherPriorityAgents(
        int32 AgentId,
        const TMap<int32, TSet<int32>>& HigherThan,
        TSet<int32>& OutHigherAgents) const;

    bool CreatesCycle(
        int32 HigherAgent,
        int32 LowerAgent,
        const TMap<int32, TSet<int32>>& HigherThan) const;

    void CollectAllConflicts(
        const TMap<int32, TArray<FIntVector>>& PathsByAgent,
        TArray<FPBSConflict>& OutConflicts) const;

    FPBSConflict FindFirstConflict(
        const TMap<int32, TArray<FIntVector>>& PathsByAgent) const;

    int32 ComputeSolutionCost(
        const TMap<int32, TArray<FIntVector>>& PathsByAgent) const;

    int32 ComputeMakespan(
        const TMap<int32, TArray<FIntVector>>& PathsByAgent) const;

    int32 ComputePriorityRelationCount(
        const TMap<int32, TSet<int32>>& HigherThan) const;

    FPBSConflict SelectConflictForBranching(
        const TArray<FPBSConflict>& Conflicts,
        const TMap<int32, TArray<FIntVector>>& PathsByAgent) const;
    void RefreshNodeState(FPBSNode& Node) const;

    FIntVector GetCellAtTime(
        const TArray<FIntVector>& Path,
        int32 TimeStep) const;

    TArray<FVector> CellPathToWorldPath(
        const FGridMap3D& GridMap,
        const TArray<FIntVector>& CellPath) const;

    FPBSConflict FindConflictBetweenTwoAgents(
        int32 AgentA,
        const TArray<FIntVector>& PathA,
        int32 AgentB,
        const TArray<FIntVector>& PathB) const;
};
