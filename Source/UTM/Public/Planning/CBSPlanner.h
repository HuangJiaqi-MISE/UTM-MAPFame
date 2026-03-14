#pragma once

#include "CoreMinimal.h"
#include "Planning/MultiAgentPlannerBase.h"

class FGridMap3D;

class FCBSPlanner : public IMultiAgentPlannerBase
{
public:
    virtual bool PlanMissions(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        TMap<int32, TArray<FVector>>& OutPaths) override;

private:
    struct FCBSConstraint
    {
        int32 AgentId = INDEX_NONE;
        FIntVector Cell = FIntVector::ZeroValue;
        int32 TimeStep = 0;

        bool bIsEdgeConstraint = false;
        FIntVector FromCell = FIntVector::ZeroValue;
        FIntVector ToCell = FIntVector::ZeroValue;
    };

    struct FCBSConflict
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

    struct FCBSNode
    {
        TArray<FCBSConstraint> Constraints;
        TMap<int32, TArray<FIntVector>> PathsByAgent;
        int32 Cost = 0;
    };

    struct FTimeAStarState
    {
        FIntVector Cell = FIntVector::ZeroValue;
        int32 TimeStep = 0;

        bool operator==(const FTimeAStarState& Other) const
        {
            return Cell == Other.Cell && TimeStep == Other.TimeStep;
        }
    };

    struct FStateKey
    {
        FIntVector Cell = FIntVector::ZeroValue;
        int32 TimeStep = 0;

        bool operator==(const FStateKey& Other) const
        {
            return Cell == Other.Cell && TimeStep == Other.TimeStep;
        }

        friend uint32 GetTypeHash(const FStateKey& Key)
        {
            return HashCombine(GetTypeHash(Key.Cell), GetTypeHash(Key.TimeStep));
        }
    };

    struct FEdgeConstraintKey
    {
        FIntVector From = FIntVector::ZeroValue;
        FIntVector To = FIntVector::ZeroValue;

        bool operator==(const FEdgeConstraintKey& Other) const
        {
            return From == Other.From && To == Other.To;
        }

        friend uint32 GetTypeHash(const FEdgeConstraintKey& Key)
        {
            return HashCombine(GetTypeHash(Key.From), GetTypeHash(Key.To));
        }
    };

    struct FCompiledConstraints
    {
        TMap<int32, TSet<FIntVector>> VertexByTime;
        TMap<int32, TSet<FEdgeConstraintKey>> EdgeByTime;
        int32 MaxConstraintTime = 0;

        bool IsVertexForbidden(const FIntVector& Cell, int32 TimeStep) const
        {
            const TSet<FIntVector>* Cells = VertexByTime.Find(TimeStep);
            return Cells != nullptr && Cells->Contains(Cell);
        }

        bool IsEdgeForbidden(const FIntVector& From, const FIntVector& To, int32 TimeStep) const
        {
            const TSet<FEdgeConstraintKey>* Edges = EdgeByTime.Find(TimeStep);
            if (!Edges)
            {
                return false;
            }

            FEdgeConstraintKey Key;
            Key.From = From;
            Key.To = To;
            return Edges->Contains(Key);
        }

        int32 GetHoldingTime(const FIntVector& GoalCell, int32 EarliestTime) const
        {
            int32 Result = EarliestTime;

            for (const auto& KVP : VertexByTime)
            {
                if (KVP.Key >= EarliestTime && KVP.Value.Contains(GoalCell))
                {
                    Result = FMath::Max(Result, KVP.Key + 1);
                }
            }

            return Result;
        }

        int32 GetStaticTimestep(int32 HoldingTime) const
        {
            return FMath::Max(MaxConstraintTime + 1, HoldingTime);
        }
    };

    FCompiledConstraints CompileConstraintsForAgent(
        int32 AgentId,
        const TArray<FCBSConstraint>& Constraints) const;

private:
    bool BuildRootNode(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        FCBSNode& OutRootNode) const;

    bool ReplanSingleAgentForNode(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        int32 AgentId,
        FCBSNode& InOutNode) const;

    FCBSConflict FindFirstConflict(
        const TMap<int32, TArray<FIntVector>>& PathsByAgent) const;

    int32 ComputeSolutionCost(
        const TMap<int32, TArray<FIntVector>>& PathsByAgent) const;

    TArray<FIntVector> WorldPathToCellPath(
        const FGridMap3D& GridMap,
        const TArray<FVector>& WorldPath) const;

    TArray<FVector> CellPathToWorldPath(
        const FGridMap3D& GridMap,
        const TArray<FIntVector>& CellPath) const;

    FIntVector GetCellAtTime(
        const TArray<FIntVector>& Path,
        int32 TimeStep) const;

    bool IsVertexConstrained(
        int32 AgentId,
        const FIntVector& Cell,
        int32 TimeStep,
        const TArray<FCBSConstraint>& Constraints) const;

    bool IsEdgeConstrained(
        int32 AgentId,
        const FIntVector& FromCell,
        const FIntVector& ToCell,
        int32 TimeStep,
        const TArray<FCBSConstraint>& Constraints) const;

    bool LowLevelPlanForAgent(
        const FGridMap3D& GridMap,
        const FDroneMissionConfig& Mission,
        const TArray<FCBSConstraint>& Constraints,
        const TArray<FIntVector>* ExistingPath,
        TArray<FIntVector>& OutPath) const;
};
