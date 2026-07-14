#include "Execution/ExecutionReplanCandidateSelector.h"

#include "Planning/UTMSafetyModel.h"

namespace
{
    FIntVector GetCellAtTime(const TArray<FIntVector>& Cells, int32 TimeStep)
    {
        if (Cells.Num() <= 0)
        {
            return FIntVector::ZeroValue;
        }

        if (TimeStep <= 0)
        {
            return Cells[0];
        }

        if (TimeStep < Cells.Num())
        {
            return Cells[TimeStep];
        }

        return Cells.Last();
    }

    FIntVector GetPredictedCellAtOffset(const FExecutionAgentSnapshot& Agent, int32 Offset)
    {
        if (Offset <= 0 || Agent.PlannedCells.Num() <= 0)
        {
            return Agent.ObservedCell;
        }

        const int32 BaseIndex = FMath::Clamp(Agent.ExecutedPlanIndex, 0, Agent.PlannedCells.Num() - 1);
        return GetCellAtTime(Agent.PlannedCells, BaseIndex + Offset);
    }

    int32 GetCellDistance(const FIntVector& A, const FIntVector& B)
    {
        return FMath::Max3(
            FMath::Abs(A.X - B.X),
            FMath::Abs(A.Y - B.Y),
            FMath::Abs(A.Z - B.Z));
    }

    bool IsActiveMission(
        const TMap<int32, const FExecutionAgentSnapshot*>& AgentsByMissionId,
        int32 MissionId)
    {
        const FExecutionAgentSnapshot* const* Agent = AgentsByMissionId.Find(MissionId);
        return Agent && *Agent && !(*Agent)->bFinished;
    }

    bool HaveStaticUTMCoupling(
        const FExecutionReplanCandidateSelectionInput& Input,
        const FIntVector& CellA,
        int32 MissionIdA,
        const FIntVector& CellB,
        int32 MissionIdB)
    {
        if (!Input.bCheckStaticUTMSafety)
        {
            return false;
        }

        const FDroneMissionConfig* MissionConfigA = Input.MissionConfigsById.Find(MissionIdA);
        const FDroneMissionConfig* MissionConfigB = Input.MissionConfigsById.Find(MissionIdB);
        if (!MissionConfigA || !MissionConfigB)
        {
            return false;
        }

        return FUTMSafetyModel::HasStaticUTMConfigConflict(CellA, *MissionConfigA, CellB, *MissionConfigB);
    }

    bool AreCurrentStartsCoupled(
        const FExecutionReplanCandidateSelectionInput& Input,
        const TMap<int32, const FExecutionAgentSnapshot*>& AgentsByMissionId,
        int32 MissionIdA,
        int32 MissionIdB)
    {
        if (MissionIdA == MissionIdB)
        {
            return false;
        }

        const FExecutionAgentSnapshot* const* AgentA = AgentsByMissionId.Find(MissionIdA);
        const FExecutionAgentSnapshot* const* AgentB = AgentsByMissionId.Find(MissionIdB);
        if (!AgentA || !*AgentA || !AgentB || !*AgentB || (*AgentA)->bFinished || (*AgentB)->bFinished)
        {
            return false;
        }

        return (*AgentA)->ObservedCell == (*AgentB)->ObservedCell
            || HaveStaticUTMCoupling(
                Input,
                (*AgentA)->ObservedCell,
                MissionIdA,
                (*AgentB)->ObservedCell,
                MissionIdB);
    }

    bool AreWithinSpatialExpansion(
        const TMap<int32, const FExecutionAgentSnapshot*>& AgentsByMissionId,
        int32 MissionIdA,
        int32 MissionIdB,
        int32 SpatialRadiusCells)
    {
        if (SpatialRadiusCells <= 0 || MissionIdA == MissionIdB)
        {
            return false;
        }

        const FExecutionAgentSnapshot* const* AgentA = AgentsByMissionId.Find(MissionIdA);
        const FExecutionAgentSnapshot* const* AgentB = AgentsByMissionId.Find(MissionIdB);
        if (!AgentA || !*AgentA || !AgentB || !*AgentB || (*AgentA)->bFinished || (*AgentB)->bFinished)
        {
            return false;
        }

        return GetCellDistance((*AgentA)->ObservedCell, (*AgentB)->ObservedCell) <= SpatialRadiusCells;
    }

    bool HaveFutureWindowCoupling(
        const FExecutionReplanCandidateSelectionInput& Input,
        const TMap<int32, const FExecutionAgentSnapshot*>& AgentsByMissionId,
        int32 MissionIdA,
        int32 MissionIdB,
        int32 LookaheadSteps)
    {
        if (LookaheadSteps <= 0 || MissionIdA == MissionIdB)
        {
            return false;
        }

        const FExecutionAgentSnapshot* const* AgentA = AgentsByMissionId.Find(MissionIdA);
        const FExecutionAgentSnapshot* const* AgentB = AgentsByMissionId.Find(MissionIdB);
        if (!AgentA || !*AgentA || !AgentB || !*AgentB || (*AgentA)->bFinished || (*AgentB)->bFinished)
        {
            return false;
        }

        for (int32 Offset = 0; Offset <= LookaheadSteps; ++Offset)
        {
            const FIntVector CellA = GetPredictedCellAtOffset(**AgentA, Offset);
            const FIntVector CellB = GetPredictedCellAtOffset(**AgentB, Offset);

            if (CellA == CellB || HaveStaticUTMCoupling(Input, CellA, MissionIdA, CellB, MissionIdB))
            {
                return true;
            }

            if (Offset > 0)
            {
                const FIntVector PrevA = GetPredictedCellAtOffset(**AgentA, Offset - 1);
                const FIntVector PrevB = GetPredictedCellAtOffset(**AgentB, Offset - 1);
                if (PrevA == CellB && PrevB == CellA && CellA != CellB)
                {
                    return true;
                }
            }
        }

        return false;
    }
}

FExecutionReplanCandidateSelectionResult FExecutionReplanCandidateSelector::Select(
    const FExecutionReplanCandidateSelectionInput& Input)
{
    FExecutionReplanCandidateSelectionResult Result;

    TMap<int32, const FExecutionAgentSnapshot*> AgentsByMissionId;
    AgentsByMissionId.Reserve(Input.Snapshot.Agents.Num());
    for (const FExecutionAgentSnapshot& Agent : Input.Snapshot.Agents)
    {
        AgentsByMissionId.Add(Agent.MissionId, &Agent);
    }

    for (const int32 MissionId : Input.RequestedMissionIds)
    {
        if (IsActiveMission(AgentsByMissionId, MissionId))
        {
            Result.ActiveRequestedMissionCount++;
        }
    }

    for (const FExecutionAgentSnapshot& Agent : Input.Snapshot.Agents)
    {
        if (Agent.bFinished)
        {
            continue;
        }

        if (Input.bGlobalReplan || Input.RequestedMissionIds.Contains(Agent.MissionId))
        {
            Result.CandidateMissionIds.Add(Agent.MissionId);
        }
    }

    if (Input.bGlobalReplan)
    {
        return Result;
    }

    bool bExpandedLocalComponent = true;
    while (bExpandedLocalComponent)
    {
        bExpandedLocalComponent = false;

        for (const FExecutionAgentSnapshot& Agent : Input.Snapshot.Agents)
        {
            if (Agent.bFinished || Result.CandidateMissionIds.Contains(Agent.MissionId))
            {
                continue;
            }

            bool bCoupledWithCandidate = false;
            for (const int32 CandidateMissionId : Result.CandidateMissionIds)
            {
                if (AreCurrentStartsCoupled(Input, AgentsByMissionId, CandidateMissionId, Agent.MissionId)
                    || AreWithinSpatialExpansion(AgentsByMissionId, CandidateMissionId, Agent.MissionId, Input.SpatialRadiusCells)
                    || HaveFutureWindowCoupling(Input, AgentsByMissionId, CandidateMissionId, Agent.MissionId, Input.LookaheadSteps))
                {
                    bCoupledWithCandidate = true;
                    break;
                }
            }

            if (bCoupledWithCandidate)
            {
                Result.CandidateMissionIds.Add(Agent.MissionId);
                Result.bExpandedLocalComponent = true;
                bExpandedLocalComponent = true;
            }
        }
    }

    return Result;
}
