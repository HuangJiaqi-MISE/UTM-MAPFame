#include "Execution/ExecutionReplanPostCheckPolicy.h"

#include "Planning/UTMSafetyModel.h"

namespace
{
    FIntVector PostCheckClampCellToGrid(const FIntVector& Cell, const FIntVector& GridDim)
    {
        return FIntVector(
            FMath::Clamp(Cell.X, 0, FMath::Max(0, GridDim.X - 1)),
            FMath::Clamp(Cell.Y, 0, FMath::Max(0, GridDim.Y - 1)),
            FMath::Clamp(Cell.Z, 0, FMath::Max(0, GridDim.Z - 1)));
    }

    FIntVector PostCheckGetCellAtTime(const TArray<FIntVector>& Cells, int32 TimeStep)
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

    FIntVector PostCheckGetCell(
        const FExecutionReplanPostCheckInput& Input,
        const FExecutionAgentSnapshot& Agent,
        int32 Offset)
    {
        if (const TArray<FIntVector>* ReplannedCellPath = Input.ReplannedCellPathsByMission.Find(Agent.MissionId))
        {
            return PostCheckClampCellToGrid(PostCheckGetCellAtTime(*ReplannedCellPath, Offset), Input.GridDim);
        }

        if (Offset <= 0 || Agent.PlannedCells.Num() <= 0)
        {
            return Agent.ObservedCell;
        }

        const int32 BaseIndex = FMath::Clamp(Agent.ExecutedPlanIndex, 0, Agent.PlannedCells.Num() - 1);
        return PostCheckClampCellToGrid(PostCheckGetCellAtTime(Agent.PlannedCells, BaseIndex + Offset), Input.GridDim);
    }

    EExecutionPredictedConflictType PostCheckConvertUTMConflictType(EStaticUTMConflictType Type)
    {
        switch (Type)
        {
        case EStaticUTMConflictType::ProtectionFootprint:
            return EExecutionPredictedConflictType::ProtectionFootprint;
        case EStaticUTMConflictType::Downwash:
            return EExecutionPredictedConflictType::Downwash;
        default:
            return EExecutionPredictedConflictType::None;
        }
    }
}

FExecutionReplanPostCheckResult FExecutionReplanPostCheckPolicy::Validate(
    const FExecutionReplanPostCheckInput& Input)
{
    FExecutionReplanPostCheckResult Result;

    TMap<int32, const FExecutionAgentSnapshot*> AgentsByMissionId;
    AgentsByMissionId.Reserve(Input.Snapshot.Agents.Num());
    TArray<int32> ValidationMissionIds;
    ValidationMissionIds.Reserve(Input.Snapshot.Agents.Num());

    for (const FExecutionAgentSnapshot& Agent : Input.Snapshot.Agents)
    {
        AgentsByMissionId.Add(Agent.MissionId, &Agent);
        ValidationMissionIds.Add(Agent.MissionId);
    }

    ValidationMissionIds.Sort();

    const int32 PostCheckLookaheadSteps = FMath::Max(0, Input.LookaheadSteps);
    for (int32 Offset = 0; Offset <= PostCheckLookaheadSteps; ++Offset)
    {
        for (int32 I = 0; I < ValidationMissionIds.Num(); ++I)
        {
            const FExecutionAgentSnapshot* const* AgentA = AgentsByMissionId.Find(ValidationMissionIds[I]);
            if (!AgentA || !*AgentA || (*AgentA)->PlannedCells.Num() <= 0)
            {
                continue;
            }

            for (int32 J = I + 1; J < ValidationMissionIds.Num(); ++J)
            {
                const FExecutionAgentSnapshot* const* AgentB = AgentsByMissionId.Find(ValidationMissionIds[J]);
                if (!AgentB || !*AgentB || (*AgentB)->PlannedCells.Num() <= 0)
                {
                    continue;
                }

                if (!Input.CandidateMissionIds.Contains((*AgentA)->MissionId)
                    && !Input.CandidateMissionIds.Contains((*AgentB)->MissionId))
                {
                    continue;
                }

                const FIntVector CellA = PostCheckGetCell(Input, **AgentA, Offset);
                const FIntVector CellB = PostCheckGetCell(Input, **AgentB, Offset);

                if (CellA == CellB)
                {
                    Result.bHasConflict = true;
                    Result.Conflict.Type = EExecutionPredictedConflictType::Vertex;
                    Result.Conflict.AgentA = (*AgentA)->MissionId;
                    Result.Conflict.AgentB = (*AgentB)->MissionId;
                    Result.Conflict.Cell = CellA;
                    Result.ConflictOffset = Offset;
                    return Result;
                }

                if (Offset > 0)
                {
                    const FIntVector PrevA = PostCheckGetCell(Input, **AgentA, Offset - 1);
                    const FIntVector PrevB = PostCheckGetCell(Input, **AgentB, Offset - 1);
                    const bool bEdgeConflict =
                        (PrevA == CellB) &&
                        (PrevB == CellA) &&
                        (CellA != CellB);

                    if (bEdgeConflict)
                    {
                        Result.bHasConflict = true;
                        Result.Conflict.Type = EExecutionPredictedConflictType::Edge;
                        Result.Conflict.AgentA = (*AgentA)->MissionId;
                        Result.Conflict.AgentB = (*AgentB)->MissionId;
                        Result.Conflict.Cell = CellA;
                        Result.ConflictOffset = Offset;
                        return Result;
                    }
                }

                if (Input.bCheckStaticUTMSafety)
                {
                    const FDroneMissionConfig* MissionConfigA = Input.MissionConfigsById.Find((*AgentA)->MissionId);
                    const FDroneMissionConfig* MissionConfigB = Input.MissionConfigsById.Find((*AgentB)->MissionId);
                    if (MissionConfigA && MissionConfigB)
                    {
                        const EStaticUTMConflictType UTMConflictType =
                            FUTMSafetyModel::GetStaticUTMConfigConflictType(CellA, *MissionConfigA, CellB, *MissionConfigB);

                        const EExecutionPredictedConflictType ConflictType = PostCheckConvertUTMConflictType(UTMConflictType);
                        if (ConflictType != EExecutionPredictedConflictType::None)
                        {
                            Result.bHasConflict = true;
                            Result.Conflict.Type = ConflictType;
                            Result.Conflict.AgentA = (*AgentA)->MissionId;
                            Result.Conflict.AgentB = (*AgentB)->MissionId;
                            Result.Conflict.Cell = CellA;
                            Result.ConflictOffset = Offset;
                            return Result;
                        }
                    }
                }
            }
        }
    }

    return Result;
}
