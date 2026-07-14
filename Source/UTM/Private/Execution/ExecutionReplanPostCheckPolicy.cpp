#include "Execution/ExecutionReplanPostCheckPolicy.h"

#include "Execution/ConflictPredictionPolicy.h"

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

    FExecutionConflictCheckItem PostCheckMakeConflictCheckItem(
        int32 MissionId,
        const FIntVector& PreviousCell,
        const FIntVector& CurrentCell)
    {
        FExecutionConflictCheckItem Item;
        Item.MissionId = MissionId;
        Item.bValid = true;
        Item.ObservedCell = PreviousCell;
        Item.TargetCell = CurrentCell;
        return Item;
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

    FExecutionConflictPredictionInput ConflictPredictionInput;
    ConflictPredictionInput.MissionConfigsById = &Input.MissionConfigsById;
    ConflictPredictionInput.bCheckStaticUTMSafety = Input.bCheckStaticUTMSafety;

    const int32 PostCheckLookaheadSteps = FMath::Max(0, Input.LookaheadSteps);
    for (int32 Offset = 0; Offset <= PostCheckLookaheadSteps; ++Offset)
    {
        FConflictPredictionSettings ConflictPredictionSettings;
        ConflictPredictionSettings.bCheckEdgeConflicts = (Offset > 0);
        const FConflictPredictionPolicy ConflictPredictionPolicy(ConflictPredictionSettings);

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
                const FIntVector PreviousCellA = Offset > 0
                    ? PostCheckGetCell(Input, **AgentA, Offset - 1)
                    : CellA;
                const FIntVector PreviousCellB = Offset > 0
                    ? PostCheckGetCell(Input, **AgentB, Offset - 1)
                    : CellB;

                FExecutionPredictedConflict Conflict;
                if (ConflictPredictionPolicy.FindPairConflict(
                    PostCheckMakeConflictCheckItem((*AgentA)->MissionId, PreviousCellA, CellA),
                    PostCheckMakeConflictCheckItem((*AgentB)->MissionId, PreviousCellB, CellB),
                    ConflictPredictionInput,
                    Conflict))
                {
                    Result.bHasConflict = true;
                    Result.Conflict = Conflict;
                    Result.ConflictOffset = Offset;
                    return Result;
                }
            }
        }
    }

    return Result;
}
