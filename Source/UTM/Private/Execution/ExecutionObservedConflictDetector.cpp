#include "Execution/ExecutionObservedConflictDetector.h"

namespace
{
    FIntVector GetObservedConflictCellAtTime(
        const TArray<FIntVector>& Cells,
        int32 TimeStep)
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
}

FExecutionObservedConflictDetectionResult
FExecutionObservedConflictDetector::Detect(
    const FExecutionObservedConflictDetectionRequest& Request)
{
    FExecutionObservedConflictDetectionResult Result;
    if (!Request.AgentStatesByMissionId)
    {
        return Result;
    }

    TArray<int32> MissionIds;
    Request.AgentStatesByMissionId->GetKeys(MissionIds);

    for (int32 I = 0; I < MissionIds.Num(); ++I)
    {
        const FExecutionAgentState* A =
            Request.AgentStatesByMissionId->Find(MissionIds[I]);
        if (!A)
        {
            continue;
        }

        for (int32 J = I + 1; J < MissionIds.Num(); ++J)
        {
            const FExecutionAgentState* B =
                Request.AgentStatesByMissionId->Find(MissionIds[J]);
            if (!B)
            {
                continue;
            }

            const FIntVector ACell =
                GetObservedConflictCellAtTime(
                    A->ActualCells,
                    Request.TimeStep);
            const FIntVector BCell =
                GetObservedConflictCellAtTime(
                    B->ActualCells,
                    Request.TimeStep);

            if (ACell == BCell)
            {
                FExecutionConflict Conflict;
                Conflict.TimeStep = Request.TimeStep;
                Conflict.AgentA = A->MissionId;
                Conflict.AgentB = B->MissionId;
                Conflict.bIsEdgeConflict = false;
                Conflict.Cell = ACell;
                Result.Conflicts.Add(Conflict);
            }

            if (Request.TimeStep <= 0)
            {
                continue;
            }

            const FIntVector APrev =
                GetObservedConflictCellAtTime(
                    A->ActualCells,
                    Request.TimeStep - 1);
            const FIntVector BPrev =
                GetObservedConflictCellAtTime(
                    B->ActualCells,
                    Request.TimeStep - 1);
            const bool bEdgeConflict =
                (APrev == BCell) &&
                (BPrev == ACell) &&
                (ACell != BCell);
            if (!bEdgeConflict)
            {
                continue;
            }

            FExecutionConflict Conflict;
            Conflict.TimeStep = Request.TimeStep;
            Conflict.AgentA = A->MissionId;
            Conflict.AgentB = B->MissionId;
            Conflict.bIsEdgeConflict = true;
            Conflict.FromA = APrev;
            Conflict.ToA = ACell;
            Conflict.FromB = BPrev;
            Conflict.ToB = BCell;
            Result.Conflicts.Add(Conflict);
        }
    }

    return Result;
}
