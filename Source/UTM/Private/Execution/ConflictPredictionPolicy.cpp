#include "Execution/ConflictPredictionPolicy.h"

FConflictPredictionPolicy::FConflictPredictionPolicy(const FConflictPredictionSettings& InSettings)
    : Settings(InSettings)
{
}

void FConflictPredictionPolicy::SetSettings(const FConflictPredictionSettings& InSettings)
{
    Settings = InSettings;
}

bool FConflictPredictionPolicy::FindFirstConflict(
    const TArray<FExecutionStepDecision>& Decisions,
    FExecutionPredictedConflict& OutConflict) const
{
    OutConflict = FExecutionPredictedConflict();

    for (int32 AIndex = 0; AIndex < Decisions.Num(); ++AIndex)
    {
        const FExecutionStepDecision& A = Decisions[AIndex];
        if (!A.bValid)
        {
            continue;
        }

        for (int32 BIndex = AIndex + 1; BIndex < Decisions.Num(); ++BIndex)
        {
            const FExecutionStepDecision& B = Decisions[BIndex];
            if (!B.bValid)
            {
                continue;
            }

            if (Settings.bCheckVertexConflicts && A.TargetCell == B.TargetCell)
            {
                OutConflict.Type = EExecutionPredictedConflictType::Vertex;
                OutConflict.AgentA = A.MissionId;
                OutConflict.AgentB = B.MissionId;
                OutConflict.Cell = A.TargetCell;
                OutConflict.AgentAFromCell = A.ObservedCell;
                OutConflict.AgentAToCell = A.TargetCell;
                OutConflict.AgentBFromCell = B.ObservedCell;
                OutConflict.AgentBToCell = B.TargetCell;
                return true;
            }

            const bool bEdgeSwap =
                A.ObservedCell == B.TargetCell &&
                B.ObservedCell == A.TargetCell &&
                A.TargetCell != A.ObservedCell &&
                B.TargetCell != B.ObservedCell;

            if (Settings.bCheckEdgeConflicts && bEdgeSwap)
            {
                OutConflict.Type = EExecutionPredictedConflictType::Edge;
                OutConflict.AgentA = A.MissionId;
                OutConflict.AgentB = B.MissionId;
                OutConflict.AgentAFromCell = A.ObservedCell;
                OutConflict.AgentAToCell = A.TargetCell;
                OutConflict.AgentBFromCell = B.ObservedCell;
                OutConflict.AgentBToCell = B.TargetCell;
                return true;
            }
        }
    }

    return false;
}

TArray<FExecutionPredictedConflict> FConflictPredictionPolicy::FindConflicts(
    const TArray<FExecutionStepDecision>& Decisions) const
{
    TArray<FExecutionPredictedConflict> Conflicts;

    for (int32 AIndex = 0; AIndex < Decisions.Num(); ++AIndex)
    {
        const FExecutionStepDecision& A = Decisions[AIndex];
        if (!A.bValid)
        {
            continue;
        }

        for (int32 BIndex = AIndex + 1; BIndex < Decisions.Num(); ++BIndex)
        {
            const FExecutionStepDecision& B = Decisions[BIndex];
            if (!B.bValid)
            {
                continue;
            }

            if (Settings.bCheckVertexConflicts && A.TargetCell == B.TargetCell)
            {
                FExecutionPredictedConflict Conflict;
                Conflict.Type = EExecutionPredictedConflictType::Vertex;
                Conflict.AgentA = A.MissionId;
                Conflict.AgentB = B.MissionId;
                Conflict.Cell = A.TargetCell;
                Conflict.AgentAFromCell = A.ObservedCell;
                Conflict.AgentAToCell = A.TargetCell;
                Conflict.AgentBFromCell = B.ObservedCell;
                Conflict.AgentBToCell = B.TargetCell;
                Conflicts.Add(Conflict);
                continue;
            }

            const bool bEdgeSwap =
                A.ObservedCell == B.TargetCell &&
                B.ObservedCell == A.TargetCell &&
                A.TargetCell != A.ObservedCell &&
                B.TargetCell != B.ObservedCell;

            if (Settings.bCheckEdgeConflicts && bEdgeSwap)
            {
                FExecutionPredictedConflict Conflict;
                Conflict.Type = EExecutionPredictedConflictType::Edge;
                Conflict.AgentA = A.MissionId;
                Conflict.AgentB = B.MissionId;
                Conflict.AgentAFromCell = A.ObservedCell;
                Conflict.AgentAToCell = A.TargetCell;
                Conflict.AgentBFromCell = B.ObservedCell;
                Conflict.AgentBToCell = B.TargetCell;
                Conflicts.Add(Conflict);
            }
        }
    }

    return Conflicts;
}

