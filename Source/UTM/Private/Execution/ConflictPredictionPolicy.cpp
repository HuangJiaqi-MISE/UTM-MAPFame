#include "Execution/ConflictPredictionPolicy.h"

#include "Planning/UTMSafetyModel.h"

namespace
{
    EExecutionPredictedConflictType ConflictPredictionConvertUTMConflictType(EStaticUTMConflictType Type)
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

    bool ConflictPredictionFindPairConflict(
        const FExecutionConflictCheckItem& A,
        const FExecutionConflictCheckItem& B,
        const FConflictPredictionSettings& Settings,
        const FExecutionConflictPredictionInput& Input,
        FExecutionPredictedConflict& OutConflict)
    {
        OutConflict = FExecutionPredictedConflict();

        if (!A.bValid || !B.bValid)
        {
            return false;
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
            A.TargetCell != B.TargetCell;

        if (Settings.bCheckEdgeConflicts && bEdgeSwap)
        {
            OutConflict.Type = EExecutionPredictedConflictType::Edge;
            OutConflict.AgentA = A.MissionId;
            OutConflict.AgentB = B.MissionId;
            OutConflict.Cell = A.TargetCell;
            OutConflict.AgentAFromCell = A.ObservedCell;
            OutConflict.AgentAToCell = A.TargetCell;
            OutConflict.AgentBFromCell = B.ObservedCell;
            OutConflict.AgentBToCell = B.TargetCell;
            return true;
        }

        if (Input.bCheckStaticUTMSafety && Input.MissionConfigsById)
        {
            const FDroneMissionConfig* MissionConfigA = Input.MissionConfigsById->Find(A.MissionId);
            const FDroneMissionConfig* MissionConfigB = Input.MissionConfigsById->Find(B.MissionId);
            if (MissionConfigA && MissionConfigB)
            {
                const EStaticUTMConflictType UTMConflictType =
                    FUTMSafetyModel::GetStaticUTMConfigConflictType(
                        A.TargetCell,
                        *MissionConfigA,
                        B.TargetCell,
                        *MissionConfigB);

                const EExecutionPredictedConflictType ConflictType =
                    ConflictPredictionConvertUTMConflictType(UTMConflictType);
                if (ConflictType != EExecutionPredictedConflictType::None)
                {
                    OutConflict.Type = ConflictType;
                    OutConflict.AgentA = A.MissionId;
                    OutConflict.AgentB = B.MissionId;
                    OutConflict.Cell = A.TargetCell;
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

    FExecutionConflictPredictionInput ConflictPredictionBuildInputFromDecisions(
        const TArray<FExecutionStepDecision>& Decisions)
    {
        FExecutionConflictPredictionInput Input;
        Input.Items.Reserve(Decisions.Num());

        for (const FExecutionStepDecision& Decision : Decisions)
        {
            FExecutionConflictCheckItem Item;
            Item.MissionId = Decision.MissionId;
            Item.bValid = Decision.bValid;
            Item.ObservedCell = Decision.ObservedCell;
            Item.TargetCell = Decision.TargetCell;
            Input.Items.Add(Item);
        }

        return Input;
    }
}

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
    const FExecutionConflictPredictionInput Input =
        ConflictPredictionBuildInputFromDecisions(Decisions);
    return FindFirstConflict(Input, OutConflict);
}

bool FConflictPredictionPolicy::FindFirstConflict(
    const FExecutionConflictPredictionInput& Input,
    FExecutionPredictedConflict& OutConflict) const
{
    OutConflict = FExecutionPredictedConflict();

    for (int32 AIndex = 0; AIndex < Input.Items.Num(); ++AIndex)
    {
        const FExecutionConflictCheckItem& A = Input.Items[AIndex];
        if (!A.bValid)
        {
            continue;
        }

        for (int32 BIndex = AIndex + 1; BIndex < Input.Items.Num(); ++BIndex)
        {
            const FExecutionConflictCheckItem& B = Input.Items[BIndex];
            if (!B.bValid)
            {
                continue;
            }

            if (ConflictPredictionFindPairConflict(A, B, Settings, Input, OutConflict))
            {
                return true;
            }
        }
    }

    return false;
}

TArray<FExecutionPredictedConflict> FConflictPredictionPolicy::FindConflicts(
    const TArray<FExecutionStepDecision>& Decisions) const
{
    const FExecutionConflictPredictionInput Input =
        ConflictPredictionBuildInputFromDecisions(Decisions);
    return FindConflicts(Input);
}

TArray<FExecutionPredictedConflict> FConflictPredictionPolicy::FindConflicts(
    const FExecutionConflictPredictionInput& Input) const
{
    TArray<FExecutionPredictedConflict> Conflicts;

    for (int32 AIndex = 0; AIndex < Input.Items.Num(); ++AIndex)
    {
        const FExecutionConflictCheckItem& A = Input.Items[AIndex];
        if (!A.bValid)
        {
            continue;
        }

        for (int32 BIndex = AIndex + 1; BIndex < Input.Items.Num(); ++BIndex)
        {
            const FExecutionConflictCheckItem& B = Input.Items[BIndex];
            if (!B.bValid)
            {
                continue;
            }

            FExecutionPredictedConflict Conflict;
            if (ConflictPredictionFindPairConflict(A, B, Settings, Input, Conflict))
            {
                Conflicts.Add(Conflict);
            }
        }
    }

    return Conflicts;
}

