#include "Execution/ExecutionConflictResolutionPolicy.h"

#include "Execution/ConflictPredictionPolicy.h"

namespace
{
    const TCHAR* ConflictTypeToString(EExecutionPredictedConflictType Type)
    {
        switch (Type)
        {
        case EExecutionPredictedConflictType::Vertex:
            return TEXT("Vertex");
        case EExecutionPredictedConflictType::Edge:
            return TEXT("Edge");
        case EExecutionPredictedConflictType::ProtectionFootprint:
            return TEXT("ProtectionFootprint");
        case EExecutionPredictedConflictType::Downwash:
            return TEXT("Downwash");
        default:
            return TEXT("None");
        }
    }

    FExecutionConflictPredictionInput BuildConflictPredictionInput(
        const FExecutionConflictResolutionInput& Input,
        const TMap<int32, FExecutionStepProposal>& StepProposals)
    {
        FExecutionConflictPredictionInput PredictionInput;
        PredictionInput.Items.Reserve(StepProposals.Num());
        PredictionInput.MissionConfigsById = Input.MissionConfigsById;
        PredictionInput.bCheckStaticUTMSafety = Input.Settings.bCheckStaticUTMSafety;

        TArray<int32> MissionIds;
        StepProposals.GetKeys(MissionIds);
        MissionIds.Sort();

        for (const int32 MissionId : MissionIds)
        {
            const FExecutionStepProposal* Proposal = StepProposals.Find(MissionId);
            if (!Proposal)
            {
                continue;
            }

            FExecutionConflictCheckItem Item;
            Item.MissionId = Proposal->MissionId;
            Item.bValid = true;
            Item.ObservedCell = Proposal->ObservedCell;
            Item.TargetCell = Proposal->ProposedCell;
            PredictionInput.Items.Add(Item);
        }

        return PredictionInput;
    }

    const FExecutionConflictResolutionAgentState* FindAgentState(
        const FExecutionConflictResolutionInput& Input,
        int32 MissionId)
    {
        return Input.AgentStatesByMissionId.Find(MissionId);
    }

    int32 ChooseYieldingMissionId(
        const FExecutionConflictResolutionInput& Input,
        const FExecutionStepProposal& ProposalA,
        const FExecutionStepProposal& ProposalB)
    {
        const bool bAStays = (ProposalA.ProposedCell == ProposalA.ObservedCell);
        const bool bBStays = (ProposalB.ProposedCell == ProposalB.ObservedCell);
        if (bAStays != bBStays)
        {
            return bAStays ? ProposalB.MissionId : ProposalA.MissionId;
        }

        const FExecutionConflictResolutionAgentState* StateA =
            FindAgentState(Input, ProposalA.MissionId);
        const FExecutionConflictResolutionAgentState* StateB =
            FindAgentState(Input, ProposalB.MissionId);
        const bool bAGoalHold = StateA &&
            (StateA->bFinished || ProposalA.FinalAction == EExecutionPolicyAction::GoalHold);
        const bool bBGoalHold = StateB &&
            (StateB->bFinished || ProposalB.FinalAction == EExecutionPolicyAction::GoalHold);
        if (bAGoalHold != bBGoalHold)
        {
            return bAGoalHold ? ProposalB.MissionId : ProposalA.MissionId;
        }

        return ProposalA.MissionId > ProposalB.MissionId
            ? ProposalA.MissionId
            : ProposalB.MissionId;
    }
}

FExecutionConflictResolutionResult FExecutionConflictResolutionPolicy::Resolve(
    const FExecutionConflictResolutionInput& Input,
    TMap<int32, FExecutionStepProposal>& InOutStepProposals)
{
    FExecutionConflictResolutionResult Result;

    if (!Input.Settings.bEnabled || InOutStepProposals.Num() <= 1)
    {
        return Result;
    }

    const FConflictPredictionPolicy ConflictPredictionPolicy;
    auto FindFirstPredictedConflict = [&](FExecutionPredictedConflict& OutConflict) -> bool
        {
            return ConflictPredictionPolicy.FindFirstConflict(
                BuildConflictPredictionInput(Input, InOutStepProposals),
                OutConflict);
        };

    const int32 MaxPasses = FMath::Max(1, Input.Settings.MaxResolutionPasses);
    bool bNeedsAnotherPass = true;

    for (int32 Pass = 0; Pass < MaxPasses && bNeedsAnotherPass; ++Pass)
    {
        bNeedsAnotherPass = false;

        FExecutionPredictedConflict Conflict;
        if (!FindFirstPredictedConflict(Conflict))
        {
            break;
        }

        FExecutionStepProposal* ProposalA = InOutStepProposals.Find(Conflict.AgentA);
        FExecutionStepProposal* ProposalB = InOutStepProposals.Find(Conflict.AgentB);
        if (!ProposalA || !ProposalB)
        {
            Result.ReplanMissionIds.Add(Conflict.AgentA);
            Result.ReplanMissionIds.Add(Conflict.AgentB);
            break;
        }

        const int32 YieldMissionId = ChooseYieldingMissionId(Input, *ProposalA, *ProposalB);
        FExecutionStepProposal* YieldProposal = InOutStepProposals.Find(YieldMissionId);
        const int32 KeepMissionId = YieldMissionId == Conflict.AgentA
            ? Conflict.AgentB
            : Conflict.AgentA;

        if (!YieldProposal)
        {
            Result.ReplanMissionIds.Add(Conflict.AgentA);
            Result.ReplanMissionIds.Add(Conflict.AgentB);
            break;
        }

        const bool bAlreadyHolding =
            YieldProposal->ProposedCell == YieldProposal->ObservedCell &&
            YieldProposal->FinalAction == EExecutionPolicyAction::HoldForPredictedConflict;
        if (bAlreadyHolding)
        {
            Result.ReplanMissionIds.Add(Conflict.AgentA);
            Result.ReplanMissionIds.Add(Conflict.AgentB);

            FExecutionConflictResolutionEvent Event;
            Event.Type = EExecutionConflictResolutionEventType::UnresolvedConflict;
            Event.Conflict = Conflict;
            Result.Events.Add(Event);
            break;
        }

        YieldProposal->ProposedCell = YieldProposal->ObservedCell;
        YieldProposal->ProposedPlanIndex = YieldProposal->ReferencePlanIndex;
        YieldProposal->bHeldForPredictedConflict = true;
        YieldProposal->FinalAction = EExecutionPolicyAction::HoldForPredictedConflict;
        YieldProposal->ResolutionReason = FString::Printf(
            TEXT("yield to Mission %d for predicted %s conflict"),
            KeepMissionId,
            ConflictTypeToString(Conflict.Type));

        FExecutionConflictResolutionEvent Event;
        Event.Type = EExecutionConflictResolutionEventType::YieldApplied;
        Event.Conflict = Conflict;
        Event.YieldMissionId = YieldProposal->MissionId;
        Event.KeepMissionId = KeepMissionId;
        Result.Events.Add(Event);

        bNeedsAnotherPass = true;
    }

    FExecutionPredictedConflict RemainingConflict;
    if (FindFirstPredictedConflict(RemainingConflict))
    {
        Result.ReplanMissionIds.Add(RemainingConflict.AgentA);
        Result.ReplanMissionIds.Add(RemainingConflict.AgentB);

        FExecutionConflictResolutionEvent Event;
        Event.Type = EExecutionConflictResolutionEventType::RemainingConflict;
        Event.Conflict = RemainingConflict;
        Result.Events.Add(Event);
    }

    const int32 ConflictHoldBudget =
        FMath::Max(1, Input.Settings.ConflictHoldThresholdForReplan);
    TArray<int32> MissionIds;
    InOutStepProposals.GetKeys(MissionIds);
    MissionIds.Sort();

    for (const int32 MissionId : MissionIds)
    {
        const FExecutionStepProposal* Proposal = InOutStepProposals.Find(MissionId);
        const FExecutionConflictResolutionAgentState* State = FindAgentState(Input, MissionId);
        if (!Proposal || !State || !Proposal->bHeldForPredictedConflict)
        {
            continue;
        }

        if (State->ConsecutiveConflictHoldCount + 1 >= ConflictHoldBudget)
        {
            Result.ReplanMissionIds.Add(MissionId);
        }
    }

    return Result;
}
