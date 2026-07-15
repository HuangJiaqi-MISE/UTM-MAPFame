#include "Execution/ExecutionFinalSafetyGatePolicy.h"

#include "Execution/ConflictPredictionPolicy.h"

FExecutionFinalSafetyGateResult FExecutionFinalSafetyGatePolicy::EvaluateAndApplyHold(
    const FExecutionFinalSafetyGateInput& Input,
    TMap<int32, FExecutionStepProposal>& InOutStepProposals)
{
    FExecutionFinalSafetyGateResult Result;

    if (!Input.Settings.bEnabled || InOutStepProposals.Num() <= 1)
    {
        return Result;
    }

    const FExecutionFinalSafetyGateConflictCheckResult InitialCheck =
        CheckConflicts(Input, InOutStepProposals);
    if (!InitialCheck.bHasConflict)
    {
        return Result;
    }

    Result.bConflictDetected = true;
    Result.InitialConflict = InitialCheck.FirstConflict;
    Result.HoldMissionIds = InitialCheck.ConflictMissionIds;
    Result.InitialHoldMissionCount = Result.HoldMissionIds.Num();

    for (int32 Pass = 0; Pass < Input.OrderedMissionIds.Num(); ++Pass)
    {
        ApplyHold(Result.HoldMissionIds, InOutStepProposals);

        const FExecutionFinalSafetyGateConflictCheckResult RemainingCheck =
            CheckConflicts(Input, InOutStepProposals);
        if (!RemainingCheck.bHasConflict)
        {
            Result.bHoldConfigurationSafe = true;
            break;
        }

        const int32 PreviousHoldCount = Result.HoldMissionIds.Num();
        for (const int32 MissionId : RemainingCheck.ConflictMissionIds)
        {
            Result.HoldMissionIds.Add(MissionId);
        }

        if (Result.HoldMissionIds.Num() > PreviousHoldCount)
        {
            FExecutionFinalSafetyGateHoldExpansion Expansion;
            Expansion.PreviousHoldCount = PreviousHoldCount;
            Expansion.ExpandedHoldCount = Result.HoldMissionIds.Num();
            Expansion.RemainingConflict = RemainingCheck.FirstConflict;
            Result.HoldExpansions.Add(Expansion);
            continue;
        }

        Result.UnresolvedHoldConflict = RemainingCheck.FirstConflict;
        break;
    }

    if (!Result.bHoldConfigurationSafe)
    {
        return Result;
    }

    ApplyHold(Result.HoldMissionIds, InOutStepProposals);

    Result.HoldBudget = FMath::Max(1, Input.Settings.MaxHoldSteps);
    for (const int32 MissionId : Result.HoldMissionIds)
    {
        const FExecutionFinalSafetyGateAgentState* AgentState =
            Input.AgentStatesByMissionId.Find(MissionId);
        if (AgentState &&
            AgentState->ConsecutiveSafetyGateHoldCount + 1 >= Result.HoldBudget)
        {
            Result.bForceGlobalReplan = true;
            Result.HoldLimitMissionId = MissionId;
            break;
        }
    }

    return Result;
}

FExecutionFinalSafetyGateConflictCheckResult FExecutionFinalSafetyGatePolicy::CheckConflicts(
    const FExecutionFinalSafetyGateInput& Input,
    const TMap<int32, FExecutionStepProposal>& StepProposals)
{
    FExecutionConflictPredictionInput PredictionInput;
    PredictionInput.Items.Reserve(StepProposals.Num());
    PredictionInput.MissionConfigsById = Input.MissionConfigsById;
    PredictionInput.bCheckStaticUTMSafety = Input.Settings.bCheckStaticUTMSafety;

    for (const int32 MissionId : Input.OrderedMissionIds)
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

    FExecutionFinalSafetyGateConflictCheckResult Result;
    const FConflictPredictionPolicy ConflictPredictionPolicy;
    const TArray<FExecutionPredictedConflict> Conflicts =
        ConflictPredictionPolicy.FindConflicts(PredictionInput);

    for (const FExecutionPredictedConflict& Conflict : Conflicts)
    {
        if (!Result.bHasConflict)
        {
            Result.bHasConflict = true;
            Result.FirstConflict = Conflict;
        }

        Result.ConflictMissionIds.Add(Conflict.AgentA);
        Result.ConflictMissionIds.Add(Conflict.AgentB);
    }

    return Result;
}

void FExecutionFinalSafetyGatePolicy::ApplyHold(
    const TSet<int32>& HoldMissionIds,
    TMap<int32, FExecutionStepProposal>& InOutStepProposals)
{
    for (const int32 MissionId : HoldMissionIds)
    {
        FExecutionStepProposal* Proposal = InOutStepProposals.Find(MissionId);
        if (!Proposal)
        {
            continue;
        }

        Proposal->bValid = true;
        Proposal->bHeldForPredictedConflict = false;
        Proposal->bHeldForReplan = false;
        Proposal->bRequiresReplan = true;
        Proposal->FinalAction = EExecutionPolicyAction::HoldForSafetyGate;
        Proposal->ProposedPlanIndex = Proposal->ReferencePlanIndex;
        Proposal->ProposedCell = Proposal->ObservedCell;
        Proposal->ResolutionReason = TEXT("final safety gate hold before replanning");
    }
}
