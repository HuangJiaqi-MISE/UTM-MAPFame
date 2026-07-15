#include "Execution/ExecutionFinalSafetyGateCoordinator.h"

namespace
{
    void EmitFinalSafetyGateEvent(
        const FExecutionFinalSafetyGateCoordinatorCallbacks& Callbacks,
        const FExecutionFinalSafetyGateEvent& Event)
    {
        if (Callbacks.OnEvent)
        {
            Callbacks.OnEvent(Event);
        }
    }
}

FExecutionFinalSafetyGateCoordinatorResult FExecutionFinalSafetyGateCoordinator::Run(
    const FExecutionFinalSafetyGateCoordinatorRequest& Request,
    const FExecutionFinalSafetyGateCoordinatorCallbacks& Callbacks,
    TMap<int32, FExecutionStepProposal>& InOutStepProposals)
{
    FExecutionFinalSafetyGateCoordinatorResult Result;
    if (!Request.SafetyGateInput)
    {
        return Result;
    }

    const FExecutionFinalSafetyGateResult SafetyGateResult =
        FExecutionFinalSafetyGatePolicy::EvaluateAndApplyHold(
            *Request.SafetyGateInput,
            InOutStepProposals);
    if (!SafetyGateResult.bConflictDetected)
    {
        return Result;
    }

    Result.bConflictDetected = true;

    FExecutionFinalSafetyGateEvent UnsafeEvent;
    UnsafeEvent.Type = EExecutionFinalSafetyGateEventType::UnsafeProposalDetected;
    UnsafeEvent.Conflict = SafetyGateResult.InitialConflict;
    UnsafeEvent.MissionCount = SafetyGateResult.InitialHoldMissionCount;
    EmitFinalSafetyGateEvent(Callbacks, UnsafeEvent);

    for (const FExecutionFinalSafetyGateHoldExpansion& Expansion : SafetyGateResult.HoldExpansions)
    {
        FExecutionFinalSafetyGateEvent ExpansionEvent;
        ExpansionEvent.Type = EExecutionFinalSafetyGateEventType::HoldSetExpanded;
        ExpansionEvent.Conflict = Expansion.RemainingConflict;
        ExpansionEvent.PreviousMissionCount = Expansion.PreviousHoldCount;
        ExpansionEvent.MissionCount = Expansion.ExpandedHoldCount;
        EmitFinalSafetyGateEvent(Callbacks, ExpansionEvent);
    }

    if (!SafetyGateResult.bHoldConfigurationSafe)
    {
        FExecutionFinalSafetyGateEvent UnsafeHoldEvent;
        UnsafeHoldEvent.Type = EExecutionFinalSafetyGateEventType::HoldFallbackUnsafe;
        UnsafeHoldEvent.Conflict = SafetyGateResult.UnresolvedHoldConflict;
        EmitFinalSafetyGateEvent(Callbacks, UnsafeHoldEvent);
        Result.bStopExecution = true;
        return Result;
    }

    Result.RequestedReplanMissionIds = SafetyGateResult.HoldMissionIds;

    if (SafetyGateResult.bForceGlobalReplan)
    {
        FExecutionFinalSafetyGateEvent HoldLimitEvent;
        HoldLimitEvent.Type = EExecutionFinalSafetyGateEventType::HoldLimitReached;
        HoldLimitEvent.MissionId = SafetyGateResult.HoldLimitMissionId;
        HoldLimitEvent.HoldBudget = SafetyGateResult.HoldBudget;
        EmitFinalSafetyGateEvent(Callbacks, HoldLimitEvent);
    }

    FExecutionFinalSafetyGateEvent ReplanRequestedEvent;
    ReplanRequestedEvent.Type = EExecutionFinalSafetyGateEventType::SafeHoldReplanRequested;
    ReplanRequestedEvent.MissionCount = SafetyGateResult.HoldMissionIds.Num();
    ReplanRequestedEvent.bForceGlobalReplan = SafetyGateResult.bForceGlobalReplan;
    EmitFinalSafetyGateEvent(Callbacks, ReplanRequestedEvent);

    bool bSafetyGateReplanSucceeded = false;
    TSet<int32> SafetyGateSuccessfulReplanMissionIds;
    if (Request.ReplanMode != EExecutionPolicyReplanMode::Disabled)
    {
        const bool bUseGlobalReplan =
            SafetyGateResult.bForceGlobalReplan ||
            Request.ReplanMode == EExecutionPolicyReplanMode::GlobalUnfinished;
        if (Callbacks.RunReplan)
        {
            bSafetyGateReplanSucceeded = Callbacks.RunReplan(
                SafetyGateResult.HoldMissionIds,
                bUseGlobalReplan,
                SafetyGateSuccessfulReplanMissionIds);
        }

        if (!bSafetyGateReplanSucceeded &&
            !SafetyGateResult.bForceGlobalReplan &&
            Request.ReplanMode == EExecutionPolicyReplanMode::LocalConflictSet)
        {
            FExecutionFinalSafetyGateEvent UpgradeEvent;
            UpgradeEvent.Type = EExecutionFinalSafetyGateEventType::LocalReplanFailedUpgradeGlobal;
            EmitFinalSafetyGateEvent(Callbacks, UpgradeEvent);

            if (Callbacks.RunReplan)
            {
                bSafetyGateReplanSucceeded = Callbacks.RunReplan(
                    SafetyGateResult.HoldMissionIds,
                    true,
                    SafetyGateSuccessfulReplanMissionIds);
            }
        }
    }
    else
    {
        FExecutionFinalSafetyGateEvent DisabledEvent;
        DisabledEvent.Type = EExecutionFinalSafetyGateEventType::ReplanDisabledSafeHold;
        EmitFinalSafetyGateEvent(Callbacks, DisabledEvent);
    }

    if (bSafetyGateReplanSucceeded)
    {
        Result.bReplanSucceeded = true;
        Result.SuccessfulReplanMissionIds = SafetyGateSuccessfulReplanMissionIds;

        if (Callbacks.ApplyReplanResult)
        {
            Callbacks.ApplyReplanResult(
                SafetyGateResult.HoldMissionIds,
                SafetyGateSuccessfulReplanMissionIds,
                InOutStepProposals);
        }

        const FExecutionFinalSafetyGateConflictCheckResult FinalCheck =
            FExecutionFinalSafetyGatePolicy::CheckConflicts(
                *Request.SafetyGateInput,
                InOutStepProposals);
        if (FinalCheck.bHasConflict)
        {
            FExecutionFinalSafetyGateEvent FinalUnsafeEvent;
            FinalUnsafeEvent.Type = EExecutionFinalSafetyGateEventType::FinalProposalUnsafe;
            FinalUnsafeEvent.Conflict = FinalCheck.FirstConflict;
            EmitFinalSafetyGateEvent(Callbacks, FinalUnsafeEvent);
            Result.bStopExecution = true;
        }
    }
    else if (SafetyGateResult.bForceGlobalReplan)
    {
        FExecutionFinalSafetyGateEvent GlobalFailureEvent;
        GlobalFailureEvent.Type =
            EExecutionFinalSafetyGateEventType::GlobalReplanFailedAfterHoldLimit;
        EmitFinalSafetyGateEvent(Callbacks, GlobalFailureEvent);
        Result.bStopExecution = true;
    }

    return Result;
}
