#include "Execution/ExecutionStateTransition.h"

FExecutionStateTransitionResult FExecutionStateTransition::Compute(
    const FExecutionStateTransitionInput& Input,
    const FExecutionStepProposal& Proposal)
{
    FExecutionStateTransitionResult Result;
    Result.NextState = Input.CurrentState;
    Result.CommittedCell = Proposal.ProposedCell;

    if (Input.PlannedCellCount <= 0)
    {
        return Result;
    }

    Result.NextState.ExecutedPlanIndex = FMath::Clamp(
        Proposal.ProposedPlanIndex,
        0,
        Input.PlannedCellCount - 1);
    Result.NextState.MaxAlignmentSpatialError = FMath::Max(
        Result.NextState.MaxAlignmentSpatialError,
        Proposal.AlignmentDecision.SpatialErrorCells);
    Result.NextState.MaxAlignmentTemporalError = FMath::Max(
        Result.NextState.MaxAlignmentTemporalError,
        FMath::Abs(Proposal.AlignmentDecision.TemporalErrorSteps));

    if (Proposal.FinalAction == EExecutionPolicyAction::SnapToPlanIndex)
    {
        Result.NextState.AlignmentSnapCount++;
    }
    else if (Proposal.FinalAction == EExecutionPolicyAction::RecoverTowardPlan)
    {
        Result.NextState.AlignmentCorrectionCount++;
    }
    else if (Proposal.FinalAction == EExecutionPolicyAction::HoldForAlignment ||
        Proposal.FinalAction == EExecutionPolicyAction::HoldForPredictedConflict ||
        Proposal.FinalAction == EExecutionPolicyAction::HoldForSafetyGate ||
        Proposal.FinalAction == EExecutionPolicyAction::HoldForReplan)
    {
        Result.NextState.AlignmentHoldCount++;
    }

    if (Proposal.bHeldForPredictedConflict)
    {
        Result.NextState.AlignmentConflictHoldCount++;
        Result.NextState.ConsecutiveConflictHoldCount++;
    }
    else if (Proposal.FinalAction != EExecutionPolicyAction::HoldForReplan)
    {
        Result.NextState.ConsecutiveConflictHoldCount = 0;
    }

    if (Proposal.FinalAction == EExecutionPolicyAction::HoldForSafetyGate)
    {
        Result.NextState.ConsecutiveSafetyGateHoldCount++;
    }
    else if (Proposal.FinalAction != EExecutionPolicyAction::HoldForReplan)
    {
        Result.NextState.ConsecutiveSafetyGateHoldCount = 0;
    }

    if (Input.bReplanRequestedForState)
    {
        Result.NextState.AlignmentReplanRequestCount++;
    }

    if (Input.bReplannedForState)
    {
        Result.NextState.AlignmentSuccessfulReplanCount++;
        Result.NextState.bAlignmentLost = false;
        Result.NextState.ConsecutiveConflictHoldCount = 0;
        Result.NextState.ConsecutiveSafetyGateHoldCount = 0;
    }
    else if ((Proposal.bRequiresReplan ||
        Proposal.bInitialAlignmentInvalid ||
        Input.bOriginallyRequestedForReplan) &&
        !Input.bReplanSucceeded)
    {
        Result.NextState.bAlignmentLost = true;
    }

    if (Proposal.bDelayRequested)
    {
        Result.NextState.TotalDelaySteps++;
    }

    Result.NextState.bFinished =
        Result.NextState.ExecutedPlanIndex >= Input.PlannedCellCount - 1 &&
        Result.CommittedCell == Input.FinalPlannedCell;

    return Result;
}
