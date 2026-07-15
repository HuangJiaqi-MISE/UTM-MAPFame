#include "Execution/ExecutionStepProposalBuilder.h"

FExecutionStepProposalBuildResult FExecutionStepProposalBuilder::Build(
    const FExecutionStepProposalBuildInput& Input,
    const FExecutionStepDecision& AlignmentDecision)
{
    FExecutionStepProposalBuildResult Result;
    if (Input.PlannedCellCount <= 0)
    {
        return Result;
    }

    FExecutionStepProposal& Proposal = Result.Proposal;
    Proposal.MissionId = Input.MissionId;
    Proposal.ObservedCell = Input.ObservedCell;
    Proposal.bDelayRequested = Input.bDelayRequested;
    Proposal.AlignmentDecision = AlignmentDecision;
    Proposal.ReferencePlanIndex = FMath::Clamp(
        Input.CurrentPlanIndex,
        0,
        Input.PlannedCellCount - 1);
    Proposal.ProposedPlanIndex = Proposal.ReferencePlanIndex;
    Proposal.ProposedCell = Input.ObservedCell;
    Proposal.FinalAction = EExecutionPolicyAction::HoldForAlignment;
    Proposal.ResolutionReason = AlignmentDecision.Reason;

    if (AlignmentDecision.bValid)
    {
        Proposal.bValid = true;
        Proposal.bRequiresReplan = AlignmentDecision.bRequiresReplan;
        Proposal.ReferencePlanIndex = FMath::Clamp(
            AlignmentDecision.ReferencePlanIndex,
            0,
            Input.PlannedCellCount - 1);
        Proposal.ProposedPlanIndex = FMath::Clamp(
            AlignmentDecision.TargetPlanIndex,
            0,
            Input.PlannedCellCount - 1);
        Proposal.ProposedCell = AlignmentDecision.TargetCell;
        Proposal.FinalAction = AlignmentDecision.Action;
    }
    else
    {
        Proposal.bInitialAlignmentInvalid = true;
        Proposal.bRequiresReplan = true;
        Proposal.ResolutionReason = AlignmentDecision.Reason.IsEmpty()
            ? TEXT("invalid alignment result")
            : AlignmentDecision.Reason;
    }

    Result.bRequestsReplan =
        Proposal.bRequiresReplan || Proposal.bInitialAlignmentInvalid;
    Result.bSuccess = true;
    return Result;
}
