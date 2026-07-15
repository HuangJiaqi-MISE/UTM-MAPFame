#include "Execution/ExecutionReplanProposalSynchronizer.h"

void FExecutionReplanProposalSynchronizer::Apply(
    const FExecutionReplanProposalSyncRequest& Request,
    TMap<int32, FExecutionStepProposal>& InOutStepProposals)
{
    for (const int32 MissionId : Request.OrderedMissionIds)
    {
        if (!Request.TargetMissionIds.Contains(MissionId))
        {
            continue;
        }

        FExecutionStepProposal* Proposal = InOutStepProposals.Find(MissionId);
        const FExecutionReplanProposalAgentState* AgentState =
            Request.AgentStatesByMissionId.Find(MissionId);
        if (!Proposal || !AgentState || AgentState->PlannedCellCount <= 0)
        {
            continue;
        }

        const bool bReplanned = Request.ReplannedMissionIds.Contains(MissionId);
        Proposal->bValid = true;
        Proposal->bHeldForReplan = true;
        Proposal->bHeldForPredictedConflict = false;
        Proposal->bRequiresReplan = false;
        Proposal->FinalAction = EExecutionPolicyAction::HoldForReplan;
        Proposal->ReferencePlanIndex = FMath::Clamp(
            AgentState->ExecutedPlanIndex,
            0,
            AgentState->PlannedCellCount - 1);
        Proposal->ProposedPlanIndex = bReplanned
            ? FMath::Min(
                Proposal->ReferencePlanIndex + 1,
                AgentState->PlannedCellCount - 1)
            : Proposal->ReferencePlanIndex;
        Proposal->ProposedCell = Proposal->ObservedCell;
        Proposal->ResolutionReason = bReplanned
            ? Request.ReplannedReason
            : Request.SynchronizedReason;
    }
}
