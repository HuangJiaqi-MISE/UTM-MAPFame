#include "Execution/ExecutionStepResultApplier.h"

#include "Execution/ExecutionStateTransition.h"

FExecutionStepResultApplyResult FExecutionStepResultApplier::Apply(
    const FExecutionStepResultApplyRequest& Request,
    const FExecutionControllerStepResult& ControllerResult)
{
    FExecutionStepResultApplyResult Result;
    Result.AppliedAgents.Reserve(Request.OrderedMissionIds.Num());

    for (const int32 MissionId : Request.OrderedMissionIds)
    {
        const FExecutionStepResultApplyAgentState* AgentState =
            Request.AgentStatesByMissionId.Find(MissionId);
        const FExecutionStepProposal* Proposal =
            ControllerResult.StepProposals.Find(MissionId);
        if (!AgentState || AgentState->PlannedCellCount <= 0 || !Proposal)
        {
            continue;
        }

        FExecutionStepAppliedAgent AppliedAgent;
        AppliedAgent.MissionId = MissionId;
        AppliedAgent.bReplanRequestedForState =
            ControllerResult.RequestedReplanMissionIds.Contains(MissionId) ||
            ControllerResult.SuccessfulReplanMissionIds.Contains(MissionId);
        AppliedAgent.bReplannedForState =
            ControllerResult.SuccessfulReplanMissionIds.Contains(MissionId);

        FExecutionStateTransitionInput TransitionInput;
        TransitionInput.CurrentState = AgentState->CurrentState;
        TransitionInput.PlannedCellCount = AgentState->PlannedCellCount;
        TransitionInput.FinalPlannedCell = AgentState->FinalPlannedCell;
        TransitionInput.bReplanRequestedForState =
            AppliedAgent.bReplanRequestedForState;
        TransitionInput.bOriginallyRequestedForReplan =
            ControllerResult.RequestedReplanMissionIds.Contains(MissionId);
        TransitionInput.bReplannedForState = AppliedAgent.bReplannedForState;
        TransitionInput.bReplanSucceeded = ControllerResult.bReplanSucceeded;

        AppliedAgent.TransitionResult =
            FExecutionStateTransition::Compute(TransitionInput, *Proposal);
        Result.bAnyActive |= !AppliedAgent.TransitionResult.NextState.bFinished;
        Result.AppliedAgents.Add(MoveTemp(AppliedAgent));
    }

    return Result;
}
