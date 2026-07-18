#include "Execution/ExecutionRuntimeSessionStepProcessor.h"

#include "Execution/ExecutionAlignmentPolicy.h"

namespace
{
    FExecutionStateTransitionState CaptureExecutionStateTransitionState(
        const FExecutionAgentState& State)
    {
        FExecutionStateTransitionState TransitionState;
        TransitionState.ExecutedPlanIndex = State.ExecutedPlanIndex;
        TransitionState.TotalDelaySteps = State.TotalDelaySteps;
        TransitionState.bFinished = State.bFinished;
        TransitionState.AlignmentCorrectionCount = State.AlignmentCorrectionCount;
        TransitionState.AlignmentHoldCount = State.AlignmentHoldCount;
        TransitionState.AlignmentConflictHoldCount = State.AlignmentConflictHoldCount;
        TransitionState.AlignmentSnapCount = State.AlignmentSnapCount;
        TransitionState.AlignmentReplanRequestCount = State.AlignmentReplanRequestCount;
        TransitionState.AlignmentSuccessfulReplanCount = State.AlignmentSuccessfulReplanCount;
        TransitionState.MaxAlignmentSpatialError = State.MaxAlignmentSpatialError;
        TransitionState.MaxAlignmentTemporalError = State.MaxAlignmentTemporalError;
        TransitionState.bAlignmentLost = State.bAlignmentLost;
        TransitionState.ConsecutiveConflictHoldCount = State.ConsecutiveConflictHoldCount;
        TransitionState.ConsecutiveSafetyGateHoldCount = State.ConsecutiveSafetyGateHoldCount;
        return TransitionState;
    }

    TMap<int32, FExecutionStepResultApplyAgentState>
        CaptureExecutionStepResultApplyAgentStates(
            const TArray<int32>& OrderedMissionIds,
            const TMap<int32, FExecutionAgentState>& AgentStatesByMissionId)
    {
        TMap<int32, FExecutionStepResultApplyAgentState> AgentStates;
        AgentStates.Reserve(OrderedMissionIds.Num());

        for (const int32 MissionId : OrderedMissionIds)
        {
            const FExecutionAgentState* State = AgentStatesByMissionId.Find(MissionId);
            if (!State)
            {
                continue;
            }

            FExecutionStepResultApplyAgentState AgentState;
            AgentState.CurrentState = CaptureExecutionStateTransitionState(*State);
            AgentState.PlannedCellCount = State->PlannedCells.Num();
            if (AgentState.PlannedCellCount > 0)
            {
                AgentState.FinalPlannedCell = State->PlannedCells.Last();
            }
            AgentStates.Add(MissionId, MoveTemp(AgentState));
        }

        return AgentStates;
    }

    void CommitExecutionStateTransition(
        FExecutionAgentState& State,
        const FExecutionStepProposal& Proposal,
        const FExecutionStateTransitionResult& Result)
    {
        State.ExecutedPlanIndex = Result.NextState.ExecutedPlanIndex;
        State.TotalDelaySteps = Result.NextState.TotalDelaySteps;
        State.bFinished = Result.NextState.bFinished;
        State.AlignmentCorrectionCount = Result.NextState.AlignmentCorrectionCount;
        State.AlignmentHoldCount = Result.NextState.AlignmentHoldCount;
        State.AlignmentConflictHoldCount = Result.NextState.AlignmentConflictHoldCount;
        State.AlignmentSnapCount = Result.NextState.AlignmentSnapCount;
        State.AlignmentReplanRequestCount = Result.NextState.AlignmentReplanRequestCount;
        State.AlignmentSuccessfulReplanCount = Result.NextState.AlignmentSuccessfulReplanCount;
        State.MaxAlignmentSpatialError = Result.NextState.MaxAlignmentSpatialError;
        State.MaxAlignmentTemporalError = Result.NextState.MaxAlignmentTemporalError;
        State.bAlignmentLost = Result.NextState.bAlignmentLost;
        State.ConsecutiveConflictHoldCount = Result.NextState.ConsecutiveConflictHoldCount;
        State.ConsecutiveSafetyGateHoldCount = Result.NextState.ConsecutiveSafetyGateHoldCount;
        State.DisplayToCell = Result.CommittedCell;
        State.LastAlignmentAction = FExecutionAlignmentPolicy::LexToString(Proposal.FinalAction);
        State.ActualCells.Add(Result.CommittedCell);
    }
}

FExecutionRuntimeStepPrepareResult FExecutionRuntimeSessionStepProcessor::PrepareStep(
    const FExecutionRuntimeStepPrepareRequest& Request)
{
    FExecutionRuntimeStepPrepareResult Result;
    Result.ConflictResolutionInput.MissionConfigsById = Request.MissionConfigsById;
    if (!Request.AgentStatesByMissionId)
    {
        return Result;
    }

    Request.AgentStatesByMissionId->GetKeys(Result.OrderedMissionIds);
    Result.OrderedMissionIds.Sort();
    Result.OrderedAgentSnapshots.Reserve(Result.OrderedMissionIds.Num());
    Result.ConflictResolutionInput.AgentStatesByMissionId.Reserve(
        Result.OrderedMissionIds.Num());

    for (const int32 MissionId : Result.OrderedMissionIds)
    {
        FExecutionAgentState* State = Request.AgentStatesByMissionId->Find(MissionId);
        if (!State)
        {
            continue;
        }

        FExecutionConflictResolutionAgentState ConflictAgentState;
        ConflictAgentState.bFinished = State->bFinished;
        ConflictAgentState.ConsecutiveConflictHoldCount =
            State->ConsecutiveConflictHoldCount;
        Result.ConflictResolutionInput.AgentStatesByMissionId.Add(
            MissionId,
            ConflictAgentState);

        if (State->PlannedCells.Num() <= 0)
        {
            continue;
        }

        const FIntVector ObservedCell = Request.ResolveObservedCell
            ? Request.ResolveObservedCell(*State)
            : State->LastObservedCell;
        State->LastObservedCell = ObservedCell;
        State->DisplayFromCell = ObservedCell;

        const bool bCanAdvance =
            (State->ExecutedPlanIndex + 1 < State->PlannedCells.Num());
        const bool bDelay = bCanAdvance && Request.ShouldDelay
            ? Request.ShouldDelay(*State, Request.TimeStep)
            : false;

        FExecutionAgentSnapshot AgentSnapshot;
        AgentSnapshot.MissionId = MissionId;
        AgentSnapshot.bFinished = State->bFinished;
        AgentSnapshot.bDelayRequested = bDelay;
        AgentSnapshot.ObservedCell = ObservedCell;
        AgentSnapshot.TimeStep = Request.TimeStep;
        AgentSnapshot.ExecutedPlanIndex = State->ExecutedPlanIndex;
        AgentSnapshot.ConsecutiveConflictHoldCount =
            State->ConsecutiveConflictHoldCount;
        AgentSnapshot.ConsecutiveSafetyGateHoldCount =
            State->ConsecutiveSafetyGateHoldCount;
        AgentSnapshot.PlannedCells = State->PlannedCells;
        Result.OrderedAgentSnapshots.Add(MoveTemp(AgentSnapshot));
    }

    return Result;
}

TMap<int32, FExecutionReplanProposalAgentState>
FExecutionRuntimeSessionStepProcessor::CaptureReplanProposalAgentStates(
    const TArray<int32>& OrderedMissionIds,
    const TMap<int32, FExecutionAgentState>& AgentStatesByMissionId)
{
    TMap<int32, FExecutionReplanProposalAgentState> AgentStates;
    AgentStates.Reserve(OrderedMissionIds.Num());

    for (const int32 MissionId : OrderedMissionIds)
    {
        const FExecutionAgentState* State = AgentStatesByMissionId.Find(MissionId);
        if (!State)
        {
            continue;
        }

        FExecutionReplanProposalAgentState AgentState;
        AgentState.ExecutedPlanIndex = State->ExecutedPlanIndex;
        AgentState.PlannedCellCount = State->PlannedCells.Num();
        AgentStates.Add(MissionId, AgentState);
    }

    return AgentStates;
}

FExecutionFinalSafetyGateInput
FExecutionRuntimeSessionStepProcessor::BuildFinalSafetyGateInput(
    const TArray<int32>& OrderedMissionIds,
    const TMap<int32, FDroneMissionConfig>& MissionConfigsById,
    const TMap<int32, FExecutionAgentState>& AgentStatesByMissionId)
{
    FExecutionFinalSafetyGateInput Input;
    Input.OrderedMissionIds = OrderedMissionIds;
    Input.MissionConfigsById = &MissionConfigsById;

    for (const int32 MissionId : OrderedMissionIds)
    {
        const FExecutionAgentState* State = AgentStatesByMissionId.Find(MissionId);
        if (!State)
        {
            continue;
        }

        FExecutionFinalSafetyGateAgentState AgentState;
        AgentState.ConsecutiveSafetyGateHoldCount =
            State->ConsecutiveSafetyGateHoldCount;
        Input.AgentStatesByMissionId.Add(MissionId, AgentState);
    }

    return Input;
}

FExecutionStepResultApplyResult
FExecutionRuntimeSessionStepProcessor::ApplyControllerResult(
    const TArray<int32>& OrderedMissionIds,
    TMap<int32, FExecutionAgentState>& AgentStatesByMissionId,
    const FExecutionControllerStepResult& ControllerResult)
{
    FExecutionStepResultApplyRequest ApplyRequest;
    ApplyRequest.OrderedMissionIds = OrderedMissionIds;
    ApplyRequest.AgentStatesByMissionId =
        CaptureExecutionStepResultApplyAgentStates(
            OrderedMissionIds,
            AgentStatesByMissionId);

    FExecutionStepResultApplyResult ApplyResult =
        FExecutionStepResultApplier::Apply(ApplyRequest, ControllerResult);

    for (const FExecutionStepAppliedAgent& AppliedAgent : ApplyResult.AppliedAgents)
    {
        FExecutionAgentState* State =
            AgentStatesByMissionId.Find(AppliedAgent.MissionId);
        const FExecutionStepProposal* Proposal =
            ControllerResult.StepProposals.Find(AppliedAgent.MissionId);
        if (!State || !Proposal)
        {
            continue;
        }

        CommitExecutionStateTransition(
            *State,
            *Proposal,
            AppliedAgent.TransitionResult);
    }

    return ApplyResult;
}
