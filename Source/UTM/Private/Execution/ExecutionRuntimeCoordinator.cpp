#include "Execution/ExecutionRuntimeCoordinator.h"

#include "Execution/ExecutionControllerRegistry.h"
#include "Execution/ExecutionObservedConflictDetector.h"
#include "Execution/ExecutionRuntimeSessionStepProcessor.h"

FExecutionRuntimeCoordinatorResult FExecutionRuntimeCoordinator::Advance(
    const FExecutionRuntimeCoordinatorRequest& Request,
    const FExecutionRuntimeCoordinatorCallbacks& Callbacks)
{
    FExecutionRuntimeCoordinatorResult Result;
    if (!Request.Session)
    {
        Result.FailureReason =
            TEXT("Execution runtime coordinator request has no session");
        return Result;
    }

    if (!Request.GridMap)
    {
        Result.FailureReason =
            TEXT("Execution runtime coordinator request has no grid map");
        return Result;
    }

    FExecutionRuntimeSession& Session = *Request.Session;
    Session.TimeStep++;
    Result.TimeStep = Session.TimeStep;

    FExecutionRuntimeStepPrepareRequest StepPrepareRequest;
    StepPrepareRequest.AgentStatesByMissionId =
        &Session.AgentStatesByMissionId;
    StepPrepareRequest.MissionConfigsById = &Session.MissionConfigsById;
    StepPrepareRequest.TimeStep = Session.TimeStep;
    StepPrepareRequest.ResolveObservedCell = Callbacks.ResolveObservedCell;
    StepPrepareRequest.ShouldDelay = Callbacks.ShouldDelay;
    FExecutionRuntimeStepPrepareResult StepPrepareResult =
        FExecutionRuntimeSessionStepProcessor::PrepareStep(StepPrepareRequest);
    const TArray<int32>& MissionIds = StepPrepareResult.OrderedMissionIds;

    FExecutionControllerStepRequest ControllerRequest;
    ControllerRequest.OrderedMissionIds = MissionIds;
    ControllerRequest.OrderedAgentSnapshots =
        MoveTemp(StepPrepareResult.OrderedAgentSnapshots);
    ControllerRequest.GridMap = Request.GridMap;
    ControllerRequest.RuntimeConfig = Request.RuntimeConfig;
    ControllerRequest.ConflictResolutionInput =
        &StepPrepareResult.ConflictResolutionInput;

    FExecutionControllerStepCallbacks ControllerCallbacks;
    ControllerCallbacks.RunReplan = Callbacks.RunReplan;
    ControllerCallbacks.CaptureReplanProposalAgentStates =
        [&Session, &MissionIds]()
        {
            return FExecutionRuntimeSessionStepProcessor::
                CaptureReplanProposalAgentStates(
                    MissionIds,
                    Session.AgentStatesByMissionId);
        };
    ControllerCallbacks.BuildFinalSafetyGateInput =
        [&Session, &MissionIds]()
        {
            return FExecutionRuntimeSessionStepProcessor::
                BuildFinalSafetyGateInput(
                    MissionIds,
                    Session.MissionConfigsById,
                    Session.AgentStatesByMissionId);
        };
    ControllerCallbacks.OnConflictResolutionEvent =
        Callbacks.OnConflictResolutionEvent;
    ControllerCallbacks.OnFinalSafetyGateEvent =
        Callbacks.OnFinalSafetyGateEvent;

    Result.ControllerResult = FExecutionControllerRegistry::RunStep(
        Request.ControllerType,
        ControllerRequest,
        ControllerCallbacks);
    Result.bStepProcessed = true;

    if (Result.ControllerResult.bStopExecution)
    {
        return Result;
    }

    Result.ApplyResult =
        FExecutionRuntimeSessionStepProcessor::ApplyControllerResult(
            MissionIds,
            Session.AgentStatesByMissionId,
            Result.ControllerResult);
    Result.ObservedConflicts =
        RecordObservedConflicts(Session, Session.TimeStep);
    return Result;
}

TArray<FExecutionConflict>
FExecutionRuntimeCoordinator::RecordObservedConflicts(
    FExecutionRuntimeSession& Session,
    int32 TimeStep) const
{
    FExecutionObservedConflictDetectionRequest DetectionRequest;
    DetectionRequest.AgentStatesByMissionId =
        &Session.AgentStatesByMissionId;
    DetectionRequest.TimeStep = TimeStep;
    FExecutionObservedConflictDetectionResult DetectionResult =
        FExecutionObservedConflictDetector::Detect(DetectionRequest);

    Session.Conflicts.Append(DetectionResult.Conflicts);
    return MoveTemp(DetectionResult.Conflicts);
}
