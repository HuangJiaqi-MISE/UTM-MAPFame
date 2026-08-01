#include "Execution/ExecutionRuntimeCoordinator.h"

#include "Execution/ExecutionControllerRegistry.h"
#include "Execution/ExecutionDelayPolicy.h"
#include "Execution/ExecutionObservedConflictDetector.h"
#include "Execution/ExecutionReplanService.h"
#include "Execution/ExecutionRuntimeSessionStepProcessor.h"

FExecutionRuntimeCoordinator::FExecutionRuntimeCoordinator()
    : ReplanService(MakeUnique<FDefaultExecutionReplanService>())
{
}

FExecutionRuntimeCoordinator::~FExecutionRuntimeCoordinator()
{
    ResetController();
}

bool FExecutionRuntimeCoordinator::InitializeController(
    const FExecutionRuntimeCoordinatorInitializeRequest& Request,
    FString& OutFailureReason)
{
    ResetController();
    if (ReplanService)
    {
        ReplanService->Reset();
    }

    if (!Request.Session)
    {
        OutFailureReason =
            TEXT("Execution controller initialization has no session");
        return false;
    }

    if (!Request.GridMap)
    {
        OutFailureReason =
            TEXT("Execution controller initialization has no grid map");
        return false;
    }

    TUniquePtr<IExecutionController> Controller =
        FExecutionControllerRegistry::CreateController(Request.ControllerType);
    if (!Controller)
    {
        OutFailureReason = FString::Printf(
            TEXT("Failed to create execution controller: %s"),
            *FExecutionControllerRegistry::GetControllerTypeName(
                Request.ControllerType));
        return false;
    }

    FExecutionControllerInitializeRequest ControllerInitializeRequest;
    Request.Session->AgentStatesByMissionId.GetKeys(
        ControllerInitializeRequest.OrderedMissionIds);
    ControllerInitializeRequest.OrderedMissionIds.Sort();
    ControllerInitializeRequest.GridMap = Request.GridMap;
    ControllerInitializeRequest.RuntimeConfig = Request.RuntimeConfig;

    OutFailureReason.Reset();
    if (!Controller->Initialize(
            ControllerInitializeRequest,
            OutFailureReason))
    {
        if (OutFailureReason.IsEmpty())
        {
            OutFailureReason = FString::Printf(
                TEXT("Execution controller initialization failed: %s"),
                *Controller->GetName());
        }

        Controller->Reset();
        return false;
    }

    ActiveController = MoveTemp(Controller);
    return true;
}

bool FExecutionRuntimeCoordinator::SetReplanService(
    TUniquePtr<IExecutionReplanService>&& InReplanService)
{
    if (ActiveController || !InReplanService)
    {
        return false;
    }

    if (ReplanService)
    {
        ReplanService->Reset();
    }

    ReplanService = MoveTemp(InReplanService);
    return true;
}

void FExecutionRuntimeCoordinator::ResetController()
{
    if (ActiveController)
    {
        ActiveController->Reset();
    }

    ActiveController.Reset();
}

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

    if (!ActiveController)
    {
        Result.FailureReason =
            TEXT("Execution runtime coordinator has no active controller");
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
    StepPrepareRequest.ShouldDelay =
        [&Request, &Session](
            const FExecutionAgentState& AgentState,
            int32 TimeStep)
        {
            FExecutionDelayPolicyInput DelayInput;
            DelayInput.AgentState = &AgentState;
            DelayInput.Settings = &Request.RuntimeConfig.Delay;
            DelayInput.Random = &Session.Random;
            DelayInput.TimeStep = TimeStep;
            return FExecutionDelayPolicy::ShouldDelay(DelayInput);
        };
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
    ControllerCallbacks.RunReplan =
        [this, &Request, &Callbacks](
            const TSet<int32>& RequestedMissionIds,
            bool bGlobalReplan,
            TSet<int32>& ReplannedMissionIds) -> bool
        {
            FExecutionReplanServiceRequest ServiceRequest;
            ServiceRequest.GridMap = Request.GridMap;
            ServiceRequest.Session = Request.Session;
            ServiceRequest.PlannedCellPathsByMissionId =
                Request.ReplanContext.PlannedCellPathsByMissionId;
            ServiceRequest.PlannedWorldPathsByMissionId =
                Request.ReplanContext.PlannedWorldPathsByMissionId;
            ServiceRequest.PlannerType = Request.ReplanContext.PlannerType;
            if (Callbacks.BuildPlannerRuntimeConfig)
            {
                ServiceRequest.PlannerConfig =
                    Callbacks.BuildPlannerRuntimeConfig();
            }
            ServiceRequest.RuntimeConfig = Request.RuntimeConfig;
            ServiceRequest.RequestedMissionIds = RequestedMissionIds;
            ServiceRequest.bGlobalReplan = bGlobalReplan;

            FExecutionReplanServiceCallbacks ServiceCallbacks;
            ServiceCallbacks.OnAttemptFailure =
                Callbacks.OnReplanAttemptFailure;
            ServiceCallbacks.OnCoordinatorEvent =
                Callbacks.OnReplanCoordinatorEvent;

            FExecutionReplanServiceResult ServiceResult;
            if (ReplanService)
            {
                ServiceResult = ReplanService->Run(
                    ServiceRequest,
                    ServiceCallbacks);
            }
            else
            {
                ServiceResult.FailureReason =
                    TEXT("execution runtime coordinator has no replan service");
            }
            if (Callbacks.OnReplanServiceResult)
            {
                Callbacks.OnReplanServiceResult(ServiceResult);
            }

            ReplannedMissionIds = ServiceResult.ReplannedMissionIds;
            return ServiceResult.bSuccess;
        };
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

    Result.ControllerResult = ActiveController->RunStep(
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
