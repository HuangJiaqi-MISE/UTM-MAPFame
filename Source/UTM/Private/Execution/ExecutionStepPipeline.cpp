#include "Execution/ExecutionStepPipeline.h"

namespace
{
    void AddMissionIds(
        const TSet<int32>& MissionIds,
        TSet<int32>& InOutMissionIds)
    {
        for (const int32 MissionId : MissionIds)
        {
            InOutMissionIds.Add(MissionId);
        }
    }

    void ApplyReplanProposalSync(
        const FExecutionStepPipelineRequest& Request,
        const FExecutionStepPipelineCallbacks& Callbacks,
        const TSet<int32>& TargetMissionIds,
        const TSet<int32>& ReplannedMissionIds,
        const TCHAR* ReplannedReason,
        const TCHAR* SynchronizedReason,
        TMap<int32, FExecutionStepProposal>& InOutStepProposals)
    {
        if (!Callbacks.CaptureReplanProposalAgentStates)
        {
            return;
        }

        FExecutionReplanProposalSyncRequest SyncRequest;
        SyncRequest.OrderedMissionIds = Request.OrderedMissionIds;
        SyncRequest.TargetMissionIds = TargetMissionIds;
        SyncRequest.ReplannedMissionIds = ReplannedMissionIds;
        SyncRequest.AgentStatesByMissionId =
            Callbacks.CaptureReplanProposalAgentStates();
        SyncRequest.ReplannedReason = ReplannedReason;
        SyncRequest.SynchronizedReason = SynchronizedReason;
        FExecutionReplanProposalSynchronizer::Apply(
            SyncRequest,
            InOutStepProposals);
    }
}

FExecutionStepPipelineResult FExecutionStepPipeline::Run(
    const FExecutionStepPipelineRequest& Request,
    const FExecutionStepPipelineCallbacks& Callbacks,
    TMap<int32, FExecutionStepProposal>& InOutStepProposals)
{
    FExecutionStepPipelineResult Result;
    Result.RequestedReplanMissionIds = Request.InitialRequestedReplanMissionIds;

    if (Request.ConflictResolutionInput)
    {
        const FExecutionConflictResolutionResult ConflictResolutionResult =
            FExecutionConflictResolutionPolicy::Resolve(
                *Request.ConflictResolutionInput,
                InOutStepProposals);
        AddMissionIds(
            ConflictResolutionResult.ReplanMissionIds,
            Result.RequestedReplanMissionIds);

        if (Callbacks.OnConflictResolutionEvent)
        {
            for (const FExecutionConflictResolutionEvent& Event :
                ConflictResolutionResult.Events)
            {
                Callbacks.OnConflictResolutionEvent(Event);
            }
        }
    }

    FExecutionStepReplanCoordinatorRequest StepReplanRequest;
    StepReplanRequest.RequestedMissionIds = Result.RequestedReplanMissionIds;
    StepReplanRequest.ReplanMode = Request.ReplanMode;

    FExecutionStepReplanCoordinatorCallbacks StepReplanCallbacks;
    StepReplanCallbacks.RunReplan = Callbacks.RunReplan;
    StepReplanCallbacks.ApplyReplanResult =
        [&Request, &Callbacks](
            const TSet<int32>& ReplannedMissionIds,
            TMap<int32, FExecutionStepProposal>& InOutProposals)
        {
            TSet<int32> TargetMissionIds;
            for (const int32 MissionId : Request.OrderedMissionIds)
            {
                TargetMissionIds.Add(MissionId);
            }

            ApplyReplanProposalSync(
                Request,
                Callbacks,
                TargetMissionIds,
                ReplannedMissionIds,
                TEXT("hold while applying replanned trajectory"),
                TEXT("hold to synchronize with replanned agents"),
                InOutProposals);
        };

    const FExecutionStepReplanCoordinatorResult StepReplanResult =
        FExecutionStepReplanCoordinator::Run(
            StepReplanRequest,
            StepReplanCallbacks,
            InOutStepProposals);
    Result.bReplanSucceeded = StepReplanResult.bSuccess;
    Result.SuccessfulReplanMissionIds = StepReplanResult.ReplannedMissionIds;

    if (!Callbacks.BuildFinalSafetyGateInput)
    {
        return Result;
    }

    FExecutionFinalSafetyGateInput SafetyGateInput =
        Callbacks.BuildFinalSafetyGateInput();
    FExecutionFinalSafetyGateCoordinatorRequest SafetyGateRequest;
    SafetyGateRequest.SafetyGateInput = &SafetyGateInput;
    SafetyGateRequest.ReplanMode = Request.ReplanMode;

    FExecutionFinalSafetyGateCoordinatorCallbacks SafetyGateCallbacks;
    SafetyGateCallbacks.RunReplan = Callbacks.RunReplan;
    SafetyGateCallbacks.ApplyReplanResult =
        [&Request, &Callbacks](
            const TSet<int32>& HoldMissionIds,
            const TSet<int32>& ReplannedMissionIds,
            TMap<int32, FExecutionStepProposal>& InOutProposals)
        {
            TSet<int32> TargetMissionIds = HoldMissionIds;
            AddMissionIds(ReplannedMissionIds, TargetMissionIds);
            ApplyReplanProposalSync(
                Request,
                Callbacks,
                TargetMissionIds,
                ReplannedMissionIds,
                TEXT("hold while applying safety-gate replanned trajectory"),
                TEXT("hold to synchronize with safety-gate replanned agents"),
                InOutProposals);
        };
    SafetyGateCallbacks.OnEvent = Callbacks.OnFinalSafetyGateEvent;

    const FExecutionFinalSafetyGateCoordinatorResult SafetyGateResult =
        FExecutionFinalSafetyGateCoordinator::Run(
            SafetyGateRequest,
            SafetyGateCallbacks,
            InOutStepProposals);
    AddMissionIds(
        SafetyGateResult.RequestedReplanMissionIds,
        Result.RequestedReplanMissionIds);
    if (SafetyGateResult.bReplanSucceeded)
    {
        Result.bReplanSucceeded = true;
        AddMissionIds(
            SafetyGateResult.SuccessfulReplanMissionIds,
            Result.SuccessfulReplanMissionIds);
    }
    Result.bStopExecution = SafetyGateResult.bStopExecution;
    return Result;
}
