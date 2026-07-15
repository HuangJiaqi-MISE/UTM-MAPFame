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

    void BuildInitialStepProposals(
        const FExecutionStepPipelineRequest& Request,
        FExecutionStepPipelineResult& InOutResult)
    {
        if (!Request.GridMap)
        {
            return;
        }

        const FExecutionAlignmentPolicy AlignmentPolicy(Request.AlignmentSettings);
        for (const FExecutionAgentSnapshot& AgentSnapshot : Request.OrderedAgentSnapshots)
        {
            if (AgentSnapshot.PlannedCells.Num() <= 0)
            {
                continue;
            }

            const FExecutionStepDecision AlignmentDecision =
                AlignmentPolicy.Decide(*Request.GridMap, AgentSnapshot);

            FExecutionStepProposalBuildInput ProposalBuildInput;
            ProposalBuildInput.MissionId = AgentSnapshot.MissionId;
            ProposalBuildInput.ObservedCell = AgentSnapshot.ObservedCell;
            ProposalBuildInput.bDelayRequested = AgentSnapshot.bDelayRequested;
            ProposalBuildInput.CurrentPlanIndex = AgentSnapshot.ExecutedPlanIndex;
            ProposalBuildInput.PlannedCellCount = AgentSnapshot.PlannedCells.Num();

            const FExecutionStepProposalBuildResult ProposalBuildResult =
                FExecutionStepProposalBuilder::Build(
                    ProposalBuildInput,
                    AlignmentDecision);
            if (!ProposalBuildResult.bSuccess)
            {
                continue;
            }

            if (ProposalBuildResult.bRequestsReplan)
            {
                InOutResult.RequestedReplanMissionIds.Add(AgentSnapshot.MissionId);
            }

            InOutResult.StepProposals.Add(
                AgentSnapshot.MissionId,
                ProposalBuildResult.Proposal);
        }
    }
}

FExecutionStepPipelineResult FExecutionStepPipeline::Run(
    const FExecutionStepPipelineRequest& Request,
    const FExecutionStepPipelineCallbacks& Callbacks)
{
    FExecutionStepPipelineResult Result;
    BuildInitialStepProposals(Request, Result);

    if (Request.ConflictResolutionInput)
    {
        const FExecutionConflictResolutionResult ConflictResolutionResult =
            FExecutionConflictResolutionPolicy::Resolve(
                *Request.ConflictResolutionInput,
                Result.StepProposals);
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
            Result.StepProposals);
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
            Result.StepProposals);
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
