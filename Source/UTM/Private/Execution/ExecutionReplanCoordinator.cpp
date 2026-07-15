#include "Execution/ExecutionReplanCoordinator.h"

#include "Execution/ExecutionReplanCandidateSelector.h"
#include "HAL/PlatformTime.h"
#include "Misc/ScopeExit.h"

namespace
{
    const FExecutionAgentSnapshot* FindAgentSnapshot(
        const FExecutionSnapshot& Snapshot,
        int32 MissionId)
    {
        for (const FExecutionAgentSnapshot& Agent : Snapshot.Agents)
        {
            if (Agent.MissionId == MissionId)
            {
                return &Agent;
            }
        }

        return nullptr;
    }

    void EmitCoordinatorEvent(
        const FExecutionReplanCoordinatorCallbacks& Callbacks,
        const FExecutionReplanCoordinatorEvent& Event)
    {
        if (Callbacks.OnEvent)
        {
            Callbacks.OnEvent(Event);
        }
    }
}

FExecutionReplanCoordinatorResult FExecutionReplanCoordinator::Run(
    const FExecutionReplanCoordinatorRequest& Request,
    const FExecutionReplanCoordinatorCallbacks& Callbacks)
{
    FExecutionReplanCoordinatorResult Result;

    if (!Request.Snapshot ||
        !Request.MissionConfigsById ||
        Request.RequestedMissionIds.Num() <= 0 ||
        !Callbacks.RunAttempt ||
        !Callbacks.ApplyAttemptResult)
    {
        return Result;
    }

    const FExecutionSnapshot& Snapshot = *Request.Snapshot;
    const TMap<int32, FDroneMissionConfig>& MissionConfigsById = *Request.MissionConfigsById;
    const int32 AttemptCount = Request.bGlobalReplan
        ? 1
        : FMath::Max(1, Request.MaxExpansionRounds);
    const int32 BaseSpatialRadiusCells = FMath::Max(0, Request.BaseSpatialRadiusCells);
    const int32 BaseLookaheadSteps = FMath::Max(0, Request.BaseLookaheadSteps);

    auto TryPlanCandidateSet =
        [&](TSet<int32> CandidateMissionIdSet,
            int32 AttemptIndex,
            int32 SpatialRadiusCells,
            int32 LookaheadSteps) -> bool
        {
            const double AttemptStartSeconds = FPlatformTime::Seconds();
            ON_SCOPE_EXIT
            {
                const double AttemptTimeMs = (FPlatformTime::Seconds() - AttemptStartSeconds) * 1000.0;
                Result.TimedAttemptCount++;
                Result.TotalAttemptTimeMs += AttemptTimeMs;
                Result.MaxAttemptTimeMs = FMath::Max(Result.MaxAttemptTimeMs, AttemptTimeMs);
            };

            const int32 MaxPostCheckTargetedRetries = Request.bGlobalReplan ? 0 : 1;
            int32 PostCheckTargetedRetryCount = 0;
            TSet<int32> ForcedAnchorMissionIdSet;

            while (true)
            {
                FExecutionReplanAttemptInput AttemptInput;
                AttemptInput.Snapshot = Request.Snapshot;
                AttemptInput.CandidateMissionIdSet = CandidateMissionIdSet;
                AttemptInput.ForcedAnchorMissionIdSet = ForcedAnchorMissionIdSet;
                AttemptInput.Spec.bGlobalReplan = Request.bGlobalReplan;
                AttemptInput.Spec.AttemptIndex = AttemptIndex;
                AttemptInput.Spec.AttemptCount = AttemptCount;
                AttemptInput.Spec.SpatialRadiusCells = SpatialRadiusCells;
                AttemptInput.Spec.LookaheadSteps = LookaheadSteps;
                AttemptInput.Spec.bCheckStaticUTMSafety = Request.bCheckStaticUTMSafety;

                FExecutionReplanAttemptResult AttemptResult;
                const bool bAttemptSuccess = Callbacks.RunAttempt(AttemptInput, AttemptResult);

                if (!bAttemptSuccess)
                {
                    if (AttemptResult.Status != EExecutionReplanAttemptStatus::PostCheckFailed)
                    {
                        return false;
                    }

                    FExecutionReplanCoordinatorEvent PostCheckEvent;
                    PostCheckEvent.Type = EExecutionReplanCoordinatorEventType::PostCheckFailed;
                    PostCheckEvent.bGlobalReplan = Request.bGlobalReplan;
                    PostCheckEvent.AttemptIndex = AttemptIndex;
                    PostCheckEvent.AttemptCount = AttemptCount;
                    PostCheckEvent.SpatialRadiusCells = SpatialRadiusCells;
                    PostCheckEvent.LookaheadSteps = LookaheadSteps;
                    PostCheckEvent.CandidateMissionCount = AttemptResult.CandidateMissionIds.Num();
                    PostCheckEvent.AnchorMissionCount = AttemptResult.AnchorMissionIds.Num();
                    PostCheckEvent.StaticAnchorBlockedCellCount = AttemptResult.StaticAnchorBlockedCellCount;
                    PostCheckEvent.Conflict = AttemptResult.PostCheckConflict;
                    PostCheckEvent.ConflictOffset = AttemptResult.PostCheckConflictOffset;
                    EmitCoordinatorEvent(Callbacks, PostCheckEvent);

                    const int32 PreviousMovableCount = AttemptResult.CandidateMissionIds.Num();
                    const int32 PreviousForcedAnchorCount = ForcedAnchorMissionIdSet.Num();
                    bool bTargetedRetryExpanded = false;

                    auto AddPostCheckRetryMission = [&](int32 MissionId)
                        {
                            const FExecutionAgentSnapshot* Agent = FindAgentSnapshot(Snapshot, MissionId);
                            if (Agent && !Agent->bFinished)
                            {
                                if (!CandidateMissionIdSet.Contains(MissionId))
                                {
                                    CandidateMissionIdSet.Add(MissionId);
                                    bTargetedRetryExpanded = true;
                                }
                                return;
                            }

                            if (Request.bCheckStaticUTMSafety &&
                                Agent &&
                                Agent->bFinished &&
                                !ForcedAnchorMissionIdSet.Contains(MissionId))
                            {
                                ForcedAnchorMissionIdSet.Add(MissionId);
                                bTargetedRetryExpanded = true;
                            }
                        };

                    if (PostCheckTargetedRetryCount < MaxPostCheckTargetedRetries)
                    {
                        AddPostCheckRetryMission(AttemptResult.PostCheckConflict.AgentA);
                        AddPostCheckRetryMission(AttemptResult.PostCheckConflict.AgentB);

                        if (bTargetedRetryExpanded)
                        {
                            FExecutionReplanCandidateSelectionInput TargetedSelectionInput;
                            TargetedSelectionInput.Snapshot = Snapshot;
                            TargetedSelectionInput.MissionConfigsById = MissionConfigsById;
                            TargetedSelectionInput.RequestedMissionIds = CandidateMissionIdSet;
                            TargetedSelectionInput.bGlobalReplan = false;
                            TargetedSelectionInput.bCheckStaticUTMSafety = Request.bCheckStaticUTMSafety;
                            TargetedSelectionInput.SpatialRadiusCells = SpatialRadiusCells;
                            TargetedSelectionInput.LookaheadSteps = LookaheadSteps;

                            CandidateMissionIdSet =
                                FExecutionReplanCandidateSelector::Select(TargetedSelectionInput).CandidateMissionIds;
                        }

                        if (bTargetedRetryExpanded)
                        {
                            PostCheckTargetedRetryCount++;

                            FExecutionReplanCoordinatorEvent RetryEvent;
                            RetryEvent.Type = EExecutionReplanCoordinatorEventType::TargetedRetryExpanded;
                            RetryEvent.bGlobalReplan = Request.bGlobalReplan;
                            RetryEvent.AttemptIndex = AttemptIndex;
                            RetryEvent.AttemptCount = AttemptCount;
                            RetryEvent.SpatialRadiusCells = SpatialRadiusCells;
                            RetryEvent.LookaheadSteps = LookaheadSteps;
                            RetryEvent.TargetedRetryIndex = PostCheckTargetedRetryCount;
                            RetryEvent.MaxTargetedRetryCount = MaxPostCheckTargetedRetries;
                            RetryEvent.PreviousCandidateMissionCount = PreviousMovableCount;
                            RetryEvent.CandidateMissionCount = CandidateMissionIdSet.Num();
                            RetryEvent.PreviousForcedAnchorMissionCount = PreviousForcedAnchorCount;
                            RetryEvent.ForcedAnchorMissionCount = ForcedAnchorMissionIdSet.Num();
                            RetryEvent.Conflict = AttemptResult.PostCheckConflict;
                            EmitCoordinatorEvent(Callbacks, RetryEvent);
                            continue;
                        }
                    }

                    return false;
                }

                TSet<int32> ReplannedMissionIds;
                if (!Callbacks.ApplyAttemptResult(AttemptResult, ReplannedMissionIds))
                {
                    return false;
                }

                Result.ReplannedMissionIds = ReplannedMissionIds;
                Result.AppliedReplanCount = 1;

                FExecutionReplanCoordinatorEvent SuccessEvent;
                SuccessEvent.Type = EExecutionReplanCoordinatorEventType::AttemptSucceeded;
                SuccessEvent.bGlobalReplan = Request.bGlobalReplan;
                SuccessEvent.AttemptIndex = AttemptIndex;
                SuccessEvent.AttemptCount = AttemptCount;
                SuccessEvent.SpatialRadiusCells = SpatialRadiusCells;
                SuccessEvent.LookaheadSteps = LookaheadSteps;
                SuccessEvent.AnchorMissionCount = AttemptResult.AnchorMissionIds.Num();
                SuccessEvent.StaticAnchorBlockedCellCount = AttemptResult.StaticAnchorBlockedCellCount;
                SuccessEvent.ReplannedMissionCount = ReplannedMissionIds.Num();
                SuccessEvent.ExecutionTimeStep = Request.ExecutionTimeStep;
                SuccessEvent.TotalReplanCount = Request.CurrentTotalReplanCount + 1;
                EmitCoordinatorEvent(Callbacks, SuccessEvent);

                return ReplannedMissionIds.Num() > 0;
            }
        };

    for (int32 AttemptIndex = 0; AttemptIndex < AttemptCount; ++AttemptIndex)
    {
        const int32 SpatialRadiusCells = Request.bGlobalReplan
            ? BaseSpatialRadiusCells
            : BaseSpatialRadiusCells * (AttemptIndex + 1);
        const int32 LookaheadSteps = Request.bGlobalReplan
            ? BaseLookaheadSteps
            : BaseLookaheadSteps * (AttemptIndex + 1);

        FExecutionReplanCandidateSelectionInput CandidateSelectionInput;
        CandidateSelectionInput.Snapshot = Snapshot;
        CandidateSelectionInput.MissionConfigsById = MissionConfigsById;
        CandidateSelectionInput.RequestedMissionIds = Request.RequestedMissionIds;
        CandidateSelectionInput.bGlobalReplan = Request.bGlobalReplan;
        CandidateSelectionInput.bCheckStaticUTMSafety = Request.bCheckStaticUTMSafety;
        CandidateSelectionInput.SpatialRadiusCells = SpatialRadiusCells;
        CandidateSelectionInput.LookaheadSteps = LookaheadSteps;

        const FExecutionReplanCandidateSelectionResult CandidateSelectionResult =
            FExecutionReplanCandidateSelector::Select(CandidateSelectionInput);
        TSet<int32> CandidateMissionIdSet = CandidateSelectionResult.CandidateMissionIds;

        if (!Request.bGlobalReplan &&
            CandidateMissionIdSet.Num() > CandidateSelectionResult.ActiveRequestedMissionCount)
        {
            FExecutionReplanCoordinatorEvent ExpandedEvent;
            ExpandedEvent.Type = EExecutionReplanCoordinatorEventType::CandidateSetExpanded;
            ExpandedEvent.bGlobalReplan = false;
            ExpandedEvent.AttemptIndex = AttemptIndex;
            ExpandedEvent.AttemptCount = AttemptCount;
            ExpandedEvent.SpatialRadiusCells = SpatialRadiusCells;
            ExpandedEvent.LookaheadSteps = LookaheadSteps;
            ExpandedEvent.ActiveRequestedMissionCount = CandidateSelectionResult.ActiveRequestedMissionCount;
            ExpandedEvent.CandidateMissionCount = CandidateMissionIdSet.Num();
            EmitCoordinatorEvent(Callbacks, ExpandedEvent);
        }

        if (TryPlanCandidateSet(
            MoveTemp(CandidateMissionIdSet),
            AttemptIndex,
            SpatialRadiusCells,
            LookaheadSteps))
        {
            Result.bSuccess = true;
            return Result;
        }

        Result.ReplannedMissionIds.Reset();
        Result.AppliedReplanCount = 0;
    }

    return Result;
}
