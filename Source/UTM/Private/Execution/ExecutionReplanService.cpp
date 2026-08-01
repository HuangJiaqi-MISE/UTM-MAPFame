#include "Execution/ExecutionReplanService.h"

#include "Execution/ExecutionReplanAttemptRunner.h"
#include "Execution/ExecutionRuntimeSession.h"
#include "Execution/ExecutionRuntimeSessionBuilder.h"
#include "Execution/ExecutionRuntimeSessionReplanCommitter.h"
#include "Planning/GridMap3D.h"

FExecutionReplanServiceResult FExecutionReplanService::Run(
    const FExecutionReplanServiceRequest& Request,
    const FExecutionReplanServiceCallbacks& Callbacks)
{
    FExecutionReplanServiceResult Result;
    Result.MaxReplanCount = Request.RuntimeConfig.ReplanService.MaxReplanCount;

    if (!Request.GridMap ||
        !Request.Session ||
        !Request.PlannedCellPathsByMissionId)
    {
        Result.FailureReason = TEXT("invalid execution replan service context");
        return Result;
    }

    FExecutionRuntimeSession& Session = *Request.Session;
    Result.CurrentTotalReplanCount = Session.TotalReplanCount;

    if (Request.RequestedMissionIds.Num() <= 0)
    {
        Result.Status = EExecutionReplanServiceStatus::EmptyRequest;
        return Result;
    }

    if (Request.RuntimeConfig.ReplanMode ==
        EExecutionPolicyReplanMode::Disabled)
    {
        Result.Status = EExecutionReplanServiceStatus::Disabled;
        return Result;
    }

    if (Result.MaxReplanCount >= 0 &&
        Session.TotalReplanCount >= Result.MaxReplanCount)
    {
        Result.Status = EExecutionReplanServiceStatus::ReplanLimitReached;
        return Result;
    }

    FExecutionRuntimeSnapshotBuildRequest SnapshotRequest;
    SnapshotRequest.GridMap = Request.GridMap;
    SnapshotRequest.AgentStatesByMissionId = &Session.AgentStatesByMissionId;
    SnapshotRequest.TimeStep = Session.TimeStep;
    SnapshotRequest.TotalReplanCount = Session.TotalReplanCount;
    SnapshotRequest.ResolveObservedCell =
        [](const FExecutionAgentState& State)
        {
            return State.LastObservedCell;
        };
    const FExecutionSnapshot ExecutionSnapshot =
        FExecutionRuntimeSessionBuilder::BuildSnapshot(SnapshotRequest);

    FExecutionReplanCoordinatorRequest CoordinatorRequest;
    CoordinatorRequest.Snapshot = &ExecutionSnapshot;
    CoordinatorRequest.MissionConfigsById = &Session.MissionConfigsById;
    CoordinatorRequest.RequestedMissionIds = Request.RequestedMissionIds;
    CoordinatorRequest.bGlobalReplan = Request.bGlobalReplan;
    CoordinatorRequest.bCheckStaticUTMSafety =
        Request.RuntimeConfig.bCheckStaticUTMSafety;
    CoordinatorRequest.MaxExpansionRounds =
        Request.RuntimeConfig.ReplanService.LocalMaxExpansionRounds;
    CoordinatorRequest.BaseSpatialRadiusCells =
        Request.RuntimeConfig.ReplanService.LocalSpatialExpansionRadiusCells;
    CoordinatorRequest.BaseLookaheadSteps =
        Request.RuntimeConfig.ReplanService.LocalLookaheadSteps;
    CoordinatorRequest.ExecutionTimeStep = Session.TimeStep;
    CoordinatorRequest.CurrentTotalReplanCount = Session.TotalReplanCount;

    FExecutionReplanCoordinatorCallbacks CoordinatorCallbacks;
    CoordinatorCallbacks.RunAttempt =
        [&Request, &Session, &Callbacks](
            const FExecutionReplanAttemptInput& AttemptInput,
            FExecutionReplanAttemptResult& AttemptResult) -> bool
        {
            const FPlannerRuntimeConfig PlannerConfig = Request.PlannerConfig;

            FExecutionReplanAttemptContext AttemptContext;
            AttemptContext.BaseGrid = Request.GridMap;
            AttemptContext.MissionConfigsById = &Session.MissionConfigsById;
            AttemptContext.PlannerType = Request.PlannerType;
            AttemptContext.PlannerConfig = &PlannerConfig;

            const bool bAttemptSuccess = FExecutionReplanAttemptRunner::Run(
                AttemptContext,
                AttemptInput,
                AttemptResult);
            if (!bAttemptSuccess && Callbacks.OnAttemptFailure)
            {
                Callbacks.OnAttemptFailure(AttemptInput, AttemptResult);
            }

            return bAttemptSuccess;
        };
    CoordinatorCallbacks.ApplyAttemptResult =
        [&Request, &Session](
            const FExecutionReplanAttemptResult& AttemptResult,
            TSet<int32>& ReplannedMissionIds) -> bool
        {
            FExecutionRuntimeReplanAttemptCommitRequest CommitRequest;
            CommitRequest.AttemptResult = &AttemptResult;
            CommitRequest.MissionConfigsById = &Session.MissionConfigsById;
            CommitRequest.AgentStatesByMissionId =
                &Session.AgentStatesByMissionId;
            CommitRequest.PlannedCellPathsByMissionId =
                Request.PlannedCellPathsByMissionId;
            const FExecutionRuntimeReplanAttemptCommitResult CommitResult =
                FExecutionRuntimeSessionReplanCommitter::CommitAttemptResult(
                    CommitRequest);

            if (Request.PlannedWorldPathsByMissionId)
            {
                for (const int32 MissionId : AttemptResult.CandidateMissionIds)
                {
                    if (!CommitResult.ReplannedMissionIds.Contains(MissionId))
                    {
                        continue;
                    }

                    const TArray<FIntVector>* TimelineCells =
                        Request.PlannedCellPathsByMissionId->Find(MissionId);
                    if (!TimelineCells)
                    {
                        continue;
                    }

                    TArray<FVector> TimelineWorld;
                    TimelineWorld.Reserve(TimelineCells->Num());
                    for (const FIntVector& Cell : *TimelineCells)
                    {
                        TimelineWorld.Add(Request.GridMap->CellToWorld(Cell));
                    }

                    Request.PlannedWorldPathsByMissionId->Add(
                        MissionId,
                        TimelineWorld);
                }
            }

            ReplannedMissionIds = CommitResult.ReplannedMissionIds;
            return CommitResult.bSuccess;
        };
    CoordinatorCallbacks.OnEvent =
        [&Callbacks](const FExecutionReplanCoordinatorEvent& Event)
        {
            if (Callbacks.OnCoordinatorEvent)
            {
                Callbacks.OnCoordinatorEvent(Event);
            }
        };

    const FExecutionReplanCoordinatorResult CoordinatorResult =
        FExecutionReplanCoordinator::Run(
            CoordinatorRequest,
            CoordinatorCallbacks);

    FExecutionRuntimeSessionReplanCommitter::CommitCoordinatorResult(
        Request.bGlobalReplan,
        CoordinatorResult,
        Session.ReplanTimingStats,
        Session.TotalReplanCount);

    Result.ReplannedMissionIds = CoordinatorResult.ReplannedMissionIds;
    Result.bSuccess = CoordinatorResult.bSuccess;
    Result.Status = Result.bSuccess
        ? EExecutionReplanServiceStatus::Success
        : EExecutionReplanServiceStatus::CoordinatorFailed;
    return Result;
}
