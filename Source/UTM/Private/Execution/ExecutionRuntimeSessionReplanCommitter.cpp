#include "Execution/ExecutionRuntimeSessionReplanCommitter.h"

#include "Execution/ExecutionReplanPathIntegrator.h"

FExecutionRuntimeReplanAttemptCommitResult
FExecutionRuntimeSessionReplanCommitter::CommitAttemptResult(
    const FExecutionRuntimeReplanAttemptCommitRequest& Request)
{
    FExecutionRuntimeReplanAttemptCommitResult Result;
    if (!Request.AttemptResult ||
        !Request.MissionConfigsById ||
        !Request.AgentStatesByMissionId ||
        !Request.PlannedCellPathsByMissionId)
    {
        return Result;
    }

    for (const int32 MissionId : Request.AttemptResult->CandidateMissionIds)
    {
        FExecutionAgentState* State =
            Request.AgentStatesByMissionId->Find(MissionId);
        const FDroneMissionConfig* MissionConfig =
            Request.MissionConfigsById->Find(MissionId);
        const TArray<FIntVector>* ReplannedCellPath =
            Request.AttemptResult->ReplannedCellPathsByMission.Find(MissionId);
        if (!State || !MissionConfig || !ReplannedCellPath)
        {
            return Result;
        }

        const FExecutionReplanPathIntegrationResult IntegrationResult =
            FExecutionReplanPathIntegrator::Integrate(
                State->ActualCells,
                State->LastObservedCell,
                *ReplannedCellPath);
        if (!IntegrationResult.bSuccess)
        {
            return Result;
        }

        State->PlannedCells = IntegrationResult.TimelineCells;
        State->ExecutedPlanIndex = IntegrationResult.ExecutedPlanIndex;
        State->GoalCell = IntegrationResult.GoalCell;
        State->GoalWorld = MissionConfig->GoalWorld;
        State->ConsecutiveConflictHoldCount = 0;
        State->bAlignmentLost = false;

        Request.PlannedCellPathsByMissionId->Add(
            MissionId,
            IntegrationResult.TimelineCells);
        Result.ReplannedMissionIds.Add(MissionId);
    }

    Result.bSuccess = (Result.ReplannedMissionIds.Num() > 0);
    return Result;
}

void FExecutionRuntimeSessionReplanCommitter::CommitCoordinatorResult(
    bool bGlobalReplan,
    const FExecutionReplanCoordinatorResult& CoordinatorResult,
    FExecutionReplanTimingStats& InOutTimingStats,
    int32& InOutTotalReplanCount)
{
    if (bGlobalReplan)
    {
        InOutTimingStats.GlobalAttemptCount += CoordinatorResult.TimedAttemptCount;
        InOutTimingStats.GlobalTotalTimeMs += CoordinatorResult.TotalAttemptTimeMs;
        InOutTimingStats.GlobalMaxTimeMs = FMath::Max(
            InOutTimingStats.GlobalMaxTimeMs,
            CoordinatorResult.MaxAttemptTimeMs);
    }
    else
    {
        InOutTimingStats.LocalAttemptCount += CoordinatorResult.TimedAttemptCount;
        InOutTimingStats.LocalTotalTimeMs += CoordinatorResult.TotalAttemptTimeMs;
        InOutTimingStats.LocalMaxTimeMs = FMath::Max(
            InOutTimingStats.LocalMaxTimeMs,
            CoordinatorResult.MaxAttemptTimeMs);
    }

    InOutTotalReplanCount += CoordinatorResult.AppliedReplanCount;
}
