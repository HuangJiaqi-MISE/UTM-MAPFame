#include "Execution/ExecutionRuntimeSessionReplanCommitter.h"

#include "Execution/ExecutionReplanPathIntegrator.h"

namespace
{
    struct FPreparedExecutionReplanCommit
    {
        int32 MissionId = INDEX_NONE;
        FExecutionAgentState* State = nullptr;
        FVector GoalWorld = FVector::ZeroVector;
        FExecutionReplanPathIntegrationResult IntegrationResult;
    };
}

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
        Result.FailureReason =
            TEXT("invalid execution replan commit request");
        return Result;
    }

    if (Request.AttemptResult->CandidateMissionIds.Num() <= 0)
    {
        Result.Status =
            EExecutionRuntimeReplanAttemptCommitStatus::EmptyCandidateSet;
        Result.FailureReason =
            TEXT("execution replan commit has no candidate missions");
        return Result;
    }

    TArray<FPreparedExecutionReplanCommit> PreparedCommits;
    PreparedCommits.Reserve(
        Request.AttemptResult->CandidateMissionIds.Num());

    for (const int32 MissionId : Request.AttemptResult->CandidateMissionIds)
    {
        FExecutionAgentState* State =
            Request.AgentStatesByMissionId->Find(MissionId);
        const FDroneMissionConfig* MissionConfig =
            Request.MissionConfigsById->Find(MissionId);
        const TArray<FIntVector>* ReplannedCellPath =
            Request.AttemptResult->ReplannedCellPathsByMission.Find(MissionId);
        if (!State)
        {
            Result.Status =
                EExecutionRuntimeReplanAttemptCommitStatus::MissingAgentState;
            Result.FailedMissionId = MissionId;
            Result.FailureReason = FString::Printf(
                TEXT("execution replan commit is missing Agent State for Mission %d"),
                MissionId);
            return Result;
        }

        if (!MissionConfig)
        {
            Result.Status =
                EExecutionRuntimeReplanAttemptCommitStatus::MissingMissionConfig;
            Result.FailedMissionId = MissionId;
            Result.FailureReason = FString::Printf(
                TEXT("execution replan commit is missing Mission Config for Mission %d"),
                MissionId);
            return Result;
        }

        if (!ReplannedCellPath)
        {
            Result.Status =
                EExecutionRuntimeReplanAttemptCommitStatus::MissingReplannedPath;
            Result.FailedMissionId = MissionId;
            Result.FailureReason = FString::Printf(
                TEXT("execution replan commit is missing replanned path for Mission %d"),
                MissionId);
            return Result;
        }

        FExecutionReplanPathIntegrationResult IntegrationResult =
            FExecutionReplanPathIntegrator::Integrate(
                State->ActualCells,
                State->LastObservedCell,
                *ReplannedCellPath);
        if (!IntegrationResult.bSuccess)
        {
            Result.Status =
                EExecutionRuntimeReplanAttemptCommitStatus::PathIntegrationFailed;
            Result.FailedMissionId = MissionId;
            Result.FailureReason = FString::Printf(
                TEXT("execution replan path integration failed for Mission %d"),
                MissionId);
            return Result;
        }

        FPreparedExecutionReplanCommit PreparedCommit;
        PreparedCommit.MissionId = MissionId;
        PreparedCommit.State = State;
        PreparedCommit.GoalWorld = MissionConfig->GoalWorld;
        PreparedCommit.IntegrationResult = MoveTemp(IntegrationResult);
        PreparedCommits.Add(MoveTemp(PreparedCommit));
    }

    for (const FPreparedExecutionReplanCommit& PreparedCommit :
         PreparedCommits)
    {
        FExecutionAgentState& State = *PreparedCommit.State;
        const FExecutionReplanPathIntegrationResult& IntegrationResult =
            PreparedCommit.IntegrationResult;

        State.PlannedCells = IntegrationResult.TimelineCells;
        State.ExecutedPlanIndex = IntegrationResult.ExecutedPlanIndex;
        State.GoalCell = IntegrationResult.GoalCell;
        State.GoalWorld = PreparedCommit.GoalWorld;
        State.ConsecutiveConflictHoldCount = 0;
        State.bAlignmentLost = false;

        Request.PlannedCellPathsByMissionId->Add(
            PreparedCommit.MissionId,
            IntegrationResult.TimelineCells);
        Result.ReplannedMissionIds.Add(PreparedCommit.MissionId);
    }

    Result.Status = EExecutionRuntimeReplanAttemptCommitStatus::Success;
    Result.bSuccess = true;
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
