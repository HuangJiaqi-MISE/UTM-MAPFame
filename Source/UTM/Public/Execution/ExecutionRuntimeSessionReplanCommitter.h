#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionReplanAttemptTypes.h"
#include "Execution/ExecutionReplanCoordinator.h"
#include "Execution/ExecutionRuntimeStateTypes.h"
#include "Planning/DroneMissionTypes.h"

struct FExecutionRuntimeReplanAttemptCommitRequest
{
    const FExecutionReplanAttemptResult* AttemptResult = nullptr;
    const TMap<int32, FDroneMissionConfig>* MissionConfigsById = nullptr;
    TMap<int32, FExecutionAgentState>* AgentStatesByMissionId = nullptr;
    TMap<int32, TArray<FIntVector>>* PlannedCellPathsByMissionId = nullptr;
};

enum class EExecutionRuntimeReplanAttemptCommitStatus : uint8
{
    Success,
    InvalidRequest,
    EmptyCandidateSet,
    MissingAgentState,
    MissingMissionConfig,
    MissingReplannedPath,
    PathIntegrationFailed
};

struct FExecutionRuntimeReplanAttemptCommitResult
{
    EExecutionRuntimeReplanAttemptCommitStatus Status =
        EExecutionRuntimeReplanAttemptCommitStatus::InvalidRequest;
    bool bSuccess = false;
    int32 FailedMissionId = INDEX_NONE;
    FString FailureReason;
    TSet<int32> ReplannedMissionIds;
};

class FExecutionRuntimeSessionReplanCommitter
{
public:
    static FExecutionRuntimeReplanAttemptCommitResult CommitAttemptResult(
        const FExecutionRuntimeReplanAttemptCommitRequest& Request);

    static void CommitCoordinatorResult(
        bool bGlobalReplan,
        const FExecutionReplanCoordinatorResult& CoordinatorResult,
        FExecutionReplanTimingStats& InOutTimingStats,
        int32& InOutTotalReplanCount);
};
