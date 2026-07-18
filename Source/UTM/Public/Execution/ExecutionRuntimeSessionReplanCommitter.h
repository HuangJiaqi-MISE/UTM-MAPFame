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

struct FExecutionRuntimeReplanAttemptCommitResult
{
    bool bSuccess = false;
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
