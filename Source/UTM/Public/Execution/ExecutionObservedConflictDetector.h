#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionRuntimeStateTypes.h"

struct FExecutionObservedConflictDetectionRequest
{
    const TMap<int32, FExecutionAgentState>* AgentStatesByMissionId = nullptr;
    int32 TimeStep = 0;
};

struct FExecutionObservedConflictDetectionResult
{
    TArray<FExecutionConflict> Conflicts;
};

class FExecutionObservedConflictDetector
{
public:
    static FExecutionObservedConflictDetectionResult Detect(
        const FExecutionObservedConflictDetectionRequest& Request);
};
