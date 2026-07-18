#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionControllerTypes.h"
#include "Execution/ExecutionRuntimeStateTypes.h"
#include "Execution/ExecutionStepResultApplier.h"

struct FExecutionRuntimeStepPrepareRequest
{
    TMap<int32, FExecutionAgentState>* AgentStatesByMissionId = nullptr;
    const TMap<int32, FDroneMissionConfig>* MissionConfigsById = nullptr;
    int32 TimeStep = 0;
    TFunction<FIntVector(const FExecutionAgentState&)> ResolveObservedCell;
    TFunction<bool(const FExecutionAgentState&, int32)> ShouldDelay;
};

struct FExecutionRuntimeStepPrepareResult
{
    TArray<int32> OrderedMissionIds;
    TArray<FExecutionAgentSnapshot> OrderedAgentSnapshots;
    FExecutionConflictResolutionInput ConflictResolutionInput;
};

class FExecutionRuntimeSessionStepProcessor
{
public:
    static FExecutionRuntimeStepPrepareResult PrepareStep(
        const FExecutionRuntimeStepPrepareRequest& Request);

    static TMap<int32, FExecutionReplanProposalAgentState>
        CaptureReplanProposalAgentStates(
            const TArray<int32>& OrderedMissionIds,
            const TMap<int32, FExecutionAgentState>& AgentStatesByMissionId);

    static FExecutionFinalSafetyGateInput BuildFinalSafetyGateInput(
        const TArray<int32>& OrderedMissionIds,
        const TMap<int32, FDroneMissionConfig>& MissionConfigsById,
        const TMap<int32, FExecutionAgentState>& AgentStatesByMissionId);

    static FExecutionStepResultApplyResult ApplyControllerResult(
        const TArray<int32>& OrderedMissionIds,
        TMap<int32, FExecutionAgentState>& AgentStatesByMissionId,
        const FExecutionControllerStepResult& ControllerResult);
};
