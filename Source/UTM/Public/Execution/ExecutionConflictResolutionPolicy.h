#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionStepTypes.h"
#include "Planning/DroneMissionTypes.h"

struct FExecutionConflictResolutionAgentState
{
    bool bFinished = false;
    int32 ConsecutiveConflictHoldCount = 0;
};

struct FExecutionConflictResolutionSettings
{
    bool bEnabled = true;
    bool bCheckStaticUTMSafety = false;
    int32 MaxResolutionPasses = 1;
    int32 ConflictHoldThresholdForReplan = 2;
};

struct FExecutionConflictResolutionInput
{
    const TMap<int32, FDroneMissionConfig>* MissionConfigsById = nullptr;
    TMap<int32, FExecutionConflictResolutionAgentState> AgentStatesByMissionId;
    FExecutionConflictResolutionSettings Settings;
};

enum class EExecutionConflictResolutionEventType : uint8
{
    UnresolvedConflict,
    YieldApplied,
    RemainingConflict
};

struct FExecutionConflictResolutionEvent
{
    EExecutionConflictResolutionEventType Type = EExecutionConflictResolutionEventType::UnresolvedConflict;
    FExecutionPredictedConflict Conflict;
    int32 YieldMissionId = INDEX_NONE;
    int32 KeepMissionId = INDEX_NONE;
};

struct FExecutionConflictResolutionResult
{
    TSet<int32> ReplanMissionIds;
    TArray<FExecutionConflictResolutionEvent> Events;
};

class FExecutionConflictResolutionPolicy
{
public:
    static FExecutionConflictResolutionResult Resolve(
        const FExecutionConflictResolutionInput& Input,
        TMap<int32, FExecutionStepProposal>& InOutStepProposals);
};
