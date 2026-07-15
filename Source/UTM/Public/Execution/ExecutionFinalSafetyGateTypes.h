#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"
#include "Planning/DroneMissionTypes.h"

struct FExecutionFinalSafetyGateAgentState
{
    int32 ConsecutiveSafetyGateHoldCount = 0;
};

struct FExecutionFinalSafetyGateSettings
{
    bool bEnabled = true;
    bool bCheckStaticUTMSafety = false;
    int32 MaxHoldSteps = 1;
};

struct FExecutionFinalSafetyGateInput
{
    TArray<int32> OrderedMissionIds;
    const TMap<int32, FDroneMissionConfig>* MissionConfigsById = nullptr;
    TMap<int32, FExecutionFinalSafetyGateAgentState> AgentStatesByMissionId;
    FExecutionFinalSafetyGateSettings Settings;
};

struct FExecutionFinalSafetyGateConflictCheckResult
{
    bool bHasConflict = false;
    TSet<int32> ConflictMissionIds;
    FExecutionPredictedConflict FirstConflict;
};

struct FExecutionFinalSafetyGateHoldExpansion
{
    int32 PreviousHoldCount = 0;
    int32 ExpandedHoldCount = 0;
    FExecutionPredictedConflict RemainingConflict;
};

struct FExecutionFinalSafetyGateResult
{
    bool bConflictDetected = false;
    bool bHoldConfigurationSafe = false;
    bool bForceGlobalReplan = false;
    int32 InitialHoldMissionCount = 0;
    int32 HoldBudget = 1;
    int32 HoldLimitMissionId = INDEX_NONE;
    TSet<int32> HoldMissionIds;
    FExecutionPredictedConflict InitialConflict;
    FExecutionPredictedConflict UnresolvedHoldConflict;
    TArray<FExecutionFinalSafetyGateHoldExpansion> HoldExpansions;
};
