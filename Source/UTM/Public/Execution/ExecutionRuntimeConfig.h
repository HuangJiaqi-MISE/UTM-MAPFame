#pragma once

#include "CoreMinimal.h"
#include "Execution/DiscreteAlignmentManager.h"
#include "Execution/ExecutionConflictResolutionPolicy.h"
#include "Execution/ExecutionDelayPolicyTypes.h"
#include "Execution/ExecutionFinalSafetyGateTypes.h"
#include "Execution/ExecutionTypes.h"

struct FExecutionReplanServiceSettings
{
    int32 MaxReplanCount = 8;
    int32 LocalSpatialExpansionRadiusCells = 2;
    int32 LocalLookaheadSteps = 5;
    int32 LocalMaxExpansionRounds = 2;
};

struct FExecutionRuntimeConfig
{
    FExecutionDelayPolicySettings Delay;
    FDiscreteAlignmentSettings Alignment;
    FExecutionConflictResolutionSettings ConflictResolution;
    FExecutionFinalSafetyGateSettings FinalSafetyGate;
    FExecutionReplanServiceSettings ReplanService;
    EExecutionPolicyReplanMode ReplanMode =
        EExecutionPolicyReplanMode::GlobalUnfinished;
    bool bCheckStaticUTMSafety = false;
};
