#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionControllerTypes.h"
#include "Execution/ExecutionRuntimeConfig.h"
#include "Execution/ExecutionRuntimeSession.h"
#include "Execution/ExecutionStepResultApplier.h"

class FGridMap3D;

struct FExecutionRuntimeCoordinatorRequest
{
    FExecutionRuntimeSession* Session = nullptr;
    const FGridMap3D* GridMap = nullptr;
    FExecutionRuntimeConfig RuntimeConfig;
    EExecutionControllerType ControllerType =
        EExecutionControllerType::DefaultPipeline;
};

struct FExecutionRuntimeCoordinatorCallbacks
{
    TFunction<FIntVector(const FExecutionAgentState&)> ResolveObservedCell;
    TFunction<bool(const FExecutionAgentState&, int32)> ShouldDelay;
    TFunction<bool(const TSet<int32>&, bool, TSet<int32>&)> RunReplan;
    TFunction<void(const FExecutionConflictResolutionEvent&)>
        OnConflictResolutionEvent;
    TFunction<void(const FExecutionFinalSafetyGateEvent&)>
        OnFinalSafetyGateEvent;
};

struct FExecutionRuntimeCoordinatorResult
{
    bool bStepProcessed = false;
    FString FailureReason;
    int32 TimeStep = 0;
    FExecutionControllerStepResult ControllerResult;
    FExecutionStepResultApplyResult ApplyResult;
};
