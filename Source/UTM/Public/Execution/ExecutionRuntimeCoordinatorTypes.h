#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionControllerTypes.h"
#include "Execution/ExecutionReplanServiceTypes.h"
#include "Execution/ExecutionRuntimeConfig.h"
#include "Execution/ExecutionRuntimeSession.h"
#include "Execution/ExecutionStepResultApplier.h"

class FGridMap3D;

struct FExecutionRuntimeReplanContext
{
    EPlannerType PlannerType = EPlannerType::AStar;
    TMap<int32, TArray<FIntVector>>* PlannedCellPathsByMissionId = nullptr;
    TMap<int32, TArray<FVector>>* PlannedWorldPathsByMissionId = nullptr;
};

struct FExecutionRuntimeCoordinatorInitializeRequest
{
    EExecutionControllerType ControllerType =
        EExecutionControllerType::DefaultPipeline;
    const FExecutionRuntimeSession* Session = nullptr;
    const FGridMap3D* GridMap = nullptr;
    FExecutionRuntimeConfig RuntimeConfig;
};

struct FExecutionRuntimeCoordinatorRequest
{
    FExecutionRuntimeSession* Session = nullptr;
    const FGridMap3D* GridMap = nullptr;
    FExecutionRuntimeConfig RuntimeConfig;
    FExecutionRuntimeReplanContext ReplanContext;
};

struct FExecutionRuntimeCoordinatorCallbacks
{
    TFunction<FIntVector(const FExecutionAgentState&)> ResolveObservedCell;
    TFunction<FPlannerRuntimeConfig()> BuildPlannerRuntimeConfig;
    TFunction<void(
        const FExecutionReplanAttemptInput&,
        const FExecutionReplanAttemptResult&)> OnReplanAttemptFailure;
    TFunction<void(const FExecutionReplanCoordinatorEvent&)>
        OnReplanCoordinatorEvent;
    TFunction<void(const FExecutionReplanServiceResult&)>
        OnReplanServiceResult;
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
    TArray<FExecutionConflict> ObservedConflicts;
};
