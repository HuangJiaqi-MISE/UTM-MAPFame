#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionReplanCoordinator.h"
#include "Execution/ExecutionRuntimeConfig.h"
#include "Planning/PlannerTypes.h"

class FGridMap3D;
struct FExecutionRuntimeSession;

enum class EExecutionReplanServiceStatus : uint8
{
    Success,
    EmptyRequest,
    Disabled,
    ReplanLimitReached,
    InvalidContext,
    CoordinatorFailed
};

struct FExecutionReplanServiceRequest
{
    const FGridMap3D* GridMap = nullptr;
    FExecutionRuntimeSession* Session = nullptr;
    TMap<int32, TArray<FIntVector>>* PlannedCellPathsByMissionId = nullptr;
    TMap<int32, TArray<FVector>>* PlannedWorldPathsByMissionId = nullptr;
    EPlannerType PlannerType = EPlannerType::AStar;
    FPlannerRuntimeConfig PlannerConfig;
    FExecutionRuntimeConfig RuntimeConfig;
    TSet<int32> RequestedMissionIds;
    bool bGlobalReplan = false;
};

struct FExecutionReplanServiceCallbacks
{
    TFunction<void(
        const FExecutionReplanAttemptInput&,
        const FExecutionReplanAttemptResult&)> OnAttemptFailure;
    TFunction<void(const FExecutionReplanCoordinatorEvent&)> OnCoordinatorEvent;
};

struct FExecutionReplanServiceResult
{
    EExecutionReplanServiceStatus Status =
        EExecutionReplanServiceStatus::InvalidContext;
    bool bSuccess = false;
    FString FailureReason;
    TSet<int32> ReplannedMissionIds;
    int32 CurrentTotalReplanCount = 0;
    int32 MaxReplanCount = 0;
};
