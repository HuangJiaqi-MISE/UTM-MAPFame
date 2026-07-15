#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionConflictResolutionPolicy.h"
#include "Execution/ExecutionFinalSafetyGateCoordinator.h"
#include "Execution/ExecutionReplanProposalSynchronizer.h"
#include "Execution/ExecutionRuntimeConfig.h"
#include "Execution/ExecutionStepTypes.h"
#include "ExecutionControllerTypes.generated.h"

class FGridMap3D;

UENUM()
enum class EExecutionControllerType : uint8
{
    DefaultPipeline UMETA(DisplayName = "Default Execution Pipeline")
};

struct FExecutionControllerStepRequest
{
    TArray<int32> OrderedMissionIds;
    TArray<FExecutionAgentSnapshot> OrderedAgentSnapshots;
    const FGridMap3D* GridMap = nullptr;
    FExecutionRuntimeConfig RuntimeConfig;
    const FExecutionConflictResolutionInput* ConflictResolutionInput = nullptr;
};

struct FExecutionControllerStepCallbacks
{
    TFunction<bool(const TSet<int32>&, bool, TSet<int32>&)> RunReplan;
    TFunction<TMap<int32, FExecutionReplanProposalAgentState>()>
        CaptureReplanProposalAgentStates;
    TFunction<FExecutionFinalSafetyGateInput()> BuildFinalSafetyGateInput;
    TFunction<void(const FExecutionConflictResolutionEvent&)> OnConflictResolutionEvent;
    TFunction<void(const FExecutionFinalSafetyGateEvent&)> OnFinalSafetyGateEvent;
};

struct FExecutionControllerStepResult
{
    bool bReplanSucceeded = false;
    bool bStopExecution = false;
    TSet<int32> RequestedReplanMissionIds;
    TSet<int32> SuccessfulReplanMissionIds;
    TMap<int32, FExecutionStepProposal> StepProposals;
};
