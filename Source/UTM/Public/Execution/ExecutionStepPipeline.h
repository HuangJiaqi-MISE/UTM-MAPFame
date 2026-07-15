#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionAlignmentPolicy.h"
#include "Execution/ExecutionConflictResolutionPolicy.h"
#include "Execution/ExecutionFinalSafetyGateCoordinator.h"
#include "Execution/ExecutionReplanProposalSynchronizer.h"
#include "Execution/ExecutionStepProposalBuilder.h"
#include "Execution/ExecutionStepReplanCoordinator.h"

struct FExecutionStepPipelineRequest
{
    TArray<int32> OrderedMissionIds;
    TArray<FExecutionAgentSnapshot> OrderedAgentSnapshots;
    const FGridMap3D* GridMap = nullptr;
    FDiscreteAlignmentSettings AlignmentSettings;
    const FExecutionConflictResolutionInput* ConflictResolutionInput = nullptr;
    EExecutionPolicyReplanMode ReplanMode = EExecutionPolicyReplanMode::Disabled;
};

struct FExecutionStepPipelineCallbacks
{
    TFunction<bool(const TSet<int32>&, bool, TSet<int32>&)> RunReplan;
    TFunction<TMap<int32, FExecutionReplanProposalAgentState>()>
        CaptureReplanProposalAgentStates;
    TFunction<FExecutionFinalSafetyGateInput()> BuildFinalSafetyGateInput;
    TFunction<void(const FExecutionConflictResolutionEvent&)> OnConflictResolutionEvent;
    TFunction<void(const FExecutionFinalSafetyGateEvent&)> OnFinalSafetyGateEvent;
};

struct FExecutionStepPipelineResult
{
    bool bReplanSucceeded = false;
    bool bStopExecution = false;
    TSet<int32> RequestedReplanMissionIds;
    TSet<int32> SuccessfulReplanMissionIds;
    TMap<int32, FExecutionStepProposal> StepProposals;
};

class FExecutionStepPipeline
{
public:
    static FExecutionStepPipelineResult Run(
        const FExecutionStepPipelineRequest& Request,
        const FExecutionStepPipelineCallbacks& Callbacks);
};
