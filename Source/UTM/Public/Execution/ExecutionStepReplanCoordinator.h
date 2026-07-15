#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionStepTypes.h"

struct FExecutionStepReplanCoordinatorRequest
{
    TSet<int32> RequestedMissionIds;
    EExecutionPolicyReplanMode ReplanMode = EExecutionPolicyReplanMode::Disabled;
};

struct FExecutionStepReplanCoordinatorCallbacks
{
    TFunction<bool(const TSet<int32>&, bool, TSet<int32>&)> RunReplan;
    TFunction<void(
        const TSet<int32>&,
        TMap<int32, FExecutionStepProposal>&)> ApplyReplanResult;
};

struct FExecutionStepReplanCoordinatorResult
{
    bool bSuccess = false;
    TSet<int32> ReplannedMissionIds;
};

class FExecutionStepReplanCoordinator
{
public:
    static FExecutionStepReplanCoordinatorResult Run(
        const FExecutionStepReplanCoordinatorRequest& Request,
        const FExecutionStepReplanCoordinatorCallbacks& Callbacks,
        TMap<int32, FExecutionStepProposal>& InOutStepProposals);
};
