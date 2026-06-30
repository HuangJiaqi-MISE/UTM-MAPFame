#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"

class FExecutionReplanPolicy
{
public:
    explicit FExecutionReplanPolicy(const FExecutionReplanPolicySettings& InSettings = FExecutionReplanPolicySettings());

    void SetSettings(const FExecutionReplanPolicySettings& InSettings);

    FExecutionReplanRequest BuildReplanRequest(
        const FExecutionSnapshot& Snapshot,
        const TArray<FExecutionStepDecision>& Decisions,
        const TArray<FExecutionPredictedConflict>& PredictedConflicts) const;

private:
    FExecutionReplanPolicySettings Settings;
};

