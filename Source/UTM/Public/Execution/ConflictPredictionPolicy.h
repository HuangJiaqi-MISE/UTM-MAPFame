#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"

struct FConflictPredictionSettings
{
    bool bCheckVertexConflicts = true;
    bool bCheckEdgeConflicts = true;
};

class FConflictPredictionPolicy
{
public:
    explicit FConflictPredictionPolicy(const FConflictPredictionSettings& InSettings = FConflictPredictionSettings());

    void SetSettings(const FConflictPredictionSettings& InSettings);

    bool FindFirstConflict(
        const TArray<FExecutionStepDecision>& Decisions,
        FExecutionPredictedConflict& OutConflict) const;

    TArray<FExecutionPredictedConflict> FindConflicts(
        const TArray<FExecutionStepDecision>& Decisions) const;

private:
    FConflictPredictionSettings Settings;
};

