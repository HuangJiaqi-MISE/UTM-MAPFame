#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"
#include "Planning/DroneMissionTypes.h"

struct FConflictPredictionSettings
{
    bool bCheckVertexConflicts = true;
    bool bCheckEdgeConflicts = true;
};

struct FExecutionConflictCheckItem
{
    int32 MissionId = INDEX_NONE;
    bool bValid = false;
    FIntVector ObservedCell = FIntVector::ZeroValue;
    FIntVector TargetCell = FIntVector::ZeroValue;
};

struct FExecutionConflictPredictionInput
{
    TArray<FExecutionConflictCheckItem> Items;
    const TMap<int32, FDroneMissionConfig>* MissionConfigsById = nullptr;
    bool bCheckStaticUTMSafety = false;
};

class FConflictPredictionPolicy
{
public:
    explicit FConflictPredictionPolicy(const FConflictPredictionSettings& InSettings = FConflictPredictionSettings());

    void SetSettings(const FConflictPredictionSettings& InSettings);

    bool FindFirstConflict(
        const TArray<FExecutionStepDecision>& Decisions,
        FExecutionPredictedConflict& OutConflict) const;

    bool FindFirstConflict(
        const FExecutionConflictPredictionInput& Input,
        FExecutionPredictedConflict& OutConflict) const;

    TArray<FExecutionPredictedConflict> FindConflicts(
        const TArray<FExecutionStepDecision>& Decisions) const;

    TArray<FExecutionPredictedConflict> FindConflicts(
        const FExecutionConflictPredictionInput& Input) const;

private:
    FConflictPredictionSettings Settings;
};

