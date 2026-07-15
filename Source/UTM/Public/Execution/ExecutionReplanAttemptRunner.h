#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionReplanAttemptTypes.h"
#include "Planning/DroneMissionTypes.h"
#include "Planning/PlannerTypes.h"

class FGridMap3D;

struct FExecutionReplanAttemptContext
{
    const FGridMap3D* BaseGrid = nullptr;
    const TMap<int32, FDroneMissionConfig>* MissionConfigsById = nullptr;
    EPlannerType PlannerType = EPlannerType::AStar;
    const FPlannerRuntimeConfig* PlannerConfig = nullptr;
};

class FExecutionReplanAttemptRunner
{
public:
    static bool Run(
        const FExecutionReplanAttemptContext& Context,
        const FExecutionReplanAttemptInput& Input,
        FExecutionReplanAttemptResult& OutResult);
};
