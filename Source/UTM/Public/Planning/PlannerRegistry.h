#pragma once

#include "CoreMinimal.h"
#include "Planning/DroneMissionTypes.h"
#include "Planning/MultiAgentPlannerBase.h"
#include "Planning/PathPlannerBase.h"
#include "Planning/PlannerTypes.h"

class FGridMap3D;

class FPlannerRegistry
{
public:
    static bool IsMultiAgentPlannerType(EPlannerType PlannerType);
    static FString GetPlannerTypeName(EPlannerType PlannerType);

    static TUniquePtr<IPathPlannerBase> CreateSingleAgentPlanner(
        EPlannerType PlannerType,
        const FPlannerRuntimeConfig& Config);

    static TUniquePtr<IMultiAgentPlannerBase> CreateMultiAgentPlanner(
        EPlannerType PlannerType,
        const FPlannerRuntimeConfig& Config);

    static bool PlanMultiAgentMissions(
        EPlannerType PlannerType,
        const FPlannerRuntimeConfig& Config,
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        TMap<int32, TArray<FVector>>& OutPaths);
};
