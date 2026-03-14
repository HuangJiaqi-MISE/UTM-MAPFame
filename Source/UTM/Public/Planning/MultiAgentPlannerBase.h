#pragma once

#include "CoreMinimal.h"
#include "Planning/DroneMissionTypes.h"

class FGridMap3D;

class IMultiAgentPlannerBase
{
public:
    virtual ~IMultiAgentPlannerBase() = default;

    virtual bool PlanMissions(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        TMap<int32, TArray<FVector>>& OutPaths) = 0;
};