#pragma once

#include "CoreMinimal.h"

class FGridMap3D;

class IPathPlannerBase
{
public:
    virtual ~IPathPlannerBase() = default;

    virtual bool Plan(
        const FGridMap3D& GridMap,
        const FVector& StartWorld,
        const FVector& GoalWorld,
        TArray<FVector>& OutPath) = 0;
};