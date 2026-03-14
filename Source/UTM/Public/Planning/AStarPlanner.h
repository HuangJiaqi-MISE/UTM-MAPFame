#pragma once

#include "CoreMinimal.h"
#include "Planning/PathPlannerBase.h"

class FGridMap3D;

class FAStarPlanner : public IPathPlannerBase
{
public:
    virtual bool Plan(
        const FGridMap3D& GridMap,
        const FVector& StartWorld,
        const FVector& GoalWorld,
        TArray<FVector>& OutPath) override;

private:
    float Heuristic(const FIntVector& A, const FIntVector& B) const;
    FIntVector IndexToCell(const FGridMap3D& GridMap, int32 Idx) const;
};