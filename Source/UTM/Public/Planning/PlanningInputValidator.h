#pragma once

#include "CoreMinimal.h"

class AActor;
class FGridMap3D;

class FPlanningInputValidator
{
public:
    bool ValidatePointInGrid(
        const FGridMap3D& GridMap,
        const FVector& WorldPos,
        const TCHAR* Label) const;

    bool ValidatePointNotBlocked(
        const FGridMap3D& GridMap,
        const FVector& WorldPos,
        const TCHAR* Label) const;

    bool ValidateStartGoalPair(
        const FGridMap3D& GridMap,
        const FVector& StartWorld,
        const FVector& GoalWorld,
        int32 PairId,
        const AActor* StartActor,
        const AActor* GoalActor) const;
};