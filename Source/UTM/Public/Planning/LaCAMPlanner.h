#pragma once

#include "CoreMinimal.h"
#include "Planning/MultiAgentPlannerBase.h"

class FLaCAMPlanner : public IMultiAgentPlannerBase
{
public:
    explicit FLaCAMPlanner(
        double InTimeLimitMs = 5000.0,
        int32 InRandomSeed = 12345,
        bool bInAnytime = false,
        int32 InVerboseLevel = 0);

    virtual bool PlanMissions(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        TMap<int32, TArray<FVector>>& OutPaths) override;

private:
    double TimeLimitMs = 5000.0;
    int32 RandomSeed = 12345;
    bool bAnytime = false;
    int32 VerboseLevel = 0;
};
