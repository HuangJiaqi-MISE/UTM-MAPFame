#pragma once

#include "CoreMinimal.h"
#include "Planning/MultiAgentPlannerBase.h"
#include "Planning/TemporalNoFlyZoneTypes.h"

class FLaCAMUTMPlanner : public IMultiAgentPlannerBase
{
public:
    explicit FLaCAMUTMPlanner(
        double InTimeLimitMs = 5000.0,
        int32 InRandomSeed = 12345,
        bool bInAnytime = false,
        int32 InVerboseLevel = 0,
        const TArray<FTemporalNoFlyZoneConfig>& InNoFlyZoneConfigs = TArray<FTemporalNoFlyZoneConfig>());

    virtual bool PlanMissions(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& Missions,
        TMap<int32, TArray<FVector>>& OutPaths) override;

private:
    double TimeLimitMs = 8000.0;
    int32 RandomSeed = 12345;
    bool bAnytime = false;
    int32 VerboseLevel = 0;
    TArray<FTemporalNoFlyZoneConfig> NoFlyZoneConfigs;
};
