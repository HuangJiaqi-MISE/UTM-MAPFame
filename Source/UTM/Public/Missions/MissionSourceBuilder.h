#pragma once

#include "CoreMinimal.h"
#include "Missions/MissionSourceTypes.h"

class FMissionSourceBuilder
{
public:
    static FMissionSourceBuildResult BuildFromMissionConfigs(
        const TArray<FDroneMissionConfig>& MissionConfigs);

    static FMissionSourceBuildResult BuildFromStartGoalWorldPairs(
        const TArray<int32>& MissionIds,
        const TMap<int32, FVector>& StartsByMissionId,
        const TMap<int32, FVector>& GoalsByMissionId);
};
