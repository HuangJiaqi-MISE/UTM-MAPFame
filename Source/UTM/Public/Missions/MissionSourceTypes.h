#pragma once

#include "CoreMinimal.h"
#include "Planning/DroneMissionTypes.h"

enum class EMissionSourceKind : uint8
{
    MissionConfigs,
    StartGoalWorldPairs
};

struct FMissionSourceBuildResult
{
    EMissionSourceKind SourceKind = EMissionSourceKind::MissionConfigs;
    bool bSuccess = true;
    FString ErrorMessage;
    TArray<FDroneMissionConfig> Missions;
    TArray<int32> SkippedMissionIds;
};
