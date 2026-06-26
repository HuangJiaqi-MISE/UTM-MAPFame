#pragma once

#include "CoreMinimal.h"
#include "Planning/DroneMissionTypes.h"
#include "MissionSchedulerTypes.generated.h"

class FGridMap3D;

UENUM()
enum class EMissionSchedulerType : uint8
{
    Static       UMETA(DisplayName = "Static Scheduler"),
    NearestFirst UMETA(DisplayName = "Nearest-First Scheduler")
};

struct FMissionSchedulerContext
{
    const FGridMap3D* GridMap = nullptr;
    int32 RequestedMissionCount = 0;
};
