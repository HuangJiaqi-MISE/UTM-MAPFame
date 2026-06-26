#pragma once

#include "CoreMinimal.h"
#include "Planning/TemporalNoFlyZoneTypes.h"
#include "PlannerTypes.generated.h"

UENUM(BlueprintType)
enum class EPlannerType : uint8
{
    AStar      UMETA(DisplayName = "A*"),
    SIPP       UMETA(DisplayName = "SIPP"),
    DStarLite  UMETA(DisplayName = "D* Lite"),
    JPS        UMETA(DisplayName = "JPS"),
    CBS        UMETA(DisplayName = "CBS"),
    ECBS       UMETA(DisplayName = "ECBS"),
    PBS        UMETA(DisplayName = "PBS"),
    LaCAM      UMETA(DisplayName = "LaCAM"),
    LaCAMUTM   UMETA(DisplayName = "LaCAM-UTM")
};

struct FPlannerRuntimeConfig
{
    float ECBSSuboptimalityBound = 1.5f;
    int32 LaCAMTimeLimitMs = 8000;
    int32 LaCAMRandomSeed = 12345;
    bool bLaCAMAnytime = false;
    int32 LaCAMVerboseLevel = 0;
    TArray<FTemporalNoFlyZoneConfig> NoFlyZoneConfigs;
};
