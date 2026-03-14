#pragma once

#include "CoreMinimal.h"
#include "DroneMissionTypes.generated.h"

USTRUCT(BlueprintType)
struct FDroneMissionConfig
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
    int32 MissionId = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
    FVector StartWorld = FVector(50.f, 50.f, 50.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
    FVector GoalWorld = FVector(850.f, 850.f, 50.f);
};