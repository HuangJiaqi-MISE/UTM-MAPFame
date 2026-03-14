#pragma once

#include "CoreMinimal.h"
#include "TemporalNoFlyZoneTypes.generated.h"

USTRUCT(BlueprintType)
struct FTemporalNoFlyZoneConfig
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone")
    int32 ZoneId = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone")
    bool bEnabled = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone")
    FIntVector MinCell = FIntVector::ZeroValue;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone")
    FIntVector MaxCell = FIntVector::ZeroValue;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone", meta = (ClampMin = "0"))
    int32 StartTimeStep = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone", meta = (ClampMin = "0"))
    int32 EndTimeStep = 10;
};
