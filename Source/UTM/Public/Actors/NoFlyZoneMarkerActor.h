#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Planning/TemporalNoFlyZoneTypes.h"
#include "NoFlyZoneMarkerActor.generated.h"

class UBoxComponent;
class USceneComponent;

UCLASS()
class UTM_API ANoFlyZoneMarkerActor : public AActor
{
    GENERATED_BODY()

public:
    ANoFlyZoneMarkerActor();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TObjectPtr<UBoxComponent> BoxComponent;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone")
    FTemporalNoFlyZoneConfig ZoneConfig;

    UFUNCTION(BlueprintCallable, Category = "No-Fly Zone")
    void UpdateVisual();
};
