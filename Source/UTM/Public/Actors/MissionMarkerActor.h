#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MissionMarkerActor.generated.h"

class USceneComponent;
class UStaticMeshComponent;


UENUM(BlueprintType)
enum class EMissionMarkerType : uint8
{
    Start,
    Goal
};

UCLASS()
class UTM_API AMissionMarkerActor : public AActor
{
    GENERATED_BODY()

public:
    AMissionMarkerActor();

public:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TObjectPtr<UStaticMeshComponent> MeshComponent;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
    int32 MissionId = -1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
    EMissionMarkerType MarkerType = EMissionMarkerType::Start;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
    FIntVector Cell = FIntVector::ZeroValue;

    UFUNCTION(BlueprintCallable, Category = "Mission")
    void UpdateVisual();

};