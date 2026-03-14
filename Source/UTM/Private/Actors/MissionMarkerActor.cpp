#include "Actors/MissionMarkerActor.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"

#include "Materials/MaterialInstanceDynamic.h"

AMissionMarkerActor::AMissionMarkerActor()
{
    PrimaryActorTick.bCanEverTick = false;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
    RootComponent = SceneRoot;

    MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComponent"));
    MeshComponent->SetupAttachment(SceneRoot);
    MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);

}



// 起点终点根据 MarkerType 设置不同颜色和大小
void AMissionMarkerActor::UpdateVisual()
{
    UE_LOG(LogTemp, Warning, TEXT("UpdateVisual called. MissionId=%d Type=%d"), MissionId, (int32)MarkerType);

    if (!MeshComponent)
    {
        UE_LOG(LogTemp, Error, TEXT("UpdateVisual: MeshComponent is null"));
        return;
    }

    UMaterialInstanceDynamic* MID = MeshComponent->CreateAndSetMaterialInstanceDynamic(0);
    if (!MID)
    {
        UE_LOG(LogTemp, Error, TEXT("UpdateVisual: Failed to create MID"));
        return;
    }

    const FLinearColor MarkerColor =
        (MarkerType == EMissionMarkerType::Start)
        ? FLinearColor::Green
        : FLinearColor::Red;

    MID->SetVectorParameterValue(TEXT("Color"), MarkerColor);

    UE_LOG(LogTemp, Warning, TEXT("UpdateVisual: Set color done"));
}