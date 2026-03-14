#include "Actors/NoFlyZoneMarkerActor.h"

#include "Components/BoxComponent.h"
#include "Components/SceneComponent.h"

ANoFlyZoneMarkerActor::ANoFlyZoneMarkerActor()
{
    PrimaryActorTick.bCanEverTick = false;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
    RootComponent = SceneRoot;

    BoxComponent = CreateDefaultSubobject<UBoxComponent>(TEXT("BoxComponent"));
    BoxComponent->SetupAttachment(SceneRoot);
    BoxComponent->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
    BoxComponent->SetCollisionResponseToAllChannels(ECR_Ignore);
    BoxComponent->SetHiddenInGame(false);
    BoxComponent->SetLineThickness(2.f);
    BoxComponent->ShapeColor = FColor(255, 160, 0, 255);
}

void ANoFlyZoneMarkerActor::UpdateVisual()
{
    if (!BoxComponent)
    {
        return;
    }

    const bool bActive = ZoneConfig.bEnabled;
    BoxComponent->ShapeColor = bActive ? FColor(255, 160, 0, 255) : FColor(96, 96, 96, 255);
}
