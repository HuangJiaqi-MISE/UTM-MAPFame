#include "Actors/DroneActor.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"

namespace
{
    constexpr float MinDiscreteStepDuration = 0.01f;
}

ADroneActor::ADroneActor()
{
    PrimaryActorTick.bCanEverTick = true;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
    RootComponent = SceneRoot;

    DroneMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneMesh"));
    DroneMesh->SetupAttachment(SceneRoot);

    DroneMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void ADroneActor::BeginPlay()
{
    Super::BeginPlay();
}

void ADroneActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (bIsMoving)
    {
        MoveAlongPath(DeltaTime);
    }
}

void ADroneActor::SetPath(const TArray<FVector>& InPath)
{
    PathPoints = InPath;
    CurrentPathIndex = 0;
    bIsMoving = false;
    SegmentElapsedTime = 0.f;
    SegmentStartLocation = PathPoints.Num() > 0 ? PathPoints[0] : GetActorLocation();

    if (PathPoints.Num() > 0)
    {
        SetActorLocation(PathPoints[0]);
    }
}

void ADroneActor::ConfigureDiscretePlanTiming(bool bInUseDiscretePlanTiming, float InDiscreteStepDuration)
{
    bUseDiscretePlanTiming = bInUseDiscretePlanTiming;

    if (InDiscreteStepDuration > MinDiscreteStepDuration)
    {
        DiscreteStepDuration = InDiscreteStepDuration;
    }
}

void ADroneActor::StartMove()
{
    if (PathPoints.Num() <= 1)
    {
        UE_LOG(LogTemp, Warning, TEXT("DroneActor: path is too short to move"));
        bIsMoving = false;
        return;
    }

    CurrentPathIndex = 1;
    SegmentElapsedTime = 0.f;
    SegmentStartLocation = PathPoints[0];
    SetActorLocation(PathPoints[0]);
    bIsMoving = true;

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("DroneActor: movement started. NumPoints=%d DiscreteTiming=%s StepDuration=%.3f"),
        PathPoints.Num(),
        bUseDiscretePlanTiming ? TEXT("true") : TEXT("false"),
        DiscreteStepDuration
    );
}

void ADroneActor::StopMove()
{
    bIsMoving = false;
    SegmentElapsedTime = 0.f;
    SegmentStartLocation = GetActorLocation();
    UE_LOG(LogTemp, Warning, TEXT("DroneActor: movement stopped"));
}

bool ADroneActor::IsMoving() const
{
    return bIsMoving;
}

void ADroneActor::MoveAlongPath(float DeltaTime)
{
    if (PathPoints.Num() == 0)
    {
        bIsMoving = false;
        return;
    }

    if (CurrentPathIndex < 0 || CurrentPathIndex >= PathPoints.Num())
    {
        bIsMoving = false;
        UE_LOG(LogTemp, Warning, TEXT("DroneActor: reached end of path"));
        return;
    }

    if (!bUseDiscretePlanTiming)
    {
        const FVector CurrentLocation = GetActorLocation();
        const FVector TargetLocation = PathPoints[CurrentPathIndex];

        const FVector NewLocation = FMath::VInterpConstantTo(
            CurrentLocation,
            TargetLocation,
            DeltaTime,
            MoveSpeed
        );

        SetActorLocation(NewLocation);

        if (bAutoFaceMovement)
        {
            const FVector MoveDir = TargetLocation - CurrentLocation;
            if (!MoveDir.IsNearlyZero())
            {
                const FRotator TargetRotation = MoveDir.Rotation();
                SetActorRotation(TargetRotation);
            }
        }

        const float DistSq = FVector::DistSquared(NewLocation, TargetLocation);
        if (DistSq <= AcceptRadius * AcceptRadius)
        {
            CurrentPathIndex++;

            if (CurrentPathIndex >= PathPoints.Num())
            {
                bIsMoving = false;

                UE_LOG(LogTemp, Warning, TEXT("DroneActor: path finished"));

                if (GEngine)
                {
                    GEngine->AddOnScreenDebugMessage(
                        -1,
                        3.f,
                        FColor::Green,
                        TEXT("Drone finished path")
                    );
                }
            }
        }

        return;
    }

    float RemainingTime = DeltaTime;
    const float StepDuration = FMath::Max(DiscreteStepDuration, MinDiscreteStepDuration);

    while (RemainingTime > 0.f && bIsMoving)
    {
        if (CurrentPathIndex < 0 || CurrentPathIndex >= PathPoints.Num())
        {
            bIsMoving = false;
            UE_LOG(LogTemp, Warning, TEXT("DroneActor: reached end of path"));
            return;
        }

        const FVector TargetLocation = PathPoints[CurrentPathIndex];
        const float TimeToSegmentEnd = StepDuration - SegmentElapsedTime;
        const float ConsumedTime = FMath::Min(RemainingTime, TimeToSegmentEnd);

        SegmentElapsedTime += ConsumedTime;
        RemainingTime -= ConsumedTime;

        const float Alpha = FMath::Clamp(SegmentElapsedTime / StepDuration, 0.f, 1.f);
        const FVector NewLocation = FMath::Lerp(SegmentStartLocation, TargetLocation, Alpha);
        SetActorLocation(NewLocation);

        if (bAutoFaceMovement)
        {
            const FVector MoveDir = TargetLocation - SegmentStartLocation;
            if (!MoveDir.IsNearlyZero())
            {
                SetActorRotation(MoveDir.Rotation());
            }
        }

        if (Alpha < 1.f)
        {
            break;
        }

        SetActorLocation(TargetLocation);
        SegmentStartLocation = TargetLocation;
        SegmentElapsedTime = 0.f;
        ++CurrentPathIndex;

        if (CurrentPathIndex >= PathPoints.Num())
        {
            bIsMoving = false;

            UE_LOG(LogTemp, Warning, TEXT("DroneActor: path finished"));

            if (GEngine)
            {
                GEngine->AddOnScreenDebugMessage(
                    -1,
                    3.f,
                    FColor::Green,
                    TEXT("Drone finished path")
                );
            }
        }
    }
}