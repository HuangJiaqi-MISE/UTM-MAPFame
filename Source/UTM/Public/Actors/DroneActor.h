#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DroneActor.generated.h"

class USceneComponent;
class UStaticMeshComponent;

UCLASS()
class UTM_API ADroneActor : public AActor
{
    GENERATED_BODY()

public:
    ADroneActor();

protected:
    virtual void BeginPlay() override;

public:
    virtual void Tick(float DeltaTime) override;

public:
    UFUNCTION(BlueprintCallable, Category = "Drone")
    void SetPath(const TArray<FVector>& InPath);

    UFUNCTION(BlueprintCallable, Category = "Drone")
    void ConfigureDiscretePlanTiming(bool bInUseDiscretePlanTiming, float InDiscreteStepDuration);

    UFUNCTION(BlueprintCallable, Category = "Drone")
    void StartMove();

    UFUNCTION(BlueprintCallable, Category = "Drone")
    void StopMove();

    UFUNCTION(BlueprintCallable, Category = "Drone")
    bool IsMoving() const;

protected:
    void MoveAlongPath(float DeltaTime);

public:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    TObjectPtr<UStaticMeshComponent> DroneMesh;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Move")
    float MoveSpeed = 10.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Move")
    float AcceptRadius = 20.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Move")
    bool bAutoFaceMovement = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Move")
    bool bUseDiscretePlanTiming = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Move", meta = (ClampMin = "0.01"))
    float DiscreteStepDuration = 0.333f;

private:
    TArray<FVector> PathPoints;
    int32 CurrentPathIndex = 0;
    bool bIsMoving = false;
    float SegmentElapsedTime = 0.f;
    FVector SegmentStartLocation = FVector::ZeroVector;
};