#pragma once

#include "CoreMinimal.h"
#include "Actors/NoFlyZoneMarkerActor.h"
#include "Planning/TemporalNoFlyZoneTypes.h"

class AActor;
class FGridMap3D;
class UWorld;

struct FNoFlyZoneEditorAddConfigRequest
{
    FIntVector GridDim = FIntVector::ZeroValue;
    int32 DefaultSizeCells = 1;
    int32 DefaultDuration = 1;
    TArray<FTemporalNoFlyZoneConfig>* ZoneConfigs = nullptr;
};

struct FNoFlyZoneEditorSpawnMarkersRequest
{
    UWorld* World = nullptr;
    AActor* Owner = nullptr;
    FGridMap3D* GridMap = nullptr;
    FVector GridOrigin = FVector::ZeroVector;
    FIntVector GridDim = FIntVector::ZeroValue;
    float CellSize = 100.f;
    float MarkerZOffset = 0.f;
    TSubclassOf<ANoFlyZoneMarkerActor> MarkerClass;
    const TArray<FTemporalNoFlyZoneConfig>* ZoneConfigs = nullptr;
};

struct FNoFlyZoneEditorReadMarkersRequest
{
    UWorld* World = nullptr;
    FGridMap3D* GridMap = nullptr;
    FVector GridOrigin = FVector::ZeroVector;
    FIntVector GridDim = FIntVector::ZeroValue;
    float CellSize = 100.f;
    float MarkerZOffset = 0.f;
    TArray<FTemporalNoFlyZoneConfig>* ZoneConfigs = nullptr;
};

struct FNoFlyZoneEditorValidateRequest
{
    const FGridMap3D* GridMap = nullptr;
    const TArray<FTemporalNoFlyZoneConfig>* ZoneConfigs = nullptr;
};

struct FNoFlyZoneEditorGenerateRandomRequest
{
    const FGridMap3D* GridMap = nullptr;
    FIntVector GridDim = FIntVector::ZeroValue;
    int32 ZoneCount = 0;
    int32 RandomSeed = 0;
    int32 MinSizeCells = 1;
    int32 MaxSizeCells = 1;
    int32 MinStartTimeStep = 0;
    int32 MaxStartTimeStep = 0;
    int32 MinDuration = 1;
    int32 MaxDuration = 1;
    TArray<FTemporalNoFlyZoneConfig>* ZoneConfigs = nullptr;
};

class FNoFlyZoneEditorService
{
public:
    static void AddNoFlyZoneConfig(
        const FNoFlyZoneEditorAddConfigRequest& Request);

    static void ClearNoFlyZoneMarkers(UWorld* World);

    static void SpawnNoFlyZoneMarkers(
        const FNoFlyZoneEditorSpawnMarkersRequest& Request);

    static void ReadNoFlyZoneMarkersToConfigs(
        const FNoFlyZoneEditorReadMarkersRequest& Request);

    static void ValidateNoFlyZones(
        const FNoFlyZoneEditorValidateRequest& Request);

    static void GenerateRandomNoFlyZoneConfigs(
        const FNoFlyZoneEditorGenerateRandomRequest& Request);
};
