#pragma once

#include "CoreMinimal.h"
#include "Actors/MissionMarkerActor.h"
#include "Planning/DroneMissionTypes.h"

class AActor;
class FGridMap3D;
class FPlanningInputValidator;
class UWorld;

struct FMissionEditorGenerateRandomRequest
{
    const FGridMap3D* GridMap = nullptr;
    FIntVector GridDim = FIntVector::ZeroValue;
    int32 RandomMissionCount = 0;
    int32 RandomSeed = 0;
    int32 MinStartGoalCellDistance = 0;
    bool bAllowDuplicateStartCells = false;
    bool bAllowDuplicateGoalCells = false;
    TArray<FDroneMissionConfig>* MissionConfigs = nullptr;
};

struct FMissionEditorSpawnMarkersRequest
{
    UWorld* World = nullptr;
    AActor* Owner = nullptr;
    const FGridMap3D* GridMap = nullptr;
    TSubclassOf<AMissionMarkerActor> MarkerClass;
    float MarkerZOffset = 0.f;
    const TArray<FDroneMissionConfig>* MissionConfigs = nullptr;
};

struct FMissionEditorValidateRequest
{
    const FGridMap3D* GridMap = nullptr;
    const FPlanningInputValidator* InputValidator = nullptr;
    const TArray<FDroneMissionConfig>* MissionConfigs = nullptr;
};

struct FMissionEditorReadMarkersRequest
{
    UWorld* World = nullptr;
    const FGridMap3D* GridMap = nullptr;
    float MarkerZOffset = 0.f;
    TArray<FDroneMissionConfig>* MissionConfigs = nullptr;
};

class FMissionEditorService
{
public:
    static void AppendMissionMarkerActors(
        UWorld* World,
        TArray<AActor*>& InOutActors);

    static void GenerateRandomMissionConfigs(
        const FMissionEditorGenerateRandomRequest& Request);

    static void ClearMissionMarkers(UWorld* World);

    static void SpawnMissionMarkers(
        const FMissionEditorSpawnMarkersRequest& Request);

    static void ValidateMissionConfigs(
        const FMissionEditorValidateRequest& Request);

    static void ReadMissionMarkersToConfigs(
        const FMissionEditorReadMarkersRequest& Request);
};
