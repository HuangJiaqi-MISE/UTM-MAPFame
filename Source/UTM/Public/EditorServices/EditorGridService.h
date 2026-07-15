#pragma once

#include "CoreMinimal.h"

class AActor;
class FGridMap3D;
class UWorld;

struct FEditorGridBuildRequest
{
    UWorld* World = nullptr;
    FGridMap3D* GridMap = nullptr;
    FVector GridOrigin = FVector::ZeroVector;
    FIntVector GridDim = FIntVector::ZeroValue;
    float CellSize = 100.f;
    TArray<AActor*> IgnoreActors;
    bool bDrawOccupiedCells = false;
    bool bDrawFreeCells = false;
    float DebugDrawTime = 0.f;
};

class FEditorGridService
{
public:
    static void BuildGridForMissionEditing(
        const FEditorGridBuildRequest& Request);
};
