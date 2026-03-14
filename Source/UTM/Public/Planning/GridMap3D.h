#pragma once

#include "CoreMinimal.h"

class AActor;
class UWorld;

class FGridMap3D
{
public:
    FVector GridOrigin = FVector::ZeroVector;
    FIntVector GridDim = FIntVector(20, 20, 6);
    float CellSize = 100.f;

    // 0 = free, 1 = blocked
    TArray<uint8> Occupancy;

public:
    int32 ToIndex(int32 X, int32 Y, int32 Z) const;
    bool IsInside(int32 X, int32 Y, int32 Z) const;

    FIntVector WorldToCell(const FVector& WorldPos) const;
    FVector CellToWorld(const FIntVector& Cell) const;

    bool IsBlocked(int32 X, int32 Y, int32 Z) const;

    bool IsCellBlockedByWorld(
        UWorld* World,
        int32 X,
        int32 Y,
        int32 Z,
        const TArray<AActor*>& ActorsToIgnore) const;

    void BuildOccupancyGrid(
        UWorld* World,
        const TArray<AActor*>& ActorsToIgnore,
        bool bDrawOccupiedCells,
        bool bDrawFreeCells,
        float DebugDrawTime);
};