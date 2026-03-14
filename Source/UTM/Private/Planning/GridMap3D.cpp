#include "Planning/GridMap3D.h"

#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Engine/World.h"

int32 FGridMap3D::ToIndex(int32 X, int32 Y, int32 Z) const
{
    return X + GridDim.X * (Y + GridDim.Y * Z);
}

bool FGridMap3D::IsInside(int32 X, int32 Y, int32 Z) const
{
    return X >= 0 && X < GridDim.X
        && Y >= 0 && Y < GridDim.Y
        && Z >= 0 && Z < GridDim.Z;
}

FIntVector FGridMap3D::WorldToCell(const FVector& WorldPos) const
{
    const FVector Local = WorldPos - GridOrigin;

    return FIntVector(
        FMath::FloorToInt(Local.X / CellSize),
        FMath::FloorToInt(Local.Y / CellSize),
        FMath::FloorToInt(Local.Z / CellSize)
    );
}

FVector FGridMap3D::CellToWorld(const FIntVector& Cell) const
{
    return GridOrigin + FVector(
        (Cell.X + 0.5f) * CellSize,
        (Cell.Y + 0.5f) * CellSize,
        (Cell.Z + 0.5f) * CellSize
    );
}

bool FGridMap3D::IsBlocked(int32 X, int32 Y, int32 Z) const
{
    if (!IsInside(X, Y, Z))
    {
        return true;
    }

    if (Occupancy.Num() <= 0)
    {
        return true;
    }

    const int32 Idx = ToIndex(X, Y, Z);
    if (!Occupancy.IsValidIndex(Idx))
    {
        return true;
    }

    return Occupancy[Idx] != 0;
}

bool FGridMap3D::IsCellBlockedByWorld(
    UWorld* World,
    int32 X,
    int32 Y,
    int32 Z,
    const TArray<AActor*>& ActorsToIgnore) const
{
    if (!World)
    {
        return true;
    }

    if (!IsInside(X, Y, Z))
    {
        return true;
    }

    const FVector Center = CellToWorld(FIntVector(X, Y, Z));
    const FVector HalfSize(CellSize * 0.45f);

    FHitResult Hit;

    const bool bHit = UKismetSystemLibrary::BoxTraceSingle(
        World,
        Center,
        Center,
        HalfSize,
        FRotator::ZeroRotator,
        UEngineTypes::ConvertToTraceType(ECC_Visibility),
        false,
        ActorsToIgnore,
        EDrawDebugTrace::None,
        Hit,
        true
    );

    return bHit;
}

void FGridMap3D::BuildOccupancyGrid(
    UWorld* World,
    const TArray<AActor*>& ActorsToIgnore,
    bool bDrawOccupiedCells,
    bool bDrawFreeCells,
    float DebugDrawTime)
{
    const int32 TotalCells = GridDim.X * GridDim.Y * GridDim.Z;
    Occupancy.Init(0, TotalCells);

    int32 BlockedCount = 0;

    for (int32 Z = 0; Z < GridDim.Z; ++Z)
    {
        for (int32 Y = 0; Y < GridDim.Y; ++Y)
        {
            for (int32 X = 0; X < GridDim.X; ++X)
            {
                const int32 Idx = ToIndex(X, Y, Z);
                const bool bBlocked = IsCellBlockedByWorld(World, X, Y, Z, ActorsToIgnore);

                Occupancy[Idx] = bBlocked ? 1 : 0;

                if (bBlocked)
                {
                    BlockedCount++;
                }

                if (World && (bDrawOccupiedCells || bDrawFreeCells))
                {
                    const FVector Center = CellToWorld(FIntVector(X, Y, Z));
                    const FVector Extent(CellSize * 0.45f);

                    if (bBlocked && bDrawOccupiedCells)
                    {
                        DrawDebugBox(
                            World,
                            Center,
                            Extent,
                            FColor::Red,
                            false,
                            DebugDrawTime,
                            0,
                            2.0f
                        );
                    }
                    else if (!bBlocked && bDrawFreeCells)
                    {
                        DrawDebugBox(
                            World,
                            Center,
                            Extent,
                            FColor::Green,
                            false,
                            DebugDrawTime,
                            0,
                            0.5f
                        );
                    }
                }
            }
        }
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("Occupancy built. TotalCells=%d Blocked=%d Free=%d"),
        TotalCells,
        BlockedCount,
        TotalCells - BlockedCount
    );
}