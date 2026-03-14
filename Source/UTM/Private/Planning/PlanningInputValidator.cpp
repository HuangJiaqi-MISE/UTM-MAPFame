#include "Planning/PlanningInputValidator.h"
#include "Planning/GridMap3D.h"
#include "GameFramework/Actor.h"

bool FPlanningInputValidator::ValidatePointInGrid(
    const FGridMap3D& GridMap,
    const FVector& WorldPos,
    const TCHAR* Label) const
{
    const FIntVector Cell = GridMap.WorldToCell(WorldPos);

    const FVector MinWorld = GridMap.GridOrigin;
    const FVector MaxWorld =
        GridMap.GridOrigin +
        FVector(
            GridMap.GridDim.X * GridMap.CellSize,
            GridMap.GridDim.Y * GridMap.CellSize,
            GridMap.GridDim.Z * GridMap.CellSize
        );

    if (!GridMap.IsInside(Cell.X, Cell.Y, Cell.Z))
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("%s is outside grid. WorldPos=(%.1f, %.1f, %.1f), Cell=(%d, %d, %d), GridWorldMin=(%.1f, %.1f, %.1f), GridWorldMax=(%.1f, %.1f, %.1f), GridDim=(%d, %d, %d), CellSize=%.1f"),
            Label,
            WorldPos.X, WorldPos.Y, WorldPos.Z,
            Cell.X, Cell.Y, Cell.Z,
            MinWorld.X, MinWorld.Y, MinWorld.Z,
            MaxWorld.X, MaxWorld.Y, MaxWorld.Z,
            GridMap.GridDim.X, GridMap.GridDim.Y, GridMap.GridDim.Z,
            GridMap.CellSize
        );
        return false;
    }

    return true;
}

bool FPlanningInputValidator::ValidatePointNotBlocked(
    const FGridMap3D& GridMap,
    const FVector& WorldPos,
    const TCHAR* Label) const
{
    const FIntVector Cell = GridMap.WorldToCell(WorldPos);

    if (!GridMap.IsInside(Cell.X, Cell.Y, Cell.Z))
    {
        return false;
    }

    if (GridMap.IsBlocked(Cell.X, Cell.Y, Cell.Z))
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("%s is blocked. WorldPos=(%.1f, %.1f, %.1f), Cell=(%d, %d, %d)"),
            Label,
            WorldPos.X, WorldPos.Y, WorldPos.Z,
            Cell.X, Cell.Y, Cell.Z
        );
        return false;
    }

    return true;
}

bool FPlanningInputValidator::ValidateStartGoalPair(
    const FGridMap3D& GridMap,
    const FVector& StartWorld,
    const FVector& GoalWorld,
    int32 PairId,
    const AActor* StartActor,
    const AActor* GoalActor) const
{
    UE_LOG(
        LogTemp,
        Warning,
        TEXT("Validating pair %d. StartActor=%s StartWorld=(%.1f, %.1f, %.1f), GoalActor=%s GoalWorld=(%.1f, %.1f, %.1f)"),
        PairId,
        StartActor ? *StartActor->GetName() : TEXT("None"),
        StartWorld.X, StartWorld.Y, StartWorld.Z,
        GoalActor ? *GoalActor->GetName() : TEXT("None"),
        GoalWorld.X, GoalWorld.Y, GoalWorld.Z
    );

    const bool bStartInGrid = ValidatePointInGrid(GridMap, StartWorld, TEXT("Start"));
    const bool bGoalInGrid = ValidatePointInGrid(GridMap, GoalWorld, TEXT("Goal"));

    if (!bStartInGrid || !bGoalInGrid)
    {
        UE_LOG(LogTemp, Error, TEXT("Pair %d failed range validation"), PairId);
        return false;
    }

    const bool bStartFree = ValidatePointNotBlocked(GridMap, StartWorld, TEXT("Start"));
    const bool bGoalFree = ValidatePointNotBlocked(GridMap, GoalWorld, TEXT("Goal"));

    if (!bStartFree || !bGoalFree)
    {
        UE_LOG(LogTemp, Error, TEXT("Pair %d failed occupancy validation"), PairId);
        return false;
    }

    return true;
}