#include "EditorServices/EditorGridService.h"

#include "Planning/GridMap3D.h"

void FEditorGridService::BuildGridForMissionEditing(
    const FEditorGridBuildRequest& Request)
{
    if (!Request.GridMap)
    {
        return;
    }

    Request.GridMap->GridOrigin = Request.GridOrigin;
    Request.GridMap->GridDim = Request.GridDim;
    Request.GridMap->CellSize = Request.CellSize;
    Request.GridMap->BuildOccupancyGrid(
        Request.World,
        Request.IgnoreActors,
        Request.bDrawOccupiedCells,
        Request.bDrawFreeCells,
        Request.DebugDrawTime);

    UE_LOG(LogTemp, Warning, TEXT("EditorBuildGridForMissionEditing done"));
}
