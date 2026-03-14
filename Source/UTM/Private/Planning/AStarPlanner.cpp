#include "Planning/AStarPlanner.h"
#include "Planning/GridMap3D.h"

float FAStarPlanner::Heuristic(const FIntVector& A, const FIntVector& B) const
{
    return
        FMath::Abs(A.X - B.X) +
        FMath::Abs(A.Y - B.Y) +
        FMath::Abs(A.Z - B.Z);
}

FIntVector FAStarPlanner::IndexToCell(const FGridMap3D& GridMap, int32 Idx) const
{
    const int32 XY = GridMap.GridDim.X * GridMap.GridDim.Y;
    const int32 Z = Idx / XY;
    const int32 Rem = Idx % XY;
    const int32 Y = Rem / GridMap.GridDim.X;
    const int32 X = Rem % GridMap.GridDim.X;

    return FIntVector(X, Y, Z);
}

bool FAStarPlanner::Plan(
    const FGridMap3D& GridMap,
    const FVector& StartWorld,
    const FVector& GoalWorld,
    TArray<FVector>& OutPath)
{
    OutPath.Reset();

    const FIntVector StartCell = GridMap.WorldToCell(StartWorld);
    const FIntVector GoalCell = GridMap.WorldToCell(GoalWorld);

    if (!GridMap.IsInside(StartCell.X, StartCell.Y, StartCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("StartCell is outside grid"));
        return false;
    }

    if (!GridMap.IsInside(GoalCell.X, GoalCell.Y, GoalCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("GoalCell is outside grid"));
        return false;
    }

    if (GridMap.IsBlocked(StartCell.X, StartCell.Y, StartCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("StartCell is blocked"));
        return false;
    }

    if (GridMap.IsBlocked(GoalCell.X, GoalCell.Y, GoalCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("GoalCell is blocked"));
        return false;
    }

    const int32 TotalCells = GridMap.GridDim.X * GridMap.GridDim.Y * GridMap.GridDim.Z;
    const int32 StartIdx = GridMap.ToIndex(StartCell.X, StartCell.Y, StartCell.Z);
    const int32 GoalIdx = GridMap.ToIndex(GoalCell.X, GoalCell.Y, GoalCell.Z);

    TArray<float> GScore;
    TArray<int32> Parent;
    TArray<uint8> Closed;

    GScore.Init(FLT_MAX, TotalCells);
    Parent.Init(INDEX_NONE, TotalCells);
    Closed.Init(0, TotalCells);

    TArray<int32> OpenSet;
    TSet<int32> InOpen;

    GScore[StartIdx] = 0.f;
    OpenSet.Add(StartIdx);
    InOpen.Add(StartIdx);

    while (OpenSet.Num() > 0)
    {
        int32 BestPos = 0;
        float BestF = FLT_MAX;

        for (int32 i = 0; i < OpenSet.Num(); ++i)
        {
            const int32 Idx = OpenSet[i];
            const float F = GScore[Idx] + Heuristic(IndexToCell(GridMap, Idx), GoalCell);

            if (F < BestF)
            {
                BestF = F;
                BestPos = i;
            }
        }

        const int32 Current = OpenSet[BestPos];
        OpenSet.RemoveAtSwap(BestPos);
        InOpen.Remove(Current);

        if (Current == GoalIdx)
        {
            for (int32 P = GoalIdx; P != INDEX_NONE; P = Parent[P])
            {
                OutPath.Insert(GridMap.CellToWorld(IndexToCell(GridMap, P)), 0);
            }
            return true;
        }

        Closed[Current] = 1;

        const FIntVector C = IndexToCell(GridMap, Current);
        const FIntVector Dirs[6] =
        {
            FIntVector(1, 0, 0),
            FIntVector(-1, 0, 0),
            FIntVector(0, 1, 0),
            FIntVector(0,-1, 0),
            FIntVector(0, 0, 1),
            FIntVector(0, 0,-1)
        };

        for (const FIntVector& D : Dirs)
        {
            const FIntVector NCell = C + D;

            if (!GridMap.IsInside(NCell.X, NCell.Y, NCell.Z))
            {
                continue;
            }

            if (GridMap.IsBlocked(NCell.X, NCell.Y, NCell.Z))
            {
                continue;
            }

            const int32 NIdx = GridMap.ToIndex(NCell.X, NCell.Y, NCell.Z);

            if (Closed[NIdx])
            {
                continue;
            }

            const float TentativeG = GScore[Current] + 1.f;

            if (TentativeG < GScore[NIdx])
            {
                GScore[NIdx] = TentativeG;
                Parent[NIdx] = Current;

                if (!InOpen.Contains(NIdx))
                {
                    OpenSet.Add(NIdx);
                    InOpen.Add(NIdx);
                }
            }
        }
    }

    return false;
}