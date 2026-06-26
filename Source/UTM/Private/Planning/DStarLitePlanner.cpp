#include "Planning/DStarLitePlanner.h"
#include "Planning/GridMap3D.h"

namespace
{
    constexpr float DStarLiteInfCost = 1e9f;
}

float FDStarLitePlanner::Heuristic(const FIntVector& A, const FIntVector& B) const
{
    return
        FMath::Abs(A.X - B.X) +
        FMath::Abs(A.Y - B.Y) +
        FMath::Abs(A.Z - B.Z);
}

FIntVector FDStarLitePlanner::IndexToCell(const FGridMap3D& GridMap, int32 Idx) const
{
    const int32 XY = GridMap.GridDim.X * GridMap.GridDim.Y;
    const int32 Z = Idx / XY;
    const int32 Rem = Idx % XY;
    const int32 Y = Rem / GridMap.GridDim.X;
    const int32 X = Rem % GridMap.GridDim.X;

    return FIntVector(X, Y, Z);
}

TArray<int32> FDStarLitePlanner::GetNeighbors(const FGridMap3D& GridMap, int32 NodeIdx) const
{
    TArray<int32> Result;

    const FIntVector C = IndexToCell(GridMap, NodeIdx);
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
        const FIntVector N = C + D;
        if (!GridMap.IsInside(N.X, N.Y, N.Z))
        {
            continue;
        }

        if (GridMap.IsBlocked(N.X, N.Y, N.Z))
        {
            continue;
        }

        Result.Add(GridMap.ToIndex(N.X, N.Y, N.Z));
    }

    return Result;
}

float FDStarLitePlanner::Cost(const FGridMap3D& GridMap, int32 FromIdx, int32 ToIdx) const
{
    const FIntVector A = IndexToCell(GridMap, FromIdx);
    const FIntVector B = IndexToCell(GridMap, ToIdx);

    if (GridMap.IsBlocked(B.X, B.Y, B.Z))
    {
        return DStarLiteInfCost;
    }

    return 1.f;
}

FDStarLitePlanner::FNodeKey FDStarLitePlanner::CalculateKey(
    const FGridMap3D& GridMap,
    int32 NodeIdx,
    int32 StartIdx,
    int32 GoalIdx,
    const TArray<float>& G,
    const TArray<float>& RHS,
    float Km) const
{
    const float MinVal = FMath::Min(G[NodeIdx], RHS[NodeIdx]);
    const float H = Heuristic(IndexToCell(GridMap, StartIdx), IndexToCell(GridMap, NodeIdx));

    FNodeKey Key;
    Key.K1 = MinVal + H + Km;
    Key.K2 = MinVal;
    return Key;
}

void FDStarLitePlanner::RemoveFromOpen(int32 NodeIdx, TArray<FOpenEntry>& OpenList) const
{
    OpenList.RemoveAll([NodeIdx](const FOpenEntry& Entry)
        {
            return Entry.NodeIdx == NodeIdx;
        });
}

void FDStarLitePlanner::InsertOrUpdateOpen(
    const FGridMap3D& GridMap,
    int32 NodeIdx,
    int32 StartIdx,
    int32 GoalIdx,
    const TArray<float>& G,
    const TArray<float>& RHS,
    float Km,
    TArray<FOpenEntry>& OpenList) const
{
    RemoveFromOpen(NodeIdx, OpenList);

    FOpenEntry Entry;
    Entry.NodeIdx = NodeIdx;
    Entry.Key = CalculateKey(GridMap, NodeIdx, StartIdx, GoalIdx, G, RHS, Km);

    OpenList.Add(Entry);
}

FDStarLitePlanner::FOpenEntry FDStarLitePlanner::GetTopOpen(const TArray<FOpenEntry>& OpenList) const
{
    check(OpenList.Num() > 0);

    int32 BestIdx = 0;
    for (int32 i = 1; i < OpenList.Num(); ++i)
    {
        if (OpenList[i].Key < OpenList[BestIdx].Key)
        {
            BestIdx = i;
        }
    }

    return OpenList[BestIdx];
}

void FDStarLitePlanner::UpdateVertex(
    const FGridMap3D& GridMap,
    int32 UIdx,
    int32 StartIdx,
    int32 GoalIdx,
    TArray<float>& G,
    TArray<float>& RHS,
    float Km,
    TArray<FOpenEntry>& OpenList) const
{
    if (UIdx != GoalIdx)
    {
        float MinRhs = DStarLiteInfCost;
        const TArray<int32> Succ = GetNeighbors(GridMap, UIdx);

        for (int32 SIdx : Succ)
        {
            const float Candidate = Cost(GridMap, UIdx, SIdx) + G[SIdx];
            MinRhs = FMath::Min(MinRhs, Candidate);
        }

        RHS[UIdx] = MinRhs;
    }

    RemoveFromOpen(UIdx, OpenList);

    if (!FMath::IsNearlyEqual(G[UIdx], RHS[UIdx]))
    {
        InsertOrUpdateOpen(GridMap, UIdx, StartIdx, GoalIdx, G, RHS, Km, OpenList);
    }
}

bool FDStarLitePlanner::ComputeShortestPath(
    const FGridMap3D& GridMap,
    int32 StartIdx,
    int32 GoalIdx,
    TArray<float>& G,
    TArray<float>& RHS,
    float Km,
    TArray<FOpenEntry>& OpenList) const
{
    int32 GuardCounter = 0;
    const int32 GuardLimit = GridMap.GridDim.X * GridMap.GridDim.Y * GridMap.GridDim.Z * 20;

    while (OpenList.Num() > 0)
    {
        if (++GuardCounter > GuardLimit)
        {
            UE_LOG(LogTemp, Error, TEXT("D* Lite exceeded guard limit"));
            return false;
        }

        const FOpenEntry Top = GetTopOpen(OpenList);
        const FNodeKey StartKey = CalculateKey(GridMap, StartIdx, StartIdx, GoalIdx, G, RHS, Km);

        const bool bShouldStop =
            !(Top.Key < StartKey) &&
            FMath::IsNearlyEqual(RHS[StartIdx], G[StartIdx]);

        if (bShouldStop)
        {
            break;
        }

        RemoveFromOpen(Top.NodeIdx, OpenList);

        const FNodeKey NewKey = CalculateKey(GridMap, Top.NodeIdx, StartIdx, GoalIdx, G, RHS, Km);

        if (Top.Key < NewKey)
        {
            OpenList.Add({ Top.NodeIdx, NewKey });
        }
        else if (G[Top.NodeIdx] > RHS[Top.NodeIdx])
        {
            G[Top.NodeIdx] = RHS[Top.NodeIdx];

            const TArray<int32> Pred = GetNeighbors(GridMap, Top.NodeIdx);
            for (int32 PIdx : Pred)
            {
                UpdateVertex(GridMap, PIdx, StartIdx, GoalIdx, G, RHS, Km, OpenList);
            }
        }
        else
        {
            G[Top.NodeIdx] = DStarLiteInfCost;

            TArray<int32> Pred = GetNeighbors(GridMap, Top.NodeIdx);
            Pred.Add(Top.NodeIdx);

            for (int32 PIdx : Pred)
            {
                UpdateVertex(GridMap, PIdx, StartIdx, GoalIdx, G, RHS, Km, OpenList);
            }
        }
    }

    return RHS[StartIdx] < DStarLiteInfCost;
}

bool FDStarLitePlanner::ReconstructPath(
    const FGridMap3D& GridMap,
    int32 StartIdx,
    int32 GoalIdx,
    const TArray<float>& G,
    TArray<FVector>& OutPath) const
{
    OutPath.Reset();

    int32 Current = StartIdx;
    OutPath.Add(GridMap.CellToWorld(IndexToCell(GridMap, Current)));

    int32 GuardCounter = 0;
    const int32 GuardLimit = GridMap.GridDim.X * GridMap.GridDim.Y * GridMap.GridDim.Z * 4;

    while (Current != GoalIdx)
    {
        if (++GuardCounter > GuardLimit)
        {
            UE_LOG(LogTemp, Error, TEXT("D* Lite path reconstruction exceeded guard limit"));
            return false;
        }

        const TArray<int32> Neighbors = GetNeighbors(GridMap, Current);

        float BestCost = DStarLiteInfCost;
        int32 BestNext = INDEX_NONE;

        for (int32 NIdx : Neighbors)
        {
            const float Candidate = Cost(GridMap, Current, NIdx) + G[NIdx];
            if (Candidate < BestCost)
            {
                BestCost = Candidate;
                BestNext = NIdx;
            }
        }

        if (BestNext == INDEX_NONE || BestCost >= DStarLiteInfCost)
        {
            UE_LOG(LogTemp, Error, TEXT("D* Lite failed to reconstruct a valid path"));
            return false;
        }

        Current = BestNext;
        OutPath.Add(GridMap.CellToWorld(IndexToCell(GridMap, Current)));
    }

    return true;
}

bool FDStarLitePlanner::Plan(
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
        UE_LOG(LogTemp, Error, TEXT("D* Lite: StartCell is outside grid"));
        return false;
    }

    if (!GridMap.IsInside(GoalCell.X, GoalCell.Y, GoalCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("D* Lite: GoalCell is outside grid"));
        return false;
    }

    if (GridMap.IsBlocked(StartCell.X, StartCell.Y, StartCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("D* Lite: StartCell is blocked"));
        return false;
    }

    if (GridMap.IsBlocked(GoalCell.X, GoalCell.Y, GoalCell.Z))
    {
        UE_LOG(LogTemp, Error, TEXT("D* Lite: GoalCell is blocked"));
        return false;
    }

    const int32 TotalCells = GridMap.GridDim.X * GridMap.GridDim.Y * GridMap.GridDim.Z;
    const int32 StartIdx = GridMap.ToIndex(StartCell.X, StartCell.Y, StartCell.Z);
    const int32 GoalIdx = GridMap.ToIndex(GoalCell.X, GoalCell.Y, GoalCell.Z);

    TArray<float> G;
    TArray<float> RHS;
    G.Init(DStarLiteInfCost, TotalCells);
    RHS.Init(DStarLiteInfCost, TotalCells);

    RHS[GoalIdx] = 0.f;

    float Km = 0.f;
    TArray<FOpenEntry> OpenList;
    InsertOrUpdateOpen(GridMap, GoalIdx, StartIdx, GoalIdx, G, RHS, Km, OpenList);

    const bool bSuccess = ComputeShortestPath(GridMap, StartIdx, GoalIdx, G, RHS, Km, OpenList);
    if (!bSuccess)
    {
        UE_LOG(LogTemp, Error, TEXT("D* Lite failed to compute shortest path"));
        return false;
    }

    return ReconstructPath(GridMap, StartIdx, GoalIdx, G, OutPath);
}
