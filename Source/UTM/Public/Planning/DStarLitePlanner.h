#pragma once

#include "CoreMinimal.h"
#include "Planning/PathPlannerBase.h"

class FGridMap3D;

class FDStarLitePlanner : public IPathPlannerBase
{
public:
    virtual bool Plan(
        const FGridMap3D& GridMap,
        const FVector& StartWorld,
        const FVector& GoalWorld,
        TArray<FVector>& OutPath) override;

private:
    struct FNodeKey
    {
        float K1 = 0.f;
        float K2 = 0.f;

        bool operator<(const FNodeKey& Other) const
        {
            if (!FMath::IsNearlyEqual(K1, Other.K1))
            {
                return K1 < Other.K1;
            }
            return K2 < Other.K2;
        }

        bool operator>(const FNodeKey& Other) const
        {
            if (!FMath::IsNearlyEqual(K1, Other.K1))
            {
                return K1 > Other.K1;
            }
            return K2 > Other.K2;
        }

        bool operator==(const FNodeKey& Other) const
        {
            return FMath::IsNearlyEqual(K1, Other.K1)
                && FMath::IsNearlyEqual(K2, Other.K2);
        }
    };

    struct FOpenEntry
    {
        int32 NodeIdx = INDEX_NONE;
        FNodeKey Key;
    };

private:
    float Heuristic(const FIntVector& A, const FIntVector& B) const;
    FIntVector IndexToCell(const FGridMap3D& GridMap, int32 Idx) const;
    TArray<int32> GetNeighbors(const FGridMap3D& GridMap, int32 NodeIdx) const;
    float Cost(const FGridMap3D& GridMap, int32 FromIdx, int32 ToIdx) const;

    FNodeKey CalculateKey(
        const FGridMap3D& GridMap,
        int32 NodeIdx,
        int32 StartIdx,
        int32 GoalIdx,
        const TArray<float>& G,
        const TArray<float>& RHS,
        float Km) const;

    void RemoveFromOpen(int32 NodeIdx, TArray<FOpenEntry>& OpenList) const;
    void InsertOrUpdateOpen(
        const FGridMap3D& GridMap,
        int32 NodeIdx,
        int32 StartIdx,
        int32 GoalIdx,
        const TArray<float>& G,
        const TArray<float>& RHS,
        float Km,
        TArray<FOpenEntry>& OpenList) const;

    FOpenEntry GetTopOpen(const TArray<FOpenEntry>& OpenList) const;

    void UpdateVertex(
        const FGridMap3D& GridMap,
        int32 UIdx,
        int32 StartIdx,
        int32 GoalIdx,
        TArray<float>& G,
        TArray<float>& RHS,
        float Km,
        TArray<FOpenEntry>& OpenList) const;

    bool ComputeShortestPath(
        const FGridMap3D& GridMap,
        int32 StartIdx,
        int32 GoalIdx,
        TArray<float>& G,
        TArray<float>& RHS,
        float Km,
        TArray<FOpenEntry>& OpenList) const;

    bool ReconstructPath(
        const FGridMap3D& GridMap,
        int32 StartIdx,
        int32 GoalIdx,
        const TArray<float>& G,
        TArray<FVector>& OutPath) const;
};