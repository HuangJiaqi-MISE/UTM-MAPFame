#pragma once

#include "CoreMinimal.h"
#include "Planning/PathPlannerBase.h"
#include "Planning/TemporalNoFlyZoneTypes.h"

class FGridMap3D;

class FSIPPPlanner : public IPathPlannerBase
{
public:
    explicit FSIPPPlanner(const TArray<FTemporalNoFlyZoneConfig>& InNoFlyZoneConfigs = TArray<FTemporalNoFlyZoneConfig>());

    virtual bool Plan(
        const FGridMap3D& GridMap,
        const FVector& StartWorld,
        const FVector& GoalWorld,
        TArray<FVector>& OutPath) override;

private:
    static constexpr int32 InfiniteIntervalEnd = 1 << 29;

    struct FSippInterval
    {
        int32 Start = 0;
        int32 End = 0;
    };

    struct FSippStateKey
    {
        FIntVector Cell = FIntVector::ZeroValue;
        int32 IntervalStart = 0;
        int32 IntervalEnd = 0;

        bool operator==(const FSippStateKey& Other) const
        {
            return Cell == Other.Cell
                && IntervalStart == Other.IntervalStart
                && IntervalEnd == Other.IntervalEnd;
        }

        FORCEINLINE friend uint32 GetTypeHash(const FSippStateKey& State)
        {
            uint32 Hash = GetTypeHash(State.Cell);
            Hash = HashCombine(Hash, GetTypeHash(State.IntervalStart));
            Hash = HashCombine(Hash, GetTypeHash(State.IntervalEnd));
            return Hash;
        }
    };

    struct FSippParentInfo
    {
        FSippStateKey ParentState;
        int32 ParentArrivalTime = 0;
        bool bHasParent = false;
    };

    struct FSippOpenEntry
    {
        FSippStateKey State;
        int32 ArrivalTime = 0;
        float FScore = 0.f;

        bool operator<(const FSippOpenEntry& Other) const
        {
            return FScore < Other.FScore;
        }
    };

private:
    float Heuristic(const FIntVector& A, const FIntVector& B) const;

    void BuildBlockedIntervals(
        const FGridMap3D& GridMap,
        TMap<FIntVector, TArray<FSippInterval>>& OutBlockedIntervalsByCell,
        int32& OutMaxBlockedTime) const;

    void MergeIntervals(TArray<FSippInterval>& InOutIntervals) const;

    const TArray<FSippInterval>& GetSafeIntervalsForCell(
        const FIntVector& Cell,
        const TMap<FIntVector, TArray<FSippInterval>>& BlockedIntervalsByCell,
        TMap<FIntVector, TArray<FSippInterval>>& InOutSafeIntervalCache) const;

    bool ReconstructCellPath(
        const TMap<FSippStateKey, FSippParentInfo>& ParentByState,
        const TMap<FSippStateKey, int32>& ArrivalByState,
        const FSippStateKey& GoalState,
        TArray<FIntVector>& OutCellPath) const;

    TArray<FVector> CellPathToWorldPath(
        const FGridMap3D& GridMap,
        const TArray<FIntVector>& CellPath) const;

private:
    TArray<FTemporalNoFlyZoneConfig> NoFlyZoneConfigs;
};
