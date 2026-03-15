#pragma once

#include "CoreMinimal.h"

class FGridMap3D;

enum class EDiscreteAlignmentAction : uint8
{
    FollowPlan,
    HoldForDelay,
    SnapToPlanIndex,
    RecoverTowardPlan,
    HoldForAlignment,
    GoalHold
};

struct FDiscreteAlignmentSettings
{
    bool bEnabled = true;
    int32 SearchRadiusSteps = 6;
    int32 MaxSpatialErrorCells = 1;
    int32 MaxSnapAheadSteps = 3;
    bool bAllowRecoveryMoves = true;
    bool bHoldPositionOnFailure = true;
};

struct FDiscreteAlignmentResult
{
    bool bValid = false;
    bool bRequiresReplan = false;
    EDiscreteAlignmentAction Action = EDiscreteAlignmentAction::FollowPlan;
    FIntVector ObservedCell = FIntVector::ZeroValue;
    FIntVector ReferenceCell = FIntVector::ZeroValue;
    FIntVector NextCell = FIntVector::ZeroValue;
    int32 ReferencePlanIndex = 0;
    int32 NextPlanIndex = 0;
    int32 SpatialErrorCells = 0;
    int32 TemporalErrorSteps = 0;
    FString Reason;
};

class FDiscreteAlignmentManager
{
public:
    explicit FDiscreteAlignmentManager(const FDiscreteAlignmentSettings& InSettings = FDiscreteAlignmentSettings());

    void SetSettings(const FDiscreteAlignmentSettings& InSettings);

    FDiscreteAlignmentResult AlignStep(
        const FGridMap3D& GridMap,
        const TArray<FIntVector>& PlannedCells,
        int32 PreviousPlanIndex,
        int32 TimeStep,
        const FIntVector& ObservedCell,
        bool bDelayRequested) const;

    static const TCHAR* LexToString(EDiscreteAlignmentAction Action);

private:
    int32 FindBestPlanIndex(
        const TArray<FIntVector>& PlannedCells,
        int32 PreviousPlanIndex,
        int32 TimeStep,
        const FIntVector& ObservedCell) const;

    bool ComputeRecoveryCell(
        const FGridMap3D& GridMap,
        const FIntVector& FromCell,
        const FIntVector& TargetCell,
        FIntVector& OutRecoveryCell) const;

    static int32 ManhattanDistance(const FIntVector& A, const FIntVector& B);

    FDiscreteAlignmentSettings Settings;
};
