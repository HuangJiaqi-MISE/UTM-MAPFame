#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionReplanAttemptTypes.h"
#include "Planning/DroneMissionTypes.h"

enum class EExecutionReplanCoordinatorEventType : uint8
{
    CandidateSetExpanded,
    PostCheckFailed,
    TargetedRetryExpanded,
    AttemptSucceeded
};

struct FExecutionReplanCoordinatorEvent
{
    EExecutionReplanCoordinatorEventType Type = EExecutionReplanCoordinatorEventType::CandidateSetExpanded;
    bool bGlobalReplan = false;
    int32 AttemptIndex = 0;
    int32 AttemptCount = 1;
    int32 SpatialRadiusCells = 0;
    int32 LookaheadSteps = 0;
    int32 ActiveRequestedMissionCount = 0;
    int32 CandidateMissionCount = 0;
    int32 AnchorMissionCount = 0;
    int32 StaticAnchorBlockedCellCount = 0;
    int32 TargetedRetryIndex = 0;
    int32 MaxTargetedRetryCount = 0;
    int32 PreviousCandidateMissionCount = 0;
    int32 PreviousForcedAnchorMissionCount = 0;
    int32 ForcedAnchorMissionCount = 0;
    int32 ReplannedMissionCount = 0;
    int32 ExecutionTimeStep = 0;
    int32 TotalReplanCount = 0;
    FExecutionPredictedConflict Conflict;
    int32 ConflictOffset = 0;
};

struct FExecutionReplanCoordinatorRequest
{
    const FExecutionSnapshot* Snapshot = nullptr;
    const TMap<int32, FDroneMissionConfig>* MissionConfigsById = nullptr;
    TSet<int32> RequestedMissionIds;
    bool bGlobalReplan = false;
    bool bCheckStaticUTMSafety = false;
    int32 MaxExpansionRounds = 1;
    int32 BaseSpatialRadiusCells = 0;
    int32 BaseLookaheadSteps = 0;
    int32 ExecutionTimeStep = 0;
    int32 CurrentTotalReplanCount = 0;
};

struct FExecutionReplanCoordinatorCallbacks
{
    TFunction<bool(const FExecutionReplanAttemptInput&, FExecutionReplanAttemptResult&)> RunAttempt;
    TFunction<bool(const FExecutionReplanAttemptResult&, TSet<int32>&)> ApplyAttemptResult;
    TFunction<void(const FExecutionReplanCoordinatorEvent&)> OnEvent;
};

struct FExecutionReplanCoordinatorResult
{
    bool bSuccess = false;
    TSet<int32> ReplannedMissionIds;
    int32 AppliedReplanCount = 0;
    int32 TimedAttemptCount = 0;
    double TotalAttemptTimeMs = 0.0;
    double MaxAttemptTimeMs = 0.0;
};

class FExecutionReplanCoordinator
{
public:
    static FExecutionReplanCoordinatorResult Run(
        const FExecutionReplanCoordinatorRequest& Request,
        const FExecutionReplanCoordinatorCallbacks& Callbacks);
};
