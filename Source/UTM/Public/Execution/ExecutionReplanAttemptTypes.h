#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"

struct FExecutionReplanAttemptSpec
{
    bool bGlobalReplan = false;
    int32 AttemptIndex = 0;
    int32 AttemptCount = 1;
    int32 SpatialRadiusCells = 0;
    int32 LookaheadSteps = 0;
    bool bCheckStaticUTMSafety = false;
};

struct FExecutionReplanAttemptInput
{
    const FExecutionSnapshot* Snapshot = nullptr;
    TSet<int32> CandidateMissionIdSet;
    TSet<int32> ForcedAnchorMissionIdSet;
    FExecutionReplanAttemptSpec Spec;
};

enum class EExecutionReplanAttemptStatus : uint8
{
    Success,
    EmptyCandidateSet,
    GridBuildFailed,
    MissionBuildFailed,
    PlannerFailed,
    InvalidReplannedPath,
    PostCheckFailed
};

struct FExecutionReplanAttemptResult
{
    EExecutionReplanAttemptStatus Status = EExecutionReplanAttemptStatus::PlannerFailed;
    FString FailureReason;
    int32 FailedMissionId = INDEX_NONE;
    TArray<int32> CandidateMissionIds;
    TArray<int32> AnchorMissionIds;
    TSet<int32> AnchorMissionIdSet;
    int32 StaticAnchorBlockedCellCount = 0;
    TMap<int32, TArray<FIntVector>> ReplannedCellPathsByMission;
    FExecutionPredictedConflict PostCheckConflict;
    int32 PostCheckConflictOffset = 0;
};
