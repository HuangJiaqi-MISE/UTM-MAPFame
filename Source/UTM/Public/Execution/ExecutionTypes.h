#pragma once

#include "CoreMinimal.h"

enum class EExecutionPolicyAction : uint8
{
    FollowPlan,
    HoldForDelay,
    SnapToPlanIndex,
    RecoverTowardPlan,
    HoldForAlignment,
    HoldForPredictedConflict,
    HoldForSafetyGate,
    HoldForReplan,
    GoalHold
};

enum class EExecutionPolicyReplanMode : uint8
{
    Disabled,
    LocalConflictSet,
    GlobalUnfinished
};

enum class EExecutionPredictedConflictType : uint8
{
    None,
    Vertex,
    Edge,
    ProtectionFootprint,
    Downwash
};

struct FExecutionAgentSnapshot
{
    int32 MissionId = INDEX_NONE;
    bool bFinished = false;
    bool bDelayRequested = false;
    FIntVector ObservedCell = FIntVector::ZeroValue;
    FVector ObservedWorld = FVector::ZeroVector;
    FIntVector GoalCell = FIntVector::ZeroValue;
    FVector GoalWorld = FVector::ZeroVector;
    int32 TimeStep = 0;
    int32 ExecutedPlanIndex = 0;
    int32 ConsecutiveConflictHoldCount = 0;
    int32 ConsecutiveSafetyGateHoldCount = 0;
    TArray<FIntVector> PlannedCells;
};

struct FExecutionSnapshot
{
    int32 TimeStep = 0;
    int32 TotalReplanCount = 0;
    TArray<FExecutionAgentSnapshot> Agents;
};

struct FExecutionStepDecision
{
    int32 MissionId = INDEX_NONE;
    bool bValid = false;
    bool bRequiresReplan = false;
    EExecutionPolicyAction Action = EExecutionPolicyAction::FollowPlan;
    FIntVector ObservedCell = FIntVector::ZeroValue;
    FIntVector ReferenceCell = FIntVector::ZeroValue;
    FIntVector TargetCell = FIntVector::ZeroValue;
    int32 ReferencePlanIndex = 0;
    int32 TargetPlanIndex = 0;
    int32 SpatialErrorCells = 0;
    int32 TemporalErrorSteps = 0;
    FString Reason;
};

struct FExecutionPredictedConflict
{
    EExecutionPredictedConflictType Type = EExecutionPredictedConflictType::None;
    int32 AgentA = INDEX_NONE;
    int32 AgentB = INDEX_NONE;
    FIntVector Cell = FIntVector::ZeroValue;
    FIntVector AgentAFromCell = FIntVector::ZeroValue;
    FIntVector AgentAToCell = FIntVector::ZeroValue;
    FIntVector AgentBFromCell = FIntVector::ZeroValue;
    FIntVector AgentBToCell = FIntVector::ZeroValue;
};

struct FExecutionReplanPolicySettings
{
    EExecutionPolicyReplanMode Mode = EExecutionPolicyReplanMode::GlobalUnfinished;
    int32 MaxReplanCount = 0;
    int32 ConflictHoldThresholdForReplan = 2;
};

struct FExecutionReplanRequest
{
    bool bShouldReplan = false;
    bool bGlobalReplan = false;
    TSet<int32> RequestedMissionIds;
    FString Reason;
};
