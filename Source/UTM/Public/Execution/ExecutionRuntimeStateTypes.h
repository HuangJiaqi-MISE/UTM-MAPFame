#pragma once

#include "CoreMinimal.h"
#include "ExecutionRuntimeStateTypes.generated.h"

/*
模式 A：全局随机延迟,适合快速看效果。
模式 B：指定 agent 延迟,比如只让 Mission 2 延迟,能明确观察单个执行异常如何破坏整体 schedule。
模式 C：指定 timestep 延迟,比如让 Agent 2 在 t=4, 7, 8 原地等待,让实验可复现、可解释。
*/
UENUM(BlueprintType)
enum class EExecutionDelayMode : uint8
{
    RandomGlobal        UMETA(DisplayName = "Random Global"),
    PerAgentProbability UMETA(DisplayName = "Per-Agent Probability"),
    ScriptedTimesteps   UMETA(DisplayName = "Scripted Timesteps")
};

UENUM(BlueprintType)
enum class EExecutionReplanMode : uint8
{
    Disabled          UMETA(DisplayName = "Disabled"),
    LocalConflictSet  UMETA(DisplayName = "Local Conflict Set"),
    GlobalUnfinished  UMETA(DisplayName = "Global Unfinished")
};

USTRUCT(BlueprintType)
struct FExecutionReplanTimingStats
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Replan Timing")
    int32 LocalAttemptCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Replan Timing")
    double LocalTotalTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Replan Timing")
    double LocalMaxTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Replan Timing")
    int32 GlobalAttemptCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Replan Timing")
    double GlobalTotalTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Replan Timing")
    double GlobalMaxTimeMs = 0.0;
};

USTRUCT(BlueprintType)
struct FExecutionAgentState
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 MissionId = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    TArray<FIntVector> PlannedCells;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    TArray<FIntVector> ActualCells;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 ExecutedPlanIndex = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 TotalDelaySteps = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    bool bFinished = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment")
    int32 AlignmentCorrectionCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment")
    int32 AlignmentHoldCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment")
    int32 AlignmentConflictHoldCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment")
    int32 AlignmentSnapCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment")
    int32 AlignmentReplanRequestCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment")
    int32 AlignmentSuccessfulReplanCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment")
    int32 MaxAlignmentSpatialError = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment")
    int32 MaxAlignmentTemporalError = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment")
    bool bAlignmentLost = false;

    FIntVector DisplayFromCell = FIntVector::ZeroValue;
    FIntVector DisplayToCell = FIntVector::ZeroValue;
    FIntVector LastObservedCell = FIntVector::ZeroValue;
    FIntVector GoalCell = FIntVector::ZeroValue;
    FVector GoalWorld = FVector::ZeroVector;
    int32 ConsecutiveConflictHoldCount = 0;
    int32 ConsecutiveSafetyGateHoldCount = 0;
    FString LastAlignmentAction;
};

USTRUCT(BlueprintType)
struct FExecutionConflict
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 TimeStep = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 AgentA = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 AgentB = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    bool bIsEdgeConflict = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    FIntVector Cell = FIntVector::ZeroValue;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    FIntVector FromA = FIntVector::ZeroValue;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    FIntVector ToA = FIntVector::ZeroValue;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    FIntVector FromB = FIntVector::ZeroValue;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    FIntVector ToB = FIntVector::ZeroValue;
};

USTRUCT(BlueprintType)
struct FAgentDelayConfig
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution")
    int32 MissionId = INDEX_NONE;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float DelayProbability = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution")
    TArray<int32> ForcedDelaySteps;
};
