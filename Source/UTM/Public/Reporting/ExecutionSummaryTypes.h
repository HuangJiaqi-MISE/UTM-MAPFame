#pragma once

#include "CoreMinimal.h"
#include "ExecutionSummaryTypes.generated.h"

USTRUCT(BlueprintType)
struct FExecutionAgentSummary
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 MissionId = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 PlannedCellCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 ActualCellCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 PlannedMakespan = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 ActualMakespan = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 TotalDelaySteps = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 FirstMismatchTime = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    bool bReachedGoal = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentCorrectionCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentHoldCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentConflictHoldCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentSnapCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentReplanRequestCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentSuccessfulReplanCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 MaxAlignmentSpatialError = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 MaxAlignmentTemporalError = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    bool bAlignmentLost = false;
};

USTRUCT(BlueprintType)
struct FExecutionSummary
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 AgentCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 CompletedAgentCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 PlannedMakespan = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 ActualMakespan = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 TotalDelaySteps = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 VertexConflictCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 EdgeConflictCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 FirstConflictTime = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 UTMStaticConflictCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 UTMProtectionConflictCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 UTMDownwashConflictCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 FirstUTMConflictTime = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentCorrectionCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentHoldCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentConflictHoldCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentSnapCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentReplanRequestCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Alignment Summary")
    int32 AlignmentSuccessfulReplanCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    TArray<FExecutionAgentSummary> AgentSummaries;
};
