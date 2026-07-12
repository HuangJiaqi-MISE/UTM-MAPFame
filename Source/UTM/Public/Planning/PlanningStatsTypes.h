#pragma once

#include "CoreMinimal.h"
#include "PlanningStatsTypes.generated.h"

// 用于记录单个任务的规划统计数据
USTRUCT(BlueprintType)
struct FSingleMissionTimingStats
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    int32 MissionId = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    bool bSuccess = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    int32 PathPointCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double SolveTimeMs = 0.0;
};

// 用于记录整体规划过程的统计数据，包括每个任务的统计
USTRUCT(BlueprintType)
struct FPlanningTimingStats
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    FString PlannerName;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    bool bMultiAgent = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    bool bSuccess = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    int32 MissionCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double TotalTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double BuildGridTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double InputPreparationTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double SolveTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double PostProcessTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    TArray<FSingleMissionTimingStats> MissionStats;
};
