#pragma once

#include "CoreMinimal.h"
#include "Planning/DroneMissionTypes.h"
#include "Planning/MissionSchedulerTypes.h"
#include "Planning/PlanningStatsTypes.h"
#include "Planning/PlannerTypes.h"

class FGridMap3D;

enum class EPlanningPipelineFailureStage : uint8
{
    None,
    InvalidRequest,
    Scheduler,
    EmptySchedule,
    Planner
};

struct FMultiAgentPlanningPipelineRequest
{
    const FGridMap3D* GridMap = nullptr;
    TArray<FDroneMissionConfig> RawMissions;
    EMissionSchedulerType SchedulerType = EMissionSchedulerType::Static;
    EPlannerType PlannerType = EPlannerType::CBS;
    FPlannerRuntimeConfig PlannerConfig;
    FString PlannerName;
};

struct FMultiAgentPlanningPipelineResult
{
    bool bSuccess = false;
    EPlanningPipelineFailureStage FailureStage = EPlanningPipelineFailureStage::None;
    FString FailureReason;
    TArray<FDroneMissionConfig> ScheduledMissions;
    TMap<int32, TArray<FVector>> PathsByMissionId;
    TArray<FSingleMissionTimingStats> MissionStats;
    double SolveTimeMs = 0.0;
};
