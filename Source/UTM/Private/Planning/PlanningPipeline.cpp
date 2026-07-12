#include "Planning/PlanningPipeline.h"

#include "HAL/PlatformTime.h"
#include "Planning/MissionSchedulerRegistry.h"
#include "Planning/PlannerRegistry.h"

FMultiAgentPlanningPipelineResult FPlanningPipeline::RunMultiAgent(
    const FMultiAgentPlanningPipelineRequest& Request)
{
    FMultiAgentPlanningPipelineResult Result;

    const FString PlannerName = Request.PlannerName.IsEmpty()
        ? FPlannerRegistry::GetPlannerTypeName(Request.PlannerType)
        : Request.PlannerName;

    if (!Request.GridMap)
    {
        Result.FailureStage = EPlanningPipelineFailureStage::InvalidRequest;
        Result.FailureReason = TEXT("Planning pipeline request has no grid map");
        UE_LOG(LogTemp, Error, TEXT("%s"), *Result.FailureReason);
        return Result;
    }

    FMissionSchedulerContext SchedulerContext;
    SchedulerContext.GridMap = Request.GridMap;
    SchedulerContext.RequestedMissionCount = Request.RawMissions.Num();

    const bool bScheduled = FMissionSchedulerRegistry::BuildSchedule(
        Request.SchedulerType,
        SchedulerContext,
        Request.RawMissions,
        Result.ScheduledMissions);

    if (!bScheduled)
    {
        Result.FailureStage = EPlanningPipelineFailureStage::Scheduler;
        Result.FailureReason = FString::Printf(
            TEXT("Mission scheduler %s failed"),
            *FMissionSchedulerRegistry::GetSchedulerTypeName(Request.SchedulerType));
        UE_LOG(LogTemp, Error, TEXT("%s"), *Result.FailureReason);
        return Result;
    }

    UE_LOG(LogTemp, Warning, TEXT("Mission scheduler %s produced %d scheduled missions from %d raw missions"),
        *FMissionSchedulerRegistry::GetSchedulerTypeName(Request.SchedulerType),
        Result.ScheduledMissions.Num(),
        Request.RawMissions.Num());

    if (Result.ScheduledMissions.Num() <= 0)
    {
        Result.FailureStage = EPlanningPipelineFailureStage::EmptySchedule;
        Result.FailureReason = FString::Printf(
            TEXT("%s mission scheduler produced no missions to plan"),
            *PlannerName);
        UE_LOG(LogTemp, Warning, TEXT("%s"), *Result.FailureReason);
        return Result;
    }

    const double SolveStart = FPlatformTime::Seconds();
    Result.bSuccess = FPlannerRegistry::PlanMultiAgentMissions(
        Request.PlannerType,
        Request.PlannerConfig,
        *Request.GridMap,
        Result.ScheduledMissions,
        Result.PathsByMissionId);
    Result.SolveTimeMs = (FPlatformTime::Seconds() - SolveStart) * 1000.0;

    Result.MissionStats.Reserve(Result.ScheduledMissions.Num());
    for (const FDroneMissionConfig& Mission : Result.ScheduledMissions)
    {
        const TArray<FVector>* PathPoints = Result.PathsByMissionId.Find(Mission.MissionId);

        FSingleMissionTimingStats Item;
        Item.MissionId = Mission.MissionId;
        Item.bSuccess = (PathPoints != nullptr && PathPoints->Num() > 0);
        Item.PathPointCount = PathPoints ? PathPoints->Num() : 0;
        Item.SolveTimeMs = 0.0;
        Result.MissionStats.Add(Item);
    }

    if (!Result.bSuccess)
    {
        Result.FailureStage = EPlanningPipelineFailureStage::Planner;
        Result.FailureReason = FString::Printf(
            TEXT("%s failed or conflict unresolved in multi-agent planning pipeline"),
            *PlannerName);
        return Result;
    }

    Result.FailureStage = EPlanningPipelineFailureStage::None;
    return Result;
}
