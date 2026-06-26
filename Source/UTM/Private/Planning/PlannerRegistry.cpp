#include "Planning/PlannerRegistry.h"

#include "Planning/AStarPlanner.h"
#include "Planning/CBSPlanner.h"
#include "Planning/DStarLitePlanner.h"
#include "Planning/ECBSPlanner.h"
#include "Planning/LaCAMPlanner.h"
#include "Planning/LaCAMUTM.h"
#include "Planning/PBSPlanner.h"
#include "Planning/SIPPPlanner.h"

bool FPlannerRegistry::IsMultiAgentPlannerType(EPlannerType PlannerType)
{
    return PlannerType == EPlannerType::CBS
        || PlannerType == EPlannerType::ECBS
        || PlannerType == EPlannerType::PBS
        || PlannerType == EPlannerType::LaCAM
        || PlannerType == EPlannerType::LaCAMUTM;
}

FString FPlannerRegistry::GetPlannerTypeName(EPlannerType PlannerType)
{
    switch (PlannerType)
    {
    case EPlannerType::AStar:
        return TEXT("A*");
    case EPlannerType::SIPP:
        return TEXT("SIPP");
    case EPlannerType::DStarLite:
        return TEXT("D* Lite");
    case EPlannerType::JPS:
        return TEXT("JPS");
    case EPlannerType::CBS:
        return TEXT("CBS");
    case EPlannerType::ECBS:
        return TEXT("ECBS");
    case EPlannerType::PBS:
        return TEXT("PBS");
    case EPlannerType::LaCAM:
        return TEXT("LaCAM");
    case EPlannerType::LaCAMUTM:
        return TEXT("LaCAM-UTM");
    default:
        return TEXT("Unknown");
    }
}

TUniquePtr<IPathPlannerBase> FPlannerRegistry::CreateSingleAgentPlanner(
    EPlannerType PlannerType,
    const FPlannerRuntimeConfig& Config)
{
    switch (PlannerType)
    {
    case EPlannerType::AStar:
        UE_LOG(LogTemp, Warning, TEXT("Planner created: FAStarPlanner"));
        return MakeUnique<FAStarPlanner>();

    case EPlannerType::SIPP:
        UE_LOG(LogTemp, Warning, TEXT("Planner created: FSIPPPlanner"));
        return MakeUnique<FSIPPPlanner>(Config.NoFlyZoneConfigs);

    case EPlannerType::DStarLite:
        UE_LOG(LogTemp, Warning, TEXT("Planner created: FDStarLitePlanner"));
        return MakeUnique<FDStarLitePlanner>();

    case EPlannerType::JPS:
        UE_LOG(LogTemp, Warning, TEXT("JPS planner not implemented yet. Fallback to A*"));
        UE_LOG(LogTemp, Warning, TEXT("Planner created: FAStarPlanner"));
        return MakeUnique<FAStarPlanner>();

    case EPlannerType::CBS:
    case EPlannerType::ECBS:
    case EPlannerType::PBS:
    case EPlannerType::LaCAM:
    case EPlannerType::LaCAMUTM:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("%s is a multi-agent planner and cannot be created as a single-agent planner"),
            *GetPlannerTypeName(PlannerType));
        return nullptr;

    default:
        UE_LOG(LogTemp, Warning, TEXT("Unknown planner type. Fallback to A*"));
        UE_LOG(LogTemp, Warning, TEXT("Planner created: FAStarPlanner"));
        return MakeUnique<FAStarPlanner>();
    }
}

TUniquePtr<IMultiAgentPlannerBase> FPlannerRegistry::CreateMultiAgentPlanner(
    EPlannerType PlannerType,
    const FPlannerRuntimeConfig& Config)
{
    switch (PlannerType)
    {
    case EPlannerType::CBS:
        return MakeUnique<FCBSPlanner>();

    case EPlannerType::ECBS:
        return MakeUnique<FECBSPlanner>(Config.ECBSSuboptimalityBound);

    case EPlannerType::PBS:
        return MakeUnique<FPBSPlanner>();

    case EPlannerType::LaCAM:
        return MakeUnique<FLaCAMPlanner>(
            Config.LaCAMTimeLimitMs,
            Config.LaCAMRandomSeed,
            Config.bLaCAMAnytime,
            Config.LaCAMVerboseLevel);

    case EPlannerType::LaCAMUTM:
        return MakeUnique<FLaCAMUTMPlanner>(
            Config.LaCAMTimeLimitMs,
            Config.LaCAMRandomSeed,
            Config.bLaCAMAnytime,
            Config.LaCAMVerboseLevel,
            Config.NoFlyZoneConfigs);

    default:
        UE_LOG(LogTemp, Error, TEXT("PlannerType=%d is not a multi-agent planner"), static_cast<int32>(PlannerType));
        return nullptr;
    }
}

bool FPlannerRegistry::PlanMultiAgentMissions(
    EPlannerType PlannerType,
    const FPlannerRuntimeConfig& Config,
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    TMap<int32, TArray<FVector>>& OutPaths)
{
    TUniquePtr<IMultiAgentPlannerBase> Planner = CreateMultiAgentPlanner(PlannerType, Config);
    if (!Planner)
    {
        OutPaths.Reset();
        return false;
    }

    return Planner->PlanMissions(GridMap, Missions, OutPaths);
}
