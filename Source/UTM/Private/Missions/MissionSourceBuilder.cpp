#include "Missions/MissionSourceBuilder.h"

FMissionSourceBuildResult FMissionSourceBuilder::BuildFromMissionConfigs(
    const TArray<FDroneMissionConfig>& MissionConfigs)
{
    FMissionSourceBuildResult Result;
    Result.SourceKind = EMissionSourceKind::MissionConfigs;
    Result.Missions = MissionConfigs;
    return Result;
}

FMissionSourceBuildResult FMissionSourceBuilder::BuildFromStartGoalWorldPairs(
    const TArray<int32>& MissionIds,
    const TMap<int32, FVector>& StartsByMissionId,
    const TMap<int32, FVector>& GoalsByMissionId)
{
    FMissionSourceBuildResult Result;
    Result.SourceKind = EMissionSourceKind::StartGoalWorldPairs;
    Result.Missions.Reserve(MissionIds.Num());

    for (const int32 MissionId : MissionIds)
    {
        const FVector* StartWorld = StartsByMissionId.Find(MissionId);
        const FVector* GoalWorld = GoalsByMissionId.Find(MissionId);
        if (!StartWorld || !GoalWorld)
        {
            Result.SkippedMissionIds.Add(MissionId);
            continue;
        }

        FDroneMissionConfig Mission;
        Mission.MissionId = MissionId;
        Mission.StartWorld = *StartWorld;
        Mission.GoalWorld = *GoalWorld;
        Result.Missions.Add(Mission);
    }

    Result.bSuccess = (Result.Missions.Num() > 0);
    if (!Result.bSuccess)
    {
        Result.ErrorMessage = TEXT("Mission source produced no valid missions");
    }

    return Result;
}
