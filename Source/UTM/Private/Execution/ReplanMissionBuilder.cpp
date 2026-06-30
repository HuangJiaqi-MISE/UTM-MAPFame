#include "Execution/ReplanMissionBuilder.h"

FReplanMissionBuildResult FReplanMissionBuilder::Build(const FReplanMissionBuildInput& Input)
{
    FReplanMissionBuildResult Result;

    for (const FExecutionAgentSnapshot& Agent : Input.Agents)
    {
        const bool bIncludeMission =
            Input.bGlobalReplan
                ? !Agent.bFinished
                : Input.RequestedMissionIds.Contains(Agent.MissionId);

        if (!bIncludeMission)
        {
            continue;
        }

        const FDroneMissionConfig* MissionConfig = Input.MissionConfigsById.Find(Agent.MissionId);
        if (!MissionConfig)
        {
            Result.FailureReason = FString::Printf(TEXT("missing mission config for MissionId=%d"), Agent.MissionId);
            return Result;
        }

        FDroneMissionConfig ReplanMission = *MissionConfig;
        ReplanMission.StartWorld = Agent.ObservedWorld;
        ReplanMission.GoalWorld = Agent.GoalWorld;
        Result.ReplanMissions.Add(ReplanMission);
    }

    Result.bSuccess = Result.ReplanMissions.Num() > 0;
    if (!Result.bSuccess)
    {
        Result.FailureReason = TEXT("no missions selected for execution replan");
    }

    return Result;
}

