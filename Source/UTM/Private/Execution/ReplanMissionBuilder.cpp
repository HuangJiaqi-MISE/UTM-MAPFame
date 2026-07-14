#include "Execution/ReplanMissionBuilder.h"

FReplanMissionBuildResult FReplanMissionBuilder::Build(const FReplanMissionBuildInput& Input)
{
    FReplanMissionBuildResult Result;

    TMap<int32, const FExecutionAgentSnapshot*> AgentsByMissionId;
    AgentsByMissionId.Reserve(Input.Agents.Num());
    for (const FExecutionAgentSnapshot& Agent : Input.Agents)
    {
        AgentsByMissionId.Add(Agent.MissionId, &Agent);
    }

    TArray<int32> SelectedMissionIds;
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

        SelectedMissionIds.Add(Agent.MissionId);
    }

    SelectedMissionIds.Sort();
    Result.ReplanMissions.Reserve(SelectedMissionIds.Num());

    for (const int32 MissionId : SelectedMissionIds)
    {
        const FExecutionAgentSnapshot* const* Agent = AgentsByMissionId.Find(MissionId);
        if (!Agent || !*Agent)
        {
            Result.FailureReason = FString::Printf(TEXT("missing execution snapshot for MissionId=%d"), MissionId);
            return Result;
        }

        const FDroneMissionConfig* MissionConfig = Input.MissionConfigsById.Find(MissionId);
        if (!MissionConfig)
        {
            Result.FailureReason = FString::Printf(TEXT("missing mission config for MissionId=%d"), MissionId);
            return Result;
        }

        FDroneMissionConfig ReplanMission = *MissionConfig;
        ReplanMission.StartWorld = (*Agent)->ObservedWorld;
        ReplanMission.bStationaryAnchor = false;
        Result.ReplanMissions.Add(ReplanMission);
    }

    Result.bSuccess = Result.ReplanMissions.Num() > 0;
    if (!Result.bSuccess)
    {
        Result.FailureReason = TEXT("no missions selected for execution replan");
    }

    return Result;
}

