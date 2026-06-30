#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"
#include "Planning/DroneMissionTypes.h"

struct FReplanMissionBuildInput
{
    TArray<FExecutionAgentSnapshot> Agents;
    TMap<int32, FDroneMissionConfig> MissionConfigsById;
    TSet<int32> RequestedMissionIds;
    bool bGlobalReplan = false;
};

struct FReplanMissionBuildResult
{
    bool bSuccess = false;
    TArray<FDroneMissionConfig> ReplanMissions;
    FString FailureReason;
};

class FReplanMissionBuilder
{
public:
    static FReplanMissionBuildResult Build(const FReplanMissionBuildInput& Input);
};

