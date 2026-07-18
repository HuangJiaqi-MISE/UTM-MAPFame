#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionRuntimeStateTypes.h"
#include "Planning/DroneMissionTypes.h"

struct FExecutionRuntimeSession
{
    TMap<int32, FDroneMissionConfig> MissionConfigsById;
    TMap<int32, FExecutionAgentState> AgentStatesByMissionId;
    TArray<FExecutionConflict> Conflicts;
    FRandomStream Random;
    bool bRunning = false;
    int32 TimeStep = 0;
    int32 TotalReplanCount = 0;
    FExecutionReplanTimingStats ReplanTimingStats;

    void Reset();
    void PrepareForExecution(int32 RandomSeed);
};
