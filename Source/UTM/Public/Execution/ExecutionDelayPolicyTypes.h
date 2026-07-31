#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionRuntimeStateTypes.h"

struct FExecutionDelayPolicySettings
{
    EExecutionDelayMode Mode = EExecutionDelayMode::RandomGlobal;
    float GlobalDelayProbability = 0.20f;
    TArray<FAgentDelayConfig> AgentConfigs;
};

struct FExecutionDelayPolicyInput
{
    const FExecutionAgentState* AgentState = nullptr;
    const FExecutionDelayPolicySettings* Settings = nullptr;
    FRandomStream* Random = nullptr;
    int32 TimeStep = 0;
};
