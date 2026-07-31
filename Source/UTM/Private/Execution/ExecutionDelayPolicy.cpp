#include "Execution/ExecutionDelayPolicy.h"

namespace
{
    const FAgentDelayConfig* FindAgentConfig(
        const TArray<FAgentDelayConfig>& Configs,
        int32 MissionId)
    {
        for (const FAgentDelayConfig& Config : Configs)
        {
            if (Config.MissionId == MissionId)
            {
                return &Config;
            }
        }

        return nullptr;
    }
}

bool FExecutionDelayPolicy::ShouldDelay(
    const FExecutionDelayPolicyInput& Input)
{
    if (!Input.AgentState || !Input.Settings || Input.AgentState->bFinished)
    {
        return false;
    }

    switch (Input.Settings->Mode)
    {
    case EExecutionDelayMode::RandomGlobal:
    {
        const float Probability = FMath::Clamp(
            Input.Settings->GlobalDelayProbability,
            0.f,
            1.f);
        return Probability > 0.f &&
            Input.Random &&
            Input.Random->FRand() < Probability;
    }

    case EExecutionDelayMode::PerAgentProbability:
    {
        const FAgentDelayConfig* Config = FindAgentConfig(
            Input.Settings->AgentConfigs,
            Input.AgentState->MissionId);
        if (!Config)
        {
            return false;
        }

        const float Probability = FMath::Clamp(
            Config->DelayProbability,
            0.f,
            1.f);
        return Probability > 0.f &&
            Input.Random &&
            Input.Random->FRand() < Probability;
    }

    case EExecutionDelayMode::ScriptedTimesteps:
    {
        const FAgentDelayConfig* Config = FindAgentConfig(
            Input.Settings->AgentConfigs,
            Input.AgentState->MissionId);
        return Config && Config->ForcedDelaySteps.Contains(Input.TimeStep);
    }

    default:
        return false;
    }
}
