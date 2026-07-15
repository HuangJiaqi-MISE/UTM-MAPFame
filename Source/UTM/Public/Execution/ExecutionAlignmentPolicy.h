#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"
#include "Execution/DiscreteAlignmentManager.h"

class FGridMap3D;

class FExecutionAlignmentPolicy
{
public:
    explicit FExecutionAlignmentPolicy(const FDiscreteAlignmentSettings& InSettings = FDiscreteAlignmentSettings());

    void SetSettings(const FDiscreteAlignmentSettings& InSettings);

    FExecutionStepDecision Decide(
        const FGridMap3D& GridMap,
        const FExecutionAgentSnapshot& AgentSnapshot) const;

    static const TCHAR* LexToString(EExecutionPolicyAction Action);

private:
    static EExecutionPolicyAction ConvertAction(EDiscreteAlignmentAction Action);

    FDiscreteAlignmentSettings Settings;
};

