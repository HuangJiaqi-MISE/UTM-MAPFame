#pragma once

#include "CoreMinimal.h"
#include "Planning/PlanningPipelineTypes.h"

class FPlanningPipeline
{
public:
    static FMultiAgentPlanningPipelineResult RunMultiAgent(
        const FMultiAgentPlanningPipelineRequest& Request);
};
