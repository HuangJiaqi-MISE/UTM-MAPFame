#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionControllerTypes.h"
#include "Execution/ExecutionStateTransitionTypes.h"

struct FExecutionStepResultApplyAgentState
{
    FExecutionStateTransitionState CurrentState;
    int32 PlannedCellCount = 0;
    FIntVector FinalPlannedCell = FIntVector::ZeroValue;
};

struct FExecutionStepResultApplyRequest
{
    TArray<int32> OrderedMissionIds;
    TMap<int32, FExecutionStepResultApplyAgentState> AgentStatesByMissionId;
};

struct FExecutionStepAppliedAgent
{
    int32 MissionId = INDEX_NONE;
    bool bReplanRequestedForState = false;
    bool bReplannedForState = false;
    FExecutionStateTransitionResult TransitionResult;
};

struct FExecutionStepResultApplyResult
{
    TArray<FExecutionStepAppliedAgent> AppliedAgents;
    bool bAnyActive = false;
};

class FExecutionStepResultApplier
{
public:
    static FExecutionStepResultApplyResult Apply(
        const FExecutionStepResultApplyRequest& Request,
        const FExecutionControllerStepResult& ControllerResult);
};
