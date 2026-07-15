#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionStepTypes.h"

struct FExecutionReplanProposalAgentState
{
    int32 ExecutedPlanIndex = 0;
    int32 PlannedCellCount = 0;
};

struct FExecutionReplanProposalSyncRequest
{
    TArray<int32> OrderedMissionIds;
    TSet<int32> TargetMissionIds;
    TSet<int32> ReplannedMissionIds;
    TMap<int32, FExecutionReplanProposalAgentState> AgentStatesByMissionId;
    FString ReplannedReason;
    FString SynchronizedReason;
};

class FExecutionReplanProposalSynchronizer
{
public:
    static void Apply(
        const FExecutionReplanProposalSyncRequest& Request,
        TMap<int32, FExecutionStepProposal>& InOutStepProposals);
};
