#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionStepTypes.h"

struct FExecutionStepProposalBuildInput
{
    int32 MissionId = INDEX_NONE;
    FIntVector ObservedCell = FIntVector::ZeroValue;
    bool bDelayRequested = false;
    int32 CurrentPlanIndex = 0;
    int32 PlannedCellCount = 0;
};

struct FExecutionStepProposalBuildResult
{
    bool bSuccess = false;
    bool bRequestsReplan = false;
    FExecutionStepProposal Proposal;
};

class FExecutionStepProposalBuilder
{
public:
    static FExecutionStepProposalBuildResult Build(
        const FExecutionStepProposalBuildInput& Input,
        const FExecutionStepDecision& AlignmentDecision);
};
