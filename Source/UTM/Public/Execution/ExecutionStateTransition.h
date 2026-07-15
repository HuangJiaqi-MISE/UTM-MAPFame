#pragma once

#include "Execution/ExecutionStateTransitionTypes.h"
#include "Execution/ExecutionStepTypes.h"

class FExecutionStateTransition
{
public:
    static FExecutionStateTransitionResult Compute(
        const FExecutionStateTransitionInput& Input,
        const FExecutionStepProposal& Proposal);
};
