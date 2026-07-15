#pragma once

#include "Execution/ExecutionFinalSafetyGateTypes.h"
#include "Execution/ExecutionStepTypes.h"

class FExecutionFinalSafetyGatePolicy
{
public:
    static FExecutionFinalSafetyGateResult EvaluateAndApplyHold(
        const FExecutionFinalSafetyGateInput& Input,
        TMap<int32, FExecutionStepProposal>& InOutStepProposals);

    static FExecutionFinalSafetyGateConflictCheckResult CheckConflicts(
        const FExecutionFinalSafetyGateInput& Input,
        const TMap<int32, FExecutionStepProposal>& StepProposals);

private:
    static void ApplyHold(
        const TSet<int32>& HoldMissionIds,
        TMap<int32, FExecutionStepProposal>& InOutStepProposals);
};
