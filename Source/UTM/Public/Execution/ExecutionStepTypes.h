#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"

struct FExecutionStepProposal
{
    int32 MissionId = INDEX_NONE;
    FIntVector ObservedCell = FIntVector::ZeroValue;
    FIntVector ProposedCell = FIntVector::ZeroValue;
    int32 ReferencePlanIndex = 0;
    int32 ProposedPlanIndex = 0;
    bool bDelayRequested = false;
    bool bValid = false;
    bool bRequiresReplan = false;
    bool bInitialAlignmentInvalid = false;
    bool bHeldForPredictedConflict = false;
    bool bHeldForReplan = false;
    FExecutionStepDecision AlignmentDecision;
    EExecutionPolicyAction FinalAction = EExecutionPolicyAction::FollowPlan;
    FString ResolutionReason;
};
