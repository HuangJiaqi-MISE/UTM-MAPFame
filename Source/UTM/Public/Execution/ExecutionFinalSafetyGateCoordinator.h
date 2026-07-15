#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionFinalSafetyGatePolicy.h"

enum class EExecutionFinalSafetyGateEventType : uint8
{
    UnsafeProposalDetected,
    HoldSetExpanded,
    HoldLimitReached,
    SafeHoldReplanRequested,
    LocalReplanFailedUpgradeGlobal,
    ReplanDisabledSafeHold,
    FinalProposalUnsafe,
    GlobalReplanFailedAfterHoldLimit,
    HoldFallbackUnsafe
};

struct FExecutionFinalSafetyGateEvent
{
    EExecutionFinalSafetyGateEventType Type =
        EExecutionFinalSafetyGateEventType::UnsafeProposalDetected;
    FExecutionPredictedConflict Conflict;
    int32 MissionId = INDEX_NONE;
    int32 MissionCount = 0;
    int32 PreviousMissionCount = 0;
    int32 HoldBudget = 1;
    bool bForceGlobalReplan = false;
};

struct FExecutionFinalSafetyGateCoordinatorRequest
{
    const FExecutionFinalSafetyGateInput* SafetyGateInput = nullptr;
    EExecutionPolicyReplanMode ReplanMode = EExecutionPolicyReplanMode::Disabled;
};

struct FExecutionFinalSafetyGateCoordinatorCallbacks
{
    TFunction<bool(const TSet<int32>&, bool, TSet<int32>&)> RunReplan;
    TFunction<void(
        const TSet<int32>&,
        const TSet<int32>&,
        TMap<int32, FExecutionStepProposal>&)> ApplyReplanResult;
    TFunction<void(const FExecutionFinalSafetyGateEvent&)> OnEvent;
};

struct FExecutionFinalSafetyGateCoordinatorResult
{
    bool bConflictDetected = false;
    bool bReplanSucceeded = false;
    bool bStopExecution = false;
    TSet<int32> RequestedReplanMissionIds;
    TSet<int32> SuccessfulReplanMissionIds;
};

class FExecutionFinalSafetyGateCoordinator
{
public:
    static FExecutionFinalSafetyGateCoordinatorResult Run(
        const FExecutionFinalSafetyGateCoordinatorRequest& Request,
        const FExecutionFinalSafetyGateCoordinatorCallbacks& Callbacks,
        TMap<int32, FExecutionStepProposal>& InOutStepProposals);
};
