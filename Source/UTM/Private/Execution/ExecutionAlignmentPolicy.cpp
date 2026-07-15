#include "Execution/ExecutionAlignmentPolicy.h"

#include "Planning/GridMap3D.h"

FExecutionAlignmentPolicy::FExecutionAlignmentPolicy(const FDiscreteAlignmentSettings& InSettings)
    : Settings(InSettings)
{
}

void FExecutionAlignmentPolicy::SetSettings(const FDiscreteAlignmentSettings& InSettings)
{
    Settings = InSettings;
}

FExecutionStepDecision FExecutionAlignmentPolicy::Decide(
    const FGridMap3D& GridMap,
    const FExecutionAgentSnapshot& AgentSnapshot) const
{
    FExecutionStepDecision Decision;
    Decision.MissionId = AgentSnapshot.MissionId;
    Decision.ObservedCell = AgentSnapshot.ObservedCell;

    const FDiscreteAlignmentManager AlignmentManager(Settings);
    const FDiscreteAlignmentResult AlignmentResult = AlignmentManager.AlignStep(
        GridMap,
        AgentSnapshot.PlannedCells,
        AgentSnapshot.ExecutedPlanIndex,
        AgentSnapshot.TimeStep,
        AgentSnapshot.ObservedCell,
        AgentSnapshot.bDelayRequested);

    Decision.bValid = AlignmentResult.bValid;
    Decision.bRequiresReplan = AlignmentResult.bRequiresReplan;
    Decision.Action = ConvertAction(AlignmentResult.Action);
    Decision.ReferenceCell = AlignmentResult.ReferenceCell;
    Decision.TargetCell = AlignmentResult.NextCell;
    Decision.ReferencePlanIndex = AlignmentResult.ReferencePlanIndex;
    Decision.TargetPlanIndex = AlignmentResult.NextPlanIndex;
    Decision.SpatialErrorCells = AlignmentResult.SpatialErrorCells;
    Decision.TemporalErrorSteps = AlignmentResult.TemporalErrorSteps;
    Decision.Reason = AlignmentResult.Reason;

    return Decision;
}

const TCHAR* FExecutionAlignmentPolicy::LexToString(EExecutionPolicyAction Action)
{
    switch (Action)
    {
    case EExecutionPolicyAction::FollowPlan:
        return TEXT("FollowPlan");
    case EExecutionPolicyAction::HoldForDelay:
        return TEXT("HoldForDelay");
    case EExecutionPolicyAction::SnapToPlanIndex:
        return TEXT("SnapToPlanIndex");
    case EExecutionPolicyAction::RecoverTowardPlan:
        return TEXT("RecoverTowardPlan");
    case EExecutionPolicyAction::HoldForAlignment:
        return TEXT("HoldForAlignment");
    case EExecutionPolicyAction::HoldForPredictedConflict:
        return TEXT("HoldForPredictedConflict");
    case EExecutionPolicyAction::HoldForSafetyGate:
        return TEXT("HoldForSafetyGate");
    case EExecutionPolicyAction::HoldForReplan:
        return TEXT("HoldForReplan");
    case EExecutionPolicyAction::GoalHold:
        return TEXT("GoalHold");
    default:
        return TEXT("Unknown");
    }
}

EExecutionPolicyAction FExecutionAlignmentPolicy::ConvertAction(EDiscreteAlignmentAction Action)
{
    switch (Action)
    {
    case EDiscreteAlignmentAction::FollowPlan:
        return EExecutionPolicyAction::FollowPlan;
    case EDiscreteAlignmentAction::HoldForDelay:
        return EExecutionPolicyAction::HoldForDelay;
    case EDiscreteAlignmentAction::SnapToPlanIndex:
        return EExecutionPolicyAction::SnapToPlanIndex;
    case EDiscreteAlignmentAction::RecoverTowardPlan:
        return EExecutionPolicyAction::RecoverTowardPlan;
    case EDiscreteAlignmentAction::HoldForAlignment:
        return EExecutionPolicyAction::HoldForAlignment;
    case EDiscreteAlignmentAction::HoldForPredictedConflict:
        return EExecutionPolicyAction::HoldForPredictedConflict;
    case EDiscreteAlignmentAction::HoldForSafetyGate:
        return EExecutionPolicyAction::HoldForSafetyGate;
    case EDiscreteAlignmentAction::HoldForReplan:
        return EExecutionPolicyAction::HoldForReplan;
    case EDiscreteAlignmentAction::GoalHold:
        return EExecutionPolicyAction::GoalHold;
    default:
        return EExecutionPolicyAction::HoldForAlignment;
    }
}
