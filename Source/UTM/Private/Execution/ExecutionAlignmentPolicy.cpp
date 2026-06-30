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
