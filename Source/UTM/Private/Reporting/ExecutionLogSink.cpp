#include "Reporting/ExecutionLogSink.h"

#include "Execution/ExecutionAlignmentPolicy.h"
#include "Execution/ExecutionConflictResolutionPolicy.h"
#include "Execution/ExecutionControllerTypes.h"
#include "Execution/ExecutionFinalSafetyGateCoordinator.h"
#include "Execution/ExecutionReplanAttemptTypes.h"
#include "Execution/ExecutionReplanCoordinator.h"
#include "Execution/ExecutionReplanServiceTypes.h"
#include "Execution/ExecutionRuntimeSession.h"
#include "Execution/ExecutionStepResultApplier.h"

namespace
{
    const TCHAR* LexToString(EExecutionPredictedConflictType Type)
    {
        switch (Type)
        {
        case EExecutionPredictedConflictType::Vertex:
            return TEXT("Vertex");
        case EExecutionPredictedConflictType::Edge:
            return TEXT("Edge");
        case EExecutionPredictedConflictType::ProtectionFootprint:
            return TEXT("ProtectionFootprint");
        case EExecutionPredictedConflictType::Downwash:
            return TEXT("Downwash");
        default:
            return TEXT("None");
        }
    }

    FIntVector GetCellAtTimeForLog(
        const TArray<FIntVector>& Cells,
        int32 TimeStep)
    {
        if (Cells.Num() <= 0)
        {
            return FIntVector::ZeroValue;
        }

        if (TimeStep <= 0)
        {
            return Cells[0];
        }

        if (TimeStep < Cells.Num())
        {
            return Cells[TimeStep];
        }

        return Cells.Last();
    }
}

void FExecutionLogSink::SetSettings(
    const FExecutionDiagnosticsSettings& InSettings)
{
    Settings = InSettings;
}

void FExecutionLogSink::HandleConflictResolutionEvent(
    int32 TimeStep,
    const FExecutionConflictResolutionEvent& Event)
{
    if (!Settings.bLogConflictPredictionEvents)
    {
        return;
    }

    switch (Event.Type)
    {
    case EExecutionConflictResolutionEventType::UnresolvedConflict:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[AlignmentConflictPrediction] t=%d unresolved predicted %s conflict between Mission %d and Mission %d, escalate to replan"),
            TimeStep,
            LexToString(Event.Conflict.Type),
            Event.Conflict.AgentA,
            Event.Conflict.AgentB);
        break;

    case EExecutionConflictResolutionEventType::YieldApplied:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[AlignmentConflictPrediction] t=%d Mission=%d hold for predicted %s conflict with Mission=%d at Cell=(%d,%d,%d)"),
            TimeStep,
            Event.YieldMissionId,
            LexToString(Event.Conflict.Type),
            Event.KeepMissionId,
            Event.Conflict.Cell.X,
            Event.Conflict.Cell.Y,
            Event.Conflict.Cell.Z);
        break;

    case EExecutionConflictResolutionEventType::RemainingConflict:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[AlignmentConflictPrediction] t=%d remaining predicted %s conflict between Mission %d and Mission %d after arbitration"),
            TimeStep,
            LexToString(Event.Conflict.Type),
            Event.Conflict.AgentA,
            Event.Conflict.AgentB);
        break;
    }
}

void FExecutionLogSink::HandleFinalSafetyGateEvent(
    int32 TimeStep,
    const FExecutionFinalSafetyGateEvent& Event)
{
    switch (Event.Type)
    {
    case EExecutionFinalSafetyGateEventType::UnsafeProposalDetected:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[FinalSafetyGate] t=%d unsafe proposed %s conflict between Mission %d and Mission %d at Cell=(%d,%d,%d); forcing %d missions to hold"),
            TimeStep,
            LexToString(Event.Conflict.Type),
            Event.Conflict.AgentA,
            Event.Conflict.AgentB,
            Event.Conflict.Cell.X,
            Event.Conflict.Cell.Y,
            Event.Conflict.Cell.Z,
            Event.MissionCount);
        break;

    case EExecutionFinalSafetyGateEventType::HoldSetExpanded:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[FinalSafetyGate] t=%d hold set expanded from %d to %d missions due to remaining %s conflict between Mission %d and Mission %d"),
            TimeStep,
            Event.PreviousMissionCount,
            Event.MissionCount,
            LexToString(Event.Conflict.Type),
            Event.Conflict.AgentA,
            Event.Conflict.AgentB);
        break;

    case EExecutionFinalSafetyGateEventType::HoldLimitReached:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[FinalSafetyGate] t=%d Mission %d reached safety-gate hold limit %d; upgrade to global replan"),
            TimeStep,
            Event.MissionId,
            Event.HoldBudget);
        break;

    case EExecutionFinalSafetyGateEventType::SafeHoldReplanRequested:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[FinalSafetyGate] t=%d hold fallback is safe for %d missions; trigger %s execution replan"),
            TimeStep,
            Event.MissionCount,
            Event.bForceGlobalReplan ? TEXT("global") : TEXT("configured"));
        break;

    case EExecutionFinalSafetyGateEventType::LocalReplanFailedUpgradeGlobal:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[FinalSafetyGate] t=%d local safety-gate replan failed; upgrade to global replan"),
            TimeStep);
        break;

    case EExecutionFinalSafetyGateEventType::ReplanDisabledSafeHold:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[FinalSafetyGate] t=%d execution replan disabled; committing safe hold only"),
            TimeStep);
        break;

    case EExecutionFinalSafetyGateEventType::FinalProposalUnsafe:
        UE_LOG(
            LogTemp,
            Error,
            TEXT("[FinalSafetyGate] t=%d final proposal remains unsafe after safety-gate replan: %s conflict between Mission %d and Mission %d at Cell=(%d,%d,%d); mark execution failed instead of committing unsafe state"),
            TimeStep,
            LexToString(Event.Conflict.Type),
            Event.Conflict.AgentA,
            Event.Conflict.AgentB,
            Event.Conflict.Cell.X,
            Event.Conflict.Cell.Y,
            Event.Conflict.Cell.Z);
        break;

    case EExecutionFinalSafetyGateEventType::GlobalReplanFailedAfterHoldLimit:
        UE_LOG(
            LogTemp,
            Error,
            TEXT("[FinalSafetyGate] t=%d global replan failed after safety-gate hold limit; mark execution failed instead of committing unsafe state"),
            TimeStep);
        break;

    case EExecutionFinalSafetyGateEventType::HoldFallbackUnsafe:
        UE_LOG(
            LogTemp,
            Error,
            TEXT("[FinalSafetyGate] t=%d hold fallback remains unsafe: %s conflict between Mission %d and Mission %d at Cell=(%d,%d,%d); dirty-start recovery required but unavailable, mark execution failed"),
            TimeStep,
            LexToString(Event.Conflict.Type),
            Event.Conflict.AgentA,
            Event.Conflict.AgentB,
            Event.Conflict.Cell.X,
            Event.Conflict.Cell.Y,
            Event.Conflict.Cell.Z);
        break;
    }
}

void FExecutionLogSink::HandleReplanAttemptFailure(
    const FExecutionReplanAttemptInput& Input,
    const FExecutionReplanAttemptResult& Result)
{
    switch (Result.Status)
    {
    case EExecutionReplanAttemptStatus::GridBuildFailed:
        if (!Result.FailureReason.IsEmpty())
        {
            UE_LOG(LogTemp, Warning, TEXT("[AlignmentReplan] %s"), *Result.FailureReason);
        }
        break;

    case EExecutionReplanAttemptStatus::MissionBuildFailed:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[AlignmentReplan] failed to build replan missions: %s"),
            *Result.FailureReason);
        break;

    case EExecutionReplanAttemptStatus::PlannerFailed:
        if (Input.Spec.bGlobalReplan)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[AlignmentReplan] global replan failed for %d movable missions (Anchors=%d StaticBlocked=%d)"),
                Result.CandidateMissionIds.Num(),
                Result.AnchorMissionIds.Num(),
                Result.StaticAnchorBlockedCellCount);
        }
        else
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[AlignmentReplan] local replan attempt %d/%d failed for %d movable missions (Anchors=%d StaticBlocked=%d K=%d W=%d)"),
                Input.Spec.AttemptIndex + 1,
                Input.Spec.AttemptCount,
                Result.CandidateMissionIds.Num(),
                Result.AnchorMissionIds.Num(),
                Result.StaticAnchorBlockedCellCount,
                Input.Spec.SpatialRadiusCells,
                Input.Spec.LookaheadSteps);
        }
        break;

    case EExecutionReplanAttemptStatus::InvalidReplannedPath:
        if (Result.FailedMissionId != INDEX_NONE)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[AlignmentReplan] invalid replanned path for Mission %d"),
                Result.FailedMissionId);
        }
        break;

    default:
        break;
    }
}

void FExecutionLogSink::HandleReplanCoordinatorEvent(
    const FExecutionReplanCoordinatorEvent& Event)
{
    switch (Event.Type)
    {
    case EExecutionReplanCoordinatorEventType::CandidateSetExpanded:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[AlignmentReplan] local conflict component attempt %d/%d expanded from %d to %d missions (K=%d W=%d)"),
            Event.AttemptIndex + 1,
            Event.AttemptCount,
            Event.ActiveRequestedMissionCount,
            Event.CandidateMissionCount,
            Event.SpatialRadiusCells,
            Event.LookaheadSteps);
        break;

    case EExecutionReplanCoordinatorEventType::PostCheckFailed:
        if (Event.bGlobalReplan)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[AlignmentReplan] global replan post-check failed: predicted %s conflict between Mission %d and Mission %d at +%d Cell=(%d,%d,%d) (Movable=%d Anchors=%d StaticBlocked=%d)"),
                LexToString(Event.Conflict.Type),
                Event.Conflict.AgentA,
                Event.Conflict.AgentB,
                Event.ConflictOffset,
                Event.Conflict.Cell.X,
                Event.Conflict.Cell.Y,
                Event.Conflict.Cell.Z,
                Event.CandidateMissionCount,
                Event.AnchorMissionCount,
                Event.StaticAnchorBlockedCellCount);
        }
        else
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[AlignmentReplan] local replan attempt %d/%d post-check failed: predicted %s conflict between Mission %d and Mission %d at +%d Cell=(%d,%d,%d) (Movable=%d Anchors=%d StaticBlocked=%d K=%d W=%d)"),
                Event.AttemptIndex + 1,
                Event.AttemptCount,
                LexToString(Event.Conflict.Type),
                Event.Conflict.AgentA,
                Event.Conflict.AgentB,
                Event.ConflictOffset,
                Event.Conflict.Cell.X,
                Event.Conflict.Cell.Y,
                Event.Conflict.Cell.Z,
                Event.CandidateMissionCount,
                Event.AnchorMissionCount,
                Event.StaticAnchorBlockedCellCount,
                Event.SpatialRadiusCells,
                Event.LookaheadSteps);
        }
        break;

    case EExecutionReplanCoordinatorEventType::TargetedRetryExpanded:
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[AlignmentReplan] local post-check targeted retry %d/%d after %s conflict between Mission %d and Mission %d: movable %d->%d forced anchors %d->%d (K=%d W=%d)"),
            Event.TargetedRetryIndex,
            Event.MaxTargetedRetryCount,
            LexToString(Event.Conflict.Type),
            Event.Conflict.AgentA,
            Event.Conflict.AgentB,
            Event.PreviousCandidateMissionCount,
            Event.CandidateMissionCount,
            Event.PreviousForcedAnchorMissionCount,
            Event.ForcedAnchorMissionCount,
            Event.SpatialRadiusCells,
            Event.LookaheadSteps);
        break;

    case EExecutionReplanCoordinatorEventType::AttemptSucceeded:
        if (Event.bGlobalReplan)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[AlignmentReplan] global replan succeeded for %d movable missions at t=%d (Total=%d, Anchors=%d StaticBlocked=%d)"),
                Event.ReplannedMissionCount,
                Event.ExecutionTimeStep,
                Event.TotalReplanCount,
                Event.AnchorMissionCount,
                Event.StaticAnchorBlockedCellCount);
        }
        else
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[AlignmentReplan] local replan succeeded for %d movable missions at t=%d (Total=%d, Attempt=%d/%d, Anchors=%d StaticBlocked=%d K=%d W=%d)"),
                Event.ReplannedMissionCount,
                Event.ExecutionTimeStep,
                Event.TotalReplanCount,
                Event.AttemptIndex + 1,
                Event.AttemptCount,
                Event.AnchorMissionCount,
                Event.StaticAnchorBlockedCellCount,
                Event.SpatialRadiusCells,
                Event.LookaheadSteps);
        }
        break;
    }
}

void FExecutionLogSink::HandleReplanServiceResult(
    const FExecutionReplanServiceResult& Result)
{
    if (Result.Status != EExecutionReplanServiceStatus::ReplanLimitReached)
    {
        return;
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("[AlignmentReplan] skipped because total replan count %d reached limit %d"),
        Result.CurrentTotalReplanCount,
        Result.MaxReplanCount);
}

void FExecutionLogSink::HandleAppliedStep(
    int32 TimeStep,
    const FExecutionRuntimeSession& Session,
    const FExecutionControllerStepResult& ControllerResult,
    const FExecutionStepResultApplyResult& ApplyResult)
{
    for (const FExecutionStepAppliedAgent& AppliedAgent : ApplyResult.AppliedAgents)
    {
        const int32 MissionId = AppliedAgent.MissionId;
        const FExecutionAgentState* State =
            Session.AgentStatesByMissionId.Find(MissionId);
        const FExecutionStepProposal* Proposal =
            ControllerResult.StepProposals.Find(MissionId);
        if (!State || !Proposal)
        {
            continue;
        }

        if (Proposal->bDelayRequested && Settings.bLogDelayEvents)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[ExecutionDelay] t=%d Mission=%d stay at Cell=(%d,%d,%d)"),
                TimeStep,
                State->MissionId,
                Proposal->ObservedCell.X,
                Proposal->ObservedCell.Y,
                Proposal->ObservedCell.Z
            );
        }

        if (Settings.bLogAlignmentEvents &&
            Proposal->FinalAction != EExecutionPolicyAction::FollowPlan)
        {
            const FIntVector ReferenceCell = GetCellAtTimeForLog(
                State->PlannedCells,
                Proposal->ReferencePlanIndex);
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[Alignment] t=%d Mission=%d Action=%s Observed=(%d,%d,%d) RefIndex=%d RefCell=(%d,%d,%d) NextCell=(%d,%d,%d) SpatialError=%d TemporalError=%d Replan=%s Reason=%s"),
                TimeStep,
                State->MissionId,
                FExecutionAlignmentPolicy::LexToString(Proposal->FinalAction),
                Proposal->AlignmentDecision.ObservedCell.X,
                Proposal->AlignmentDecision.ObservedCell.Y,
                Proposal->AlignmentDecision.ObservedCell.Z,
                Proposal->ReferencePlanIndex,
                ReferenceCell.X,
                ReferenceCell.Y,
                ReferenceCell.Z,
                Proposal->ProposedCell.X,
                Proposal->ProposedCell.Y,
                Proposal->ProposedCell.Z,
                Proposal->AlignmentDecision.SpatialErrorCells,
                Proposal->AlignmentDecision.TemporalErrorSteps,
                AppliedAgent.bReplanRequestedForState ? TEXT("true") : TEXT("false"),
                *Proposal->ResolutionReason);
        }
    }
}

void FExecutionLogSink::HandleObservedConflicts(
    const TArray<FExecutionConflict>& Conflicts)
{
    for (const FExecutionConflict& Conflict : Conflicts)
    {
        if (!Conflict.bIsEdgeConflict)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("[ExecutionConflict][Vertex] t=%d Agent=%d Agent=%d Cell=(%d,%d,%d)"),
                Conflict.TimeStep,
                Conflict.AgentA,
                Conflict.AgentB,
                Conflict.Cell.X,
                Conflict.Cell.Y,
                Conflict.Cell.Z
            );
            continue;
        }

        UE_LOG(
            LogTemp,
            Error,
            TEXT("[ExecutionConflict][Edge] t=%d Agent=%d (%d,%d,%d)->(%d,%d,%d), Agent=%d (%d,%d,%d)->(%d,%d,%d)"),
            Conflict.TimeStep,
            Conflict.AgentA,
            Conflict.FromA.X, Conflict.FromA.Y, Conflict.FromA.Z,
            Conflict.ToA.X, Conflict.ToA.Y, Conflict.ToA.Z,
            Conflict.AgentB,
            Conflict.FromB.X, Conflict.FromB.Y, Conflict.FromB.Z,
            Conflict.ToB.X, Conflict.ToB.Y, Conflict.ToB.Z
        );
    }
}
