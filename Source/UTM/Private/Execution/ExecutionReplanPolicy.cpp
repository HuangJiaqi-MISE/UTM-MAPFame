#include "Execution/ExecutionReplanPolicy.h"

FExecutionReplanPolicy::FExecutionReplanPolicy(const FExecutionReplanPolicySettings& InSettings)
    : Settings(InSettings)
{
}

void FExecutionReplanPolicy::SetSettings(const FExecutionReplanPolicySettings& InSettings)
{
    Settings = InSettings;
}

FExecutionReplanRequest FExecutionReplanPolicy::BuildReplanRequest(
    const FExecutionSnapshot& Snapshot,
    const TArray<FExecutionStepDecision>& Decisions,
    const TArray<FExecutionPredictedConflict>& PredictedConflicts) const
{
    FExecutionReplanRequest Request;

    if (Settings.Mode == EExecutionPolicyReplanMode::Disabled)
    {
        Request.Reason = TEXT("execution replanning disabled");
        return Request;
    }

    if (Settings.MaxReplanCount > 0 && Snapshot.TotalReplanCount >= Settings.MaxReplanCount)
    {
        Request.Reason = TEXT("maximum execution replan count reached");
        return Request;
    }

    for (const FExecutionStepDecision& Decision : Decisions)
    {
        if (Decision.bRequiresReplan)
        {
            Request.RequestedMissionIds.Add(Decision.MissionId);
        }
    }

    for (const FExecutionPredictedConflict& Conflict : PredictedConflicts)
    {
        if (Conflict.Type != EExecutionPredictedConflictType::None)
        {
            Request.RequestedMissionIds.Add(Conflict.AgentA);
            Request.RequestedMissionIds.Add(Conflict.AgentB);
        }
    }

    for (const FExecutionAgentSnapshot& Agent : Snapshot.Agents)
    {
        if (Settings.ConflictHoldThresholdForReplan > 0 &&
            Agent.ConsecutiveConflictHoldCount >= Settings.ConflictHoldThresholdForReplan)
        {
            Request.RequestedMissionIds.Add(Agent.MissionId);
        }
    }

    Request.bShouldReplan = Request.RequestedMissionIds.Num() > 0;
    Request.bGlobalReplan = Request.bShouldReplan && Settings.Mode == EExecutionPolicyReplanMode::GlobalUnfinished;

    if (Request.bGlobalReplan)
    {
        for (const FExecutionAgentSnapshot& Agent : Snapshot.Agents)
        {
            if (!Agent.bFinished)
            {
                Request.RequestedMissionIds.Add(Agent.MissionId);
            }
        }
    }

    Request.Reason = Request.bShouldReplan
        ? (Request.bGlobalReplan ? TEXT("global unfinished execution replan requested") : TEXT("local execution replan requested"))
        : TEXT("no execution replan requested");

    return Request;
}

