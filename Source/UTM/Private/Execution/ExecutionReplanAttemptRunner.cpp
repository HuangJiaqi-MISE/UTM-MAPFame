#include "Execution/ExecutionReplanAttemptRunner.h"

#include "Execution/ExecutionReplanGridBuilder.h"
#include "Execution/ExecutionReplanPostCheckPolicy.h"
#include "Execution/ReplanMissionBuilder.h"
#include "Planning/GridMap3D.h"
#include "Planning/PlannerRegistry.h"

namespace
{
    TArray<FIntVector> BuildReplannedCellPath(
        const FGridMap3D& GridMap,
        const TArray<FVector>& WorldPath)
    {
        TArray<FIntVector> CellPath;
        CellPath.Reserve(WorldPath.Num());

        for (const FVector& WorldPoint : WorldPath)
        {
            CellPath.Add(GridMap.WorldToCell(WorldPoint));
        }

        return CellPath;
    }
}

bool FExecutionReplanAttemptRunner::Run(
    const FExecutionReplanAttemptContext& Context,
    const FExecutionReplanAttemptInput& Input,
    FExecutionReplanAttemptResult& OutResult)
{
    OutResult = FExecutionReplanAttemptResult();

    if (!Context.BaseGrid || !Context.MissionConfigsById || !Context.PlannerConfig || !Input.Snapshot)
    {
        OutResult.Status = EExecutionReplanAttemptStatus::GridBuildFailed;
        OutResult.FailureReason = TEXT("invalid execution replan attempt context");
        return false;
    }

    OutResult.CandidateMissionIds.Reserve(Input.CandidateMissionIdSet.Num());
    for (const int32 CandidateMissionId : Input.CandidateMissionIdSet)
    {
        OutResult.CandidateMissionIds.Add(CandidateMissionId);
    }

    OutResult.CandidateMissionIds.Sort();

    if (OutResult.CandidateMissionIds.Num() <= 0)
    {
        OutResult.Status = EExecutionReplanAttemptStatus::EmptyCandidateSet;
        return false;
    }

    FExecutionReplanGridBuildInput GridBuildInput;
    GridBuildInput.BaseGrid = Context.BaseGrid;
    GridBuildInput.Snapshot = *Input.Snapshot;
    GridBuildInput.MissionConfigsById = *Context.MissionConfigsById;
    GridBuildInput.CandidateMissionIds = Input.CandidateMissionIdSet;
    GridBuildInput.ForcedAnchorMissionIds = Input.ForcedAnchorMissionIdSet;
    GridBuildInput.bGlobalReplan = Input.Spec.bGlobalReplan;
    GridBuildInput.bCheckStaticUTMSafety = Input.Spec.bCheckStaticUTMSafety;
    GridBuildInput.SpatialRadiusCells = Input.Spec.SpatialRadiusCells;
    GridBuildInput.LookaheadSteps = Input.Spec.LookaheadSteps;

    const FExecutionReplanGridBuildResult GridBuildResult =
        FExecutionReplanGridBuilder::Build(GridBuildInput);
    if (!GridBuildResult.bSuccess)
    {
        OutResult.Status = EExecutionReplanAttemptStatus::GridBuildFailed;
        OutResult.FailureReason = GridBuildResult.FailureReason;
        return false;
    }

    OutResult.AnchorMissionIds = GridBuildResult.AnchorMissionIds;
    OutResult.AnchorMissionIdSet = GridBuildResult.AnchorMissionIdSet;
    OutResult.StaticAnchorBlockedCellCount = GridBuildResult.StaticAnchorBlockedCellCount;

    FReplanMissionBuildInput ReplanMissionBuildInput;
    ReplanMissionBuildInput.Agents = Input.Snapshot->Agents;
    ReplanMissionBuildInput.MissionConfigsById = *Context.MissionConfigsById;
    ReplanMissionBuildInput.RequestedMissionIds = Input.CandidateMissionIdSet;
    ReplanMissionBuildInput.bGlobalReplan = false;

    const FReplanMissionBuildResult ReplanMissionBuildResult =
        FReplanMissionBuilder::Build(ReplanMissionBuildInput);
    if (!ReplanMissionBuildResult.bSuccess)
    {
        OutResult.Status = EExecutionReplanAttemptStatus::MissionBuildFailed;
        OutResult.FailureReason = ReplanMissionBuildResult.FailureReason;
        return false;
    }

    const TArray<FDroneMissionConfig>& ReplanMissions = ReplanMissionBuildResult.ReplanMissions;

    TMap<int32, TArray<FVector>> ReplannedWorldPaths;
    const bool bPlanningSuccess = FPlannerRegistry::PlanMultiAgentMissions(
        Context.PlannerType,
        *Context.PlannerConfig,
        GridBuildResult.ReplanGrid,
        ReplanMissions,
        ReplannedWorldPaths);
    if (!bPlanningSuccess)
    {
        OutResult.Status = EExecutionReplanAttemptStatus::PlannerFailed;
        return false;
    }

    TMap<int32, const FExecutionAgentSnapshot*> SnapshotByMissionId;
    SnapshotByMissionId.Reserve(Input.Snapshot->Agents.Num());
    for (const FExecutionAgentSnapshot& AgentSnapshot : Input.Snapshot->Agents)
    {
        SnapshotByMissionId.Add(AgentSnapshot.MissionId, &AgentSnapshot);
    }

    OutResult.ReplannedCellPathsByMission.Reserve(ReplanMissions.Num());

    for (const FDroneMissionConfig& Mission : ReplanMissions)
    {
        const FExecutionAgentSnapshot* const* AgentSnapshot = SnapshotByMissionId.Find(Mission.MissionId);
        const bool bStationaryAnchor = OutResult.AnchorMissionIdSet.Contains(Mission.MissionId);
        const TArray<FVector>* ReplannedWorldPath = ReplannedWorldPaths.Find(Mission.MissionId);
        if (!AgentSnapshot || (!bStationaryAnchor && (!ReplannedWorldPath || ReplannedWorldPath->Num() <= 0)))
        {
            OutResult.Status = EExecutionReplanAttemptStatus::InvalidReplannedPath;
            OutResult.FailedMissionId = Mission.MissionId;
            return false;
        }

        const FIntVector ObservedCell = (*AgentSnapshot)->ObservedCell;
        TArray<FIntVector> ReplannedCellPath;
        if (bStationaryAnchor)
        {
            ReplannedCellPath.Add(ObservedCell);
        }
        else
        {
            ReplannedCellPath = BuildReplannedCellPath(*Context.BaseGrid, *ReplannedWorldPath);
            if (ReplannedCellPath.Num() <= 0)
            {
                OutResult.Status = EExecutionReplanAttemptStatus::InvalidReplannedPath;
                return false;
            }

            if (ReplannedCellPath[0] != ObservedCell)
            {
                ReplannedCellPath.Insert(ObservedCell, 0);
            }
        }

        OutResult.ReplannedCellPathsByMission.Add(Mission.MissionId, MoveTemp(ReplannedCellPath));
    }

    FExecutionReplanPostCheckInput PostCheckInput;
    PostCheckInput.Snapshot = *Input.Snapshot;
    PostCheckInput.MissionConfigsById = *Context.MissionConfigsById;
    PostCheckInput.ReplannedCellPathsByMission = OutResult.ReplannedCellPathsByMission;
    PostCheckInput.CandidateMissionIds = Input.CandidateMissionIdSet;
    PostCheckInput.GridDim = Context.BaseGrid->GridDim;
    PostCheckInput.bCheckStaticUTMSafety = Input.Spec.bCheckStaticUTMSafety;
    PostCheckInput.LookaheadSteps = Input.Spec.LookaheadSteps;

    const FExecutionReplanPostCheckResult PostCheckResult =
        FExecutionReplanPostCheckPolicy::Validate(PostCheckInput);
    if (PostCheckResult.bHasConflict)
    {
        OutResult.Status = EExecutionReplanAttemptStatus::PostCheckFailed;
        OutResult.PostCheckConflict = PostCheckResult.Conflict;
        OutResult.PostCheckConflictOffset = PostCheckResult.ConflictOffset;
        return false;
    }

    OutResult.Status = EExecutionReplanAttemptStatus::Success;
    return true;
}
