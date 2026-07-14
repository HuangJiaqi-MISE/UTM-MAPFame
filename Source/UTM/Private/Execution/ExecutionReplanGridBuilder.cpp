#include "Execution/ExecutionReplanGridBuilder.h"

#include "Planning/UTMSafetyModel.h"

namespace
{
    FIntVector GetCellAtTime(const TArray<FIntVector>& Cells, int32 TimeStep)
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

    FIntVector GetPredictedCellAtOffset(const FExecutionAgentSnapshot& Agent, int32 Offset)
    {
        if (Offset <= 0 || Agent.PlannedCells.Num() <= 0)
        {
            return Agent.ObservedCell;
        }

        const int32 BaseIndex = FMath::Clamp(Agent.ExecutedPlanIndex, 0, Agent.PlannedCells.Num() - 1);
        return GetCellAtTime(Agent.PlannedCells, BaseIndex + Offset);
    }

    int32 GetCellDistance(const FIntVector& A, const FIntVector& B)
    {
        return FMath::Max3(
            FMath::Abs(A.X - B.X),
            FMath::Abs(A.Y - B.Y),
            FMath::Abs(A.Z - B.Z));
    }

    bool HaveStaticUTMCoupling(
        const FExecutionReplanGridBuildInput& Input,
        const FIntVector& CellA,
        int32 MissionIdA,
        const FIntVector& CellB,
        int32 MissionIdB)
    {
        if (!Input.bCheckStaticUTMSafety)
        {
            return false;
        }

        const FDroneMissionConfig* MissionConfigA = Input.MissionConfigsById.Find(MissionIdA);
        const FDroneMissionConfig* MissionConfigB = Input.MissionConfigsById.Find(MissionIdB);
        if (!MissionConfigA || !MissionConfigB)
        {
            return false;
        }

        return FUTMSafetyModel::HasStaticUTMConfigConflict(CellA, *MissionConfigA, CellB, *MissionConfigB);
    }

    bool IsAnchorRelevantToCandidates(
        const FExecutionReplanGridBuildInput& Input,
        const TArray<int32>& CandidateMissionIds,
        const TMap<int32, const FExecutionAgentSnapshot*>& AgentsByMissionId,
        const FExecutionAgentSnapshot& AnchorAgent,
        const FDroneMissionConfig& AnchorConfig)
    {
        const FIntVector AnchorCell = AnchorAgent.ObservedCell;
        const int32 AnchorInfluenceRadius = FUTMSafetyModel::GetMissionInfluenceRadiusCells(AnchorConfig);
        const int32 AnchorLookaheadSteps = FMath::Max(0, Input.LookaheadSteps);
        const int32 AnchorSpatialRadiusCells = FMath::Max(0, Input.SpatialRadiusCells);

        for (const int32 CandidateMissionId : CandidateMissionIds)
        {
            const FExecutionAgentSnapshot* const* CandidateAgent = AgentsByMissionId.Find(CandidateMissionId);
            const FDroneMissionConfig* CandidateConfig = Input.MissionConfigsById.Find(CandidateMissionId);
            if (!CandidateAgent || !*CandidateAgent || !CandidateConfig)
            {
                continue;
            }

            const int32 EffectiveRadiusCells = AnchorSpatialRadiusCells
                + AnchorInfluenceRadius
                + FUTMSafetyModel::GetMissionInfluenceRadiusCells(*CandidateConfig);

            auto IsCandidateCellRelevant = [&](const FIntVector& CandidateCell) -> bool
                {
                    return CandidateCell == AnchorCell
                        || HaveStaticUTMCoupling(Input, CandidateCell, CandidateMissionId, AnchorCell, AnchorAgent.MissionId)
                        || (EffectiveRadiusCells > 0 && GetCellDistance(CandidateCell, AnchorCell) <= EffectiveRadiusCells);
                };

            for (int32 Offset = 0; Offset <= AnchorLookaheadSteps; ++Offset)
            {
                if (IsCandidateCellRelevant(GetPredictedCellAtOffset(**CandidateAgent, Offset)))
                {
                    return true;
                }
            }

            if (IsCandidateCellRelevant((*CandidateAgent)->GoalCell))
            {
                return true;
            }
        }

        return false;
    }

    void MarkBlockedCell(FGridMap3D& ReplanGrid, const FIntVector& Cell)
    {
        if (!ReplanGrid.IsInside(Cell.X, Cell.Y, Cell.Z) || ReplanGrid.Occupancy.Num() <= 0)
        {
            return;
        }

        const int32 Index = ReplanGrid.ToIndex(Cell.X, Cell.Y, Cell.Z);
        if (ReplanGrid.Occupancy.IsValidIndex(Index))
        {
            ReplanGrid.Occupancy[Index] = 1;
        }
    }

    void MarkStaticAnchorFootprint(
        FExecutionReplanGridBuildResult& Result,
        const FExecutionReplanGridBuildInput& Input,
        const TArray<int32>& CandidateMissionIds,
        const FExecutionAgentSnapshot& AnchorAgent,
        const FDroneMissionConfig& AnchorConfig,
        TSet<FIntVector>& StaticAnchorBlockedCells)
    {
        const FIntVector AnchorCell = AnchorAgent.ObservedCell;
        const int32 AnchorInfluenceRadius = FUTMSafetyModel::GetMissionInfluenceRadiusCells(AnchorConfig);

        for (const int32 CandidateMissionId : CandidateMissionIds)
        {
            const FDroneMissionConfig* CandidateConfig = Input.MissionConfigsById.Find(CandidateMissionId);
            if (!CandidateConfig)
            {
                continue;
            }

            const int32 SearchRadiusCells = FMath::Max(
                0,
                AnchorInfluenceRadius + FUTMSafetyModel::GetMissionInfluenceRadiusCells(*CandidateConfig));

            for (int32 Z = AnchorCell.Z - SearchRadiusCells; Z <= AnchorCell.Z + SearchRadiusCells; ++Z)
            {
                for (int32 Y = AnchorCell.Y - SearchRadiusCells; Y <= AnchorCell.Y + SearchRadiusCells; ++Y)
                {
                    for (int32 X = AnchorCell.X - SearchRadiusCells; X <= AnchorCell.X + SearchRadiusCells; ++X)
                    {
                        const FIntVector CandidateCell(X, Y, Z);
                        if (!Result.ReplanGrid.IsInside(CandidateCell.X, CandidateCell.Y, CandidateCell.Z))
                        {
                            continue;
                        }

                        if (!FUTMSafetyModel::HasStaticUTMConfigConflict(CandidateCell, *CandidateConfig, AnchorCell, AnchorConfig))
                        {
                            continue;
                        }

                        if (!StaticAnchorBlockedCells.Contains(CandidateCell))
                        {
                            StaticAnchorBlockedCells.Add(CandidateCell);
                            Result.StaticAnchorBlockedCellCount++;
                        }
                        MarkBlockedCell(Result.ReplanGrid, CandidateCell);
                    }
                }
            }
        }
    }
}

FExecutionReplanGridBuildResult FExecutionReplanGridBuilder::Build(
    const FExecutionReplanGridBuildInput& Input)
{
    FExecutionReplanGridBuildResult Result;

    if (!Input.BaseGrid)
    {
        Result.FailureReason = TEXT("missing base grid");
        return Result;
    }

    Result.ReplanGrid = *Input.BaseGrid;

    TArray<int32> CandidateMissionIds;
    CandidateMissionIds.Reserve(Input.CandidateMissionIds.Num());
    for (const int32 CandidateMissionId : Input.CandidateMissionIds)
    {
        CandidateMissionIds.Add(CandidateMissionId);
    }
    CandidateMissionIds.Sort();

    TMap<int32, const FExecutionAgentSnapshot*> AgentsByMissionId;
    AgentsByMissionId.Reserve(Input.Snapshot.Agents.Num());
    for (const FExecutionAgentSnapshot& Agent : Input.Snapshot.Agents)
    {
        AgentsByMissionId.Add(Agent.MissionId, &Agent);
    }

    if (Input.bCheckStaticUTMSafety)
    {
        Result.AnchorMissionIds.Reserve(Input.Snapshot.Agents.Num());
        for (const FExecutionAgentSnapshot& Agent : Input.Snapshot.Agents)
        {
            if (!Agent.bFinished || Input.CandidateMissionIds.Contains(Agent.MissionId))
            {
                continue;
            }

            const FDroneMissionConfig* MissionConfig = Input.MissionConfigsById.Find(Agent.MissionId);
            const bool bForcedAnchor = Input.ForcedAnchorMissionIds.Contains(Agent.MissionId);
            if (!MissionConfig
                || (!bForcedAnchor && !IsAnchorRelevantToCandidates(Input, CandidateMissionIds, AgentsByMissionId, Agent, *MissionConfig)))
            {
                continue;
            }

            Result.AnchorMissionIds.Add(Agent.MissionId);
            Result.AnchorMissionIdSet.Add(Agent.MissionId);
        }
    }

    Result.AnchorMissionIds.Sort();

    if (!Input.bGlobalReplan)
    {
        for (const FExecutionAgentSnapshot& Agent : Input.Snapshot.Agents)
        {
            if (Agent.bFinished || Input.CandidateMissionIds.Contains(Agent.MissionId))
            {
                continue;
            }

            MarkBlockedCell(Result.ReplanGrid, Agent.ObservedCell);
        }
    }

    TSet<FIntVector> StaticAnchorBlockedCells;
    for (const int32 AnchorMissionId : Result.AnchorMissionIds)
    {
        const FExecutionAgentSnapshot* const* AnchorAgent = AgentsByMissionId.Find(AnchorMissionId);
        const FDroneMissionConfig* AnchorConfig = Input.MissionConfigsById.Find(AnchorMissionId);
        if (!AnchorAgent || !*AnchorAgent || !AnchorConfig)
        {
            Result.FailureReason = FString::Printf(
                TEXT("missing static-anchor state or mission config for Mission %d"),
                AnchorMissionId);
            return Result;
        }

        MarkStaticAnchorFootprint(Result, Input, CandidateMissionIds, **AnchorAgent, *AnchorConfig, StaticAnchorBlockedCells);
    }

    Result.bSuccess = true;
    return Result;
}
