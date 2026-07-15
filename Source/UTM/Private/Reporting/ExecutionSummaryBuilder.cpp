#include "Reporting/ExecutionSummaryBuilder.h"

#include "Planning/UTMSafetyModel.h"

namespace
{
    FIntVector GetCellAtTime(
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

    int32 ComputeFirstMismatchTime(const FExecutionSummaryAgentInput& Agent)
    {
        const int32 MaxSteps =
            FMath::Max(Agent.PlannedCells.Num(), Agent.ActualCells.Num());
        for (int32 TimeStep = 0; TimeStep < MaxSteps; ++TimeStep)
        {
            if (GetCellAtTime(Agent.PlannedCells, TimeStep) !=
                GetCellAtTime(Agent.ActualCells, TimeStep))
            {
                return TimeStep;
            }
        }

        return -1;
    }

    FExecutionAgentSummary BuildAgentSummary(
        const FExecutionSummaryAgentInput& Agent)
    {
        FExecutionAgentSummary Item;
        Item.MissionId = Agent.MissionId;
        Item.PlannedCellCount = Agent.PlannedCells.Num();
        Item.ActualCellCount = Agent.ActualCells.Num();
        Item.PlannedMakespan = FMath::Max(0, Agent.PlannedCells.Num() - 1);
        Item.ActualMakespan = FMath::Max(0, Agent.ActualCells.Num() - 1);
        Item.TotalDelaySteps = Agent.TotalDelaySteps;
        Item.FirstMismatchTime = ComputeFirstMismatchTime(Agent);
        Item.bReachedGoal =
            Agent.PlannedCells.Num() > 0 &&
            Agent.ActualCells.Num() > 0 &&
            Agent.PlannedCells.Last() == Agent.ActualCells.Last();
        Item.AlignmentCorrectionCount = Agent.AlignmentCorrectionCount;
        Item.AlignmentHoldCount = Agent.AlignmentHoldCount;
        Item.AlignmentConflictHoldCount = Agent.AlignmentConflictHoldCount;
        Item.AlignmentSnapCount = Agent.AlignmentSnapCount;
        Item.AlignmentReplanRequestCount = Agent.AlignmentReplanRequestCount;
        Item.AlignmentSuccessfulReplanCount = Agent.AlignmentSuccessfulReplanCount;
        Item.MaxAlignmentSpatialError = Agent.MaxAlignmentSpatialError;
        Item.MaxAlignmentTemporalError = Agent.MaxAlignmentTemporalError;
        Item.bAlignmentLost = Agent.bAlignmentLost;
        return Item;
    }

    void AddAgentSummary(
        const FExecutionAgentSummary& Item,
        FExecutionSummary& InOutSummary)
    {
        if (Item.bReachedGoal)
        {
            InOutSummary.CompletedAgentCount++;
        }

        InOutSummary.PlannedMakespan =
            FMath::Max(InOutSummary.PlannedMakespan, Item.PlannedMakespan);
        InOutSummary.ActualMakespan =
            FMath::Max(InOutSummary.ActualMakespan, Item.ActualMakespan);
        InOutSummary.TotalDelaySteps += Item.TotalDelaySteps;
        InOutSummary.AlignmentCorrectionCount += Item.AlignmentCorrectionCount;
        InOutSummary.AlignmentHoldCount += Item.AlignmentHoldCount;
        InOutSummary.AlignmentConflictHoldCount += Item.AlignmentConflictHoldCount;
        InOutSummary.AlignmentSnapCount += Item.AlignmentSnapCount;
        InOutSummary.AlignmentReplanRequestCount += Item.AlignmentReplanRequestCount;
        InOutSummary.AlignmentSuccessfulReplanCount +=
            Item.AlignmentSuccessfulReplanCount;
        InOutSummary.AgentSummaries.Add(Item);
    }
}

FExecutionSummary FExecutionSummaryBuilder::Build(
    const FExecutionSummaryBuildRequest& Request)
{
    FExecutionSummary Summary;
    Summary.AgentCount = Request.AgentStatesByMissionId.Num();

    for (const TPair<int32, FExecutionSummaryAgentInput>& Pair :
        Request.AgentStatesByMissionId)
    {
        AddAgentSummary(BuildAgentSummary(Pair.Value), Summary);
    }

    for (const FExecutionSummaryConflictInput& Conflict : Request.Conflicts)
    {
        if (Conflict.bIsEdgeConflict)
        {
            Summary.EdgeConflictCount++;
        }
        else
        {
            Summary.VertexConflictCount++;
        }

        if (Summary.FirstConflictTime < 0 ||
            Conflict.TimeStep < Summary.FirstConflictTime)
        {
            Summary.FirstConflictTime = Conflict.TimeStep;
        }
    }

    TArray<int32> MissionIds;
    Request.AgentStatesByMissionId.GetKeys(MissionIds);
    MissionIds.Sort();

    for (int32 TimeStep = 0; TimeStep <= Summary.ActualMakespan; ++TimeStep)
    {
        for (int32 I = 0; I < MissionIds.Num(); ++I)
        {
            const int32 MissionIdA = MissionIds[I];
            const FExecutionSummaryAgentInput* AgentA =
                Request.AgentStatesByMissionId.Find(MissionIdA);
            const FDroneMissionConfig* ConfigA =
                Request.MissionConfigsByMissionId.Find(MissionIdA);
            if (!AgentA || !ConfigA || AgentA->ActualCells.Num() <= 0)
            {
                continue;
            }

            const FIntVector CellA = GetCellAtTime(AgentA->ActualCells, TimeStep);

            for (int32 J = I + 1; J < MissionIds.Num(); ++J)
            {
                const int32 MissionIdB = MissionIds[J];
                const FExecutionSummaryAgentInput* AgentB =
                    Request.AgentStatesByMissionId.Find(MissionIdB);
                const FDroneMissionConfig* ConfigB =
                    Request.MissionConfigsByMissionId.Find(MissionIdB);
                if (!AgentB || !ConfigB || AgentB->ActualCells.Num() <= 0)
                {
                    continue;
                }

                const FIntVector CellB =
                    GetCellAtTime(AgentB->ActualCells, TimeStep);
                const EStaticUTMConflictType ConflictType =
                    FUTMSafetyModel::GetStaticUTMConfigConflictType(
                        CellA,
                        *ConfigA,
                        CellB,
                        *ConfigB);
                if (ConflictType == EStaticUTMConflictType::None)
                {
                    continue;
                }

                Summary.UTMStaticConflictCount++;
                if (ConflictType == EStaticUTMConflictType::ProtectionFootprint)
                {
                    Summary.UTMProtectionConflictCount++;
                }
                else if (ConflictType == EStaticUTMConflictType::Downwash)
                {
                    Summary.UTMDownwashConflictCount++;
                }

                if (Summary.FirstUTMConflictTime < 0 ||
                    TimeStep < Summary.FirstUTMConflictTime)
                {
                    Summary.FirstUTMConflictTime = TimeStep;
                }

                if (Summary.FirstConflictTime < 0 ||
                    TimeStep < Summary.FirstConflictTime)
                {
                    Summary.FirstConflictTime = TimeStep;
                }
            }
        }
    }

    Summary.AgentSummaries.Sort(
        [](const FExecutionAgentSummary& A, const FExecutionAgentSummary& B)
        {
            return A.MissionId < B.MissionId;
        });
    return Summary;
}
