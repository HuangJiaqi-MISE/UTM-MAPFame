#include "Execution/ExecutionRuntimeSessionBuilder.h"

#include "Planning/GridMap3D.h"

TMap<int32, FDroneMissionConfig>
FExecutionRuntimeSessionBuilder::BuildMissionConfigsById(
    const TArray<FDroneMissionConfig>& Missions)
{
    TMap<int32, FDroneMissionConfig> MissionConfigsById;
    for (const FDroneMissionConfig& Mission : Missions)
    {
        MissionConfigsById.Add(Mission.MissionId, Mission);
    }

    return MissionConfigsById;
}

FExecutionRuntimeSessionInitializeResult
FExecutionRuntimeSessionBuilder::BuildInitialState(
    const FExecutionRuntimeSessionInitializeRequest& Request)
{
    FExecutionRuntimeSessionInitializeResult Result;
    if (!Request.GridMap ||
        !Request.PlannedCellPathsByMission ||
        !Request.MissionConfigsByMissionId)
    {
        return Result;
    }

    for (const TPair<int32, TArray<FIntVector>>& Pair :
         *Request.PlannedCellPathsByMission)
    {
        const int32 MissionId = Pair.Key;
        const TArray<FIntVector>& PlannedCells = Pair.Value;
        if (PlannedCells.Num() <= 0)
        {
            continue;
        }

        FExecutionAgentState State;
        State.MissionId = MissionId;
        State.PlannedCells = PlannedCells;
        State.ActualCells.Add(PlannedCells[0]);
        State.ExecutedPlanIndex = 0;
        State.TotalDelaySteps = 0;
        State.bFinished = (PlannedCells.Num() <= 1);
        State.DisplayFromCell = PlannedCells[0];
        State.DisplayToCell = PlannedCells[0];
        State.LastObservedCell = PlannedCells[0];
        State.GoalCell = PlannedCells.Last();
        State.GoalWorld = Request.GridMap->CellToWorld(PlannedCells.Last());
        State.ConsecutiveConflictHoldCount = 0;
        State.ConsecutiveSafetyGateHoldCount = 0;
        State.LastAlignmentAction = TEXT("Initialize");

        if (const FDroneMissionConfig* MissionConfig =
            Request.MissionConfigsByMissionId->Find(MissionId))
        {
            State.GoalWorld = MissionConfig->GoalWorld;
            State.GoalCell =
                Request.GridMap->WorldToCell(MissionConfig->GoalWorld);
        }

        Result.AgentStatesByMissionId.Add(MissionId, MoveTemp(State));
    }

    Result.bRunning = (Result.AgentStatesByMissionId.Num() > 0);
    return Result;
}

FExecutionSnapshot FExecutionRuntimeSessionBuilder::BuildSnapshot(
    const FExecutionRuntimeSnapshotBuildRequest& Request)
{
    FExecutionSnapshot Snapshot;
    Snapshot.TimeStep = Request.TimeStep;
    Snapshot.TotalReplanCount = Request.TotalReplanCount;
    if (!Request.GridMap ||
        !Request.AgentStatesByMissionId ||
        !Request.ResolveObservedCell)
    {
        return Snapshot;
    }

    Snapshot.Agents.Reserve(Request.AgentStatesByMissionId->Num());
    for (const TPair<int32, FExecutionAgentState>& Pair :
         *Request.AgentStatesByMissionId)
    {
        const FExecutionAgentState& State = Pair.Value;
        const FIntVector ObservedCell = Request.ResolveObservedCell(State);

        FExecutionAgentSnapshot AgentSnapshot;
        AgentSnapshot.MissionId = State.MissionId;
        AgentSnapshot.bFinished = State.bFinished;
        AgentSnapshot.ObservedCell = ObservedCell;
        AgentSnapshot.ObservedWorld = Request.GridMap->CellToWorld(ObservedCell);
        AgentSnapshot.GoalCell = State.GoalCell;
        AgentSnapshot.GoalWorld = State.GoalWorld;
        AgentSnapshot.TimeStep = Request.TimeStep;
        AgentSnapshot.ExecutedPlanIndex = State.ExecutedPlanIndex;
        AgentSnapshot.ConsecutiveConflictHoldCount =
            State.ConsecutiveConflictHoldCount;
        AgentSnapshot.ConsecutiveSafetyGateHoldCount =
            State.ConsecutiveSafetyGateHoldCount;
        AgentSnapshot.PlannedCells = State.PlannedCells;
        Snapshot.Agents.Add(MoveTemp(AgentSnapshot));
    }

    return Snapshot;
}
