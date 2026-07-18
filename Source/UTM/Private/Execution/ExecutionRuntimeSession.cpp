#include "Execution/ExecutionRuntimeSession.h"

void FExecutionRuntimeSession::Reset()
{
    MissionConfigsById.Reset();
    AgentStatesByMissionId.Reset();
    Conflicts.Reset();
    bRunning = false;
    TimeStep = 0;
    TotalReplanCount = 0;
    ReplanTimingStats = FExecutionReplanTimingStats();
}

void FExecutionRuntimeSession::PrepareForExecution(int32 RandomSeed)
{
    AgentStatesByMissionId.Reset();
    Conflicts.Reset();
    Random.Initialize(RandomSeed);
    bRunning = false;
    TimeStep = 0;
    TotalReplanCount = 0;
    ReplanTimingStats = FExecutionReplanTimingStats();
}
