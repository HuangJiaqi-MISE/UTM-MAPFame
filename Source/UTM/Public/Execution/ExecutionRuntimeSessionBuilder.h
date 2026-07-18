#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionRuntimeStateTypes.h"
#include "Execution/ExecutionTypes.h"
#include "Planning/DroneMissionTypes.h"

class FGridMap3D;

struct FExecutionRuntimeSessionInitializeRequest
{
    const FGridMap3D* GridMap = nullptr;
    const TMap<int32, TArray<FIntVector>>* PlannedCellPathsByMission = nullptr;
    const TMap<int32, FDroneMissionConfig>* MissionConfigsByMissionId = nullptr;
};

struct FExecutionRuntimeSessionInitializeResult
{
    TMap<int32, FExecutionAgentState> AgentStatesByMissionId;
    bool bRunning = false;
};

struct FExecutionRuntimeSnapshotBuildRequest
{
    const FGridMap3D* GridMap = nullptr;
    const TMap<int32, FExecutionAgentState>* AgentStatesByMissionId = nullptr;
    int32 TimeStep = 0;
    int32 TotalReplanCount = 0;
    TFunction<FIntVector(const FExecutionAgentState&)> ResolveObservedCell;
};

class FExecutionRuntimeSessionBuilder
{
public:
    static TMap<int32, FDroneMissionConfig> BuildMissionConfigsById(
        const TArray<FDroneMissionConfig>& Missions);

    static FExecutionRuntimeSessionInitializeResult BuildInitialState(
        const FExecutionRuntimeSessionInitializeRequest& Request);

    static FExecutionSnapshot BuildSnapshot(
        const FExecutionRuntimeSnapshotBuildRequest& Request);
};
