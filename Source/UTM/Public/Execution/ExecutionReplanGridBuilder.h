#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"
#include "Planning/DroneMissionTypes.h"
#include "Planning/GridMap3D.h"

struct FExecutionReplanGridBuildInput
{
    const FGridMap3D* BaseGrid = nullptr;
    FExecutionSnapshot Snapshot;
    TMap<int32, FDroneMissionConfig> MissionConfigsById;
    TSet<int32> CandidateMissionIds;
    TSet<int32> ForcedAnchorMissionIds;
    bool bGlobalReplan = false;
    bool bCheckStaticUTMSafety = false;
    int32 SpatialRadiusCells = 0;
    int32 LookaheadSteps = 0;
};

struct FExecutionReplanGridBuildResult
{
    bool bSuccess = false;
    FGridMap3D ReplanGrid;
    TArray<int32> AnchorMissionIds;
    TSet<int32> AnchorMissionIdSet;
    int32 StaticAnchorBlockedCellCount = 0;
    FString FailureReason;
};

class FExecutionReplanGridBuilder
{
public:
    static FExecutionReplanGridBuildResult Build(const FExecutionReplanGridBuildInput& Input);
};
