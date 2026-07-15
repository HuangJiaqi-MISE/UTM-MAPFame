#pragma once

#include "CoreMinimal.h"
#include "Planning/DroneMissionTypes.h"
#include "Reporting/ExecutionSummaryTypes.h"

struct FExecutionSummaryAgentInput
{
    int32 MissionId = INDEX_NONE;
    TArray<FIntVector> PlannedCells;
    TArray<FIntVector> ActualCells;
    int32 TotalDelaySteps = 0;
    int32 AlignmentCorrectionCount = 0;
    int32 AlignmentHoldCount = 0;
    int32 AlignmentConflictHoldCount = 0;
    int32 AlignmentSnapCount = 0;
    int32 AlignmentReplanRequestCount = 0;
    int32 AlignmentSuccessfulReplanCount = 0;
    int32 MaxAlignmentSpatialError = 0;
    int32 MaxAlignmentTemporalError = 0;
    bool bAlignmentLost = false;
};

struct FExecutionSummaryConflictInput
{
    int32 TimeStep = -1;
    bool bIsEdgeConflict = false;
};

struct FExecutionSummaryBuildRequest
{
    TMap<int32, FExecutionSummaryAgentInput> AgentStatesByMissionId;
    TArray<FExecutionSummaryConflictInput> Conflicts;
    TMap<int32, FDroneMissionConfig> MissionConfigsByMissionId;
};

class FExecutionSummaryBuilder
{
public:
    static FExecutionSummary Build(const FExecutionSummaryBuildRequest& Request);
};
