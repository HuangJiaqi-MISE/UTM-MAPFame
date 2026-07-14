#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"
#include "Planning/DroneMissionTypes.h"

struct FExecutionReplanCandidateSelectionInput
{
    FExecutionSnapshot Snapshot;
    TMap<int32, FDroneMissionConfig> MissionConfigsById;
    TSet<int32> RequestedMissionIds;
    bool bGlobalReplan = false;
    bool bCheckStaticUTMSafety = false;
    int32 SpatialRadiusCells = 0;
    int32 LookaheadSteps = 0;
};

struct FExecutionReplanCandidateSelectionResult
{
    TSet<int32> CandidateMissionIds;
    int32 ActiveRequestedMissionCount = 0;
    bool bExpandedLocalComponent = false;
};

class FExecutionReplanCandidateSelector
{
public:
    static FExecutionReplanCandidateSelectionResult Select(
        const FExecutionReplanCandidateSelectionInput& Input);
};
