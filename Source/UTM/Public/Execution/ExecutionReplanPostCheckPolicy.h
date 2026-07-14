#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionTypes.h"
#include "Planning/DroneMissionTypes.h"

struct FExecutionReplanPostCheckInput
{
    FExecutionSnapshot Snapshot;
    TMap<int32, FDroneMissionConfig> MissionConfigsById;
    TMap<int32, TArray<FIntVector>> ReplannedCellPathsByMission;
    TSet<int32> CandidateMissionIds;
    FIntVector GridDim = FIntVector::ZeroValue;
    bool bCheckStaticUTMSafety = false;
    int32 LookaheadSteps = 0;
};

struct FExecutionReplanPostCheckResult
{
    bool bHasConflict = false;
    FExecutionPredictedConflict Conflict;
    int32 ConflictOffset = 0;
};

class FExecutionReplanPostCheckPolicy
{
public:
    static FExecutionReplanPostCheckResult Validate(const FExecutionReplanPostCheckInput& Input);
};
