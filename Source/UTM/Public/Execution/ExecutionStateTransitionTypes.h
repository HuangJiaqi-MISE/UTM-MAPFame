#pragma once

#include "CoreMinimal.h"

struct FExecutionStateTransitionState
{
    int32 ExecutedPlanIndex = 0;
    int32 TotalDelaySteps = 0;
    bool bFinished = false;
    int32 AlignmentCorrectionCount = 0;
    int32 AlignmentHoldCount = 0;
    int32 AlignmentConflictHoldCount = 0;
    int32 AlignmentSnapCount = 0;
    int32 AlignmentReplanRequestCount = 0;
    int32 AlignmentSuccessfulReplanCount = 0;
    int32 MaxAlignmentSpatialError = 0;
    int32 MaxAlignmentTemporalError = 0;
    bool bAlignmentLost = false;
    int32 ConsecutiveConflictHoldCount = 0;
    int32 ConsecutiveSafetyGateHoldCount = 0;
};

struct FExecutionStateTransitionInput
{
    FExecutionStateTransitionState CurrentState;
    int32 PlannedCellCount = 0;
    FIntVector FinalPlannedCell = FIntVector::ZeroValue;
    bool bReplanRequestedForState = false;
    bool bOriginallyRequestedForReplan = false;
    bool bReplannedForState = false;
    bool bReplanSucceeded = false;
};

struct FExecutionStateTransitionResult
{
    FExecutionStateTransitionState NextState;
    FIntVector CommittedCell = FIntVector::ZeroValue;
};
