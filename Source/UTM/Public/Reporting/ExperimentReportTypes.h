#pragma once

#include "CoreMinimal.h"

struct FExperimentReportContext
{
    FString RunId;
    FString Phase;
    FString GroupId;
    FString GroupName;
    FString ScenarioName;
    FString MapTypeName;
    FString PlannerName;
    FString SchedulerTypeName;
    FString DelayModeName;
    FString ReplanModeName;
    FString Notes;

    bool bPlanningSuccess = false;
    bool bPlanningMultiAgent = false;
    bool bExecutionSummaryAvailable = false;
    bool bAlignmentEnabled = false;
    bool bConflictAwareAlignment = false;
    bool bAlignmentAllowRecoveryMoves = false;
    bool bAlignmentHoldPositionOnFailure = false;
    bool bValidatePathsAgainstNoFlyZones = false;
    bool bNoFlyValidationClear = true;

    int32 MissionCount = 0;
    int32 AgentCount = 0;
    int32 CitySeed = 0;
    int32 RandomSeed = 0;
    int32 ExecutionRandomSeed = 0;
    double StepDelayProbability = 0.0;
    int32 AlignmentSearchRadiusSteps = 0;
    int32 AlignmentMaxSpatialErrorCells = 0;
    int32 AlignmentMaxSnapAheadSteps = 0;
    int32 AlignmentConflictResolutionPasses = 0;
    int32 AlignmentConflictHoldThresholdForReplan = 0;
    int32 MaxExecutionReplans = 0;

    double PlanningBuildGridTimeMs = 0.0;
    double PlanningInputPreparationTimeMs = 0.0;
    double PlanningSolveTimeMs = 0.0;
    double PlanningPostProcessTimeMs = 0.0;
    double InitialPlanningWallTimeMs = 0.0;

    int32 NoFlyEnabledZoneCount = 0;
    int32 NoFlyCheckedMissionCount = 0;
    int32 NoFlyCheckedPointCount = 0;
    int32 NoFlyViolatingMissionCount = 0;
    int32 NoFlyTotalViolationCount = 0;

    int32 CompletedAgentCount = 0;
    int32 PlannedMakespan = 0;
    int32 ActualMakespan = 0;
    int32 Expansion = 0;
    int32 TotalDelaySteps = 0;

    int32 VertexConflictCount = 0;
    int32 EdgeConflictCount = 0;
    int32 FirstConflictTime = -1;

    int32 UTMStaticConflictCount = 0;
    int32 UTMProtectionConflictCount = 0;
    int32 UTMDownwashConflictCount = 0;
    int32 FirstUTMConflictTime = -1;

    int32 AlignmentCorrectionCount = 0;
    int32 AlignmentHoldCount = 0;
    int32 AlignmentConflictHoldCount = 0;
    int32 AlignmentSnapCount = 0;
    int32 AlignmentReplanRequestCount = 0;
    int32 AlignmentSuccessfulReplanCount = 0;

    int32 AppliedExecutionReplans = 0;
    int32 ExecutionReplanAttemptCount = 0;
    double ExecutionReplanTotalTimeMs = 0.0;
    double ExecutionReplanMaxTimeMs = 0.0;
    int32 ExecutionReplanLocalAttemptCount = 0;
    double ExecutionReplanLocalTotalTimeMs = 0.0;
    double ExecutionReplanLocalMaxTimeMs = 0.0;
    int32 ExecutionReplanGlobalAttemptCount = 0;
    double ExecutionReplanGlobalTotalTimeMs = 0.0;
    double ExecutionReplanGlobalMaxTimeMs = 0.0;
};
