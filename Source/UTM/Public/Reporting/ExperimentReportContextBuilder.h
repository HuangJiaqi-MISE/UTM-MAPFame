#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionRuntimeStateTypes.h"
#include "Reporting/ExperimentReportTypes.h"

struct FExecutionSummary;
struct FPlanningTimingStats;

struct FExperimentReportIdentityInput
{
    FString RunId;
    FString Phase;
    FString GroupId;
    FString GroupName;
    FString ScenarioName;
    FString MapTypeName;
    FString RequestedPlannerName;
    FString SchedulerTypeName;
    FString Notes;
};

struct FExperimentReportRuntimeSettingsInput
{
    EExecutionDelayMode DelayMode = EExecutionDelayMode::RandomGlobal;
    EExecutionReplanMode ReplanMode = EExecutionReplanMode::GlobalUnfinished;
    double StepDelayProbability = 0.0;
    bool bEnableDiscreteAlignment = false;
    bool bEnableConflictAwareAlignment = false;
    bool bAlignmentAllowRecoveryMoves = false;
    bool bAlignmentHoldPositionOnFailure = false;
    int32 AlignmentSearchRadiusSteps = 0;
    int32 AlignmentMaxSpatialErrorCells = 0;
    int32 AlignmentMaxSnapAheadSteps = 0;
    int32 AlignmentConflictResolutionPasses = 0;
    int32 AlignmentConflictHoldThresholdForReplan = 0;
    int32 MaxExecutionReplans = 0;
};

struct FExperimentReportEnvironmentInput
{
    int32 CitySeed = 0;
    int32 RandomSeed = 0;
    int32 ExecutionRandomSeed = 0;
};

struct FExperimentReportNoFlyValidationInput
{
    bool bValidationEnabled = false;
    int32 EnabledZoneCount = 0;
    int32 CheckedMissionCount = 0;
    int32 CheckedPointCount = 0;
    int32 ViolatingMissionCount = 0;
    int32 TotalViolationCount = 0;
};

struct FExperimentReportContextBuildRequest
{
    FExperimentReportIdentityInput Identity;
    FExperimentReportRuntimeSettingsInput RuntimeSettings;
    FExperimentReportEnvironmentInput Environment;
    FExperimentReportNoFlyValidationInput NoFlyValidation;
    const FPlanningTimingStats* PlanningStats = nullptr;
    const FExecutionSummary* ExecutionSummary = nullptr;
    const FExecutionReplanTimingStats* ReplanTimingStats = nullptr;
    int32 ObservedConflictCount = 0;
    int32 AppliedExecutionReplans = 0;
};

struct FExperimentReportContextBuildResult
{
    bool bSuccess = false;
    FString FailureReason;
    FExperimentReportContext Context;
};

class FExperimentReportContextBuilder
{
public:
    static FExperimentReportContextBuildResult Build(
        const FExperimentReportContextBuildRequest& Request);
};
