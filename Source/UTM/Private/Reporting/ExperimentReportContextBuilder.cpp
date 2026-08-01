#include "Reporting/ExperimentReportContextBuilder.h"

#include "Planning/PlanningStatsTypes.h"
#include "Reporting/ExecutionSummaryTypes.h"
#include "Reporting/ExperimentMetadataResolver.h"

namespace
{
    template<typename TEnum>
    FString GetExperimentReportEnumName(const TEnum Value)
    {
        if (const UEnum* Enum = StaticEnum<TEnum>())
        {
            return Enum->GetNameStringByValue(static_cast<int64>(Value));
        }

        return TEXT("Unknown");
    }

    EExperimentMetadataReplanMode ToExperimentMetadataReplanModeForReport(
        EExecutionReplanMode ReplanMode)
    {
        switch (ReplanMode)
        {
        case EExecutionReplanMode::Disabled:
            return EExperimentMetadataReplanMode::Disabled;
        case EExecutionReplanMode::LocalConflictSet:
            return EExperimentMetadataReplanMode::LocalConflictSet;
        case EExecutionReplanMode::GlobalUnfinished:
            return EExperimentMetadataReplanMode::GlobalUnfinished;
        default:
            return EExperimentMetadataReplanMode::Disabled;
        }
    }
}

FExperimentReportContextBuildResult FExperimentReportContextBuilder::Build(
    const FExperimentReportContextBuildRequest& Request)
{
    FExperimentReportContextBuildResult Result;
    if (!Request.PlanningStats)
    {
        Result.FailureReason = TEXT("Experiment report context has no planning stats");
        return Result;
    }
    if (!Request.ExecutionSummary)
    {
        Result.FailureReason = TEXT("Experiment report context has no execution summary");
        return Result;
    }
    if (!Request.ReplanTimingStats)
    {
        Result.FailureReason = TEXT("Experiment report context has no replan timing stats");
        return Result;
    }

    const FPlanningTimingStats& PlanningStats = *Request.PlanningStats;
    const FExecutionSummary& ExecutionSummary = *Request.ExecutionSummary;
    const FExecutionReplanTimingStats& ReplanTimingStats =
        *Request.ReplanTimingStats;
    const FExperimentReportRuntimeSettingsInput& Runtime =
        Request.RuntimeSettings;

    const bool bHasExecutionSummary =
        ExecutionSummary.AgentCount > 0 ||
        ExecutionSummary.AgentSummaries.Num() > 0 ||
        ExecutionSummary.CompletedAgentCount > 0 ||
        ExecutionSummary.PlannedMakespan > 0 ||
        ExecutionSummary.ActualMakespan > 0 ||
        Request.ObservedConflictCount > 0;
    const int32 EffectiveAgentCount =
        bHasExecutionSummary
            ? ExecutionSummary.AgentCount
            : PlanningStats.MissionCount;
    const int32 FallbackRunIdAgentCount =
        ExecutionSummary.AgentCount > 0
            ? ExecutionSummary.AgentCount
            : PlanningStats.MissionCount;
    const int32 Expansion =
        bHasExecutionSummary
            ? ExecutionSummary.ActualMakespan - ExecutionSummary.PlannedMakespan
            : 0;
    const FString PlannerName =
        PlanningStats.PlannerName.IsEmpty()
            ? Request.Identity.RequestedPlannerName
            : PlanningStats.PlannerName;
    const bool bNoFlyValidationClear =
        !Request.NoFlyValidation.bValidationEnabled ||
        Request.NoFlyValidation.TotalViolationCount <= 0;
    const int32 ExecutionReplanAttemptCount =
        ReplanTimingStats.LocalAttemptCount +
        ReplanTimingStats.GlobalAttemptCount;
    const double ExecutionReplanTotalTimeMs =
        ReplanTimingStats.LocalTotalTimeMs +
        ReplanTimingStats.GlobalTotalTimeMs;
    const double ExecutionReplanMaxTimeMs = FMath::Max(
        ReplanTimingStats.LocalMaxTimeMs,
        ReplanTimingStats.GlobalMaxTimeMs);

    FExperimentMetadataResolverInput MetadataInput;
    MetadataInput.RunId = Request.Identity.RunId;
    MetadataInput.Phase = Request.Identity.Phase;
    MetadataInput.GroupId = Request.Identity.GroupId;
    MetadataInput.GroupName = Request.Identity.GroupName;
    MetadataInput.ScenarioName = Request.Identity.ScenarioName;
    MetadataInput.FallbackScenarioName = Request.Identity.MapTypeName;
    MetadataInput.PlannerName = Request.Identity.RequestedPlannerName;
    MetadataInput.EffectiveAgentCount = FallbackRunIdAgentCount;
    MetadataInput.StepDelayProbability = Runtime.StepDelayProbability;
    MetadataInput.ExecutionRandomSeed = Request.Environment.ExecutionRandomSeed;
    MetadataInput.bEnableDiscreteAlignment = Runtime.bEnableDiscreteAlignment;
    MetadataInput.bEnableConflictAwareAlignment =
        Runtime.bEnableConflictAwareAlignment;
    MetadataInput.ReplanMode =
        ToExperimentMetadataReplanModeForReport(Runtime.ReplanMode);

    const FExperimentMetadata Metadata =
        FExperimentMetadataResolver::Resolve(MetadataInput);

    FExperimentReportContext& Context = Result.Context;
    Context.RunId = Metadata.RunId;
    Context.Phase = Metadata.Phase;
    Context.GroupId = Metadata.GroupId;
    Context.GroupName = Metadata.GroupName;
    Context.ScenarioName = Metadata.ScenarioName;
    Context.MapTypeName = Request.Identity.MapTypeName;
    Context.PlannerName = PlannerName;
    Context.SchedulerTypeName = Request.Identity.SchedulerTypeName;
    Context.DelayModeName = GetExperimentReportEnumName(Runtime.DelayMode);
    Context.ReplanModeName = GetExperimentReportEnumName(Runtime.ReplanMode);
    Context.Notes = Request.Identity.Notes;

    Context.bPlanningSuccess = PlanningStats.bSuccess;
    Context.bPlanningMultiAgent = PlanningStats.bMultiAgent;
    Context.bExecutionSummaryAvailable = bHasExecutionSummary;
    Context.bAlignmentEnabled = Runtime.bEnableDiscreteAlignment;
    Context.bConflictAwareAlignment = Runtime.bEnableConflictAwareAlignment;
    Context.bAlignmentAllowRecoveryMoves =
        Runtime.bAlignmentAllowRecoveryMoves;
    Context.bAlignmentHoldPositionOnFailure =
        Runtime.bAlignmentHoldPositionOnFailure;
    Context.bValidatePathsAgainstNoFlyZones =
        Request.NoFlyValidation.bValidationEnabled;
    Context.bNoFlyValidationClear = bNoFlyValidationClear;

    Context.MissionCount = PlanningStats.MissionCount;
    Context.AgentCount = EffectiveAgentCount;
    Context.CitySeed = Request.Environment.CitySeed;
    Context.RandomSeed = Request.Environment.RandomSeed;
    Context.ExecutionRandomSeed = Request.Environment.ExecutionRandomSeed;
    Context.StepDelayProbability = Runtime.StepDelayProbability;
    Context.AlignmentSearchRadiusSteps = Runtime.AlignmentSearchRadiusSteps;
    Context.AlignmentMaxSpatialErrorCells =
        Runtime.AlignmentMaxSpatialErrorCells;
    Context.AlignmentMaxSnapAheadSteps = Runtime.AlignmentMaxSnapAheadSteps;
    Context.AlignmentConflictResolutionPasses =
        Runtime.AlignmentConflictResolutionPasses;
    Context.AlignmentConflictHoldThresholdForReplan =
        Runtime.AlignmentConflictHoldThresholdForReplan;
    Context.MaxExecutionReplans = Runtime.MaxExecutionReplans;

    Context.PlanningBuildGridTimeMs = PlanningStats.BuildGridTimeMs;
    Context.PlanningInputPreparationTimeMs =
        PlanningStats.InputPreparationTimeMs;
    Context.PlanningSolveTimeMs = PlanningStats.SolveTimeMs;
    Context.PlanningPostProcessTimeMs = PlanningStats.PostProcessTimeMs;
    Context.InitialPlanningWallTimeMs = PlanningStats.TotalTimeMs;

    Context.NoFlyEnabledZoneCount = Request.NoFlyValidation.EnabledZoneCount;
    Context.NoFlyCheckedMissionCount =
        Request.NoFlyValidation.CheckedMissionCount;
    Context.NoFlyCheckedPointCount = Request.NoFlyValidation.CheckedPointCount;
    Context.NoFlyViolatingMissionCount =
        Request.NoFlyValidation.ViolatingMissionCount;
    Context.NoFlyTotalViolationCount =
        Request.NoFlyValidation.TotalViolationCount;

    Context.CompletedAgentCount = ExecutionSummary.CompletedAgentCount;
    Context.PlannedMakespan = ExecutionSummary.PlannedMakespan;
    Context.ActualMakespan = ExecutionSummary.ActualMakespan;
    Context.Expansion = Expansion;
    Context.TotalDelaySteps = ExecutionSummary.TotalDelaySteps;

    Context.VertexConflictCount = ExecutionSummary.VertexConflictCount;
    Context.EdgeConflictCount = ExecutionSummary.EdgeConflictCount;
    Context.FirstConflictTime = ExecutionSummary.FirstConflictTime;

    Context.UTMStaticConflictCount = ExecutionSummary.UTMStaticConflictCount;
    Context.UTMProtectionConflictCount =
        ExecutionSummary.UTMProtectionConflictCount;
    Context.UTMDownwashConflictCount = ExecutionSummary.UTMDownwashConflictCount;
    Context.FirstUTMConflictTime = ExecutionSummary.FirstUTMConflictTime;

    Context.AlignmentCorrectionCount =
        ExecutionSummary.AlignmentCorrectionCount;
    Context.AlignmentHoldCount = ExecutionSummary.AlignmentHoldCount;
    Context.AlignmentConflictHoldCount =
        ExecutionSummary.AlignmentConflictHoldCount;
    Context.AlignmentSnapCount = ExecutionSummary.AlignmentSnapCount;
    Context.AlignmentReplanRequestCount =
        ExecutionSummary.AlignmentReplanRequestCount;
    Context.AlignmentSuccessfulReplanCount =
        ExecutionSummary.AlignmentSuccessfulReplanCount;

    Context.AppliedExecutionReplans = Request.AppliedExecutionReplans;
    Context.ExecutionReplanAttemptCount = ExecutionReplanAttemptCount;
    Context.ExecutionReplanTotalTimeMs = ExecutionReplanTotalTimeMs;
    Context.ExecutionReplanMaxTimeMs = ExecutionReplanMaxTimeMs;
    Context.ExecutionReplanLocalAttemptCount =
        ReplanTimingStats.LocalAttemptCount;
    Context.ExecutionReplanLocalTotalTimeMs =
        ReplanTimingStats.LocalTotalTimeMs;
    Context.ExecutionReplanLocalMaxTimeMs =
        ReplanTimingStats.LocalMaxTimeMs;
    Context.ExecutionReplanGlobalAttemptCount =
        ReplanTimingStats.GlobalAttemptCount;
    Context.ExecutionReplanGlobalTotalTimeMs =
        ReplanTimingStats.GlobalTotalTimeMs;
    Context.ExecutionReplanGlobalMaxTimeMs =
        ReplanTimingStats.GlobalMaxTimeMs;

    Result.bSuccess = true;
    return Result;
}
