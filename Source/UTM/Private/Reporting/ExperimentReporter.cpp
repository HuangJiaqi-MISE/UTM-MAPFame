#include "Reporting/ExperimentReporter.h"

#include "Dom/JsonObject.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

FString FExperimentReporter::BuildStructuredSummaryJson(const FExperimentReportContext& Context)
{
    TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetStringField(TEXT("run_id"), Context.RunId);
    Root->SetStringField(TEXT("phase"), Context.Phase);
    Root->SetStringField(TEXT("group_id"), Context.GroupId);
    Root->SetStringField(TEXT("group_name"), Context.GroupName);
    Root->SetStringField(TEXT("scenario_name"), Context.ScenarioName);
    Root->SetStringField(TEXT("map_type"), Context.MapTypeName);
    Root->SetStringField(TEXT("planner_name"), Context.PlannerName);
    Root->SetStringField(TEXT("planner_type"), Context.PlannerName);
    Root->SetStringField(TEXT("scheduler_type"), Context.SchedulerTypeName);
    Root->SetStringField(TEXT("delay_mode"), Context.DelayModeName);
    Root->SetStringField(TEXT("replan_mode"), Context.ReplanModeName);
    Root->SetStringField(TEXT("execution_replan_mode"), Context.ReplanModeName);
    Root->SetStringField(TEXT("notes"), Context.Notes);

    Root->SetBoolField(TEXT("planning_success"), Context.bPlanningSuccess);
    Root->SetBoolField(TEXT("planning_multi_agent"), Context.bPlanningMultiAgent);
    Root->SetBoolField(TEXT("execution_summary_available"), Context.bExecutionSummaryAvailable);
    Root->SetBoolField(TEXT("alignment_enabled"), Context.bAlignmentEnabled);
    Root->SetBoolField(TEXT("b_enable_discrete_alignment"), Context.bAlignmentEnabled);
    Root->SetBoolField(TEXT("conflict_aware_alignment"), Context.bConflictAwareAlignment);
    Root->SetBoolField(TEXT("b_enable_conflict_aware_alignment"), Context.bConflictAwareAlignment);
    Root->SetBoolField(TEXT("b_alignment_allow_recovery_moves"), Context.bAlignmentAllowRecoveryMoves);
    Root->SetBoolField(TEXT("b_alignment_hold_position_on_failure"), Context.bAlignmentHoldPositionOnFailure);
    Root->SetBoolField(TEXT("b_validate_paths_against_no_fly_zones"), Context.bValidatePathsAgainstNoFlyZones);
    Root->SetBoolField(TEXT("no_fly_validation_clear"), Context.bNoFlyValidationClear);

    Root->SetNumberField(TEXT("mission_count"), Context.MissionCount);
    Root->SetNumberField(TEXT("agent_count"), Context.AgentCount);
    Root->SetNumberField(TEXT("city_seed"), Context.CitySeed);
    Root->SetNumberField(TEXT("random_seed"), Context.RandomSeed);
    Root->SetNumberField(TEXT("execution_random_seed"), Context.ExecutionRandomSeed);
    Root->SetNumberField(TEXT("step_delay_probability"), Context.StepDelayProbability);
    Root->SetNumberField(TEXT("alignment_search_radius_steps"), Context.AlignmentSearchRadiusSteps);
    Root->SetNumberField(TEXT("alignment_max_spatial_error_cells"), Context.AlignmentMaxSpatialErrorCells);
    Root->SetNumberField(TEXT("alignment_max_snap_ahead_steps"), Context.AlignmentMaxSnapAheadSteps);
    Root->SetNumberField(TEXT("alignment_conflict_resolution_passes"), Context.AlignmentConflictResolutionPasses);
    Root->SetNumberField(TEXT("alignment_conflict_hold_threshold_for_replan"), Context.AlignmentConflictHoldThresholdForReplan);
    Root->SetNumberField(TEXT("max_execution_replans"), Context.MaxExecutionReplans);

    Root->SetNumberField(TEXT("planning_build_grid_time_ms"), Context.PlanningBuildGridTimeMs);
    Root->SetNumberField(TEXT("planning_input_preparation_time_ms"), Context.PlanningInputPreparationTimeMs);
    Root->SetNumberField(TEXT("planning_solve_time_ms"), Context.PlanningSolveTimeMs);
    Root->SetNumberField(TEXT("planning_post_process_time_ms"), Context.PlanningPostProcessTimeMs);
    Root->SetNumberField(TEXT("initial_planning_wall_time_ms"), Context.InitialPlanningWallTimeMs);

    Root->SetNumberField(TEXT("no_fly_enabled_zone_count"), Context.NoFlyEnabledZoneCount);
    Root->SetNumberField(TEXT("no_fly_checked_mission_count"), Context.NoFlyCheckedMissionCount);
    Root->SetNumberField(TEXT("no_fly_checked_point_count"), Context.NoFlyCheckedPointCount);
    Root->SetNumberField(TEXT("no_fly_violating_mission_count"), Context.NoFlyViolatingMissionCount);
    Root->SetNumberField(TEXT("no_fly_total_violation_count"), Context.NoFlyTotalViolationCount);

    Root->SetNumberField(TEXT("completed_agent_count"), Context.CompletedAgentCount);
    Root->SetNumberField(TEXT("planned_makespan"), Context.PlannedMakespan);
    Root->SetNumberField(TEXT("actual_makespan"), Context.ActualMakespan);
    Root->SetNumberField(TEXT("expansion"), Context.Expansion);
    Root->SetNumberField(TEXT("total_delay_steps"), Context.TotalDelaySteps);

    Root->SetNumberField(TEXT("vertex_conflict_count"), Context.VertexConflictCount);
    Root->SetNumberField(TEXT("edge_conflict_count"), Context.EdgeConflictCount);
    Root->SetNumberField(TEXT("first_conflict_time"), Context.FirstConflictTime);

    Root->SetNumberField(TEXT("utm_static_conflict_count"), Context.UTMStaticConflictCount);
    Root->SetNumberField(TEXT("utm_protection_conflict_count"), Context.UTMProtectionConflictCount);
    Root->SetNumberField(TEXT("utm_downwash_conflict_count"), Context.UTMDownwashConflictCount);
    Root->SetNumberField(TEXT("first_utm_conflict_time"), Context.FirstUTMConflictTime);

    Root->SetNumberField(TEXT("alignment_correction_count"), Context.AlignmentCorrectionCount);
    Root->SetNumberField(TEXT("alignment_hold_count"), Context.AlignmentHoldCount);
    Root->SetNumberField(TEXT("alignment_conflict_hold_count"), Context.AlignmentConflictHoldCount);
    Root->SetNumberField(TEXT("alignment_snap_count"), Context.AlignmentSnapCount);
    Root->SetNumberField(TEXT("alignment_replan_request_count"), Context.AlignmentReplanRequestCount);
    Root->SetNumberField(TEXT("alignment_successful_replan_count"), Context.AlignmentSuccessfulReplanCount);
    Root->SetNumberField(TEXT("applied_execution_replans"), Context.AppliedExecutionReplans);
    Root->SetNumberField(TEXT("execution_replan_attempt_count"), Context.ExecutionReplanAttemptCount);
    Root->SetNumberField(TEXT("execution_replan_total_time_ms"), Context.ExecutionReplanTotalTimeMs);
    Root->SetNumberField(TEXT("execution_replan_max_time_ms"), Context.ExecutionReplanMaxTimeMs);
    Root->SetNumberField(TEXT("execution_replan_local_attempt_count"), Context.ExecutionReplanLocalAttemptCount);
    Root->SetNumberField(TEXT("execution_replan_local_total_time_ms"), Context.ExecutionReplanLocalTotalTimeMs);
    Root->SetNumberField(TEXT("execution_replan_local_max_time_ms"), Context.ExecutionReplanLocalMaxTimeMs);
    Root->SetNumberField(TEXT("execution_replan_global_attempt_count"), Context.ExecutionReplanGlobalAttemptCount);
    Root->SetNumberField(TEXT("execution_replan_global_total_time_ms"), Context.ExecutionReplanGlobalTotalTimeMs);
    Root->SetNumberField(TEXT("execution_replan_global_max_time_ms"), Context.ExecutionReplanGlobalMaxTimeMs);

    FString JsonString;
    const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
        TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&JsonString);

    if (!FJsonSerializer::Serialize(Root, Writer))
    {
        return FString();
    }

    return JsonString;
}
