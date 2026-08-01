#pragma once

#include "CoreMinimal.h"

struct FExecutionConflict;
struct FExecutionConflictResolutionEvent;
struct FExecutionControllerStepResult;
struct FExecutionFinalSafetyGateEvent;
struct FExecutionReplanAttemptInput;
struct FExecutionReplanAttemptResult;
struct FExecutionReplanCoordinatorEvent;
struct FExecutionReplanServiceResult;
struct FExecutionRuntimeSession;
struct FExecutionStepResultApplyResult;

struct FExecutionDiagnosticsSettings
{
    bool bLogDelayEvents = true;
    bool bLogAlignmentEvents = true;
    bool bLogConflictPredictionEvents = true;
};

class IExecutionDiagnosticsSink
{
public:
    virtual ~IExecutionDiagnosticsSink() = default;

    virtual void SetSettings(
        const FExecutionDiagnosticsSettings& InSettings) = 0;

    virtual void HandleConflictResolutionEvent(
        int32 TimeStep,
        const FExecutionConflictResolutionEvent& Event) = 0;

    virtual void HandleFinalSafetyGateEvent(
        int32 TimeStep,
        const FExecutionFinalSafetyGateEvent& Event) = 0;

    virtual void HandleReplanAttemptFailure(
        const FExecutionReplanAttemptInput& Input,
        const FExecutionReplanAttemptResult& Result) = 0;

    virtual void HandleReplanCoordinatorEvent(
        const FExecutionReplanCoordinatorEvent& Event) = 0;

    virtual void HandleReplanServiceResult(
        const FExecutionReplanServiceResult& Result) = 0;

    virtual void HandleAppliedStep(
        int32 TimeStep,
        const FExecutionRuntimeSession& Session,
        const FExecutionControllerStepResult& ControllerResult,
        const FExecutionStepResultApplyResult& ApplyResult) = 0;

    virtual void HandleObservedConflicts(
        const TArray<FExecutionConflict>& Conflicts) = 0;
};
