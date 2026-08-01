#pragma once

#include "Execution/ExecutionDiagnosticsSink.h"

class FExecutionLogSink final : public IExecutionDiagnosticsSink
{
public:
    virtual void SetSettings(
        const FExecutionDiagnosticsSettings& InSettings) override;

    virtual void HandleConflictResolutionEvent(
        int32 TimeStep,
        const FExecutionConflictResolutionEvent& Event) override;

    virtual void HandleFinalSafetyGateEvent(
        int32 TimeStep,
        const FExecutionFinalSafetyGateEvent& Event) override;

    virtual void HandleReplanAttemptFailure(
        const FExecutionReplanAttemptInput& Input,
        const FExecutionReplanAttemptResult& Result) override;

    virtual void HandleReplanCoordinatorEvent(
        const FExecutionReplanCoordinatorEvent& Event) override;

    virtual void HandleReplanServiceResult(
        const FExecutionReplanServiceResult& Result) override;

    virtual void HandleAppliedStep(
        int32 TimeStep,
        const FExecutionRuntimeSession& Session,
        const FExecutionControllerStepResult& ControllerResult,
        const FExecutionStepResultApplyResult& ApplyResult) override;

    virtual void HandleObservedConflicts(
        const TArray<FExecutionConflict>& Conflicts) override;

private:
    FExecutionDiagnosticsSettings Settings;
};
