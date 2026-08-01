#pragma once

#include "Execution/ExecutionRuntimeCoordinatorTypes.h"

class IExecutionController;
class IExecutionDiagnosticsSink;
class IExecutionReplanService;

class FExecutionRuntimeCoordinator
{
public:
    FExecutionRuntimeCoordinator();
    ~FExecutionRuntimeCoordinator();

    bool InitializeController(
        const FExecutionRuntimeCoordinatorInitializeRequest& Request,
        FString& OutFailureReason);

    bool SetReplanService(
        TUniquePtr<IExecutionReplanService>&& InReplanService);

    bool SetDiagnosticsSink(
        TUniquePtr<IExecutionDiagnosticsSink>&& InDiagnosticsSink);

    void ResetController();

    FExecutionRuntimeCoordinatorResult Advance(
        const FExecutionRuntimeCoordinatorRequest& Request,
        const FExecutionRuntimeCoordinatorCallbacks& Callbacks);

    TArray<FExecutionConflict> RecordObservedConflicts(
        FExecutionRuntimeSession& Session,
        int32 TimeStep,
        bool bEmitDiagnostics = true) const;

private:
    TUniquePtr<IExecutionController> ActiveController;
    TUniquePtr<IExecutionReplanService> ReplanService;
    TUniquePtr<IExecutionDiagnosticsSink> DiagnosticsSink;
};
