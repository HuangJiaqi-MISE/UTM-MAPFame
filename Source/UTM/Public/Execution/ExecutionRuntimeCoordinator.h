#pragma once

#include "Execution/ExecutionRuntimeCoordinatorTypes.h"

class IExecutionController;
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

    void ResetController();

    FExecutionRuntimeCoordinatorResult Advance(
        const FExecutionRuntimeCoordinatorRequest& Request,
        const FExecutionRuntimeCoordinatorCallbacks& Callbacks);

    TArray<FExecutionConflict> RecordObservedConflicts(
        FExecutionRuntimeSession& Session,
        int32 TimeStep) const;

private:
    TUniquePtr<IExecutionController> ActiveController;
    TUniquePtr<IExecutionReplanService> ReplanService;
};
