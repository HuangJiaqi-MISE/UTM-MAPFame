#pragma once

#include "Execution/ExecutionRuntimeCoordinatorTypes.h"

class IExecutionController;

class FExecutionRuntimeCoordinator
{
public:
    FExecutionRuntimeCoordinator();
    ~FExecutionRuntimeCoordinator();

    bool InitializeController(
        EExecutionControllerType ControllerType,
        FString& OutFailureReason);

    void ResetController();

    FExecutionRuntimeCoordinatorResult Advance(
        const FExecutionRuntimeCoordinatorRequest& Request,
        const FExecutionRuntimeCoordinatorCallbacks& Callbacks);

    TArray<FExecutionConflict> RecordObservedConflicts(
        FExecutionRuntimeSession& Session,
        int32 TimeStep) const;

private:
    TUniquePtr<IExecutionController> ActiveController;
};
