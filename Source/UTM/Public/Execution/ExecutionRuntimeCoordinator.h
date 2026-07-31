#pragma once

#include "Execution/ExecutionRuntimeCoordinatorTypes.h"

class FExecutionRuntimeCoordinator
{
public:
    FExecutionRuntimeCoordinatorResult Advance(
        const FExecutionRuntimeCoordinatorRequest& Request,
        const FExecutionRuntimeCoordinatorCallbacks& Callbacks);

    TArray<FExecutionConflict> RecordObservedConflicts(
        FExecutionRuntimeSession& Session,
        int32 TimeStep) const;
};
