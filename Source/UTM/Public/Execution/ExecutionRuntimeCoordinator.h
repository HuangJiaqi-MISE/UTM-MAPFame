#pragma once

#include "Execution/ExecutionRuntimeCoordinatorTypes.h"

class FExecutionRuntimeCoordinator
{
public:
    FExecutionRuntimeCoordinatorResult Advance(
        const FExecutionRuntimeCoordinatorRequest& Request,
        const FExecutionRuntimeCoordinatorCallbacks& Callbacks);
};
