#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionReplanServiceTypes.h"

class FExecutionReplanService
{
public:
    static FExecutionReplanServiceResult Run(
        const FExecutionReplanServiceRequest& Request,
        const FExecutionReplanServiceCallbacks& Callbacks);
};
