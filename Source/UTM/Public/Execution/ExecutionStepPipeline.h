#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionControllerTypes.h"

class FExecutionStepPipeline
{
public:
    static FExecutionControllerStepResult Run(
        const FExecutionControllerStepRequest& Request,
        const FExecutionControllerStepCallbacks& Callbacks);
};
