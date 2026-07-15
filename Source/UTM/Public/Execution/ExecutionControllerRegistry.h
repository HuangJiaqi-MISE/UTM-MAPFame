#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionControllerTypes.h"

class IExecutionController
{
public:
    virtual ~IExecutionController() = default;

    virtual FString GetName() const = 0;

    virtual FExecutionControllerStepResult RunStep(
        const FExecutionControllerStepRequest& Request,
        const FExecutionControllerStepCallbacks& Callbacks) const = 0;
};

class FExecutionControllerRegistry
{
public:
    static FString GetControllerTypeName(EExecutionControllerType ControllerType);

    static TUniquePtr<IExecutionController> CreateController(
        EExecutionControllerType ControllerType);

    static FExecutionControllerStepResult RunStep(
        EExecutionControllerType ControllerType,
        const FExecutionControllerStepRequest& Request,
        const FExecutionControllerStepCallbacks& Callbacks);
};
