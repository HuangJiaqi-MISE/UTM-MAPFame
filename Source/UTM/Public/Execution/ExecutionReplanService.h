#pragma once

#include "CoreMinimal.h"
#include "Execution/ExecutionReplanServiceTypes.h"

class IExecutionReplanService
{
public:
    virtual ~IExecutionReplanService() = default;

    virtual void Reset()
    {
    }

    virtual FExecutionReplanServiceResult Run(
        const FExecutionReplanServiceRequest& Request,
        const FExecutionReplanServiceCallbacks& Callbacks) = 0;
};

class FDefaultExecutionReplanService final : public IExecutionReplanService
{
public:
    virtual FExecutionReplanServiceResult Run(
        const FExecutionReplanServiceRequest& Request,
        const FExecutionReplanServiceCallbacks& Callbacks) override;
};
