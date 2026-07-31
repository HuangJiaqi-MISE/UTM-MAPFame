#include "Execution/ExecutionControllerRegistry.h"

#include "Execution/ExecutionStepPipeline.h"

namespace
{
    class FDefaultExecutionController final : public IExecutionController
    {
    public:
        virtual FString GetName() const override
        {
            return TEXT("Default Execution Pipeline");
        }

        virtual FExecutionControllerStepResult RunStep(
            const FExecutionControllerStepRequest& Request,
            const FExecutionControllerStepCallbacks& Callbacks) override
        {
            return FExecutionStepPipeline::Run(Request, Callbacks);
        }
    };
}

FString FExecutionControllerRegistry::GetControllerTypeName(
    EExecutionControllerType ControllerType)
{
    switch (ControllerType)
    {
    case EExecutionControllerType::DefaultPipeline:
        return TEXT("Default Execution Pipeline");
    default:
        return TEXT("Unknown");
    }
}

TUniquePtr<IExecutionController> FExecutionControllerRegistry::CreateController(
    EExecutionControllerType ControllerType)
{
    switch (ControllerType)
    {
    case EExecutionControllerType::DefaultPipeline:
        return MakeUnique<FDefaultExecutionController>();
    default:
        return nullptr;
    }
}

FExecutionControllerStepResult FExecutionControllerRegistry::RunStep(
    EExecutionControllerType ControllerType,
    const FExecutionControllerStepRequest& Request,
    const FExecutionControllerStepCallbacks& Callbacks)
{
    TUniquePtr<IExecutionController> Controller = CreateController(ControllerType);
    if (!Controller)
    {
        return FExecutionControllerStepResult();
    }

    FExecutionControllerInitializeRequest InitializeRequest;
    InitializeRequest.OrderedMissionIds = Request.OrderedMissionIds;
    InitializeRequest.GridMap = Request.GridMap;
    InitializeRequest.RuntimeConfig = Request.RuntimeConfig;

    FString FailureReason;
    if (!Controller->Initialize(InitializeRequest, FailureReason))
    {
        Controller->Reset();
        return FExecutionControllerStepResult();
    }

    FExecutionControllerStepResult Result =
        Controller->RunStep(Request, Callbacks);
    Controller->Reset();
    return Result;
}
