#include "Execution/ExecutionStepReplanCoordinator.h"

FExecutionStepReplanCoordinatorResult FExecutionStepReplanCoordinator::Run(
    const FExecutionStepReplanCoordinatorRequest& Request,
    const FExecutionStepReplanCoordinatorCallbacks& Callbacks,
    TMap<int32, FExecutionStepProposal>& InOutStepProposals)
{
    FExecutionStepReplanCoordinatorResult Result;
    if (Request.RequestedMissionIds.Num() <= 0 ||
        Request.ReplanMode == EExecutionPolicyReplanMode::Disabled)
    {
        return Result;
    }

    const bool bUseGlobalReplan =
        Request.ReplanMode == EExecutionPolicyReplanMode::GlobalUnfinished;
    if (Callbacks.RunReplan)
    {
        Result.bSuccess = Callbacks.RunReplan(
            Request.RequestedMissionIds,
            bUseGlobalReplan,
            Result.ReplannedMissionIds);
    }

    if (!Result.bSuccess &&
        Request.ReplanMode == EExecutionPolicyReplanMode::LocalConflictSet &&
        Callbacks.RunReplan)
    {
        Result.bSuccess = Callbacks.RunReplan(
            Request.RequestedMissionIds,
            true,
            Result.ReplannedMissionIds);
    }

    if (Result.bSuccess && Callbacks.ApplyReplanResult)
    {
        Callbacks.ApplyReplanResult(
            Result.ReplannedMissionIds,
            InOutStepProposals);
    }

    return Result;
}
