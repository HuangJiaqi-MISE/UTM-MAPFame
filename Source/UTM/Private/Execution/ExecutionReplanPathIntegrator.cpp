#include "Execution/ExecutionReplanPathIntegrator.h"

FExecutionReplanPathIntegrationResult FExecutionReplanPathIntegrator::Integrate(
    const TArray<FIntVector>& ActualCells,
    const FIntVector& LastObservedCell,
    const TArray<FIntVector>& ReplannedCellPath)
{
    FExecutionReplanPathIntegrationResult Result;
    if (ReplannedCellPath.Num() <= 0)
    {
        return Result;
    }

    Result.TimelineCells = ActualCells;
    if (Result.TimelineCells.Num() <= 0 ||
        Result.TimelineCells.Last() != LastObservedCell)
    {
        Result.TimelineCells.Add(LastObservedCell);
    }

    // Reserve the current execution step for the replan hold itself.
    Result.TimelineCells.Add(LastObservedCell);

    for (int32 Index = 1; Index < ReplannedCellPath.Num(); ++Index)
    {
        Result.TimelineCells.Add(ReplannedCellPath[Index]);
    }

    Result.ExecutedPlanIndex = FMath::Max(0, ActualCells.Num() - 1);
    Result.GoalCell = ReplannedCellPath.Last();
    Result.bSuccess = true;
    return Result;
}
