#pragma once

#include "CoreMinimal.h"

struct FExecutionReplanPathIntegrationResult
{
    bool bSuccess = false;
    TArray<FIntVector> TimelineCells;
    int32 ExecutedPlanIndex = 0;
    FIntVector GoalCell = FIntVector::ZeroValue;
};

class FExecutionReplanPathIntegrator
{
public:
    static FExecutionReplanPathIntegrationResult Integrate(
        const TArray<FIntVector>& ActualCells,
        const FIntVector& LastObservedCell,
        const TArray<FIntVector>& ReplannedCellPath);
};
