#pragma once

#include "CoreMinimal.h"
#include "Reporting/ExperimentReportTypes.h"

class FExperimentReporter
{
public:
    static FString BuildStructuredSummaryJson(const FExperimentReportContext& Context);
};
