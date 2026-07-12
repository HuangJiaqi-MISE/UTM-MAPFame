#pragma once

#include "CoreMinimal.h"

enum class EExperimentMetadataReplanMode : uint8
{
    Disabled,
    LocalConflictSet,
    GlobalUnfinished
};

struct FExperimentMetadataResolverInput
{
    FString RunId;
    FString Phase;
    FString GroupId;
    FString GroupName;
    FString ScenarioName;
    FString FallbackScenarioName;
    FString PlannerName;

    int32 EffectiveAgentCount = 0;
    float StepDelayProbability = 0.f;
    int32 ExecutionRandomSeed = 0;

    bool bEnableDiscreteAlignment = false;
    bool bEnableConflictAwareAlignment = false;
    EExperimentMetadataReplanMode ReplanMode = EExperimentMetadataReplanMode::Disabled;
};

struct FExperimentMetadata
{
    FString RunId;
    FString Phase;
    FString GroupId;
    FString GroupName;
    FString ScenarioName;
};

class FExperimentMetadataResolver
{
public:
    static FExperimentMetadata Resolve(const FExperimentMetadataResolverInput& Input);
};
