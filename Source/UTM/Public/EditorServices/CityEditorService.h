#pragma once

#include "CoreMinimal.h"

class UWorld;

struct FCityEditorGenerateRequest
{
    UWorld* World = nullptr;
    FVector GridOrigin = FVector::ZeroVector;
    int32 CitySeed = 0;
    int32 CityBlocksX = 0;
    int32 CityBlocksY = 0;
    float CityBlockSize = 0.f;
    float CityRoadWidth = 0.f;
    float BuildingWidthMin = 0.f;
    float BuildingWidthMax = 0.f;
    float BuildingDepthMin = 0.f;
    float BuildingDepthMax = 0.f;
    float BuildingHeightMin = 0.f;
    float BuildingHeightMax = 0.f;
};

class FCityEditorService
{
public:
    static void ClearCityEnvironment(UWorld* World);

    static void GenerateManhattanLayout(
        const FCityEditorGenerateRequest& Request);

    static void GenerateResidentialLayout(
        const FCityEditorGenerateRequest& Request);

    static void GenerateIndustrialLayout(
        const FCityEditorGenerateRequest& Request);

    static void GenerateMixedLayout(
        const FCityEditorGenerateRequest& Request);
};
