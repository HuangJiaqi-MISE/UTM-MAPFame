#include "EditorServices/CityEditorService.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"

namespace
{
    void SpawnCityBuilding(
        UWorld* World,
        const FVector& Center,
        const FVector& Extent)
    {
        if (!World)
        {
            UE_LOG(LogTemp, Error, TEXT("SpawnCityBuilding: World is null"));
            return;
        }

        AStaticMeshActor* Building = World->SpawnActor<AStaticMeshActor>(
            AStaticMeshActor::StaticClass(),
            Center,
            FRotator::ZeroRotator);
        if (!Building)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("SpawnCityBuilding: Failed to spawn building actor"));
            return;
        }

        Building->Tags.Add(FName(TEXT("CityBuilding")));

        UStaticMeshComponent* MeshComp = Building->GetStaticMeshComponent();
        if (!MeshComp)
        {
            UE_LOG(LogTemp, Error, TEXT("SpawnCityBuilding: MeshComp is null"));
            Building->Destroy();
            return;
        }

        static UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(
            nullptr,
            TEXT("/Engine/BasicShapes/Cube.Cube"));
        if (!CubeMesh)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("SpawnCityBuilding: Failed to load Cube mesh"));
            Building->Destroy();
            return;
        }

        MeshComp->SetStaticMesh(CubeMesh);
        MeshComp->SetMobility(EComponentMobility::Static);
        MeshComp->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        MeshComp->SetCollisionProfileName(TEXT("BlockAll"));

        Building->SetActorLocation(Center);
        Building->SetActorScale3D(FVector(
            Extent.X / 50.f,
            Extent.Y / 50.f,
            Extent.Z / 50.f));
    }
}

void FCityEditorService::ClearCityEnvironment(UWorld* World)
{
    if (!World)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("EditorClearCityEnvironment: World is null"));
        return;
    }

    int32 DeleteCount = 0;
    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (Actor && Actor->Tags.Contains(FName(TEXT("CityBuilding"))))
        {
            Actor->Destroy();
            DeleteCount++;
        }
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("EditorClearCityEnvironment deleted %d actors"),
        DeleteCount);
}

void FCityEditorService::GenerateManhattanLayout(
    const FCityEditorGenerateRequest& Request)
{
    FRandomStream RandomStream(Request.CitySeed);
    const float Pitch = Request.CityBlockSize + Request.CityRoadWidth;
    const float WidthMin =
        FMath::Min(Request.BuildingWidthMin, Request.BuildingWidthMax);
    const float WidthMax =
        FMath::Max(Request.BuildingWidthMin, Request.BuildingWidthMax);
    const float DepthMin =
        FMath::Min(Request.BuildingDepthMin, Request.BuildingDepthMax);
    const float DepthMax =
        FMath::Max(Request.BuildingDepthMin, Request.BuildingDepthMax);
    const float HeightMin = FMath::Max(
        FMath::Min(Request.BuildingHeightMin, Request.BuildingHeightMax),
        600.f);
    const float HeightMax = FMath::Max(
        FMath::Max(Request.BuildingHeightMin, Request.BuildingHeightMax),
        900.f);

    int32 SpawnedBuildingCount = 0;
    for (int32 X = 0; X < Request.CityBlocksX; ++X)
    {
        for (int32 Y = 0; Y < Request.CityBlocksY; ++Y)
        {
            const FVector BlockCenter(
                Request.GridOrigin.X + X * Pitch,
                Request.GridOrigin.Y + Y * Pitch,
                0.f);
            const int32 BuildingCount = RandomStream.RandRange(1, 4);

            for (int32 BuildingIndex = 0;
                 BuildingIndex < BuildingCount;
                 ++BuildingIndex)
            {
                const float Width =
                    RandomStream.FRandRange(WidthMin, WidthMax);
                const float Depth =
                    RandomStream.FRandRange(DepthMin, DepthMax);
                const float Height =
                    RandomStream.FRandRange(HeightMin, HeightMax);
                const float Margin = 40.f;
                const float OffsetXLimit = FMath::Max(
                    0.f,
                    Request.CityBlockSize * 0.5f - Width * 0.5f - Margin);
                const float OffsetYLimit = FMath::Max(
                    0.f,
                    Request.CityBlockSize * 0.5f - Depth * 0.5f - Margin);
                const FVector BuildingCenter(
                    BlockCenter.X + RandomStream.FRandRange(
                        -OffsetXLimit,
                        OffsetXLimit),
                    BlockCenter.Y + RandomStream.FRandRange(
                        -OffsetYLimit,
                        OffsetYLimit),
                    Height * 0.5f);
                const FVector BuildingExtent(
                    Width * 0.5f,
                    Depth * 0.5f,
                    Height * 0.5f);

                SpawnCityBuilding(
                    Request.World,
                    BuildingCenter,
                    BuildingExtent);
                SpawnedBuildingCount++;
            }
        }
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("GenerateCityLayout_Manhattan done. Buildings=%d"),
        SpawnedBuildingCount);
}

void FCityEditorService::GenerateResidentialLayout(
    const FCityEditorGenerateRequest& Request)
{
    FRandomStream RandomStream(Request.CitySeed);
    const float Pitch = Request.CityBlockSize + Request.CityRoadWidth;
    const float WidthMin =
        FMath::Min(Request.BuildingWidthMin, Request.BuildingWidthMax);
    const float WidthMax =
        FMath::Max(Request.BuildingWidthMin, Request.BuildingWidthMax) * 0.8f;
    const float DepthMin =
        FMath::Min(Request.BuildingDepthMin, Request.BuildingDepthMax);
    const float DepthMax =
        FMath::Max(Request.BuildingDepthMin, Request.BuildingDepthMax) * 0.8f;
    const float HeightMin =
        FMath::Min(Request.BuildingHeightMin, Request.BuildingHeightMax) * 0.4f;
    const float HeightMax =
        FMath::Max(Request.BuildingHeightMin, Request.BuildingHeightMax) * 0.75f;

    for (int32 X = 0; X < Request.CityBlocksX; ++X)
    {
        for (int32 Y = 0; Y < Request.CityBlocksY; ++Y)
        {
            const FVector BlockCenter(
                Request.GridOrigin.X + X * Pitch,
                Request.GridOrigin.Y + Y * Pitch,
                0.f);
            const int32 BuildingCount = RandomStream.RandRange(2, 5);

            for (int32 BuildingIndex = 0;
                 BuildingIndex < BuildingCount;
                 ++BuildingIndex)
            {
                const float Width =
                    RandomStream.FRandRange(WidthMin, WidthMax);
                const float Depth =
                    RandomStream.FRandRange(DepthMin, DepthMax);
                const float Height =
                    RandomStream.FRandRange(HeightMin, HeightMax);
                const float Margin = 70.f;
                const float OffsetXLimit = FMath::Max(
                    0.f,
                    Request.CityBlockSize * 0.5f - Width * 0.5f - Margin);
                const float OffsetYLimit = FMath::Max(
                    0.f,
                    Request.CityBlockSize * 0.5f - Depth * 0.5f - Margin);
                const FVector BuildingCenter(
                    BlockCenter.X + RandomStream.FRandRange(
                        -OffsetXLimit,
                        OffsetXLimit),
                    BlockCenter.Y + RandomStream.FRandRange(
                        -OffsetYLimit,
                        OffsetYLimit),
                    Height * 0.5f);
                const FVector BuildingExtent(
                    Width * 0.5f,
                    Depth * 0.5f,
                    Height * 0.5f);

                SpawnCityBuilding(
                    Request.World,
                    BuildingCenter,
                    BuildingExtent);
            }
        }
    }
}

void FCityEditorService::GenerateIndustrialLayout(
    const FCityEditorGenerateRequest& Request)
{
    FRandomStream RandomStream(Request.CitySeed);
    const float Pitch = Request.CityBlockSize + Request.CityRoadWidth;
    const float WidthMin =
        FMath::Min(Request.BuildingWidthMin, Request.BuildingWidthMax) * 1.4f;
    const float WidthMax =
        FMath::Max(Request.BuildingWidthMin, Request.BuildingWidthMax) * 2.0f;
    const float DepthMin =
        FMath::Min(Request.BuildingDepthMin, Request.BuildingDepthMax) * 1.4f;
    const float DepthMax =
        FMath::Max(Request.BuildingDepthMin, Request.BuildingDepthMax) * 2.0f;
    const float HeightMin =
        FMath::Min(Request.BuildingHeightMin, Request.BuildingHeightMax) * 0.7f;
    const float HeightMax =
        FMath::Max(Request.BuildingHeightMin, Request.BuildingHeightMax) * 1.0f;

    for (int32 X = 0; X < Request.CityBlocksX; ++X)
    {
        for (int32 Y = 0; Y < Request.CityBlocksY; ++Y)
        {
            const FVector BlockCenter(
                Request.GridOrigin.X + X * Pitch,
                Request.GridOrigin.Y + Y * Pitch,
                0.f);
            const int32 BuildingCount = RandomStream.RandRange(1, 2);

            for (int32 BuildingIndex = 0;
                 BuildingIndex < BuildingCount;
                 ++BuildingIndex)
            {
                const float Width =
                    RandomStream.FRandRange(WidthMin, WidthMax);
                const float Depth =
                    RandomStream.FRandRange(DepthMin, DepthMax);
                const float Height =
                    RandomStream.FRandRange(HeightMin, HeightMax);
                const float Margin = 50.f;
                const float OffsetXLimit = FMath::Max(
                    0.f,
                    Request.CityBlockSize * 0.5f - Width * 0.5f - Margin);
                const float OffsetYLimit = FMath::Max(
                    0.f,
                    Request.CityBlockSize * 0.5f - Depth * 0.5f - Margin);
                const FVector BuildingCenter(
                    BlockCenter.X + RandomStream.FRandRange(
                        -OffsetXLimit,
                        OffsetXLimit),
                    BlockCenter.Y + RandomStream.FRandRange(
                        -OffsetYLimit,
                        OffsetYLimit),
                    Height * 0.5f);
                const FVector BuildingExtent(
                    Width * 0.5f,
                    Depth * 0.5f,
                    Height * 0.5f);

                SpawnCityBuilding(
                    Request.World,
                    BuildingCenter,
                    BuildingExtent);
            }
        }
    }
}

void FCityEditorService::GenerateMixedLayout(
    const FCityEditorGenerateRequest& Request)
{
    FRandomStream RandomStream(Request.CitySeed);
    const float Pitch = Request.CityBlockSize + Request.CityRoadWidth;

    for (int32 X = 0; X < Request.CityBlocksX; ++X)
    {
        for (int32 Y = 0; Y < Request.CityBlocksY; ++Y)
        {
            const FVector BlockCenter(
                Request.GridOrigin.X + X * Pitch,
                Request.GridOrigin.Y + Y * Pitch,
                0.f);
            const int32 DistrictType = RandomStream.RandRange(0, 2);

            int32 BuildingCount = 0;
            float WidthMin = 0.f;
            float WidthMax = 0.f;
            float DepthMin = 0.f;
            float DepthMax = 0.f;
            float HeightMin = 0.f;
            float HeightMax = 0.f;
            float Margin = 50.f;

            if (DistrictType == 0)
            {
                BuildingCount = RandomStream.RandRange(1, 4);
                WidthMin = Request.BuildingWidthMin;
                WidthMax = Request.BuildingWidthMax;
                DepthMin = Request.BuildingDepthMin;
                DepthMax = Request.BuildingDepthMax;
                HeightMin = FMath::Max(
                    FMath::Min(
                        Request.BuildingHeightMin,
                        Request.BuildingHeightMax),
                    600.f);
                HeightMax = FMath::Max(
                    FMath::Max(
                        Request.BuildingHeightMin,
                        Request.BuildingHeightMax),
                    900.f);
                Margin = 40.f;
            }
            else if (DistrictType == 1)
            {
                BuildingCount = RandomStream.RandRange(2, 5);
                WidthMin = FMath::Min(
                    Request.BuildingWidthMin,
                    Request.BuildingWidthMax);
                WidthMax = FMath::Max(
                    Request.BuildingWidthMin,
                    Request.BuildingWidthMax) * 0.8f;
                DepthMin = FMath::Min(
                    Request.BuildingDepthMin,
                    Request.BuildingDepthMax);
                DepthMax = FMath::Max(
                    Request.BuildingDepthMin,
                    Request.BuildingDepthMax) * 0.8f;
                HeightMin = FMath::Min(
                    Request.BuildingHeightMin,
                    Request.BuildingHeightMax) * 0.4f;
                HeightMax = FMath::Max(
                    Request.BuildingHeightMin,
                    Request.BuildingHeightMax) * 0.75f;
                Margin = 70.f;
            }
            else
            {
                BuildingCount = RandomStream.RandRange(1, 2);
                WidthMin = FMath::Min(
                    Request.BuildingWidthMin,
                    Request.BuildingWidthMax) * 1.4f;
                WidthMax = FMath::Max(
                    Request.BuildingWidthMin,
                    Request.BuildingWidthMax) * 2.0f;
                DepthMin = FMath::Min(
                    Request.BuildingDepthMin,
                    Request.BuildingDepthMax) * 1.4f;
                DepthMax = FMath::Max(
                    Request.BuildingDepthMin,
                    Request.BuildingDepthMax) * 2.0f;
                HeightMin = FMath::Min(
                    Request.BuildingHeightMin,
                    Request.BuildingHeightMax) * 0.7f;
                HeightMax = FMath::Max(
                    Request.BuildingHeightMin,
                    Request.BuildingHeightMax) * 1.0f;
                Margin = 50.f;
            }

            for (int32 BuildingIndex = 0;
                 BuildingIndex < BuildingCount;
                 ++BuildingIndex)
            {
                const float Width =
                    RandomStream.FRandRange(WidthMin, WidthMax);
                const float Depth =
                    RandomStream.FRandRange(DepthMin, DepthMax);
                const float Height =
                    RandomStream.FRandRange(HeightMin, HeightMax);
                const float OffsetXLimit = FMath::Max(
                    0.f,
                    Request.CityBlockSize * 0.5f - Width * 0.5f - Margin);
                const float OffsetYLimit = FMath::Max(
                    0.f,
                    Request.CityBlockSize * 0.5f - Depth * 0.5f - Margin);
                const FVector BuildingCenter(
                    BlockCenter.X + RandomStream.FRandRange(
                        -OffsetXLimit,
                        OffsetXLimit),
                    BlockCenter.Y + RandomStream.FRandRange(
                        -OffsetYLimit,
                        OffsetYLimit),
                    Height * 0.5f);
                const FVector BuildingExtent(
                    Width * 0.5f,
                    Depth * 0.5f,
                    Height * 0.5f);

                SpawnCityBuilding(
                    Request.World,
                    BuildingCenter,
                    BuildingExtent);
            }
        }
    }
}
