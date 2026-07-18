#include "EditorServices/NoFlyZoneEditorService.h"

#include "Components/BoxComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "Planning/GridMap3D.h"

namespace
{
    FIntVector NormalizeMinCell(const FIntVector& A, const FIntVector& B)
    {
        return FIntVector(
            FMath::Min(A.X, B.X),
            FMath::Min(A.Y, B.Y),
            FMath::Min(A.Z, B.Z));
    }

    FIntVector NormalizeMaxCell(const FIntVector& A, const FIntVector& B)
    {
        return FIntVector(
            FMath::Max(A.X, B.X),
            FMath::Max(A.Y, B.Y),
            FMath::Max(A.Z, B.Z));
    }

    FIntVector ClampCellToGrid(
        const FIntVector& Cell,
        const FIntVector& GridDim)
    {
        return FIntVector(
            FMath::Clamp(Cell.X, 0, FMath::Max(0, GridDim.X - 1)),
            FMath::Clamp(Cell.Y, 0, FMath::Max(0, GridDim.Y - 1)),
            FMath::Clamp(Cell.Z, 0, FMath::Max(0, GridDim.Z - 1)));
    }

    void GetNoFlyZoneMarkerActors(
        UWorld* World,
        TArray<ANoFlyZoneMarkerActor*>& OutMarkers)
    {
        OutMarkers.Reset();
        if (!World)
        {
            return;
        }

        for (TActorIterator<ANoFlyZoneMarkerActor> It(World); It; ++It)
        {
            ANoFlyZoneMarkerActor* Marker = *It;
            if (Marker && Marker->Tags.Contains(FName(TEXT("NoFlyZoneMarker"))))
            {
                OutMarkers.Add(Marker);
            }
        }
    }
}

void FNoFlyZoneEditorService::AddNoFlyZoneConfig(
    const FNoFlyZoneEditorAddConfigRequest& Request)
{
    if (!Request.ZoneConfigs)
    {
        return;
    }

    FTemporalNoFlyZoneConfig Zone;
    int32 NextZoneId = 1;
    for (const FTemporalNoFlyZoneConfig& ExistingZone : *Request.ZoneConfigs)
    {
        NextZoneId = FMath::Max(NextZoneId, ExistingZone.ZoneId + 1);
    }

    const FIntVector GridCenter(
        FMath::Max(0, Request.GridDim.X / 2),
        FMath::Max(0, Request.GridDim.Y / 2),
        FMath::Max(0, Request.GridDim.Z / 2));
    const int32 HalfSpan = FMath::Max(0, Request.DefaultSizeCells - 1) / 2;
    Zone.ZoneId = NextZoneId;
    Zone.bEnabled = true;
    Zone.MinCell = ClampCellToGrid(
        GridCenter - FIntVector(HalfSpan, HalfSpan, 0),
        Request.GridDim);
    Zone.MaxCell = ClampCellToGrid(
        Zone.MinCell + FIntVector(
            FMath::Max(0, Request.DefaultSizeCells - 1),
            FMath::Max(0, Request.DefaultSizeCells - 1),
            0),
        Request.GridDim);
    Zone.StartTimeStep = 0;
    Zone.EndTimeStep =
        FMath::Max(Zone.StartTimeStep, Request.DefaultDuration - 1);
    Request.ZoneConfigs->Add(Zone);

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("EditorAddNoFlyZoneConfig added ZoneId=%d Min=(%d,%d,%d) Max=(%d,%d,%d) Time=[%d,%d]"),
        Zone.ZoneId,
        Zone.MinCell.X,
        Zone.MinCell.Y,
        Zone.MinCell.Z,
        Zone.MaxCell.X,
        Zone.MaxCell.Y,
        Zone.MaxCell.Z,
        Zone.StartTimeStep,
        Zone.EndTimeStep);
}

void FNoFlyZoneEditorService::ClearNoFlyZoneMarkers(UWorld* World)
{
    TArray<ANoFlyZoneMarkerActor*> Markers;
    GetNoFlyZoneMarkerActors(World, Markers);

    int32 DeleteCount = 0;
    for (ANoFlyZoneMarkerActor* Marker : Markers)
    {
        if (!Marker)
        {
            continue;
        }

        Marker->Destroy();
        DeleteCount++;
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("EditorClearNoFlyZoneMarkers deleted %d markers"),
        DeleteCount);
}

void FNoFlyZoneEditorService::SpawnNoFlyZoneMarkers(
    const FNoFlyZoneEditorSpawnMarkersRequest& Request)
{
    if (!Request.MarkerClass)
    {
        UE_LOG(LogTemp, Error, TEXT("NoFlyZoneMarkerClass is null"));
        return;
    }

    if (!Request.World)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("EditorSpawnNoFlyZoneMarkers: World is null"));
        return;
    }

    if (!Request.GridMap || !Request.ZoneConfigs)
    {
        return;
    }

    ClearNoFlyZoneMarkers(Request.World);
    Request.GridMap->GridOrigin = Request.GridOrigin;
    Request.GridMap->GridDim = Request.GridDim;
    Request.GridMap->CellSize = Request.CellSize;

    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = Request.Owner;
    SpawnParams.SpawnCollisionHandlingOverride =
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    for (const FTemporalNoFlyZoneConfig& ZoneConfig : *Request.ZoneConfigs)
    {
        FTemporalNoFlyZoneConfig NormalizedZone = ZoneConfig;
        NormalizedZone.MinCell = ClampCellToGrid(
            NormalizeMinCell(ZoneConfig.MinCell, ZoneConfig.MaxCell),
            Request.GridDim);
        NormalizedZone.MaxCell = ClampCellToGrid(
            NormalizeMaxCell(ZoneConfig.MinCell, ZoneConfig.MaxCell),
            Request.GridDim);
        NormalizedZone.EndTimeStep =
            FMath::Max(NormalizedZone.StartTimeStep, NormalizedZone.EndTimeStep);

        const FVector MinWorld =
            Request.GridMap->CellToWorld(NormalizedZone.MinCell);
        const FVector MaxWorld =
            Request.GridMap->CellToWorld(NormalizedZone.MaxCell);
        const FVector MarkerCenter = (MinWorld + MaxWorld) * 0.5f;
        const FVector MarkerExtent(
            (NormalizedZone.MaxCell.X - NormalizedZone.MinCell.X + 1) *
                Request.CellSize * 0.5f,
            (NormalizedZone.MaxCell.Y - NormalizedZone.MinCell.Y + 1) *
                Request.CellSize * 0.5f,
            (NormalizedZone.MaxCell.Z - NormalizedZone.MinCell.Z + 1) *
                Request.CellSize * 0.5f);

        ANoFlyZoneMarkerActor* Marker =
            Request.World->SpawnActor<ANoFlyZoneMarkerActor>(
                Request.MarkerClass,
                MarkerCenter + FVector(0.f, 0.f, Request.MarkerZOffset),
                FRotator::ZeroRotator,
                SpawnParams);
        if (!Marker)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("Failed to spawn no-fly-zone marker for ZoneId=%d"),
                NormalizedZone.ZoneId);
            continue;
        }

        Marker->ZoneConfig = NormalizedZone;
        Marker->Tags.AddUnique(FName(TEXT("NoFlyZoneMarker")));
        Marker->SetActorScale3D(FVector::OneVector);
        if (Marker->BoxComponent)
        {
            Marker->BoxComponent->SetBoxExtent(MarkerExtent, true);
        }
        Marker->UpdateVisual();
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("EditorSpawnNoFlyZoneMarkers done. Zone count=%d"),
        Request.ZoneConfigs->Num());
}

void FNoFlyZoneEditorService::ReadNoFlyZoneMarkersToConfigs(
    const FNoFlyZoneEditorReadMarkersRequest& Request)
{
    if (!Request.GridMap || !Request.ZoneConfigs)
    {
        return;
    }

    Request.GridMap->GridOrigin = Request.GridOrigin;
    Request.GridMap->GridDim = Request.GridDim;
    Request.GridMap->CellSize = Request.CellSize;

    TArray<ANoFlyZoneMarkerActor*> Markers;
    GetNoFlyZoneMarkerActors(Request.World, Markers);
    Request.ZoneConfigs->Reset();
    const FVector HalfCell(
        Request.CellSize * 0.5f,
        Request.CellSize * 0.5f,
        Request.CellSize * 0.5f);

    for (ANoFlyZoneMarkerActor* Marker : Markers)
    {
        if (!Marker || !Marker->BoxComponent)
        {
            continue;
        }

        FTemporalNoFlyZoneConfig ZoneConfig = Marker->ZoneConfig;
        const FVector CenterWorld =
            Marker->GetActorLocation() -
            FVector(0.f, 0.f, Request.MarkerZOffset);
        const FVector Extent = Marker->BoxComponent->GetScaledBoxExtent();
        const FVector MinCornerWorld = CenterWorld - Extent + HalfCell;
        const FVector MaxCornerWorld = CenterWorld + Extent - HalfCell;

        ZoneConfig.MinCell = ClampCellToGrid(
            Request.GridMap->WorldToCell(MinCornerWorld),
            Request.GridDim);
        ZoneConfig.MaxCell = ClampCellToGrid(
            Request.GridMap->WorldToCell(MaxCornerWorld),
            Request.GridDim);
        ZoneConfig.MinCell =
            NormalizeMinCell(ZoneConfig.MinCell, ZoneConfig.MaxCell);
        ZoneConfig.MaxCell =
            NormalizeMaxCell(ZoneConfig.MinCell, ZoneConfig.MaxCell);
        ZoneConfig.EndTimeStep =
            FMath::Max(ZoneConfig.StartTimeStep, ZoneConfig.EndTimeStep);
        Request.ZoneConfigs->Add(ZoneConfig);
    }

    Request.ZoneConfigs->Sort(
        [](const FTemporalNoFlyZoneConfig& A,
           const FTemporalNoFlyZoneConfig& B)
        {
            return A.ZoneId < B.ZoneId;
        });

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("EditorReadNoFlyZoneMarkersToConfigs done. Zone count=%d"),
        Request.ZoneConfigs->Num());
}

void FNoFlyZoneEditorService::ValidateNoFlyZones(
    const FNoFlyZoneEditorValidateRequest& Request)
{
    if (!Request.GridMap || !Request.ZoneConfigs)
    {
        return;
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("Validate NoFlyZoneConfigs begin. Count=%d"),
        Request.ZoneConfigs->Num());
    TSet<int32> ZoneIds;

    for (const FTemporalNoFlyZoneConfig& ZoneConfig : *Request.ZoneConfigs)
    {
        if (ZoneIds.Contains(ZoneConfig.ZoneId))
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("Duplicate ZoneId=%d"),
                ZoneConfig.ZoneId);
        }
        ZoneIds.Add(ZoneConfig.ZoneId);

        const FIntVector MinCell =
            NormalizeMinCell(ZoneConfig.MinCell, ZoneConfig.MaxCell);
        const FIntVector MaxCell =
            NormalizeMaxCell(ZoneConfig.MinCell, ZoneConfig.MaxCell);
        const bool bInside =
            Request.GridMap->IsInside(MinCell.X, MinCell.Y, MinCell.Z) &&
            Request.GridMap->IsInside(MaxCell.X, MaxCell.Y, MaxCell.Z);
        if (!bInside)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("Zone %d out of bounds. Min=(%d,%d,%d) Max=(%d,%d,%d)"),
                ZoneConfig.ZoneId,
                MinCell.X,
                MinCell.Y,
                MinCell.Z,
                MaxCell.X,
                MaxCell.Y,
                MaxCell.Z);
            continue;
        }

        if (ZoneConfig.EndTimeStep < ZoneConfig.StartTimeStep)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("Zone %d invalid time window [%d,%d]"),
                ZoneConfig.ZoneId,
                ZoneConfig.StartTimeStep,
                ZoneConfig.EndTimeStep);
        }

        int32 CellCount = 0;
        int32 BlockedCount = 0;
        for (int32 X = MinCell.X; X <= MaxCell.X; ++X)
        {
            for (int32 Y = MinCell.Y; Y <= MaxCell.Y; ++Y)
            {
                for (int32 Z = MinCell.Z; Z <= MaxCell.Z; ++Z)
                {
                    CellCount++;
                    if (Request.GridMap->IsBlocked(X, Y, Z))
                    {
                        BlockedCount++;
                    }
                }
            }
        }

        UE_LOG(
            LogTemp,
            Warning,
            TEXT("Zone %d Enabled=%s Cells=%d Blocked=%d Min=(%d,%d,%d) Max=(%d,%d,%d) Time=[%d,%d]"),
            ZoneConfig.ZoneId,
            ZoneConfig.bEnabled ? TEXT("true") : TEXT("false"),
            CellCount,
            BlockedCount,
            MinCell.X,
            MinCell.Y,
            MinCell.Z,
            MaxCell.X,
            MaxCell.Y,
            MaxCell.Z,
            ZoneConfig.StartTimeStep,
            ZoneConfig.EndTimeStep);
    }

    UE_LOG(LogTemp, Warning, TEXT("Validate NoFlyZoneConfigs done."));
}

void FNoFlyZoneEditorService::GenerateRandomNoFlyZoneConfigs(
    const FNoFlyZoneEditorGenerateRandomRequest& Request)
{
    if (!Request.GridMap || !Request.ZoneConfigs)
    {
        return;
    }

    Request.ZoneConfigs->Reset();
    if (Request.GridDim.X <= 0 ||
        Request.GridDim.Y <= 0 ||
        Request.GridDim.Z <= 0)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("EditorGenerateRandomNoFlyZoneConfigs: invalid grid dimension"));
        return;
    }

    const int32 SafeMinSize = FMath::Max(
        1,
        FMath::Min(Request.MinSizeCells, Request.MaxSizeCells));
    const int32 SafeMaxSize = FMath::Max(
        SafeMinSize,
        FMath::Max(Request.MinSizeCells, Request.MaxSizeCells));
    const int32 SafeMinStart = FMath::Max(
        0,
        FMath::Min(Request.MinStartTimeStep, Request.MaxStartTimeStep));
    const int32 SafeMaxStart = FMath::Max(
        SafeMinStart,
        FMath::Max(Request.MinStartTimeStep, Request.MaxStartTimeStep));
    const int32 SafeMinDuration = FMath::Max(
        1,
        FMath::Min(Request.MinDuration, Request.MaxDuration));
    const int32 SafeMaxDuration = FMath::Max(
        SafeMinDuration,
        FMath::Max(Request.MinDuration, Request.MaxDuration));

    FRandomStream RandomStream(Request.RandomSeed);
    for (int32 ZoneIndex = 0; ZoneIndex < Request.ZoneCount; ++ZoneIndex)
    {
        const int32 ZoneId = ZoneIndex + 1;
        bool bGenerated = false;
        for (int32 TryIndex = 0; TryIndex < 200; ++TryIndex)
        {
            const int32 SizeXY =
                RandomStream.RandRange(SafeMinSize, SafeMaxSize);
            const int32 SizeZ = FMath::Min(
                Request.GridDim.Z,
                FMath::Max(
                    1,
                    RandomStream.RandRange(1, FMath::Min(2, SizeXY))));
            if (SizeXY > Request.GridDim.X ||
                SizeXY > Request.GridDim.Y ||
                SizeZ > Request.GridDim.Z)
            {
                continue;
            }

            const int32 MinX =
                RandomStream.RandRange(0, Request.GridDim.X - SizeXY);
            const int32 MinY =
                RandomStream.RandRange(0, Request.GridDim.Y - SizeXY);
            const int32 MinZ =
                RandomStream.RandRange(0, Request.GridDim.Z - SizeZ);

            FTemporalNoFlyZoneConfig ZoneConfig;
            ZoneConfig.ZoneId = ZoneId;
            ZoneConfig.bEnabled = true;
            ZoneConfig.MinCell = FIntVector(MinX, MinY, MinZ);
            ZoneConfig.MaxCell = FIntVector(
                MinX + SizeXY - 1,
                MinY + SizeXY - 1,
                MinZ + SizeZ - 1);
            ZoneConfig.StartTimeStep =
                RandomStream.RandRange(SafeMinStart, SafeMaxStart);
            const int32 Duration =
                RandomStream.RandRange(SafeMinDuration, SafeMaxDuration);
            ZoneConfig.EndTimeStep = ZoneConfig.StartTimeStep + Duration - 1;

            int32 CellCount = 0;
            int32 FreeCount = 0;
            for (int32 X = ZoneConfig.MinCell.X;
                 X <= ZoneConfig.MaxCell.X;
                 ++X)
            {
                for (int32 Y = ZoneConfig.MinCell.Y;
                     Y <= ZoneConfig.MaxCell.Y;
                     ++Y)
                {
                    for (int32 Z = ZoneConfig.MinCell.Z;
                         Z <= ZoneConfig.MaxCell.Z;
                         ++Z)
                    {
                        CellCount++;
                        if (!Request.GridMap->IsBlocked(X, Y, Z))
                        {
                            FreeCount++;
                        }
                    }
                }
            }

            if (CellCount <= 0 || FreeCount <= 0)
            {
                continue;
            }

            Request.ZoneConfigs->Add(ZoneConfig);
            bGenerated = true;
            break;
        }

        if (!bGenerated)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("Failed to generate no-fly zone %d"),
                ZoneId);
        }
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("Generated random no-fly zones: %d (Seed=%d Size=[%d,%d] Start=[%d,%d] Duration=[%d,%d])"),
        Request.ZoneConfigs->Num(),
        Request.RandomSeed,
        SafeMinSize,
        SafeMaxSize,
        SafeMinStart,
        SafeMaxStart,
        SafeMinDuration,
        SafeMaxDuration);
}
