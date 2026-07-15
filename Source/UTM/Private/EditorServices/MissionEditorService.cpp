#include "EditorServices/MissionEditorService.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "Planning/GridMap3D.h"
#include "Planning/PlanningInputValidator.h"
#include "Planning/UTMSafetyModel.h"

namespace
{
    void GetMissionMarkerActors(
        UWorld* World,
        TArray<AMissionMarkerActor*>& OutMarkers)
    {
        OutMarkers.Reset();
        if (!World)
        {
            return;
        }

        for (TActorIterator<AMissionMarkerActor> It(World); It; ++It)
        {
            AMissionMarkerActor* Marker = *It;
            if (Marker && Marker->Tags.Contains(FName(TEXT("MissionMarker"))))
            {
                OutMarkers.Add(Marker);
            }
        }
    }

    bool HasConflictWithExistingStarts(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& MissionConfigs,
        const FIntVector& CandidateStartCell,
        const FDroneMissionConfig& CandidateMission)
    {
        for (const FDroneMissionConfig& ExistingMission : MissionConfigs)
        {
            const FIntVector ExistingStartCell =
                GridMap.WorldToCell(ExistingMission.StartWorld);
            if (FUTMSafetyModel::HasStaticUTMConfigConflict(
                CandidateStartCell,
                CandidateMission,
                ExistingStartCell,
                ExistingMission))
            {
                return true;
            }
        }

        return false;
    }

    bool HasConflictWithExistingGoals(
        const FGridMap3D& GridMap,
        const TArray<FDroneMissionConfig>& MissionConfigs,
        const FIntVector& CandidateGoalCell,
        const FDroneMissionConfig& CandidateMission)
    {
        for (const FDroneMissionConfig& ExistingMission : MissionConfigs)
        {
            const FIntVector ExistingGoalCell =
                GridMap.WorldToCell(ExistingMission.GoalWorld);
            if (FUTMSafetyModel::HasStaticUTMConfigConflict(
                CandidateGoalCell,
                CandidateMission,
                ExistingGoalCell,
                ExistingMission))
            {
                return true;
            }
        }

        return false;
    }

    bool TryGenerateSingleMission(
        const FMissionEditorGenerateRandomRequest& Request,
        FRandomStream& RandomStream,
        int32 MissionId,
        TSet<FIntVector>& UsedStarts,
        TSet<FIntVector>& UsedGoals,
        FDroneMissionConfig& OutMission)
    {
        constexpr int32 MaxTryCount = 500;
        const FGridMap3D& GridMap = *Request.GridMap;
        const TArray<FDroneMissionConfig>& MissionConfigs =
            *Request.MissionConfigs;

        for (int32 TryIndex = 0; TryIndex < MaxTryCount; ++TryIndex)
        {
            const FIntVector StartCell(
                RandomStream.RandRange(0, Request.GridDim.X - 1),
                RandomStream.RandRange(0, Request.GridDim.Y - 1),
                RandomStream.RandRange(0, Request.GridDim.Z - 1));
            if (GridMap.IsBlocked(StartCell.X, StartCell.Y, StartCell.Z))
            {
                continue;
            }

            if (!Request.bAllowDuplicateStartCells && UsedStarts.Contains(StartCell))
            {
                continue;
            }

            for (int32 GoalTry = 0; GoalTry < MaxTryCount; ++GoalTry)
            {
                const FIntVector GoalCell(
                    RandomStream.RandRange(0, Request.GridDim.X - 1),
                    RandomStream.RandRange(0, Request.GridDim.Y - 1),
                    RandomStream.RandRange(0, Request.GridDim.Z - 1));
                if (GridMap.IsBlocked(GoalCell.X, GoalCell.Y, GoalCell.Z))
                {
                    continue;
                }

                if (!Request.bAllowDuplicateGoalCells && UsedGoals.Contains(GoalCell))
                {
                    continue;
                }

                const int32 ManhattanDistance =
                    FMath::Abs(StartCell.X - GoalCell.X) +
                    FMath::Abs(StartCell.Y - GoalCell.Y) +
                    FMath::Abs(StartCell.Z - GoalCell.Z);
                if (ManhattanDistance < Request.MinStartGoalCellDistance)
                {
                    continue;
                }

                FDroneMissionConfig CandidateMission = OutMission;
                CandidateMission.MissionId = MissionId;
                CandidateMission.StartWorld = GridMap.CellToWorld(StartCell);
                CandidateMission.GoalWorld = GridMap.CellToWorld(GoalCell);
                if (HasConflictWithExistingStarts(
                    GridMap,
                    MissionConfigs,
                    StartCell,
                    CandidateMission))
                {
                    continue;
                }

                if (HasConflictWithExistingGoals(
                    GridMap,
                    MissionConfigs,
                    GoalCell,
                    CandidateMission))
                {
                    continue;
                }

                OutMission = CandidateMission;
                UsedStarts.Add(StartCell);
                UsedGoals.Add(GoalCell);
                return true;
            }
        }

        return false;
    }
}

void FMissionEditorService::AppendMissionMarkerActors(
    UWorld* World,
    TArray<AActor*>& InOutActors)
{
    TArray<AMissionMarkerActor*> Markers;
    GetMissionMarkerActors(World, Markers);
    for (AMissionMarkerActor* Marker : Markers)
    {
        if (Marker)
        {
            InOutActors.Add(Marker);
        }
    }
}

void FMissionEditorService::GenerateRandomMissionConfigs(
    const FMissionEditorGenerateRandomRequest& Request)
{
    if (!Request.GridMap || !Request.MissionConfigs)
    {
        return;
    }

    Request.MissionConfigs->Reset();
    FRandomStream RandomStream(Request.RandomSeed);
    TSet<FIntVector> UsedStarts;
    TSet<FIntVector> UsedGoals;

    for (int32 Index = 0; Index < Request.RandomMissionCount; ++Index)
    {
        FDroneMissionConfig NewMission;
        const int32 MissionId = Index + 1;
        if (!TryGenerateSingleMission(
            Request,
            RandomStream,
            MissionId,
            UsedStarts,
            UsedGoals,
            NewMission))
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("Failed to generate mission %d under current random-space and UTM safety constraints. Consider reducing RandomMissionCount or shrinking mission footprints."),
                MissionId);
            continue;
        }

        Request.MissionConfigs->Add(NewMission);
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("Generated random missions: %d"),
        Request.MissionConfigs->Num());
}

void FMissionEditorService::ClearMissionMarkers(UWorld* World)
{
    TArray<AMissionMarkerActor*> Markers;
    GetMissionMarkerActors(World, Markers);

    int32 DeleteCount = 0;
    for (AMissionMarkerActor* Marker : Markers)
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
        TEXT("EditorClearMissionMarkers deleted %d markers"),
        DeleteCount);
}

void FMissionEditorService::SpawnMissionMarkers(
    const FMissionEditorSpawnMarkersRequest& Request)
{
    if (!Request.MarkerClass)
    {
        UE_LOG(LogTemp, Error, TEXT("MissionMarkerClass is null"));
        return;
    }

    ClearMissionMarkers(Request.World);
    if (!Request.World || !Request.GridMap || !Request.MissionConfigs)
    {
        return;
    }

    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = Request.Owner;
    SpawnParams.SpawnCollisionHandlingOverride =
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    for (const FDroneMissionConfig& Mission : *Request.MissionConfigs)
    {
        const FIntVector StartCell =
            Request.GridMap->WorldToCell(Mission.StartWorld);
        const FIntVector GoalCell =
            Request.GridMap->WorldToCell(Mission.GoalWorld);

        const FVector StartLocation =
            Mission.StartWorld + FVector(0.f, 0.f, Request.MarkerZOffset);
        AMissionMarkerActor* StartMarker =
            Request.World->SpawnActor<AMissionMarkerActor>(
                Request.MarkerClass,
                StartLocation,
                FRotator::ZeroRotator,
                SpawnParams);
        if (StartMarker)
        {
            StartMarker->MissionId = Mission.MissionId;
            StartMarker->MarkerType = EMissionMarkerType::Start;
            StartMarker->Cell = StartCell;
            StartMarker->Tags.Add(FName(TEXT("MissionMarker")));
            StartMarker->UpdateVisual();
        }

        const FVector GoalLocation =
            Mission.GoalWorld + FVector(0.f, 0.f, Request.MarkerZOffset);
        AMissionMarkerActor* GoalMarker =
            Request.World->SpawnActor<AMissionMarkerActor>(
                Request.MarkerClass,
                GoalLocation,
                FRotator::ZeroRotator,
                SpawnParams);
        if (GoalMarker)
        {
            GoalMarker->MissionId = Mission.MissionId;
            GoalMarker->MarkerType = EMissionMarkerType::Goal;
            GoalMarker->Cell = GoalCell;
            GoalMarker->Tags.Add(FName(TEXT("MissionMarker")));
            GoalMarker->UpdateVisual();
        }
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("EditorSpawnMissionMarkers done. Mission count=%d"),
        Request.MissionConfigs->Num());
}

void FMissionEditorService::ValidateMissionConfigs(
    const FMissionEditorValidateRequest& Request)
{
    if (!Request.GridMap || !Request.InputValidator || !Request.MissionConfigs)
    {
        return;
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("========== Validate MissionConfigs begin. Count=%d  =========="),
        Request.MissionConfigs->Num());

    TSet<int32> MissionIds;
    TSet<FIntVector> StartCells;
    TSet<FIntVector> GoalCells;
    for (const FDroneMissionConfig& Mission : *Request.MissionConfigs)
    {
        if (MissionIds.Contains(Mission.MissionId))
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("Duplicate MissionId=%d"),
                Mission.MissionId);
        }
        MissionIds.Add(Mission.MissionId);

        const bool bValid = Request.InputValidator->ValidateStartGoalPair(
            *Request.GridMap,
            Mission.StartWorld,
            Mission.GoalWorld,
            Mission.MissionId,
            nullptr,
            nullptr);
        const FIntVector StartCell =
            Request.GridMap->WorldToCell(Mission.StartWorld);
        const FIntVector GoalCell =
            Request.GridMap->WorldToCell(Mission.GoalWorld);

        if (StartCells.Contains(StartCell))
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("Duplicate StartCell in Mission %d: (%d,%d,%d)"),
                Mission.MissionId,
                StartCell.X,
                StartCell.Y,
                StartCell.Z);
        }
        StartCells.Add(StartCell);

        if (GoalCells.Contains(GoalCell))
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("Duplicate GoalCell in Mission %d: (%d,%d,%d)"),
                Mission.MissionId,
                GoalCell.X,
                GoalCell.Y,
                GoalCell.Z);
        }
        GoalCells.Add(GoalCell);

        if (bValid)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("Mission %d valid"),
                Mission.MissionId);
        }
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("========== Validate MissionConfigs end  =========="));
}

void FMissionEditorService::ReadMissionMarkersToConfigs(
    const FMissionEditorReadMarkersRequest& Request)
{
    if (!Request.GridMap || !Request.MissionConfigs)
    {
        return;
    }

    TArray<AMissionMarkerActor*> Markers;
    GetMissionMarkerActors(Request.World, Markers);
    TMap<int32, FVector> StartMap;
    TMap<int32, FVector> GoalMap;
    for (AMissionMarkerActor* Marker : Markers)
    {
        if (!Marker)
        {
            continue;
        }

        const FVector MarkerWorld =
            Marker->GetActorLocation() - FVector(0.f, 0.f, Request.MarkerZOffset);
        const FIntVector Cell = Request.GridMap->WorldToCell(MarkerWorld);
        const FVector SnappedWorld = Request.GridMap->CellToWorld(Cell);
        if (Marker->MarkerType == EMissionMarkerType::Start)
        {
            StartMap.Add(Marker->MissionId, SnappedWorld);
        }
        else
        {
            GoalMap.Add(Marker->MissionId, SnappedWorld);
        }
    }

    Request.MissionConfigs->Reset();
    TArray<int32> MissionIds;
    StartMap.GetKeys(MissionIds);
    for (const int32 MissionId : MissionIds)
    {
        if (!GoalMap.Contains(MissionId))
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("Mission %d has start marker but no goal marker"),
                MissionId);
            continue;
        }

        FDroneMissionConfig Mission;
        Mission.MissionId = MissionId;
        Mission.StartWorld = StartMap[MissionId];
        Mission.GoalWorld = GoalMap[MissionId];
        Request.MissionConfigs->Add(Mission);
    }

    Request.MissionConfigs->Sort(
        [](const FDroneMissionConfig& A, const FDroneMissionConfig& B)
        {
            return A.MissionId < B.MissionId;
        });

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("EditorReadMissionMarkersToConfigs done. Mission count=%d"),
        Request.MissionConfigs->Num());
}
