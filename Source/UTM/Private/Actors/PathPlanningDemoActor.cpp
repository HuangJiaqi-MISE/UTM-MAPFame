#include "Actors/PathPlanningDemoActor.h"

#include "Components/SceneComponent.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"

#include "Actors/DroneActor.h"
#include "Engine/World.h"

#include "Planning/AStarPlanner.h"
#include "Planning/CBSPlanner.h"
#include "Planning/ECBSPlanner.h"
#include "Planning/LaCAMPlanner.h"
#include "Planning/LaCAMUTM.h"
#include "Planning/PBSPlanner.h"
#include "Planning/SIPPPlanner.h"

#include "Actors/MissionMarkerActor.h"
#include "Engine/StaticMeshActor.h"
#include "Kismet/GameplayStatics.h"

// 障碍物建筑构建
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

#include "HAL/PlatformTime.h"

namespace
{
    constexpr int32 MaxLoggedPathPoints = 64;
    constexpr int32 LoggedPathPreviewCount = 16;
    constexpr int32 MaxDebugDrawPathPoints = 2048;
    constexpr int32 MaxSpawnableDronePathPoints = 20000;

    TArray<FVector> BuildDebugPreviewPath(const TArray<FVector>& InPath)
    {
        if (InPath.Num() <= MaxDebugDrawPathPoints)
        {
            return InPath;
        }

        const int32 TargetPoints = FMath::Max(2, MaxDebugDrawPathPoints);
        const int32 Stride = FMath::Max(1, (InPath.Num() - 1) / (TargetPoints - 1));

        TArray<FVector> Preview;
        Preview.Reserve(TargetPoints);

        for (int32 Index = 0; Index < InPath.Num(); Index += Stride)
        {
            if (Preview.Num() >= TargetPoints - 1)
            {
                break;
            }

            const FVector& Candidate = InPath[Index];
            if (Preview.Num() <= 0 || !Preview.Last().Equals(Candidate))
            {
                Preview.Add(Candidate);
            }
        }

        if (Preview.Num() <= 0 || !Preview.Last().Equals(InPath.Last()))
        {
            Preview.Add(InPath.Last());
        }

        return Preview;
    }
}

APathPlanningDemoActor::APathPlanningDemoActor()
{
    PrimaryActorTick.bCanEverTick = true;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
    RootComponent = SceneRoot;
}

// 更新代码结构，手动控制是否在 BeginPlay 里自动运行规划逻辑，方便调试和编辑器交互
void APathPlanningDemoActor::BeginPlay()
{
    Super::BeginPlay();

    UE_LOG(LogTemp, Warning, TEXT("PathPlanningDemoActor BeginPlay called"));

    if (!bAutoRunPlanningOnBeginPlay)
    {
        UE_LOG(LogTemp, Warning, TEXT("Auto planning on BeginPlay is disabled"));
        return;
    }

    RunPlanning();
}



void APathPlanningDemoActor::RunPlanning()
{
    ResetPlanningStats();
    ResetPathValidationCache();

    const double TotalStartTime = FPlatformTime::Seconds();

    UE_LOG(LogTemp, Warning, TEXT("RunPlanning begin"));

    int32 DestroyedDroneCount = 0;
    for (TObjectPtr<ADroneActor>& SpawnedDrone : SpawnedDrones)
    {
        if (!SpawnedDrone)
        {
            continue;
        }

        SpawnedDrone->Destroy();
        DestroyedDroneCount++;
    }

    SpawnedDrones.Reset();
    SpawnedDroneByMissionId.Reset();
    ResetExecutionCache();

    if (DestroyedDroneCount > 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("Destroyed %d previously spawned drones"), DestroyedDroneCount);
    }

    UE_LOG(LogTemp, Warning, TEXT("Selected planner type: %s"), *GetPlannerTypeName());

    GridMap.GridOrigin = GridOrigin;
    GridMap.GridDim = GridDim;
    GridMap.CellSize = CellSize;

    const FVector TestWorld = GetActorLocation();
    const FIntVector TestCell = GridMap.WorldToCell(TestWorld);
    const FVector BackToWorld = GridMap.CellToWorld(TestCell);

    UE_LOG(LogTemp, Warning, TEXT("TestWorld = %s"), *TestWorld.ToString());
    UE_LOG(LogTemp, Warning, TEXT("TestCell = X=%d Y=%d Z=%d"), TestCell.X, TestCell.Y, TestCell.Z);
    UE_LOG(LogTemp, Warning, TEXT("BackToWorld = %s"), *BackToWorld.ToString());

    TArray<AActor*> IgnoreActors;
    IgnoreActors.Add(this);

    bool bOverallSuccess = false;

    if (!bUseMissionConfigs)
    {
        const double InputPrepStart = FPlatformTime::Seconds();

        TArray<int32> Ids;
        TMap<int32, TObjectPtr<AActor>> Starts;
        TMap<int32, TObjectPtr<AActor>> Goals;

        CollectStartGoalPairs(Ids, Starts, Goals);
        LastPlanningStats.MissionCount = Ids.Num();

        UE_LOG(LogTemp, Warning, TEXT("Found %d Start/Goal pairs"), Ids.Num());

        for (const auto& KVP : Starts)
        {
            if (KVP.Value)
            {
                IgnoreActors.Add(KVP.Value);
            }
        }

        for (const auto& KVP : Goals)
        {
            if (KVP.Value)
            {
                IgnoreActors.Add(KVP.Value);
            }
        }

        LastPlanningStats.InputPreparationTimeMs =
            (FPlatformTime::Seconds() - InputPrepStart) * 1000.0;

        const double BuildGridStart = FPlatformTime::Seconds();
        GridMap.BuildOccupancyGrid(
            GetWorld(),
            IgnoreActors,
            bDrawOccupiedCells,
            bDrawFreeCells,
            DebugDrawTime
        );
        LastPlanningStats.BuildGridTimeMs =
            (FPlatformTime::Seconds() - BuildGridStart) * 1000.0;

        if (IsMultiAgentPlannerType())
        {
            bOverallSuccess = ProcessStartGoalPairsMultiAgent();
        }
        else
        {
            bOverallSuccess = ProcessStartGoalPairsSingleAgent(Ids, Starts, Goals);
        }
    }
    else
    {
        LastPlanningStats.MissionCount = MissionConfigs.Num();

        const double BuildGridStart = FPlatformTime::Seconds();
        GridMap.BuildOccupancyGrid(
            GetWorld(),
            IgnoreActors,
            bDrawOccupiedCells,
            bDrawFreeCells,
            DebugDrawTime
        );
        LastPlanningStats.BuildGridTimeMs =
            (FPlatformTime::Seconds() - BuildGridStart) * 1000.0;

        if (IsMultiAgentPlannerType())
        {
            bOverallSuccess = ProcessMissionConfigsMultiAgent();
        }
        else
        {
            bOverallSuccess = ProcessMissionConfigs();
        }
    }

    LastPlanningStats.bSuccess = bOverallSuccess;
    LastPlanningStats.TotalTimeMs = (FPlatformTime::Seconds() - TotalStartTime) * 1000.0;

    LogPlanningStatsSummary();

    if (bOverallSuccess && IsMultiAgentPlannerType() && bUseCentralizedExecution)
    {
        InitializeExecutionStates();
    }

    if (bValidatePathsAgainstNoFlyZones)
    {
        ValidateLastPlannedPathsAgainstNoFlyZones();
    }

    UE_LOG(LogTemp, Warning, TEXT("RunPlanning end"));
}






void APathPlanningDemoActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (!bExecutionRunning || !IsMultiAgentPlannerType() || !bUseCentralizedExecution)
    {
        return;
    }

    ExecutionAccumulator += DeltaTime;

    while (ExecutionAccumulator >= CBSStepDuration)
    {
        ExecutionAccumulator -= CBSStepDuration;
        AdvanceExecutionOneStep();
    }

    const float Alpha = (CBSStepDuration > KINDA_SMALL_NUMBER)
        ? FMath::Clamp(ExecutionAccumulator / CBSStepDuration, 0.f, 1.f)
        : 1.f;

    UpdateExecutionVisuals(Alpha);
}

bool APathPlanningDemoActor::ParseTaggedId(AActor* Actor, const FString& Prefix, int32& OutId) const
{
    if (!Actor)
    {
        return false;
    }

    for (const FName& TagName : Actor->Tags)
    {
        const FString Tag = TagName.ToString();

        if (!Tag.StartsWith(Prefix))
        {
            continue;
        }

        const FString NumStr = Tag.Mid(Prefix.Len());
        if (NumStr.IsEmpty())
        {
            continue;
        }

        const int32 Id = FCString::Atoi(*NumStr);
        if (Id <= 0)
        {
            continue;
        }

        OutId = Id;
        return true;
    }

    return false;
}

void APathPlanningDemoActor::CollectStartGoalPairs(
    TArray<int32>& OutIds,
    TMap<int32, TObjectPtr<AActor>>& OutStarts,
    TMap<int32, TObjectPtr<AActor>>& OutGoals) const
{
    OutIds.Reset();
    OutStarts.Reset();
    OutGoals.Reset();

    for (TActorIterator<AActor> It(GetWorld()); It; ++It)
    {
        AActor* A = *It;
        if (!A)
        {
            continue;
        }

        int32 Id = 0;

        if (ParseTaggedId(A, TEXT("Start_"), Id))
        {
            if (OutStarts.Contains(Id))
            {
                UE_LOG(LogTemp, Warning, TEXT("Duplicate Start_%d: %s"), Id, *A->GetName());
            }
            else
            {
                OutStarts.Add(Id, A);
            }
        }
        else if (ParseTaggedId(A, TEXT("Goal_"), Id))
        {
            if (OutGoals.Contains(Id))
            {
                UE_LOG(LogTemp, Warning, TEXT("Duplicate Goal_%d: %s"), Id, *A->GetName());
            }
            else
            {
                OutGoals.Add(Id, A);
            }
        }
    }

    for (const auto& KVP : OutStarts)
    {
        const int32 Id = KVP.Key;
        if (OutGoals.Contains(Id))
        {
            OutIds.Add(Id);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Start_%d exists but Goal_%d missing"), Id, Id);
        }
    }

    for (const auto& KVP : OutGoals)
    {
        const int32 Id = KVP.Key;
        if (!OutStarts.Contains(Id))
        {
            UE_LOG(LogTemp, Warning, TEXT("Goal_%d exists but Start_%d missing"), Id, Id);
        }
    }

    OutIds.Sort();
}

FColor APathPlanningDemoActor::GetDebugColorById(int32 Id) const
{
    const FColor Colors[] =
    {
        FColor::Red,
        FColor::Green,
        FColor::Blue,
        FColor::Yellow,
        FColor::Cyan,
        FColor::Magenta,
        FColor(255, 128, 0),
        FColor(128, 0, 255)
    };

    const int32 Count = UE_ARRAY_COUNT(Colors);
    return Colors[(Id - 1) % Count];
}

// LogPathCoordinates开关，控制输出路径坐标到日志，方便调试验证
void APathPlanningDemoActor::LogPathCoordinates(const TArray<FVector>& InPath, int32 Id, const TCHAR* Label) const
{
    if (!bLogPathCoordinates)
    {
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("%s %d path coordinates. Points=%d"), Label, Id, InPath.Num());

    if (InPath.Num() <= MaxLoggedPathPoints)
    {
        for (int32 Index = 0; Index < InPath.Num(); ++Index)
        {
            const FVector& WorldPoint = InPath[Index];
            const FIntVector CellPoint = GridMap.WorldToCell(WorldPoint);

            UE_LOG(
                LogTemp,
                Warning,
                TEXT("  [%d] World=(%.1f, %.1f, %.1f) Cell=(%d, %d, %d)"),
                Index,
                WorldPoint.X,
                WorldPoint.Y,
                WorldPoint.Z,
                CellPoint.X,
                CellPoint.Y,
                CellPoint.Z
            );
        }
        return;
    }

    const int32 HeadCount = FMath::Min(LoggedPathPreviewCount, InPath.Num());
    const int32 TailStart = FMath::Max(HeadCount, InPath.Num() - LoggedPathPreviewCount);

    for (int32 Index = 0; Index < HeadCount; ++Index)
    {
        const FVector& WorldPoint = InPath[Index];
        const FIntVector CellPoint = GridMap.WorldToCell(WorldPoint);

        UE_LOG(
            LogTemp,
            Warning,
            TEXT("  [%d] World=(%.1f, %.1f, %.1f) Cell=(%d, %d, %d)"),
            Index,
            WorldPoint.X,
            WorldPoint.Y,
            WorldPoint.Z,
            CellPoint.X,
            CellPoint.Y,
            CellPoint.Z
        );
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("  ... omitted %d intermediate points to keep log responsive ..."),
        FMath::Max(0, TailStart - HeadCount));

    for (int32 Index = TailStart; Index < InPath.Num(); ++Index)
    {
        const FVector& WorldPoint = InPath[Index];
        const FIntVector CellPoint = GridMap.WorldToCell(WorldPoint);

        UE_LOG(
            LogTemp,
            Warning,
            TEXT("  [%d] World=(%.1f, %.1f, %.1f) Cell=(%d, %d, %d)"),
            Index,
            WorldPoint.X,
            WorldPoint.Y,
            WorldPoint.Z,
            CellPoint.X,
            CellPoint.Y,
            CellPoint.Z
        );
    }
}

void APathPlanningDemoActor::DrawPathDebug(const TArray<FVector>& InPath, const FColor& Color) const
{
    if (!GetWorld() || InPath.Num() <= 0)
    {
        return;
    }

    const FVector Offset(0.f, 0.f, 20.f);
    const TArray<FVector> DebugPath = BuildDebugPreviewPath(InPath);

    if (DebugPath.Num() != InPath.Num())
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("Path debug draw truncated from %d to %d points"),
            InPath.Num(),
            DebugPath.Num());
    }

    if (bDrawPathPoints)
    {
        for (const FVector& P : DebugPath)
        {
            DrawDebugSphere(
                GetWorld(),
                P + Offset,
                18.f,
                8,
                Color,
                false,
                PathDrawTime
            );
        }
    }

    if (bDrawPaths)
    {
        for (int32 i = 0; i < DebugPath.Num() - 1; ++i)
        {
            DrawDebugLine(
                GetWorld(),
                DebugPath[i] + Offset,
                DebugPath[i + 1] + Offset,
                Color,
                false,
                PathDrawTime,
                0,
                PathLineThickness
            );
        }
    }
}

bool APathPlanningDemoActor::IsCellInsideNoFlyZoneAtTime(
    const FIntVector& Cell,
    int32 TimeStep,
    const FTemporalNoFlyZoneConfig& ZoneConfig) const
{
    if (!ZoneConfig.bEnabled)
    {
        return false;
    }

    const int32 StartTime = FMath::Max(0, ZoneConfig.StartTimeStep);
    const int32 EndTime = FMath::Max(StartTime, ZoneConfig.EndTimeStep);
    if (TimeStep < StartTime || TimeStep > EndTime)
    {
        return false;
    }

    FIntVector MinCell(
        FMath::Min(ZoneConfig.MinCell.X, ZoneConfig.MaxCell.X),
        FMath::Min(ZoneConfig.MinCell.Y, ZoneConfig.MaxCell.Y),
        FMath::Min(ZoneConfig.MinCell.Z, ZoneConfig.MaxCell.Z));
    FIntVector MaxCell(
        FMath::Max(ZoneConfig.MinCell.X, ZoneConfig.MaxCell.X),
        FMath::Max(ZoneConfig.MinCell.Y, ZoneConfig.MaxCell.Y),
        FMath::Max(ZoneConfig.MinCell.Z, ZoneConfig.MaxCell.Z));

    MinCell.X = FMath::Clamp(MinCell.X, 0, FMath::Max(0, GridDim.X - 1));
    MinCell.Y = FMath::Clamp(MinCell.Y, 0, FMath::Max(0, GridDim.Y - 1));
    MinCell.Z = FMath::Clamp(MinCell.Z, 0, FMath::Max(0, GridDim.Z - 1));
    MaxCell.X = FMath::Clamp(MaxCell.X, 0, FMath::Max(0, GridDim.X - 1));
    MaxCell.Y = FMath::Clamp(MaxCell.Y, 0, FMath::Max(0, GridDim.Y - 1));
    MaxCell.Z = FMath::Clamp(MaxCell.Z, 0, FMath::Max(0, GridDim.Z - 1));

    return Cell.X >= MinCell.X && Cell.X <= MaxCell.X
        && Cell.Y >= MinCell.Y && Cell.Y <= MaxCell.Y
        && Cell.Z >= MinCell.Z && Cell.Z <= MaxCell.Z;
}

void APathPlanningDemoActor::ValidatePathAgainstNoFlyZones(
    int32 MissionId,
    const TArray<FVector>& PathPoints,
    FNoFlyZonePathValidationSummary& InOutSummary) const
{
    if (PathPoints.Num() <= 0)
    {
        return;
    }

    InOutSummary.CheckedMissionCount++;

    bool bMissionHasViolation = false;
    for (int32 TimeStep = 0; TimeStep < PathPoints.Num(); ++TimeStep)
    {
        const FIntVector Cell = GridMap.WorldToCell(PathPoints[TimeStep]);
        InOutSummary.CheckedPointCount++;

        for (const FTemporalNoFlyZoneConfig& ZoneConfig : NoFlyZoneConfigs)
        {
            if (!IsCellInsideNoFlyZoneAtTime(Cell, TimeStep, ZoneConfig))
            {
                continue;
            }

            if (!bMissionHasViolation)
            {
                InOutSummary.ViolatingMissionCount++;
                bMissionHasViolation = true;
            }

            InOutSummary.TotalViolationCount++;

            FNoFlyZonePathViolation Violation;
            Violation.MissionId = MissionId;
            Violation.ZoneId = ZoneConfig.ZoneId;
            Violation.TimeStep = TimeStep;
            Violation.Cell = Cell;
            InOutSummary.Violations.Add(Violation);
        }
    }
}

void APathPlanningDemoActor::LogNoFlyZonePathValidationSummary(
    const FNoFlyZonePathValidationSummary& Summary) const
{
    int32 EnabledZoneCount = 0;
    for (const FTemporalNoFlyZoneConfig& ZoneConfig : NoFlyZoneConfigs)
    {
        if (ZoneConfig.bEnabled)
        {
            EnabledZoneCount++;
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("==!!!======== No-Fly Zone Path Validation ========!!!=="));
    UE_LOG(LogTemp, Warning, TEXT("EnabledZoneCount    : %d"), EnabledZoneCount);
    UE_LOG(LogTemp, Warning, TEXT("CheckedMissionCount : %d"), Summary.CheckedMissionCount);
    UE_LOG(LogTemp, Warning, TEXT("CheckedPointCount   : %d"), Summary.CheckedPointCount);
    UE_LOG(LogTemp, Warning, TEXT("ViolatingMissionCnt : %d"), Summary.ViolatingMissionCount);
    UE_LOG(LogTemp, Warning, TEXT("TotalViolationCount : %d"), Summary.TotalViolationCount);

    if (Summary.TotalViolationCount <= 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("No-Fly Zone Path Validation result: clear and pass"));
        UE_LOG(LogTemp, Warning, TEXT("==!!!============================================!!!=="));
        return;
    }

    const int32 MaxLoggedCount = FMath::Clamp(MaxLoggedNoFlyZoneViolations, 0, Summary.Violations.Num());
    for (int32 Index = 0; Index < MaxLoggedCount; ++Index)
    {
        const FNoFlyZonePathViolation& Violation = Summary.Violations[Index];
        UE_LOG(
            LogTemp,
            Error,
            TEXT("Violation[%d] Mission=%d Zone=%d Time=%d Cell=(%d,%d,%d)"),
            Index,
            Violation.MissionId,
            Violation.ZoneId,
            Violation.TimeStep,
            Violation.Cell.X,
            Violation.Cell.Y,
            Violation.Cell.Z);
    }

    if (Summary.Violations.Num() > MaxLoggedCount)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("... omitted %d additional no-fly-zone violations ..."),
            Summary.Violations.Num() - MaxLoggedCount);
    }

    UE_LOG(LogTemp, Warning, TEXT("==============================================="));
}

void APathPlanningDemoActor::CachePlannedPath(int32 MissionId, const TArray<FVector>& PathPoints)
{
    LastPlannedPathsByMission.Add(MissionId, PathPoints);
    PlannedCellPathsByMission.Add(MissionId, BuildCellPathFromWorldPath(PathPoints));
}

void APathPlanningDemoActor::ResetPathValidationCache()
{
    LastPlannedPathsByMission.Reset();
    LastNoFlyZonePathValidation = FNoFlyZonePathValidationSummary();
}

void APathPlanningDemoActor::ValidateLastPlannedPathsAgainstNoFlyZones()
{
    LastNoFlyZonePathValidation = FNoFlyZonePathValidationSummary();

    for (const TPair<int32, TArray<FVector>>& KVP : LastPlannedPathsByMission)
    {
        ValidatePathAgainstNoFlyZones(KVP.Key, KVP.Value, LastNoFlyZonePathValidation);
    }

    LogNoFlyZonePathValidationSummary(LastNoFlyZonePathValidation);
}

// 大类：新增执行器代码
// 将世界坐标路径转换为网格坐标路径，方便后续执行和验证使用
TArray<FIntVector> APathPlanningDemoActor::BuildCellPathFromWorldPath(const TArray<FVector>& PathPoints) const
{
    TArray<FIntVector> Result;
    Result.Reserve(PathPoints.Num());

    for (const FVector& P : PathPoints)
    {
        Result.Add(GridMap.WorldToCell(P));
    }

    return Result;
}

FIntVector APathPlanningDemoActor::GetCellAtTime(const TArray<FIntVector>& Cells, int32 TimeStep) const
{
    if (Cells.Num() <= 0)
    {
        return FIntVector::ZeroValue;
    }

    if (TimeStep <= 0)
    {
        return Cells[0];
    }

    if (TimeStep < Cells.Num())
    {
        return Cells[TimeStep];
    }

    return Cells.Last();
}

void APathPlanningDemoActor::ResetExecutionCache()
{
    PlannedCellPathsByMission.Reset();
    ExecutionStates.Reset();
    ExecutionConflicts.Reset();

    ExecutionAccumulator = 0.f;
    CurrentExecutionTimeStep = 0;
    bExecutionRunning = false;

    LastExecutionSummary = FExecutionSummary();
}

void APathPlanningDemoActor::InitializeExecutionStates()
{
    ExecutionStates.Reset();
    ExecutionConflicts.Reset();

    ExecutionRandom.Initialize(ExecutionRandomSeed);
    ExecutionAccumulator = 0.f;
    CurrentExecutionTimeStep = 0;
    bExecutionRunning = false;

    for (const TPair<int32, TArray<FIntVector>>& KVP : PlannedCellPathsByMission)
    {
        const int32 MissionId = KVP.Key;
        const TArray<FIntVector>& PlannedCells = KVP.Value;

        if (PlannedCells.Num() <= 0)
        {
            continue;
        }

        FExecutionAgentState State;
        State.MissionId = MissionId;
        State.Drone = SpawnedDroneByMissionId.FindRef(MissionId);
        State.PlannedCells = PlannedCells;
        State.ActualCells.Add(PlannedCells[0]);
        State.ExecutedPlanIndex = 0;
        State.TotalDelaySteps = 0;
        State.bFinished = (PlannedCells.Num() <= 1);
        State.DisplayFromCell = PlannedCells[0];
        State.DisplayToCell = PlannedCells[0];

        if (State.Drone)
        {
            State.Drone->SetActorLocation(GridMap.CellToWorld(PlannedCells[0]));
        }

        ExecutionStates.Add(MissionId, State);
    }

    bExecutionRunning = (ExecutionStates.Num() > 0);

    DetectExecutionConflictsAtStep(0);

    if (!bExecutionRunning)
    {
        BuildExecutionSummary();

        if (bLogExecutionSummary)
        {
            LogExecutionSummary();
        }
    }
}

const FAgentDelayConfig* APathPlanningDemoActor::FindAgentDelayConfig(int32 MissionId) const
{
    for (const FAgentDelayConfig& Config : AgentDelayConfigs)
    {
        if (Config.MissionId == MissionId)
        {
            return &Config;
        }
    }

    return nullptr;
}

bool APathPlanningDemoActor::IsForcedDelayStep(const FExecutionAgentState& State, int32 TimeStep) const
{
    const FAgentDelayConfig* Config = FindAgentDelayConfig(State.MissionId);
    if (!Config)
    {
        return false;
    }

    return Config->ForcedDelaySteps.Contains(TimeStep);
}

bool APathPlanningDemoActor::ShouldDelayThisStep(const FExecutionAgentState& State, int32 TimeStep)
{
    if (State.bFinished)
    {
        return false;
    }

    switch (DelayMode)
    {
    case EExecutionDelayMode::RandomGlobal:
    {
        const float P = FMath::Clamp(StepDelayProbability, 0.f, 1.f);
        return (P > 0.f) && (ExecutionRandom.FRand() < P);
    }

    case EExecutionDelayMode::PerAgentProbability:
    {
        const FAgentDelayConfig* Config = FindAgentDelayConfig(State.MissionId);
        if (!Config)
        {
            return false;
        }

        const float P = FMath::Clamp(Config->DelayProbability, 0.f, 1.f);
        return (P > 0.f) && (ExecutionRandom.FRand() < P);
    }

    case EExecutionDelayMode::ScriptedTimesteps:
        return IsForcedDelayStep(State, TimeStep);

    default:
        return false;
    }
}

void APathPlanningDemoActor::AdvanceExecutionOneStep()
{
    CurrentExecutionTimeStep++;

    bool bAnyActive = false;

    for (TPair<int32, FExecutionAgentState>& KVP : ExecutionStates)
    {
        FExecutionAgentState& State = KVP.Value;

        if (State.PlannedCells.Num() <= 0)
        {
            continue;
        }

        const int32 CurrentIndex = FMath::Clamp(State.ExecutedPlanIndex, 0, State.PlannedCells.Num() - 1);
        const FIntVector CurrentCell = State.PlannedCells[CurrentIndex];

        State.DisplayFromCell = CurrentCell;

        const bool bCanAdvance = (State.ExecutedPlanIndex + 1 < State.PlannedCells.Num());
        const bool bDelay = bCanAdvance && ShouldDelayThisStep(State, CurrentExecutionTimeStep);

        if (bCanAdvance && !bDelay)
        {
            State.ExecutedPlanIndex++;
        }
        else if (bDelay)
        {
            State.TotalDelaySteps++;

            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[ExecutionDelay] t=%d Mission=%d stay at Cell=(%d,%d,%d)"),
                CurrentExecutionTimeStep,
                State.MissionId,
                CurrentCell.X,
                CurrentCell.Y,
                CurrentCell.Z
            );
        }

        const int32 NewIndex = FMath::Clamp(State.ExecutedPlanIndex, 0, State.PlannedCells.Num() - 1);
        const FIntVector NewCell = State.PlannedCells[NewIndex];

        State.DisplayToCell = NewCell;
        State.ActualCells.Add(NewCell);
        State.bFinished = (State.ExecutedPlanIndex >= State.PlannedCells.Num() - 1);

        if (!State.bFinished)
        {
            bAnyActive = true;
        }
    }

    DetectExecutionConflictsAtStep(CurrentExecutionTimeStep);

    if (!bAnyActive)
    {
        bExecutionRunning = false;
        UpdateExecutionVisuals(1.f);

        BuildExecutionSummary();

        if (bLogExecutionSummary)
        {
            LogExecutionSummary();
        }

        return;
    }

    bExecutionRunning = true;
}

int32 APathPlanningDemoActor::ComputeFirstMismatchTime(const FExecutionAgentState& State) const
{
    const int32 MaxSteps = FMath::Max(State.PlannedCells.Num(), State.ActualCells.Num());

    for (int32 T = 0; T < MaxSteps; ++T)
    {
        const FIntVector PlannedCell = GetCellAtTime(State.PlannedCells, T);
        const FIntVector ActualCell = GetCellAtTime(State.ActualCells, T);

        if (PlannedCell != ActualCell)
        {
            return T;
        }
    }

    return -1;
}

void APathPlanningDemoActor::BuildExecutionSummary()
{
    LastExecutionSummary = FExecutionSummary();
    LastExecutionSummary.AgentCount = ExecutionStates.Num();

    for (const TPair<int32, FExecutionAgentState>& KVP : ExecutionStates)
    {
        const FExecutionAgentState& State = KVP.Value;

        FExecutionAgentSummary Item;
        Item.MissionId = State.MissionId;
        Item.PlannedCellCount = State.PlannedCells.Num();
        Item.ActualCellCount = State.ActualCells.Num();
        Item.PlannedMakespan = FMath::Max(0, State.PlannedCells.Num() - 1);
        Item.ActualMakespan = FMath::Max(0, State.ActualCells.Num() - 1);
        Item.TotalDelaySteps = State.TotalDelaySteps;
        Item.FirstMismatchTime = ComputeFirstMismatchTime(State);
        Item.bReachedGoal =
            (State.PlannedCells.Num() > 0) &&
            (State.ActualCells.Num() > 0) &&
            (State.PlannedCells.Last() == State.ActualCells.Last());

        if (Item.bReachedGoal)
        {
            LastExecutionSummary.CompletedAgentCount++;
        }

        LastExecutionSummary.PlannedMakespan =
            FMath::Max(LastExecutionSummary.PlannedMakespan, Item.PlannedMakespan);

        LastExecutionSummary.ActualMakespan =
            FMath::Max(LastExecutionSummary.ActualMakespan, Item.ActualMakespan);

        LastExecutionSummary.TotalDelaySteps += Item.TotalDelaySteps;
        LastExecutionSummary.AgentSummaries.Add(Item);
    }

    for (const FExecutionConflict& Conflict : ExecutionConflicts)
    {
        if (Conflict.bIsEdgeConflict)
        {
            LastExecutionSummary.EdgeConflictCount++;
        }
        else
        {
            LastExecutionSummary.VertexConflictCount++;
        }

        if (LastExecutionSummary.FirstConflictTime < 0 ||
            Conflict.TimeStep < LastExecutionSummary.FirstConflictTime)
        {
            LastExecutionSummary.FirstConflictTime = Conflict.TimeStep;
        }
    }

    LastExecutionSummary.AgentSummaries.Sort(
        [](const FExecutionAgentSummary& A, const FExecutionAgentSummary& B)
        {
            return A.MissionId < B.MissionId;
        });
}

void APathPlanningDemoActor::LogExecutionSummary() const
{
    UE_LOG(LogTemp, Warning, TEXT("============= Execution Summary ============="));
    UE_LOG(LogTemp, Warning, TEXT("DelayMode = %s"),
        *UEnum::GetValueAsString(DelayMode));

    UE_LOG(LogTemp, Warning, TEXT("AgentCount = %d, CompletedAgentCount = %d"),
        LastExecutionSummary.AgentCount,
        LastExecutionSummary.CompletedAgentCount);

    UE_LOG(LogTemp, Warning, TEXT("PlannedMakespan = %d, ActualMakespan = %d, Expansion = %d"),
        LastExecutionSummary.PlannedMakespan,
        LastExecutionSummary.ActualMakespan,
        LastExecutionSummary.ActualMakespan - LastExecutionSummary.PlannedMakespan);

    UE_LOG(LogTemp, Warning, TEXT("TotalDelaySteps = %d"),
        LastExecutionSummary.TotalDelaySteps);

    UE_LOG(LogTemp, Warning, TEXT("VertexConflictCount = %d, EdgeConflictCount = %d, FirstConflictTime = %d"),
        LastExecutionSummary.VertexConflictCount,
        LastExecutionSummary.EdgeConflictCount,
        LastExecutionSummary.FirstConflictTime);

    for (const FExecutionAgentSummary& Item : LastExecutionSummary.AgentSummaries)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("Mission %d | PlannedCells=%d ActualCells=%d | PlannedMakespan=%d ActualMakespan=%d | Delay=%d | FirstMismatch=%d | ReachedGoal=%s"),
            Item.MissionId,
            Item.PlannedCellCount,
            Item.ActualCellCount,
            Item.PlannedMakespan,
            Item.ActualMakespan,
            Item.TotalDelaySteps,
            Item.FirstMismatchTime,
            Item.bReachedGoal ? TEXT("true") : TEXT("false"));
    }

    if (DelayMode == EExecutionDelayMode::PerAgentProbability ||
        DelayMode == EExecutionDelayMode::ScriptedTimesteps)
    {
        UE_LOG(LogTemp, Warning, TEXT("--------- Agent Delay Configs ---------"));

        for (const FAgentDelayConfig& Config : AgentDelayConfigs)
        {
            FString ForcedStepsStr;
            for (int32 Index = 0; Index < Config.ForcedDelaySteps.Num(); ++Index)
            {
                if (Index > 0)
                {
                    ForcedStepsStr += TEXT(",");
                }

                ForcedStepsStr += FString::FromInt(Config.ForcedDelaySteps[Index]);
            }

            UE_LOG(LogTemp, Warning,
                TEXT("Mission %d | DelayProbability=%.3f | ForcedDelaySteps=[%s]"),
                Config.MissionId,
                Config.DelayProbability,
                *ForcedStepsStr);
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("=============================================="));
}

void APathPlanningDemoActor::DetectExecutionConflictsAtStep(int32 TimeStep)
{
    TArray<int32> MissionIds;
    ExecutionStates.GetKeys(MissionIds);

    for (int32 I = 0; I < MissionIds.Num(); ++I)
    {
        const FExecutionAgentState* A = ExecutionStates.Find(MissionIds[I]);
        if (!A)
        {
            continue;
        }

        for (int32 J = I + 1; J < MissionIds.Num(); ++J)
        {
            const FExecutionAgentState* B = ExecutionStates.Find(MissionIds[J]);
            if (!B)
            {
                continue;
            }

            const FIntVector ACell = GetCellAtTime(A->ActualCells, TimeStep);
            const FIntVector BCell = GetCellAtTime(B->ActualCells, TimeStep);

            if (ACell == BCell)
            {
                FExecutionConflict Conflict;
                Conflict.TimeStep = TimeStep;
                Conflict.AgentA = A->MissionId;
                Conflict.AgentB = B->MissionId;
                Conflict.bIsEdgeConflict = false;
                Conflict.Cell = ACell;
                ExecutionConflicts.Add(Conflict);

                UE_LOG(
                    LogTemp,
                    Error,
                    TEXT("[ExecutionConflict][Vertex] t=%d Agent=%d Agent=%d Cell=(%d,%d,%d)"),
                    TimeStep,
                    A->MissionId,
                    B->MissionId,
                    ACell.X,
                    ACell.Y,
                    ACell.Z
                );
            }

            if (TimeStep > 0)
            {
                const FIntVector APrev = GetCellAtTime(A->ActualCells, TimeStep - 1);
                const FIntVector BPrev = GetCellAtTime(B->ActualCells, TimeStep - 1);

                const bool bEdgeConflict =
                    (APrev == BCell) &&
                    (BPrev == ACell) &&
                    (ACell != BCell);

                if (bEdgeConflict)
                {
                    FExecutionConflict Conflict;
                    Conflict.TimeStep = TimeStep;
                    Conflict.AgentA = A->MissionId;
                    Conflict.AgentB = B->MissionId;
                    Conflict.bIsEdgeConflict = true;
                    Conflict.FromA = APrev;
                    Conflict.ToA = ACell;
                    Conflict.FromB = BPrev;
                    Conflict.ToB = BCell;
                    ExecutionConflicts.Add(Conflict);

                    UE_LOG(
                        LogTemp,
                        Error,
                        TEXT("[ExecutionConflict][Edge] t=%d Agent=%d (%d,%d,%d)->(%d,%d,%d), Agent=%d (%d,%d,%d)->(%d,%d,%d)"),
                        TimeStep,
                        A->MissionId,
                        APrev.X, APrev.Y, APrev.Z,
                        ACell.X, ACell.Y, ACell.Z,
                        B->MissionId,
                        BPrev.X, BPrev.Y, BPrev.Z,
                        BCell.X, BCell.Y, BCell.Z
                    );
                }
            }
        }
    }
}

void APathPlanningDemoActor::UpdateExecutionVisuals(float Alpha)
{
    for (TPair<int32, FExecutionAgentState>& KVP : ExecutionStates)
    {
        FExecutionAgentState& State = KVP.Value;

        if (State.Drone)
        {
            const FVector FromWorld = GridMap.CellToWorld(State.DisplayFromCell);
            const FVector ToWorld = GridMap.CellToWorld(State.DisplayToCell);
            const FVector NewLocation = FMath::Lerp(FromWorld, ToWorld, Alpha);

            State.Drone->SetActorLocation(NewLocation);
        }

        DrawExecutionDebugForState(State, CurrentExecutionTimeStep);
    }
}

void APathPlanningDemoActor::DrawExecutionDebugForState(const FExecutionAgentState& State, int32 TimeStep) const
{
    if (!GetWorld())
    {
        return;
    }

    const FIntVector PlannedCell = GetCellAtTime(State.PlannedCells, TimeStep);
    const FIntVector ActualCell = GetCellAtTime(State.ActualCells, TimeStep);

    const FVector PlannedWorld = GridMap.CellToWorld(PlannedCell) + FVector(0.f, 0.f, 20.f);
    const FVector ActualWorld = GridMap.CellToWorld(ActualCell) + FVector(0.f, 0.f, 60.f);
    const FVector Extent(CellSize * 0.30f, CellSize * 0.30f, CellSize * 0.30f);

    if (bDrawExecutionCells)
    {
        DrawDebugBox(GetWorld(), PlannedWorld, Extent, FColor::Blue, false, ExecutionDebugDrawTime, 0, 3.f);
        DrawDebugBox(GetWorld(), ActualWorld, Extent, FColor::Red, false, ExecutionDebugDrawTime, 0, 3.f);

        DrawDebugLine(
            GetWorld(),
            PlannedWorld,
            ActualWorld,
            (PlannedCell == ActualCell) ? FColor::Green : FColor::Yellow,
            false,
            ExecutionDebugDrawTime,
            0,
            2.f
        );
    }

    if (bDrawExecutionText && State.Drone)
    {
        const FString Text = FString::Printf(
            TEXT("M%d  t=%d\nPlanned=(%d,%d,%d)\nActual=(%d,%d,%d)\nDelay=%d"),
            State.MissionId,
            TimeStep,
            PlannedCell.X, PlannedCell.Y, PlannedCell.Z,
            ActualCell.X, ActualCell.Y, ActualCell.Z,
            State.TotalDelaySteps
        );

        DrawDebugString(
            GetWorld(),
            State.Drone->GetActorLocation() + FVector(0.f, 0.f, 140.f),
            Text,
            nullptr,
            (PlannedCell == ActualCell) ? FColor::Green : FColor::Yellow,
            ExecutionDebugDrawTime,
            false
        );
    }
}


// 自动挂载无人机蓝图类
ADroneActor* APathPlanningDemoActor::SpawnDroneForPath(const TArray<FVector>& PathPoints, int32 PairId)
{
    if (!bAutoSpawnDrones)
    {
        UE_LOG(LogTemp, Warning, TEXT("Auto drone spawn disabled. Pair %d skipped spawn"), PairId);
        return nullptr;
    }

    if (!DroneClass)
    {
        UE_LOG(LogTemp, Error, TEXT("DroneClass is null. Cannot spawn drone for pair %d"), PairId);
        return nullptr;
    }

    if (PathPoints.Num() <= 0)
    {
        UE_LOG(LogTemp, Error, TEXT("PathPoints is empty. Cannot spawn drone for pair %d"), PairId);
        return nullptr;
    }

    if (PathPoints.Num() > MaxSpawnableDronePathPoints)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("PathPoints is too long (%d). Skip drone spawn for pair %d to avoid editor stall"),
            PathPoints.Num(),
            PairId);
        return nullptr;
    }

    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("World is null. Cannot spawn drone for pair %d"), PairId);
        return nullptr;
    }

    const FVector SpawnLocation = PathPoints[0];
    const FRotator SpawnRotation = FRotator::ZeroRotator;

    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    ADroneActor* NewDrone = World->SpawnActor<ADroneActor>(
        DroneClass,
        SpawnLocation,
        SpawnRotation,
        SpawnParams
    );

    if (!NewDrone)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to spawn drone for pair %d"), PairId);
        return nullptr;
    }

    // 重复路径点会原地等待完整一个时间步，不再只等一个 Tick。
    // 所有路径段都在同一时长内完成，和 CBS / ECBS 的离散 timestep 语义一致。
    // 非多智能体算法仍然走原来的连续速度模式，不会影响单智能体链路。
    // 只有SAPFA DroneActor 自己才会完美执行路径
    SpawnedDrones.Add(NewDrone);
    SpawnedDroneByMissionId.Add(PairId, NewDrone);

    if (IsMultiAgentPlannerType() && bUseCentralizedExecution)
    {
        NewDrone->StopMove();
        NewDrone->SetActorLocation(PathPoints[0]);
    }
    else
    {
        if (IsMultiAgentPlannerType())
        {
            NewDrone->ConfigureDiscretePlanTiming(true, CBSStepDuration);
        }
        else
        {
            NewDrone->ConfigureDiscretePlanTiming(false, 0.f);
        }

        NewDrone->SetPath(PathPoints);
        NewDrone->StartMove();
    }

    UE_LOG(LogTemp, Warning, TEXT("Spawned drone for pair %d: %s"), PairId, *NewDrone->GetName());

    return NewDrone;
}

// 单智能体路径规划
// UI添加起点终点以及任务序号配置后，直接用这些配置进行路径规划，
// 无需在场景里放置 Start_i / Goal_i Actor
bool APathPlanningDemoActor::ProcessMissionConfigs()
{
    UE_LOG(LogTemp, Warning, TEXT("Using MissionConfigs mode. Mission count = %d"), MissionConfigs.Num());

    bool bAnySuccess = false;

    for (const FDroneMissionConfig& Mission : MissionConfigs)
    {
        const int32 Id = Mission.MissionId;
        const FVector StartWorld = Mission.StartWorld;
        const FVector GoalWorld = Mission.GoalWorld;

        UE_LOG(
            LogTemp,
            Warning,
            TEXT("Mission %d. StartWorld=(%.1f, %.1f, %.1f), GoalWorld=(%.1f, %.1f, %.1f)"),
            Id,
            StartWorld.X, StartWorld.Y, StartWorld.Z,
            GoalWorld.X, GoalWorld.Y, GoalWorld.Z
        );

        if (!InputValidator.ValidateStartGoalPair(
            GridMap,
            StartWorld,
            GoalWorld,
            Id,
            nullptr,
            nullptr))
        {
            UE_LOG(LogTemp, Error, TEXT("Mission %d invalid input. Skip planning."), Id);

            FSingleMissionTimingStats Item;
            Item.MissionId = Id;
            Item.bSuccess = false;
            Item.PathPointCount = 0;
            Item.SolveTimeMs = 0.0;
            LastPlanningStats.MissionStats.Add(Item);
            continue;
        }

        TUniquePtr<IPathPlannerBase> Planner = CreatePlannerByType();
        if (!Planner)
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to create planner for mission %d"), Id);

            FSingleMissionTimingStats Item;
            Item.MissionId = Id;
            Item.bSuccess = false;
            Item.PathPointCount = 0;
            Item.SolveTimeMs = 0.0;
            LastPlanningStats.MissionStats.Add(Item);
            continue;
        }

        TArray<FVector> PathPoints;

        const double SolveStart = FPlatformTime::Seconds();
        const bool bFound = Planner->Plan(GridMap, StartWorld, GoalWorld, PathPoints);
        const double SolveMs = (FPlatformTime::Seconds() - SolveStart) * 1000.0;

        LastPlanningStats.SolveTimeMs += SolveMs;

        FSingleMissionTimingStats Item;
        Item.MissionId = Id;
        Item.bSuccess = bFound;
        Item.PathPointCount = PathPoints.Num();
        Item.SolveTimeMs = SolveMs;
        LastPlanningStats.MissionStats.Add(Item);

        if (!bFound)
        {
            UE_LOG(LogTemp, Error, TEXT("Mission %d: path not found"), Id);
            continue;
        }

        bAnySuccess = true;
        CachePlannedPath(Id, PathPoints);

        const double PostStart = FPlatformTime::Seconds();

        UE_LOG(LogTemp, Warning, TEXT("Mission %d: path found, points=%d"), Id, PathPoints.Num());
        LogPathCoordinates(PathPoints, Id, TEXT("Mission"));
        DrawPathDebug(PathPoints, GetDebugColorById(Id));
        SpawnDroneForPath(PathPoints, Id);

        LastPlanningStats.PostProcessTimeMs +=
            (FPlatformTime::Seconds() - PostStart) * 1000.0;
    }

    return bAnySuccess;
}

// 把场景里收集到的 Start_i / Goal_i 配对转换成 FDroneMissionConfig 数组，
// 做输入校验，然后统一调用多智能体规划器。
bool APathPlanningDemoActor::ProcessStartGoalPairsMultiAgent()
{
    TArray<int32> Ids;
    TMap<int32, TObjectPtr<AActor>> Starts;
    TMap<int32, TObjectPtr<AActor>> Goals;

    CollectStartGoalPairs(Ids, Starts, Goals);

    UE_LOG(LogTemp, Warning, TEXT("Using %s start/goal pair mode. Pair count = %d"),
        *GetPlannerTypeName(),
        Ids.Num());

    TArray<FDroneMissionConfig> Missions;
    Missions.Reserve(Ids.Num());

    for (const int32 Id : Ids)
    {
        AActor* StartActor = Starts.FindRef(Id);
        AActor* GoalActor = Goals.FindRef(Id);

        if (!StartActor || !GoalActor)
        {
            UE_LOG(LogTemp, Error, TEXT("Pair %d invalid actor reference"), Id);
            continue;
        }

        const FVector StartWorld = StartActor->GetActorLocation();
        const FVector GoalWorld = GoalActor->GetActorLocation();

        if (!InputValidator.ValidateStartGoalPair(
            GridMap,
            StartWorld,
            GoalWorld,
            Id,
            StartActor,
            GoalActor))
        {
            UE_LOG(LogTemp, Error, TEXT("Pair %d invalid input. Skip %s planning."), Id, *GetPlannerTypeName());
            continue;
        }

        FDroneMissionConfig Mission;
        Mission.MissionId = Id;
        Mission.StartWorld = StartWorld;
        Mission.GoalWorld = GoalWorld;
        Missions.Add(Mission);
    }

    if (Missions.Num() <= 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("%s start/goal pair mode has no valid missions to plan"), *GetPlannerTypeName());
        return false;
    }

    TMap<int32, TArray<FVector>> OutPaths;

    const double SolveStart = FPlatformTime::Seconds();
    const bool bSuccess = PlanMultiAgentMissions(Missions, OutPaths);
    LastPlanningStats.SolveTimeMs += (FPlatformTime::Seconds() - SolveStart) * 1000.0;

    for (const FDroneMissionConfig& Mission : Missions)
    {
        const TArray<FVector>* PathPoints = OutPaths.Find(Mission.MissionId);

        FSingleMissionTimingStats Item;
        Item.MissionId = Mission.MissionId;
        Item.bSuccess = (PathPoints != nullptr && PathPoints->Num() > 0);
        Item.PathPointCount = PathPoints ? PathPoints->Num() : 0;
        Item.SolveTimeMs = 0.0;
        LastPlanningStats.MissionStats.Add(Item);
    }

    if (!bSuccess)
    {
        UE_LOG(LogTemp, Warning, TEXT("%s failed or conflict unresolved in start/goal pair mode."), *GetPlannerTypeName());
        return false;
    }

    bool bAnyPath = false;

    for (const FDroneMissionConfig& Mission : Missions)
    {
        const TArray<FVector>* PathPoints = OutPaths.Find(Mission.MissionId);
        if (!PathPoints || PathPoints->Num() <= 0)
        {
            UE_LOG(LogTemp, Warning, TEXT("%s returned no path for pair %d"),
                *GetPlannerTypeName(),
                Mission.MissionId);
            continue;
        }

        bAnyPath = true;
        CachePlannedPath(Mission.MissionId, *PathPoints);

        const double PostStart = FPlatformTime::Seconds();

        UE_LOG(LogTemp, Warning, TEXT("%s pair %d: path found, points=%d"),
            *GetPlannerTypeName(),
            Mission.MissionId,
            PathPoints->Num());

        LogPathCoordinates(*PathPoints, Mission.MissionId, *GetPlannerTypeName());
        DrawPathDebug(*PathPoints, GetDebugColorById(Mission.MissionId));
        SpawnDroneForPath(*PathPoints, Mission.MissionId);

        LastPlanningStats.PostProcessTimeMs +=
            (FPlatformTime::Seconds() - PostStart) * 1000.0;
    }

    return bAnyPath;
}

bool APathPlanningDemoActor::ProcessMissionConfigsMultiAgent()
{
    UE_LOG(LogTemp, Warning, TEXT("Using %s multi-agent mode. Mission count = %d"),
        *GetPlannerTypeName(),
        MissionConfigs.Num());

    TMap<int32, TArray<FVector>> OutPaths;

    const double SolveStart = FPlatformTime::Seconds();
    const bool bSuccess = PlanMultiAgentMissions(MissionConfigs, OutPaths);
    LastPlanningStats.SolveTimeMs += (FPlatformTime::Seconds() - SolveStart) * 1000.0;

    for (const FDroneMissionConfig& Mission : MissionConfigs)
    {
        const TArray<FVector>* PathPoints = OutPaths.Find(Mission.MissionId);

        FSingleMissionTimingStats Item;
        Item.MissionId = Mission.MissionId;
        Item.bSuccess = (PathPoints != nullptr && PathPoints->Num() > 0);
        Item.PathPointCount = PathPoints ? PathPoints->Num() : 0;
        Item.SolveTimeMs = 0.0;
        LastPlanningStats.MissionStats.Add(Item);
    }

    if (!bSuccess)
    {
        UE_LOG(LogTemp, Warning, TEXT("%s failed or conflict unresolved in current stage."), *GetPlannerTypeName());
        return false;
    }

    bool bAnyPath = false;

    for (const FDroneMissionConfig& Mission : MissionConfigs)
    {
        const TArray<FVector>* PathPoints = OutPaths.Find(Mission.MissionId);
        if (!PathPoints || PathPoints->Num() <= 0)
        {
            UE_LOG(LogTemp, Warning, TEXT("%s returned no path for Mission %d"),
                *GetPlannerTypeName(),
                Mission.MissionId);
            continue;
        }

        bAnyPath = true;
        CachePlannedPath(Mission.MissionId, *PathPoints);

        const double PostStart = FPlatformTime::Seconds();

        UE_LOG(LogTemp, Warning, TEXT("%s Mission %d: path found, points=%d"),
            *GetPlannerTypeName(),
            Mission.MissionId,
            PathPoints->Num());

        LogPathCoordinates(*PathPoints, Mission.MissionId, *GetPlannerTypeName());
        DrawPathDebug(*PathPoints, GetDebugColorById(Mission.MissionId));
        SpawnDroneForPath(*PathPoints, Mission.MissionId);

        LastPlanningStats.PostProcessTimeMs +=
            (FPlatformTime::Seconds() - PostStart) * 1000.0;
    }

    return bAnyPath;
}

bool APathPlanningDemoActor::PlanMultiAgentMissions(
    const TArray<FDroneMissionConfig>& Missions,
    TMap<int32, TArray<FVector>>& OutPaths) const
{
    if (PlannerType == EPlannerType::CBS)
    {
        FCBSPlanner Planner;
        return Planner.PlanMissions(GridMap, Missions, OutPaths);
    }

    if (PlannerType == EPlannerType::ECBS)
    {
        FECBSPlanner Planner(ECBSSuboptimalityBound);
        return Planner.PlanMissions(GridMap, Missions, OutPaths);
    }

    if (PlannerType == EPlannerType::PBS)
    {
        FPBSPlanner Planner;
        return Planner.PlanMissions(GridMap, Missions, OutPaths);
    }

    if (PlannerType == EPlannerType::LaCAM)
    {
        FLaCAMPlanner Planner(LaCAMTimeLimitMs, LaCAMRandomSeed, bLaCAMAnytime, LaCAMVerboseLevel);
        return Planner.PlanMissions(GridMap, Missions, OutPaths);
    }

    if (PlannerType == EPlannerType::LaCAMUTM)
    {
        FLaCAMUTMPlanner Planner(LaCAMTimeLimitMs, LaCAMRandomSeed, bLaCAMAnytime, LaCAMVerboseLevel, NoFlyZoneConfigs);
        return Planner.PlanMissions(GridMap, Missions, OutPaths);
    }

    UE_LOG(LogTemp, Error, TEXT("PlannerType=%d is not a multi-agent planner"), (int32)PlannerType);
    return false;
}

bool APathPlanningDemoActor::IsMultiAgentPlannerType() const
{
    return PlannerType == EPlannerType::CBS
        || PlannerType == EPlannerType::ECBS
        || PlannerType == EPlannerType::PBS
        || PlannerType == EPlannerType::LaCAM
        || PlannerType == EPlannerType::LaCAMUTM;
}

// 根据 PlannerType 创建对应的路径规划器实例
TUniquePtr<IPathPlannerBase> APathPlanningDemoActor::CreatePlannerByType() const
{
    switch (PlannerType)
    {
    case EPlannerType::AStar:
        UE_LOG(LogTemp, Warning, TEXT("Planner created: FAStarPlanner"));
        return MakeUnique<FAStarPlanner>();

    case EPlannerType::SIPP:
        UE_LOG(LogTemp, Warning, TEXT("Planner created: FSIPPPlanner"));
        return MakeUnique<FSIPPPlanner>(NoFlyZoneConfigs);

    case EPlannerType::DStarLite:
        UE_LOG(LogTemp, Warning, TEXT("Planner created: FDStarLitePlanner"));
        return MakeUnique<FDStarLitePlanner>();

    case EPlannerType::JPS:
        UE_LOG(LogTemp, Warning, TEXT("JPS planner not implemented yet. Fallback to A*"));
        UE_LOG(LogTemp, Warning, TEXT("Planner created: FAStarPlanner"));
        return MakeUnique<FAStarPlanner>();

    case EPlannerType::PBS:
        UE_LOG(LogTemp, Warning, TEXT("PBS is a multi-agent planner and is not created by CreatePlannerByType()"));
        return nullptr;

    case EPlannerType::CBS:
    case EPlannerType::ECBS:
    case EPlannerType::LaCAM:
    case EPlannerType::LaCAMUTM:
        UE_LOG(LogTemp, Warning, TEXT("%s is a multi-agent planner and is not created by CreatePlannerByType()"), *GetPlannerTypeName());
        return nullptr;

    default:
        UE_LOG(LogTemp, Warning, TEXT("Unknown planner type. Fallback to A*"));
        UE_LOG(LogTemp, Warning, TEXT("Planner created: FAStarPlanner"));
        return MakeUnique<FAStarPlanner>();
    }
}

// 获取当前选择的路径规划算法名称，供UI显示
FString APathPlanningDemoActor::GetPlannerTypeName() const
{
    switch (PlannerType)
    {
    case EPlannerType::AStar:
        return TEXT("A*");
    case EPlannerType::SIPP:
        return TEXT("SIPP");
    case EPlannerType::DStarLite:
        return TEXT("D* Lite");
    case EPlannerType::CBS:
        return TEXT("CBS");
    case EPlannerType::ECBS:
        return TEXT("ECBS");
    case EPlannerType::PBS:
        return TEXT("PBS");
    case EPlannerType::LaCAM:
        return TEXT("LaCAM");
    case EPlannerType::LaCAMUTM:
        return TEXT("LaCAM-UTM");
    default:
        return TEXT("Unknown");
    }
}



// UI界面添加起止点、导入表格数据
void APathPlanningDemoActor::GetMissionMarkerActors(TArray<AMissionMarkerActor*>& OutMarkers) const
{
    OutMarkers.Reset();

    if (!GetWorld())
    {
        return;
    }

    for (TActorIterator<AMissionMarkerActor> It(GetWorld()); It; ++It)
    {
        AMissionMarkerActor* Marker = *It;
        if (!Marker)
        {
            continue;
        }

        if (Marker->Tags.Contains(FName(TEXT("MissionMarker"))))
        {
            OutMarkers.Add(Marker);
        }
    }
}

void APathPlanningDemoActor::EditorBuildGridForMissionEditing()
{
    TArray<AActor*> IgnoreActors;
    IgnoreActors.Add(this);

    TArray<AMissionMarkerActor*> ExistingMarkers;
    GetMissionMarkerActors(ExistingMarkers);

    for (AMissionMarkerActor* Marker : ExistingMarkers)
    {
        if (Marker)
        {
            IgnoreActors.Add(Marker);
        }
    }

    GridMap.GridOrigin = GridOrigin;
    GridMap.GridDim = GridDim;
    GridMap.CellSize = CellSize;

    GridMap.BuildOccupancyGrid(
        GetWorld(),
        IgnoreActors,
        bDrawOccupiedCells,
        bDrawFreeCells,
        DebugDrawTime
    );

    UE_LOG(LogTemp, Warning, TEXT("EditorBuildGridForMissionEditing done"));
}

bool APathPlanningDemoActor::TryGenerateSingleMission(
    FRandomStream& RandomStream,
    int32 MissionId,
    TSet<FIntVector>& UsedStarts,
    TSet<FIntVector>& UsedGoals,
    FDroneMissionConfig& OutMission) const
{
    const int32 MaxTryCount = 500;

    for (int32 TryIndex = 0; TryIndex < MaxTryCount; ++TryIndex)
    {
        const FIntVector StartCell(
            RandomStream.RandRange(0, GridDim.X - 1),
            RandomStream.RandRange(0, GridDim.Y - 1),
            RandomStream.RandRange(0, GridDim.Z - 1)
        );

        if (GridMap.IsBlocked(StartCell.X, StartCell.Y, StartCell.Z))
        {
            continue;
        }

        if (!bAllowDuplicateStartCells && UsedStarts.Contains(StartCell))
        {
            continue;
        }

        for (int32 GoalTry = 0; GoalTry < MaxTryCount; ++GoalTry)
        {
            const FIntVector GoalCell(
                RandomStream.RandRange(0, GridDim.X - 1),
                RandomStream.RandRange(0, GridDim.Y - 1),
                RandomStream.RandRange(0, GridDim.Z - 1)
            );

            if (GridMap.IsBlocked(GoalCell.X, GoalCell.Y, GoalCell.Z))
            {
                continue;
            }

            if (!bAllowDuplicateGoalCells && UsedGoals.Contains(GoalCell))
            {
                continue;
            }

            const int32 ManhattanDist =
                FMath::Abs(StartCell.X - GoalCell.X) +
                FMath::Abs(StartCell.Y - GoalCell.Y) +
                FMath::Abs(StartCell.Z - GoalCell.Z);

            if (ManhattanDist < MinStartGoalCellDistance)
            {
                continue;
            }

            OutMission.MissionId = MissionId;
            OutMission.StartWorld = GridMap.CellToWorld(StartCell);
            OutMission.GoalWorld = GridMap.CellToWorld(GoalCell);

            UsedStarts.Add(StartCell);
            UsedGoals.Add(GoalCell);
            return true;
        }
    }

    return false;
}

void APathPlanningDemoActor::EditorGenerateRandomMissionConfigs()
{
    EditorBuildGridForMissionEditing();

    MissionConfigs.Reset();

    FRandomStream RandomStream(RandomSeed);
    TSet<FIntVector> UsedStarts;
    TSet<FIntVector> UsedGoals;

    for (int32 i = 0; i < RandomMissionCount; ++i)
    {
        FDroneMissionConfig NewMission;
        const int32 MissionId = i + 1;

        if (!TryGenerateSingleMission(RandomStream, MissionId, UsedStarts, UsedGoals, NewMission))
        {
            UE_LOG(LogTemp, Warning, TEXT("Failed to generate mission %d"), MissionId);
            continue;
        }

        MissionConfigs.Add(NewMission);
    }

    UE_LOG(LogTemp, Warning, TEXT("Generated random missions: %d"), MissionConfigs.Num());
}

void APathPlanningDemoActor::EditorClearMissionMarkers()
{
    TArray<AMissionMarkerActor*> Markers;
    GetMissionMarkerActors(Markers);

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

    UE_LOG(LogTemp, Warning, TEXT("EditorClearMissionMarkers deleted %d markers"), DeleteCount);
}

void APathPlanningDemoActor::EditorSpawnMissionMarkers()
{
    if (!MissionMarkerClass)
    {
        UE_LOG(LogTemp, Error, TEXT("MissionMarkerClass is null"));
        return;
    }

    EditorClearMissionMarkers();

    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    for (const FDroneMissionConfig& Mission : MissionConfigs)
    {
        const FIntVector StartCell = GridMap.WorldToCell(Mission.StartWorld);
        const FIntVector GoalCell = GridMap.WorldToCell(Mission.GoalWorld);

        {
            const FVector SpawnLocation = Mission.StartWorld + FVector(0.f, 0.f, MarkerZOffset);
            AMissionMarkerActor* StartMarker = World->SpawnActor<AMissionMarkerActor>(
                MissionMarkerClass,
                SpawnLocation,
                FRotator::ZeroRotator,
                SpawnParams
            );

            if (StartMarker)
            {
                StartMarker->MissionId = Mission.MissionId;
                StartMarker->MarkerType = EMissionMarkerType::Start;
                StartMarker->Cell = StartCell;
                StartMarker->Tags.Add(FName(TEXT("MissionMarker")));
                StartMarker->UpdateVisual();
            }
        }

        {
            const FVector SpawnLocation = Mission.GoalWorld + FVector(0.f, 0.f, MarkerZOffset);
            AMissionMarkerActor* GoalMarker = World->SpawnActor<AMissionMarkerActor>(
                MissionMarkerClass,
                SpawnLocation,
                FRotator::ZeroRotator,
                SpawnParams
            );

            if (GoalMarker)
            {
                GoalMarker->MissionId = Mission.MissionId;
                GoalMarker->MarkerType = EMissionMarkerType::Goal;
                GoalMarker->Cell = GoalCell;
                GoalMarker->Tags.Add(FName(TEXT("MissionMarker")));
                GoalMarker->UpdateVisual();
            }
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("EditorSpawnMissionMarkers done. Mission count=%d"), MissionConfigs.Num());
}

void APathPlanningDemoActor::EditorValidateMissionConfigs()
{
    EditorBuildGridForMissionEditing();

    UE_LOG(LogTemp, Warning, TEXT("========== Validate MissionConfigs begin. Count=%d  =========="), MissionConfigs.Num());

    TSet<int32> MissionIds;
    TSet<FIntVector> StartCells;
    TSet<FIntVector> GoalCells;

    for (const FDroneMissionConfig& Mission : MissionConfigs)
    {
        if (MissionIds.Contains(Mission.MissionId))
        {
            UE_LOG(LogTemp, Error, TEXT("Duplicate MissionId=%d"), Mission.MissionId);
        }
        MissionIds.Add(Mission.MissionId);

        const bool bValid = InputValidator.ValidateStartGoalPair(
            GridMap,
            Mission.StartWorld,
            Mission.GoalWorld,
            Mission.MissionId,
            nullptr,
            nullptr
        );

        const FIntVector StartCell = GridMap.WorldToCell(Mission.StartWorld);
        const FIntVector GoalCell = GridMap.WorldToCell(Mission.GoalWorld);

        if (StartCells.Contains(StartCell))
        {
            UE_LOG(LogTemp, Warning, TEXT("Duplicate StartCell in Mission %d: (%d,%d,%d)"),
                Mission.MissionId, StartCell.X, StartCell.Y, StartCell.Z);
        }
        StartCells.Add(StartCell);

        if (GoalCells.Contains(GoalCell))
        {
            UE_LOG(LogTemp, Warning, TEXT("Duplicate GoalCell in Mission %d: (%d,%d,%d)"),
                Mission.MissionId, GoalCell.X, GoalCell.Y, GoalCell.Z);
        }
        GoalCells.Add(GoalCell);

        if (bValid)
        {
            UE_LOG(LogTemp, Warning, TEXT("Mission %d valid"), Mission.MissionId);
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("========== Validate MissionConfigs end  =========="));
}

void APathPlanningDemoActor::EditorReadMissionMarkersToConfigs()
{
    TArray<AMissionMarkerActor*> Markers;
    GetMissionMarkerActors(Markers);

    TMap<int32, FVector> StartMap;
    TMap<int32, FVector> GoalMap;

    for (AMissionMarkerActor* Marker : Markers)
    {
        if (!Marker)
        {
            continue;
        }

        const FVector MarkerWorld = Marker->GetActorLocation() - FVector(0.f, 0.f, MarkerZOffset);
        const FIntVector Cell = GridMap.WorldToCell(MarkerWorld);
        const FVector SnappedWorld = GridMap.CellToWorld(Cell);

        if (Marker->MarkerType == EMissionMarkerType::Start)
        {
            StartMap.Add(Marker->MissionId, SnappedWorld);
        }
        else
        {
            GoalMap.Add(Marker->MissionId, SnappedWorld);
        }
    }

    MissionConfigs.Reset();

    TArray<int32> MissionIds;
    StartMap.GetKeys(MissionIds);

    for (int32 MissionId : MissionIds)
    {
        if (!GoalMap.Contains(MissionId))
        {
            UE_LOG(LogTemp, Warning, TEXT("Mission %d has start marker but no goal marker"), MissionId);
            continue;
        }

        FDroneMissionConfig Mission;
        Mission.MissionId = MissionId;
        Mission.StartWorld = StartMap[MissionId];
        Mission.GoalWorld = GoalMap[MissionId];

        MissionConfigs.Add(Mission);
    }

    MissionConfigs.Sort([](const FDroneMissionConfig& A, const FDroneMissionConfig& B)
        {
            return A.MissionId < B.MissionId;
        });

    UE_LOG(LogTemp, Warning, TEXT("EditorReadMissionMarkersToConfigs done. Mission count=%d"), MissionConfigs.Num());
}


//负责删除所有由城市生成器生成的建筑 Actor
void APathPlanningDemoActor::EditorClearCityEnvironment()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorClearCityEnvironment: World is null"));
        return;
    }

    int32 DeleteCount = 0;

    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (!Actor)
        {
            continue;
        }

        if (Actor->Tags.Contains(FName(TEXT("CityBuilding"))))
        {
            Actor->Destroy();
            DeleteCount++;
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("EditorClearCityEnvironment deleted %d actors"), DeleteCount);
}

// 负责生成一个建筑块
void APathPlanningDemoActor::SpawnCityBuilding(const FVector& Center, const FVector& Extent)
{
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("SpawnCityBuilding: World is null"));
        return;
    }

    AStaticMeshActor* Building = World->SpawnActor<AStaticMeshActor>(
        AStaticMeshActor::StaticClass(),
        Center,
        FRotator::ZeroRotator
    );

    if (!Building)
    {
        UE_LOG(LogTemp, Error, TEXT("SpawnCityBuilding: Failed to spawn building actor"));
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

    static UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    if (!CubeMesh)
    {
        UE_LOG(LogTemp, Error, TEXT("SpawnCityBuilding: Failed to load Cube mesh"));
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
        Extent.Z / 50.f
    ));
}

// ManhattanCity生成器在指定区域内生成一个个建筑块，形成一个简单的城市环境
void APathPlanningDemoActor::GenerateCityLayout_Manhattan(FRandomStream& RandomStream)
{
    const float Pitch = CityBlockSize + CityRoadWidth;

    const float WidthMin = FMath::Min(BuildingWidthMin, BuildingWidthMax);
    const float WidthMax = FMath::Max(BuildingWidthMin, BuildingWidthMax);

    const float DepthMin = FMath::Min(BuildingDepthMin, BuildingDepthMax);
    const float DepthMax = FMath::Max(BuildingDepthMin, BuildingDepthMax);

    const float HeightMin = FMath::Max(FMath::Min(BuildingHeightMin, BuildingHeightMax), 600.f);
    const float HeightMax = FMath::Max(FMath::Max(BuildingHeightMin, BuildingHeightMax), 900.f);

    int32 SpawnedBuildingCount = 0;

    for (int32 ix = 0; ix < CityBlocksX; ++ix)
    {
        for (int32 iy = 0; iy < CityBlocksY; ++iy)
        {
            const FVector BlockCenter(
                GridOrigin.X + ix * Pitch,
                GridOrigin.Y + iy * Pitch,
                0.f
            );

            // 曼哈顿街区：每个 block 里 1~4 个中高层建筑
            const int32 BuildingCount = RandomStream.RandRange(1, 4);

            for (int32 k = 0; k < BuildingCount; ++k)
            {
                const float Width = RandomStream.FRandRange(WidthMin, WidthMax);
                const float Depth = RandomStream.FRandRange(DepthMin, DepthMax);
                const float Height = RandomStream.FRandRange(HeightMin, HeightMax);

                // 留一点边距，避免建筑冲出 block
                const float Margin = 40.f;

                const float OffsetXLimit = FMath::Max(0.f, CityBlockSize * 0.5f - Width * 0.5f - Margin);
                const float OffsetYLimit = FMath::Max(0.f, CityBlockSize * 0.5f - Depth * 0.5f - Margin);

                const FVector BuildingCenter(
                    BlockCenter.X + RandomStream.FRandRange(-OffsetXLimit, OffsetXLimit),
                    BlockCenter.Y + RandomStream.FRandRange(-OffsetYLimit, OffsetYLimit),
                    Height * 0.5f
                );

                const FVector BuildingExtent(
                    Width * 0.5f,
                    Depth * 0.5f,
                    Height * 0.5f
                );

                SpawnCityBuilding(BuildingCenter, BuildingExtent);
                SpawnedBuildingCount++;
            }
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("GenerateCityLayout_Manhattan done. Buildings=%d"), SpawnedBuildingCount);
}



void APathPlanningDemoActor::EditorGenerateManhattanCity()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorGenerateManhattanCity: World is null"));
        return;
    }

    if (CityBlocksX <= 0 || CityBlocksY <= 0)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorGenerateManhattanCity: Invalid block count"));
        return;
    }

    if (CityBlockSize <= 0.f)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorGenerateManhattanCity: Invalid CityBlockSize"));
        return;
    }

    EditorClearCityEnvironment();

    FRandomStream RandomStream(CitySeed);
    GenerateCityLayout_Manhattan(RandomStream);

    if (bAutoRebuildGridAfterCityGenerate)
    {
        EditorBuildGridForMissionEditing();
    }

    UE_LOG(LogTemp, Warning, TEXT("EditorGenerateManhattanCity done"));
}


/*
住宅片区：GenerateCityLayout_Residential
特征:住宅区应该体现：建筑较矮建筑较小建筑更多但更分散,道路和空地相对宽松,整体更开阔
*/
void APathPlanningDemoActor::GenerateCityLayout_Residential(FRandomStream& RandomStream)
{
    const float Pitch = CityBlockSize + CityRoadWidth;

    const float WidthMin = FMath::Min(BuildingWidthMin, BuildingWidthMax);
    const float WidthMax = FMath::Max(BuildingWidthMin, BuildingWidthMax) * 0.8f;

    const float DepthMin = FMath::Min(BuildingDepthMin, BuildingDepthMax);
    const float DepthMax = FMath::Max(BuildingDepthMin, BuildingDepthMax) * 0.8f;

    const float HeightMin = FMath::Min(BuildingHeightMin, BuildingHeightMax) * 0.4f;
    const float HeightMax = FMath::Max(BuildingHeightMin, BuildingHeightMax) * 0.75f;

    for (int32 ix = 0; ix < CityBlocksX; ++ix)
    {
        for (int32 iy = 0; iy < CityBlocksY; ++iy)
        {
            const FVector BlockCenter(
                GridOrigin.X + ix * Pitch,
                GridOrigin.Y + iy * Pitch,
                0.f
            );

            const int32 BuildingCount = RandomStream.RandRange(2, 5);

            for (int32 k = 0; k < BuildingCount; ++k)
            {
                const float Width = RandomStream.FRandRange(WidthMin, WidthMax);
                const float Depth = RandomStream.FRandRange(DepthMin, DepthMax);
                const float Height = RandomStream.FRandRange(HeightMin, HeightMax);

                const float Margin = 70.f;

                const float OffsetXLimit = FMath::Max(0.f, CityBlockSize * 0.5f - Width * 0.5f - Margin);
                const float OffsetYLimit = FMath::Max(0.f, CityBlockSize * 0.5f - Depth * 0.5f - Margin);

                const FVector BuildingCenter(
                    BlockCenter.X + RandomStream.FRandRange(-OffsetXLimit, OffsetXLimit),
                    BlockCenter.Y + RandomStream.FRandRange(-OffsetYLimit, OffsetYLimit),
                    Height * 0.5f
                );

                const FVector BuildingExtent(
                    Width * 0.5f,
                    Depth * 0.5f,
                    Height * 0.5f
                );

                SpawnCityBuilding(BuildingCenter, BuildingExtent);
            }
        }
    }
}

void APathPlanningDemoActor::EditorGenerateResidentialDistrict()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorGenerateResidentialDistrict: World is null"));
        return;
    }

    if (CityBlocksX <= 0 || CityBlocksY <= 0)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorGenerateResidentialDistrict: Invalid block count"));
        return;
    }

    EditorClearCityEnvironment();

    FRandomStream RandomStream(CitySeed);
    GenerateCityLayout_Residential(RandomStream);

    if (bAutoRebuildGridAfterCityGenerate)
    {
        EditorBuildGridForMissionEditing();
    }

    UE_LOG(LogTemp, Warning, TEXT("EditorGenerateResidentialDistrict done"));
}

/*
工业园区：GenerateCityLayout_Industrial
特征:工业区应该体现,建筑数量少,单体建筑大高度中等,留大面积开阔区,更适合测试大尺度绕行
*/
void APathPlanningDemoActor::GenerateCityLayout_Industrial(FRandomStream& RandomStream)
{
    const float Pitch = CityBlockSize + CityRoadWidth;

    const float WidthMin = FMath::Min(BuildingWidthMin, BuildingWidthMax) * 1.4f;
    const float WidthMax = FMath::Max(BuildingWidthMin, BuildingWidthMax) * 2.0f;

    const float DepthMin = FMath::Min(BuildingDepthMin, BuildingDepthMax) * 1.4f;
    const float DepthMax = FMath::Max(BuildingDepthMin, BuildingDepthMax) * 2.0f;

    const float HeightMin = FMath::Min(BuildingHeightMin, BuildingHeightMax) * 0.7f;
    const float HeightMax = FMath::Max(BuildingHeightMin, BuildingHeightMax) * 1.0f;

    for (int32 ix = 0; ix < CityBlocksX; ++ix)
    {
        for (int32 iy = 0; iy < CityBlocksY; ++iy)
        {
            const FVector BlockCenter(
                GridOrigin.X + ix * Pitch,
                GridOrigin.Y + iy * Pitch,
                0.f
            );

            const int32 BuildingCount = RandomStream.RandRange(1, 2);

            for (int32 k = 0; k < BuildingCount; ++k)
            {
                const float Width = RandomStream.FRandRange(WidthMin, WidthMax);
                const float Depth = RandomStream.FRandRange(DepthMin, DepthMax);
                const float Height = RandomStream.FRandRange(HeightMin, HeightMax);

                const float Margin = 50.f;

                const float OffsetXLimit = FMath::Max(0.f, CityBlockSize * 0.5f - Width * 0.5f - Margin);
                const float OffsetYLimit = FMath::Max(0.f, CityBlockSize * 0.5f - Depth * 0.5f - Margin);

                const FVector BuildingCenter(
                    BlockCenter.X + RandomStream.FRandRange(-OffsetXLimit, OffsetXLimit),
                    BlockCenter.Y + RandomStream.FRandRange(-OffsetYLimit, OffsetYLimit),
                    Height * 0.5f
                );

                const FVector BuildingExtent(
                    Width * 0.5f,
                    Depth * 0.5f,
                    Height * 0.5f
                );

                SpawnCityBuilding(BuildingCenter, BuildingExtent);
            }
        }
    }
}

void APathPlanningDemoActor::EditorGenerateIndustrialPark()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorGenerateIndustrialPark: World is null"));
        return;
    }

    if (CityBlocksX <= 0 || CityBlocksY <= 0)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorGenerateIndustrialPark: Invalid block count"));
        return;
    }

    EditorClearCityEnvironment();

    FRandomStream RandomStream(CitySeed);
    GenerateCityLayout_Industrial(RandomStream);

    if (bAutoRebuildGridAfterCityGenerate)
    {
        EditorBuildGridForMissionEditing();
    }

    UE_LOG(LogTemp, Warning, TEXT("EditorGenerateIndustrialPark done"));
}

/*
混合城区：GenerateCityLayout_Mixed
特征:一部分高楼密集,一部分低矮住宅,一部分大体量工业建筑,整体异质性高
*/

void APathPlanningDemoActor::GenerateCityLayout_Mixed(FRandomStream& RandomStream)
{
    const float Pitch = CityBlockSize + CityRoadWidth;

    for (int32 ix = 0; ix < CityBlocksX; ++ix)
    {
        for (int32 iy = 0; iy < CityBlocksY; ++iy)
        {
            const FVector BlockCenter(
                GridOrigin.X + ix * Pitch,
                GridOrigin.Y + iy * Pitch,
                0.f
            );

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
                // 曼哈顿风格
                BuildingCount = RandomStream.RandRange(1, 4);
                WidthMin = BuildingWidthMin;
                WidthMax = BuildingWidthMax;
                DepthMin = BuildingDepthMin;
                DepthMax = BuildingDepthMax;
                HeightMin = FMath::Max(FMath::Min(BuildingHeightMin, BuildingHeightMax), 600.f);
                HeightMax = FMath::Max(FMath::Max(BuildingHeightMin, BuildingHeightMax), 900.f);
                Margin = 40.f;
            }
            else if (DistrictType == 1)
            {
                // 住宅风格
                BuildingCount = RandomStream.RandRange(2, 5);
                WidthMin = FMath::Min(BuildingWidthMin, BuildingWidthMax);
                WidthMax = FMath::Max(BuildingWidthMin, BuildingWidthMax) * 0.8f;
                DepthMin = FMath::Min(BuildingDepthMin, BuildingDepthMax);
                DepthMax = FMath::Max(BuildingDepthMin, BuildingDepthMax) * 0.8f;
                HeightMin = FMath::Min(BuildingHeightMin, BuildingHeightMax) * 0.4f;
                HeightMax = FMath::Max(BuildingHeightMin, BuildingHeightMax) * 0.75f;
                Margin = 70.f;
            }
            else
            {
                // 工业风格
                BuildingCount = RandomStream.RandRange(1, 2);
                WidthMin = FMath::Min(BuildingWidthMin, BuildingWidthMax) * 1.4f;
                WidthMax = FMath::Max(BuildingWidthMin, BuildingWidthMax) * 2.0f;
                DepthMin = FMath::Min(BuildingDepthMin, BuildingDepthMax) * 1.4f;
                DepthMax = FMath::Max(BuildingDepthMin, BuildingDepthMax) * 2.0f;
                HeightMin = FMath::Min(BuildingHeightMin, BuildingHeightMax) * 0.7f;
                HeightMax = FMath::Max(BuildingHeightMin, BuildingHeightMax) * 1.0f;
                Margin = 50.f;
            }

            for (int32 k = 0; k < BuildingCount; ++k)
            {
                const float Width = RandomStream.FRandRange(WidthMin, WidthMax);
                const float Depth = RandomStream.FRandRange(DepthMin, DepthMax);
                const float Height = RandomStream.FRandRange(HeightMin, HeightMax);

                const float OffsetXLimit = FMath::Max(0.f, CityBlockSize * 0.5f - Width * 0.5f - Margin);
                const float OffsetYLimit = FMath::Max(0.f, CityBlockSize * 0.5f - Depth * 0.5f - Margin);

                const FVector BuildingCenter(
                    BlockCenter.X + RandomStream.FRandRange(-OffsetXLimit, OffsetXLimit),
                    BlockCenter.Y + RandomStream.FRandRange(-OffsetYLimit, OffsetYLimit),
                    Height * 0.5f
                );

                const FVector BuildingExtent(
                    Width * 0.5f,
                    Depth * 0.5f,
                    Height * 0.5f
                );

                SpawnCityBuilding(BuildingCenter, BuildingExtent);
            }
        }
    }
}

void APathPlanningDemoActor::EditorGenerateMixedUrbanArea()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorGenerateMixedUrbanArea: World is null"));
        return;
    }

    if (CityBlocksX <= 0 || CityBlocksY <= 0)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorGenerateMixedUrbanArea: Invalid block count"));
        return;
    }

    EditorClearCityEnvironment();

    FRandomStream RandomStream(CitySeed);
    GenerateCityLayout_Mixed(RandomStream);

    if (bAutoRebuildGridAfterCityGenerate)
    {
        EditorBuildGridForMissionEditing();
    }

    UE_LOG(LogTemp, Warning, TEXT("EditorGenerateMixedUrbanArea done"));
}


// 求解时间统计
void APathPlanningDemoActor::ResetPlanningStats()
{
    LastPlanningStats = FPlanningTimingStats();
    LastPlanningStats.PlannerName = GetPlannerTypeName();
    LastPlanningStats.bMultiAgent = IsMultiAgentPlannerType();
}

bool APathPlanningDemoActor::ProcessStartGoalPairsSingleAgent(
    const TArray<int32>& Ids,
    const TMap<int32, TObjectPtr<AActor>>& Starts,
    const TMap<int32, TObjectPtr<AActor>>& Goals)
{
    bool bAnySuccess = false;

    for (int32 Id : Ids)
    {
        AActor* StartActor = Starts.FindRef(Id);
        AActor* GoalActor = Goals.FindRef(Id);

        if (!StartActor || !GoalActor)
        {
            UE_LOG(LogTemp, Error, TEXT("Pair %d invalid actor reference"), Id);

            FSingleMissionTimingStats Item;
            Item.MissionId = Id;
            Item.bSuccess = false;
            Item.PathPointCount = 0;
            Item.SolveTimeMs = 0.0;
            LastPlanningStats.MissionStats.Add(Item);
            continue;
        }

        const FVector StartWorld = StartActor->GetActorLocation();
        const FVector GoalWorld = GoalActor->GetActorLocation();

        if (!InputValidator.ValidateStartGoalPair(
            GridMap,
            StartWorld,
            GoalWorld,
            Id,
            StartActor,
            GoalActor))
        {
            UE_LOG(LogTemp, Error, TEXT("Pair %d invalid input. Skip planning."), Id);

            FSingleMissionTimingStats Item;
            Item.MissionId = Id;
            Item.bSuccess = false;
            Item.PathPointCount = 0;
            Item.SolveTimeMs = 0.0;
            LastPlanningStats.MissionStats.Add(Item);
            continue;
        }

        TUniquePtr<IPathPlannerBase> Planner = CreatePlannerByType();
        if (!Planner)
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to create planner for pair %d"), Id);

            FSingleMissionTimingStats Item;
            Item.MissionId = Id;
            Item.bSuccess = false;
            Item.PathPointCount = 0;
            Item.SolveTimeMs = 0.0;
            LastPlanningStats.MissionStats.Add(Item);
            continue;
        }

        TArray<FVector> PathPoints;

        const double SolveStart = FPlatformTime::Seconds();
        const bool bFound = Planner->Plan(GridMap, StartWorld, GoalWorld, PathPoints);
        const double SolveMs = (FPlatformTime::Seconds() - SolveStart) * 1000.0;

        LastPlanningStats.SolveTimeMs += SolveMs;

        FSingleMissionTimingStats Item;
        Item.MissionId = Id;
        Item.bSuccess = bFound;
        Item.PathPointCount = PathPoints.Num();
        Item.SolveTimeMs = SolveMs;
        LastPlanningStats.MissionStats.Add(Item);

        if (!bFound)
        {
            UE_LOG(LogTemp, Error, TEXT("Pair %d: path not found"), Id);
            continue;
        }

        bAnySuccess = true;
        CachePlannedPath(Id, PathPoints);

        const double PostStart = FPlatformTime::Seconds();

        UE_LOG(LogTemp, Warning, TEXT("Pair %d: path found, points=%d"), Id, PathPoints.Num());
        LogPathCoordinates(PathPoints, Id, TEXT("Pair"));
        DrawPathDebug(PathPoints, GetDebugColorById(Id));
        SpawnDroneForPath(PathPoints, Id);

        LastPlanningStats.PostProcessTimeMs +=
            (FPlatformTime::Seconds() - PostStart) * 1000.0;
    }

    return bAnySuccess;
}

void APathPlanningDemoActor::LogPlanningStatsSummary() const
{
    if (!bEnablePlanningTimeLog)
    {
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("========== Planning Timing Summary =========="));
    UE_LOG(LogTemp, Warning, TEXT("PlannerName        : %s"), *LastPlanningStats.PlannerName);
    UE_LOG(LogTemp, Warning, TEXT("MultiAgent         : %s"), LastPlanningStats.bMultiAgent ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Warning, TEXT("Success            : %s"), LastPlanningStats.bSuccess ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Warning, TEXT("MissionCount       : %d"), LastPlanningStats.MissionCount);

    UE_LOG(LogTemp, Warning, TEXT("MapType            : %s"), *GetCityLayoutTypeName());
    UE_LOG(LogTemp, Warning, TEXT("CitySeed           : %d"), CitySeed);
    UE_LOG(LogTemp, Warning, TEXT("RandomSeed        : %d"), RandomSeed);
    UE_LOG(LogTemp, Warning, TEXT("GridDim            : X=%d Y=%d Z=%d"), GridDim.X, GridDim.Y, GridDim.Z);
    UE_LOG(LogTemp, Warning, TEXT("CellSize           : %.3f"), CellSize);

    UE_LOG(LogTemp, Warning, TEXT("BuildGridTimeMs    : %.3f"), LastPlanningStats.BuildGridTimeMs);
    UE_LOG(LogTemp, Warning, TEXT("InputPrepTimeMs    : %.3f"), LastPlanningStats.InputPreparationTimeMs);
    UE_LOG(LogTemp, Warning, TEXT("SolveTimeMs        : %.3f"), LastPlanningStats.SolveTimeMs);
    UE_LOG(LogTemp, Warning, TEXT("PostProcessTimeMs  : %.3f"), LastPlanningStats.PostProcessTimeMs);
    UE_LOG(LogTemp, Warning, TEXT("TotalTimeMs        : %.3f"), LastPlanningStats.TotalTimeMs);

    for (const FSingleMissionTimingStats& Item : LastPlanningStats.MissionStats)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("MissionId=%d Success=%s PathPoints=%d SolveTimeMs=%.3f"),
            Item.MissionId,
            Item.bSuccess ? TEXT("true") : TEXT("false"),
            Item.PathPointCount,
            Item.SolveTimeMs
        );
    }

    UE_LOG(LogTemp, Warning, TEXT("============================================"));
}

FString APathPlanningDemoActor::GetCityLayoutTypeName() const
{
    switch (CityLayoutType)
    {
    case ECityLayoutType::Manhattan:
        return TEXT("Manhattan");
    case ECityLayoutType::Residential:
        return TEXT("Residential");
    case ECityLayoutType::Industrial:
        return TEXT("Industrial");
    case ECityLayoutType::Mixed:
        return TEXT("Mixed");
    default:
        return TEXT("Unknown");
    }
}


#include "Actors/NoFlyZoneMarkerActor.h"
#include "Components/BoxComponent.h"

namespace
{
    static FIntVector NormalizeMinCell(const FIntVector& A, const FIntVector& B)
    {
        return FIntVector(
            FMath::Min(A.X, B.X),
            FMath::Min(A.Y, B.Y),
            FMath::Min(A.Z, B.Z));
    }

    static FIntVector NormalizeMaxCell(const FIntVector& A, const FIntVector& B)
    {
        return FIntVector(
            FMath::Max(A.X, B.X),
            FMath::Max(A.Y, B.Y),
            FMath::Max(A.Z, B.Z));
    }

    static FIntVector ClampCellToGrid(const FIntVector& Cell, const FIntVector& GridDim)
    {
        return FIntVector(
            FMath::Clamp(Cell.X, 0, FMath::Max(0, GridDim.X - 1)),
            FMath::Clamp(Cell.Y, 0, FMath::Max(0, GridDim.Y - 1)),
            FMath::Clamp(Cell.Z, 0, FMath::Max(0, GridDim.Z - 1)));
    }
}

void APathPlanningDemoActor::GetNoFlyZoneMarkerActors(TArray<ANoFlyZoneMarkerActor*>& OutMarkers) const
{
    OutMarkers.Reset();

    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    for (TActorIterator<ANoFlyZoneMarkerActor> It(World); It; ++It)
    {
        ANoFlyZoneMarkerActor* Marker = *It;
        if (!Marker)
        {
            continue;
        }

        if (Marker->Tags.Contains(FName(TEXT("NoFlyZoneMarker"))))
        {
            OutMarkers.Add(Marker);
        }
    }
}

void APathPlanningDemoActor::EditorAddNoFlyZoneConfig()
{
    FTemporalNoFlyZoneConfig Zone;

    int32 NextZoneId = 1;
    for (const FTemporalNoFlyZoneConfig& ExistingZone : NoFlyZoneConfigs)
    {
        NextZoneId = FMath::Max(NextZoneId, ExistingZone.ZoneId + 1);
    }

    const FIntVector GridCenter(
        FMath::Max(0, GridDim.X / 2),
        FMath::Max(0, GridDim.Y / 2),
        FMath::Max(0, GridDim.Z / 2));

    const int32 HalfSpan = FMath::Max(0, DefaultNoFlyZoneSizeCells - 1) / 2;
    Zone.ZoneId = NextZoneId;
    Zone.bEnabled = true;
    Zone.MinCell = ClampCellToGrid(GridCenter - FIntVector(HalfSpan, HalfSpan, 0), GridDim);
    Zone.MaxCell = ClampCellToGrid(
        Zone.MinCell + FIntVector(
            FMath::Max(0, DefaultNoFlyZoneSizeCells - 1),
            FMath::Max(0, DefaultNoFlyZoneSizeCells - 1),
            0),
        GridDim);
    Zone.StartTimeStep = 0;
    Zone.EndTimeStep = FMath::Max(Zone.StartTimeStep, DefaultNoFlyZoneDuration - 1);

    NoFlyZoneConfigs.Add(Zone);

    UE_LOG(LogTemp, Warning, TEXT("EditorAddNoFlyZoneConfig added ZoneId=%d Min=(%d,%d,%d) Max=(%d,%d,%d) Time=[%d,%d]"),
        Zone.ZoneId,
        Zone.MinCell.X, Zone.MinCell.Y, Zone.MinCell.Z,
        Zone.MaxCell.X, Zone.MaxCell.Y, Zone.MaxCell.Z,
        Zone.StartTimeStep, Zone.EndTimeStep);
}

void APathPlanningDemoActor::EditorClearNoFlyZoneMarkers()
{
    TArray<ANoFlyZoneMarkerActor*> Markers;
    GetNoFlyZoneMarkerActors(Markers);

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

    UE_LOG(LogTemp, Warning, TEXT("EditorClearNoFlyZoneMarkers deleted %d markers"), DeleteCount);
}

void APathPlanningDemoActor::EditorSpawnNoFlyZoneMarkers()
{
    if (!NoFlyZoneMarkerClass)
    {
        UE_LOG(LogTemp, Error, TEXT("NoFlyZoneMarkerClass is null"));
        return;
    }

    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorSpawnNoFlyZoneMarkers: World is null"));
        return;
    }

    EditorClearNoFlyZoneMarkers();

    GridMap.GridOrigin = GridOrigin;
    GridMap.GridDim = GridDim;
    GridMap.CellSize = CellSize;

    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    for (const FTemporalNoFlyZoneConfig& ZoneConfig : NoFlyZoneConfigs)
    {
        FTemporalNoFlyZoneConfig NormalizedZone = ZoneConfig;
        NormalizedZone.MinCell = ClampCellToGrid(NormalizeMinCell(ZoneConfig.MinCell, ZoneConfig.MaxCell), GridDim);
        NormalizedZone.MaxCell = ClampCellToGrid(NormalizeMaxCell(ZoneConfig.MinCell, ZoneConfig.MaxCell), GridDim);
        NormalizedZone.EndTimeStep = FMath::Max(NormalizedZone.StartTimeStep, NormalizedZone.EndTimeStep);

        const FVector MinWorld = GridMap.CellToWorld(NormalizedZone.MinCell);
        const FVector MaxWorld = GridMap.CellToWorld(NormalizedZone.MaxCell);
        const FVector MarkerCenter = (MinWorld + MaxWorld) * 0.5f;
        const FVector MarkerExtent(
            (NormalizedZone.MaxCell.X - NormalizedZone.MinCell.X + 1) * CellSize * 0.5f,
            (NormalizedZone.MaxCell.Y - NormalizedZone.MinCell.Y + 1) * CellSize * 0.5f,
            (NormalizedZone.MaxCell.Z - NormalizedZone.MinCell.Z + 1) * CellSize * 0.5f);

        ANoFlyZoneMarkerActor* Marker = World->SpawnActor<ANoFlyZoneMarkerActor>(
            NoFlyZoneMarkerClass,
            MarkerCenter + FVector(0.f, 0.f, MarkerZOffset),
            FRotator::ZeroRotator,
            SpawnParams);

        if (!Marker)
        {
            UE_LOG(LogTemp, Warning, TEXT("Failed to spawn no-fly-zone marker for ZoneId=%d"), NormalizedZone.ZoneId);
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

    UE_LOG(LogTemp, Warning, TEXT("EditorSpawnNoFlyZoneMarkers done. Zone count=%d"), NoFlyZoneConfigs.Num());
}

void APathPlanningDemoActor::EditorReadNoFlyZoneMarkersToConfigs()
{
    GridMap.GridOrigin = GridOrigin;
    GridMap.GridDim = GridDim;
    GridMap.CellSize = CellSize;

    TArray<ANoFlyZoneMarkerActor*> Markers;
    GetNoFlyZoneMarkerActors(Markers);

    NoFlyZoneConfigs.Reset();

    const FVector HalfCell(CellSize * 0.5f, CellSize * 0.5f, CellSize * 0.5f);

    for (ANoFlyZoneMarkerActor* Marker : Markers)
    {
        if (!Marker || !Marker->BoxComponent)
        {
            continue;
        }

        FTemporalNoFlyZoneConfig ZoneConfig = Marker->ZoneConfig;
        const FVector CenterWorld = Marker->GetActorLocation() - FVector(0.f, 0.f, MarkerZOffset);
        const FVector Extent = Marker->BoxComponent->GetScaledBoxExtent();

        const FVector MinCornerWorld = CenterWorld - Extent + HalfCell;
        const FVector MaxCornerWorld = CenterWorld + Extent - HalfCell;

        ZoneConfig.MinCell = ClampCellToGrid(GridMap.WorldToCell(MinCornerWorld), GridDim);
        ZoneConfig.MaxCell = ClampCellToGrid(GridMap.WorldToCell(MaxCornerWorld), GridDim);
        ZoneConfig.MinCell = NormalizeMinCell(ZoneConfig.MinCell, ZoneConfig.MaxCell);
        ZoneConfig.MaxCell = NormalizeMaxCell(ZoneConfig.MinCell, ZoneConfig.MaxCell);
        ZoneConfig.EndTimeStep = FMath::Max(ZoneConfig.StartTimeStep, ZoneConfig.EndTimeStep);

        NoFlyZoneConfigs.Add(ZoneConfig);
    }

    NoFlyZoneConfigs.Sort([](const FTemporalNoFlyZoneConfig& A, const FTemporalNoFlyZoneConfig& B)
    {
        return A.ZoneId < B.ZoneId;
    });

    UE_LOG(LogTemp, Warning, TEXT("EditorReadNoFlyZoneMarkersToConfigs done. Zone count=%d"), NoFlyZoneConfigs.Num());
}

void APathPlanningDemoActor::EditorValidateNoFlyZones()
{
    EditorBuildGridForMissionEditing();

    UE_LOG(LogTemp, Warning, TEXT("Validate NoFlyZoneConfigs begin. Count=%d"), NoFlyZoneConfigs.Num());

    TSet<int32> ZoneIds;

    for (const FTemporalNoFlyZoneConfig& ZoneConfig : NoFlyZoneConfigs)
    {
        if (ZoneIds.Contains(ZoneConfig.ZoneId))
        {
            UE_LOG(LogTemp, Error, TEXT("Duplicate ZoneId=%d"), ZoneConfig.ZoneId);
        }
        ZoneIds.Add(ZoneConfig.ZoneId);

        const FIntVector MinCell = NormalizeMinCell(ZoneConfig.MinCell, ZoneConfig.MaxCell);
        const FIntVector MaxCell = NormalizeMaxCell(ZoneConfig.MinCell, ZoneConfig.MaxCell);

        const bool bInside = GridMap.IsInside(MinCell.X, MinCell.Y, MinCell.Z)
            && GridMap.IsInside(MaxCell.X, MaxCell.Y, MaxCell.Z);

        if (!bInside)
        {
            UE_LOG(LogTemp, Error, TEXT("Zone %d out of bounds. Min=(%d,%d,%d) Max=(%d,%d,%d)"),
                ZoneConfig.ZoneId,
                MinCell.X, MinCell.Y, MinCell.Z,
                MaxCell.X, MaxCell.Y, MaxCell.Z);
            continue;
        }

        if (ZoneConfig.EndTimeStep < ZoneConfig.StartTimeStep)
        {
            UE_LOG(LogTemp, Error, TEXT("Zone %d invalid time window [%d,%d]"),
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
                    if (GridMap.IsBlocked(X, Y, Z))
                    {
                        BlockedCount++;
                    }
                }
            }
        }

        UE_LOG(LogTemp, Warning, TEXT("Zone %d Enabled=%s Cells=%d Blocked=%d Min=(%d,%d,%d) Max=(%d,%d,%d) Time=[%d,%d]"),
            ZoneConfig.ZoneId,
            ZoneConfig.bEnabled ? TEXT("true") : TEXT("false"),
            CellCount,
            BlockedCount,
            MinCell.X, MinCell.Y, MinCell.Z,
            MaxCell.X, MaxCell.Y, MaxCell.Z,
            ZoneConfig.StartTimeStep,
            ZoneConfig.EndTimeStep);
    }

    UE_LOG(LogTemp, Warning, TEXT("Validate NoFlyZoneConfigs done."));
}
void APathPlanningDemoActor::EditorGenerateRandomNoFlyZoneConfigs()
{
    EditorBuildGridForMissionEditing();

    NoFlyZoneConfigs.Reset();

    if (GridDim.X <= 0 || GridDim.Y <= 0 || GridDim.Z <= 0)
    {
        UE_LOG(LogTemp, Error, TEXT("EditorGenerateRandomNoFlyZoneConfigs: invalid grid dimension"));
        return;
    }

    const int32 SafeMinSize = FMath::Max(1, FMath::Min(RandomNoFlyZoneMinSizeCells, RandomNoFlyZoneMaxSizeCells));
    const int32 SafeMaxSize = FMath::Max(SafeMinSize, FMath::Max(RandomNoFlyZoneMinSizeCells, RandomNoFlyZoneMaxSizeCells));
    const int32 SafeMinStart = FMath::Max(0, FMath::Min(RandomNoFlyZoneMinStartTimeStep, RandomNoFlyZoneMaxStartTimeStep));
    const int32 SafeMaxStart = FMath::Max(SafeMinStart, FMath::Max(RandomNoFlyZoneMinStartTimeStep, RandomNoFlyZoneMaxStartTimeStep));
    const int32 SafeMinDuration = FMath::Max(1, FMath::Min(RandomNoFlyZoneMinDuration, RandomNoFlyZoneMaxDuration));
    const int32 SafeMaxDuration = FMath::Max(SafeMinDuration, FMath::Max(RandomNoFlyZoneMinDuration, RandomNoFlyZoneMaxDuration));

    FRandomStream RandomStream(NoFlyZoneRandomSeed);

    for (int32 ZoneIndex = 0; ZoneIndex < RandomNoFlyZoneCount; ++ZoneIndex)
    {
        const int32 ZoneId = ZoneIndex + 1;
        bool bGenerated = false;

        for (int32 TryIndex = 0; TryIndex < 200; ++TryIndex)
        {
            const int32 SizeXY = RandomStream.RandRange(SafeMinSize, SafeMaxSize);
            const int32 SizeZ = FMath::Min(GridDim.Z, FMath::Max(1, RandomStream.RandRange(1, FMath::Min(2, SizeXY))));

            if (SizeXY > GridDim.X || SizeXY > GridDim.Y || SizeZ > GridDim.Z)
            {
                continue;
            }

            const int32 MinX = RandomStream.RandRange(0, GridDim.X - SizeXY);
            const int32 MinY = RandomStream.RandRange(0, GridDim.Y - SizeXY);
            const int32 MinZ = RandomStream.RandRange(0, GridDim.Z - SizeZ);

            FTemporalNoFlyZoneConfig ZoneConfig;
            ZoneConfig.ZoneId = ZoneId;
            ZoneConfig.bEnabled = true;
            ZoneConfig.MinCell = FIntVector(MinX, MinY, MinZ);
            ZoneConfig.MaxCell = FIntVector(MinX + SizeXY - 1, MinY + SizeXY - 1, MinZ + SizeZ - 1);
            ZoneConfig.StartTimeStep = RandomStream.RandRange(SafeMinStart, SafeMaxStart);

            const int32 Duration = RandomStream.RandRange(SafeMinDuration, SafeMaxDuration);
            ZoneConfig.EndTimeStep = ZoneConfig.StartTimeStep + Duration - 1;

            int32 CellCount = 0;
            int32 FreeCount = 0;
            for (int32 X = ZoneConfig.MinCell.X; X <= ZoneConfig.MaxCell.X; ++X)
            {
                for (int32 Y = ZoneConfig.MinCell.Y; Y <= ZoneConfig.MaxCell.Y; ++Y)
                {
                    for (int32 Z = ZoneConfig.MinCell.Z; Z <= ZoneConfig.MaxCell.Z; ++Z)
                    {
                        CellCount++;
                        if (!GridMap.IsBlocked(X, Y, Z))
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

            NoFlyZoneConfigs.Add(ZoneConfig);
            bGenerated = true;
            break;
        }

        if (!bGenerated)
        {
            UE_LOG(LogTemp, Warning, TEXT("Failed to generate no-fly zone %d"), ZoneId);
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("Generated random no-fly zones: %d (Seed=%d Size=[%d,%d] Start=[%d,%d] Duration=[%d,%d])"),
        NoFlyZoneConfigs.Num(),
        NoFlyZoneRandomSeed,
        SafeMinSize,
        SafeMaxSize,
        SafeMinStart,
        SafeMaxStart,
        SafeMinDuration,
        SafeMaxDuration);
}

