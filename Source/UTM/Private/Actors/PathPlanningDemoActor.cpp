#include "Actors/PathPlanningDemoActor.h"

#include "Components/SceneComponent.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"

#include "Actors/DroneActor.h"
#include "Engine/World.h"

#include "Planning/DiscreteAlignmentManager.h"
#include "Planning/PlannerRegistry.h"

#include "Actors/MissionMarkerActor.h"
#include "Engine/StaticMeshActor.h"
#include "Dom/JsonObject.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/ScopeExit.h"

// 障碍物建筑构建
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

#include "HAL/PlatformTime.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace
{
    constexpr int32 MaxLoggedPathPoints = 64;
    constexpr int32 LoggedPathPreviewCount = 16;
    constexpr int32 MaxDebugDrawPathPoints = 2048;
    constexpr int32 MaxSpawnableDronePathPoints = 20000;

    enum class EPredictedExecutionConflictType : uint8
    {
        None,
        Vertex,
        Edge,
        ProtectionFootprint,
        Downwash
    };

    struct FExecutionStepProposal
    {
        int32 MissionId = INDEX_NONE;
        FIntVector ObservedCell = FIntVector::ZeroValue;
        FIntVector ProposedCell = FIntVector::ZeroValue;
        int32 ReferencePlanIndex = 0;
        int32 ProposedPlanIndex = 0;
        bool bDelayRequested = false;
        bool bValid = false;
        bool bRequiresReplan = false;
        bool bInitialAlignmentInvalid = false;
        bool bHeldForPredictedConflict = false;
        bool bHeldForReplan = false;
        FDiscreteAlignmentResult AlignmentResult;
        EDiscreteAlignmentAction FinalAction = EDiscreteAlignmentAction::FollowPlan;
        FString ResolutionReason;
    };

    struct FPredictedExecutionConflict
    {
        EPredictedExecutionConflictType Type = EPredictedExecutionConflictType::None;
        int32 AgentA = INDEX_NONE;
        int32 AgentB = INDEX_NONE;
        FIntVector Cell = FIntVector::ZeroValue;
    };

    const TCHAR* LexToString(EPredictedExecutionConflictType Type)
    {
        switch (Type)
        {
        case EPredictedExecutionConflictType::Vertex:
            return TEXT("Vertex");
        case EPredictedExecutionConflictType::Edge:
            return TEXT("Edge");
        case EPredictedExecutionConflictType::ProtectionFootprint:
            return TEXT("ProtectionFootprint");
        case EPredictedExecutionConflictType::Downwash:
            return TEXT("Downwash");
        default:
            return TEXT("None");
        }
    }

    template<typename TEnum>
    FString GetEnumNameString(const TEnum Value)
    {
        if (const UEnum* Enum = StaticEnum<TEnum>())
        {
            return Enum->GetNameStringByValue(static_cast<int64>(Value));
        }

        return TEXT("Unknown");
    }

    FString SanitizeExperimentToken(const FString& Input)
    {
        FString Result;
        Result.Reserve(Input.Len());

        for (const TCHAR Char : Input)
        {
            if (FChar::IsAlnum(Char))
            {
                Result.AppendChar(Char);
            }
        }

        return Result.IsEmpty() ? TEXT("Unknown") : Result;
    }

    struct FMissionFootprintBox
    {
        bool bValid = false;
        FIntVector Min = FIntVector::ZeroValue;
        FIntVector Max = FIntVector::ZeroValue;
    };

    FMissionFootprintBox MakeMissionProtectionBox(const FIntVector& CenterCell, const FDroneMissionConfig& Mission)
    {
        const int32 ProtectionXYRadiusCells = FMath::Max(0, Mission.ProtectionXYRadiusCells);
        const int32 ProtectionZUpCells = FMath::Max(0, Mission.ProtectionZUpCells);
        const int32 ProtectionZDownCells = FMath::Max(0, Mission.ProtectionZDownCells);

        FMissionFootprintBox Result;
        Result.bValid = true;
        Result.Min = FIntVector(
            CenterCell.X - ProtectionXYRadiusCells,
            CenterCell.Y - ProtectionXYRadiusCells,
            CenterCell.Z - ProtectionZDownCells);
        Result.Max = FIntVector(
            CenterCell.X + ProtectionXYRadiusCells,
            CenterCell.Y + ProtectionXYRadiusCells,
            CenterCell.Z + ProtectionZUpCells);
        return Result;
    }

    FMissionFootprintBox MakeMissionDownwashBox(const FIntVector& CenterCell, const FDroneMissionConfig& Mission)
    {
        const int32 DownwashXYRadiusCells = FMath::Max(0, Mission.DownwashXYRadiusCells);
        const int32 DownwashZBelowCells = FMath::Max(0, Mission.DownwashZBelowCells);

        if (DownwashZBelowCells <= 0)
        {
            return FMissionFootprintBox();
        }

        FMissionFootprintBox Result;
        Result.bValid = true;
        Result.Min = FIntVector(
            CenterCell.X - DownwashXYRadiusCells,
            CenterCell.Y - DownwashXYRadiusCells,
            CenterCell.Z - DownwashZBelowCells);
        Result.Max = FIntVector(
            CenterCell.X + DownwashXYRadiusCells,
            CenterCell.Y + DownwashXYRadiusCells,
            CenterCell.Z - 1);
        return Result;
    }

    bool MissionBoxesOverlap(const FMissionFootprintBox& Left, const FMissionFootprintBox& Right)
    {
        return Left.bValid
            && Right.bValid
            && Left.Min.X <= Right.Max.X && Right.Min.X <= Left.Max.X
            && Left.Min.Y <= Right.Max.Y && Right.Min.Y <= Left.Max.Y
            && Left.Min.Z <= Right.Max.Z && Right.Min.Z <= Left.Max.Z;
    }

    enum class EStaticUTMConflictType : uint8
    {
        None,
        ProtectionFootprint,
        Downwash
    };

    EStaticUTMConflictType GetStaticUTMConfigConflictType(
        const FIntVector& CellA,
        const FDroneMissionConfig& MissionA,
        const FIntVector& CellB,
        const FDroneMissionConfig& MissionB)
    {
        const FMissionFootprintBox ProtectionA = MakeMissionProtectionBox(CellA, MissionA);
        const FMissionFootprintBox ProtectionB = MakeMissionProtectionBox(CellB, MissionB);
        if (MissionBoxesOverlap(ProtectionA, ProtectionB))
        {
            return EStaticUTMConflictType::ProtectionFootprint;
        }

        if (CellA.Z > CellB.Z
            && MissionBoxesOverlap(MakeMissionDownwashBox(CellA, MissionA), ProtectionB))
        {
            return EStaticUTMConflictType::Downwash;
        }

        if (CellB.Z > CellA.Z
            && MissionBoxesOverlap(MakeMissionDownwashBox(CellB, MissionB), ProtectionA))
        {
            return EStaticUTMConflictType::Downwash;
        }

        return EStaticUTMConflictType::None;
    }

    bool HasStaticUTMConfigConflict(
        const FIntVector& CellA,
        const FDroneMissionConfig& MissionA,
        const FIntVector& CellB,
        const FDroneMissionConfig& MissionB)
    {
        return GetStaticUTMConfigConflictType(CellA, MissionA, CellB, MissionB)
            != EStaticUTMConflictType::None;
    }

    FString GetDefaultExperimentGroupNameById(const FString& GroupId)
    {
        if (GroupId == TEXT("G1"))
        {
            return TEXT("ExecutionOnly");
        }
        if (GroupId == TEXT("G2"))
        {
            return TEXT("AlignmentV1");
        }
        if (GroupId == TEXT("G3"))
        {
            return TEXT("AlignmentV2Global");
        }
        if (GroupId == TEXT("R1"))
        {
            return TEXT("V2NoReplan");
        }
        if (GroupId == TEXT("R2"))
        {
            return TEXT("V2Local");
        }
        if (GroupId == TEXT("R3"))
        {
            return TEXT("V2Global");
        }

        return FString();
    }

    bool IsKnownExperimentGroupId(const FString& GroupId)
    {
        return
            GroupId == TEXT("G1") ||
            GroupId == TEXT("G2") ||
            GroupId == TEXT("G3") ||
            GroupId == TEXT("R1") ||
            GroupId == TEXT("R2") ||
            GroupId == TEXT("R3");
    }

    bool IsKnownExperimentPhase(const FString& Phase)
    {
        return Phase == TEXT("PhaseA") || Phase == TEXT("PhaseB");
    }

    FString GetDefaultExperimentPhaseByGroupId(const FString& GroupId)
    {
        if (GroupId.StartsWith(TEXT("G")))
        {
            return TEXT("PhaseA");
        }

        if (GroupId.StartsWith(TEXT("R")))
        {
            return TEXT("PhaseB");
        }

        return FString();
    }

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

namespace
{
    FString GetExperimentPhaseBySeed(const int32 Seed)
    {
        switch (Seed)
        {
        case 1:
            return TEXT("PhaseA");
        case 3:
            return TEXT("PhaseB");
        default:
            return FString();
        }
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

    if ((!bOverallSuccess || !IsMultiAgentPlannerType() || !bUseCentralizedExecution) &&
        bLogStructuredExperimentJson)
    {
        LogStructuredExperimentSummaryJson();
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

    while (bExecutionRunning && ExecutionAccumulator >= CBSStepDuration)
    {
        ExecutionAccumulator -= CBSStepDuration;
        AdvanceExecutionOneStep();
    }

    if (!bExecutionRunning)
    {
        return;
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
    ExecutionMissionConfigsByMissionId.Reset();
    ExecutionStates.Reset();
    ExecutionConflicts.Reset();

    ExecutionAccumulator = 0.f;
    CurrentExecutionTimeStep = 0;
    TotalExecutionReplanCount = 0;
    ExecutionReplanTimingStats = FExecutionReplanTimingStats();
    bExecutionRunning = false;

    LastExecutionSummary = FExecutionSummary();
}

void APathPlanningDemoActor::CacheExecutionMissionConfigs(const TArray<FDroneMissionConfig>& Missions)
{
    ExecutionMissionConfigsByMissionId.Reset();

    for (const FDroneMissionConfig& Mission : Missions)
    {
        ExecutionMissionConfigsByMissionId.Add(Mission.MissionId, Mission);
    }
}

void APathPlanningDemoActor::InitializeExecutionStates()
{
    ExecutionStates.Reset();
    ExecutionConflicts.Reset();

    ExecutionRandom.Initialize(ExecutionRandomSeed);
    ExecutionAccumulator = 0.f;
    CurrentExecutionTimeStep = 0;
    TotalExecutionReplanCount = 0;
    ExecutionReplanTimingStats = FExecutionReplanTimingStats();
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
        State.LastObservedCell = PlannedCells[0];
        State.GoalCell = PlannedCells.Last();
        State.GoalWorld = GridMap.CellToWorld(PlannedCells.Last());
        State.ConsecutiveConflictHoldCount = 0;
        State.ConsecutiveSafetyGateHoldCount = 0;
        State.LastAlignmentAction = TEXT("Initialize");

        if (const FDroneMissionConfig* MissionConfig = ExecutionMissionConfigsByMissionId.Find(MissionId))
        {
            State.GoalWorld = MissionConfig->GoalWorld;
            State.GoalCell = GridMap.WorldToCell(MissionConfig->GoalWorld);
        }

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

FIntVector APathPlanningDemoActor::GetObservedExecutionCell(const FExecutionAgentState& State) const
{
    auto ClampCellToGrid = [&](const FIntVector& Cell) -> FIntVector
        {
            return FIntVector(
                FMath::Clamp(Cell.X, 0, FMath::Max(0, GridMap.GridDim.X - 1)),
                FMath::Clamp(Cell.Y, 0, FMath::Max(0, GridMap.GridDim.Y - 1)),
                FMath::Clamp(Cell.Z, 0, FMath::Max(0, GridMap.GridDim.Z - 1)));
        };

    if (State.Drone)
    {
        return ClampCellToGrid(GridMap.WorldToCell(State.Drone->GetActorLocation()));
    }

    if (State.ActualCells.Num() > 0)
    {
        return ClampCellToGrid(State.ActualCells.Last());
    }

    return ClampCellToGrid(GetCellAtTime(State.PlannedCells, State.ExecutedPlanIndex));
}

FDiscreteAlignmentSettings APathPlanningDemoActor::BuildDiscreteAlignmentSettings() const
{
    FDiscreteAlignmentSettings Settings;
    Settings.bEnabled = bEnableDiscreteAlignment;
    Settings.SearchRadiusSteps = FMath::Max(1, AlignmentSearchRadiusSteps);
    Settings.MaxSpatialErrorCells = FMath::Max(0, AlignmentMaxSpatialErrorCells);
    Settings.MaxSnapAheadSteps = FMath::Max(0, AlignmentMaxSnapAheadSteps);
    Settings.bAllowRecoveryMoves = bAlignmentAllowRecoveryMoves;
    Settings.bHoldPositionOnFailure = bAlignmentHoldPositionOnFailure;
    return Settings;
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
    // Snap the previous segment to its terminal cell before sampling the current position.
    UpdateExecutionVisuals(1.f);
    CurrentExecutionTimeStep++;


    const FDiscreteAlignmentManager AlignmentManager(BuildDiscreteAlignmentSettings());

    TArray<int32> MissionIds;
    ExecutionStates.GetKeys(MissionIds);
    MissionIds.Sort();

    TMap<int32, FExecutionStepProposal> StepProposals;
    TSet<int32> RequestedReplanMissionIds;

    for (const int32 MissionId : MissionIds)
    {
        FExecutionAgentState* State = ExecutionStates.Find(MissionId);
        if (!State || State->PlannedCells.Num() <= 0)
        {
            continue;
        }

        const FIntVector ObservedCell = GetObservedExecutionCell(*State);
        State->LastObservedCell = ObservedCell;
        State->DisplayFromCell = ObservedCell;

        const bool bCanAdvance = (State->ExecutedPlanIndex + 1 < State->PlannedCells.Num());
        const bool bDelay = bCanAdvance && ShouldDelayThisStep(*State, CurrentExecutionTimeStep);
        const FDiscreteAlignmentResult AlignmentResult =
            AlignmentManager.AlignStep(
                GridMap,
                State->PlannedCells,
                State->ExecutedPlanIndex,
                CurrentExecutionTimeStep,
                ObservedCell,
                bDelay);

        FExecutionStepProposal Proposal;
        Proposal.MissionId = MissionId;
        Proposal.ObservedCell = ObservedCell;
        Proposal.bDelayRequested = bDelay;
        Proposal.AlignmentResult = AlignmentResult;
        Proposal.ReferencePlanIndex = FMath::Clamp(State->ExecutedPlanIndex, 0, State->PlannedCells.Num() - 1);
        Proposal.ProposedPlanIndex = Proposal.ReferencePlanIndex;
        Proposal.ProposedCell = ObservedCell;
        Proposal.FinalAction = EDiscreteAlignmentAction::HoldForAlignment;
        Proposal.ResolutionReason = AlignmentResult.Reason;

        if (AlignmentResult.bValid)
        {
            Proposal.bValid = true;
            Proposal.bRequiresReplan = AlignmentResult.bRequiresReplan;
            Proposal.ReferencePlanIndex = FMath::Clamp(AlignmentResult.ReferencePlanIndex, 0, State->PlannedCells.Num() - 1);
            Proposal.ProposedPlanIndex = FMath::Clamp(AlignmentResult.NextPlanIndex, 0, State->PlannedCells.Num() - 1);



            Proposal.ProposedCell = AlignmentResult.NextCell;
            Proposal.FinalAction = AlignmentResult.Action;
        }
        else
        {
            Proposal.bInitialAlignmentInvalid = true;
            Proposal.bRequiresReplan = true;
            Proposal.ResolutionReason = AlignmentResult.Reason.IsEmpty()
                ? TEXT("invalid alignment result")
                : AlignmentResult.Reason;
        }

        if (Proposal.bRequiresReplan || Proposal.bInitialAlignmentInvalid)
        {
            RequestedReplanMissionIds.Add(MissionId);
        }

        StepProposals.Add(MissionId, Proposal);
    }

    auto FindFirstPredictedConflict =
        [&](FPredictedExecutionConflict& OutConflict) -> bool
        {
            for (int32 I = 0; I < MissionIds.Num(); ++I)
            {
                const FExecutionStepProposal* ProposalA = StepProposals.Find(MissionIds[I]);
                if (!ProposalA)
                {
                    continue;
                }

                for (int32 J = I + 1; J < MissionIds.Num(); ++J)
                {
                    const FExecutionStepProposal* ProposalB = StepProposals.Find(MissionIds[J]);
                    if (!ProposalB)
                    {
                        continue;
                    }

                    if (ProposalA->ProposedCell == ProposalB->ProposedCell)
                    {
                        OutConflict.Type = EPredictedExecutionConflictType::Vertex;
                        OutConflict.AgentA = ProposalA->MissionId;
                        OutConflict.AgentB = ProposalB->MissionId;
                        OutConflict.Cell = ProposalA->ProposedCell;
                        return true;
                    }

                    const bool bEdgeConflict =
                        (ProposalA->ObservedCell == ProposalB->ProposedCell) &&
                        (ProposalB->ObservedCell == ProposalA->ProposedCell) &&
                        (ProposalA->ProposedCell != ProposalB->ProposedCell);

                    if (bEdgeConflict)
                    {
                        OutConflict.Type = EPredictedExecutionConflictType::Edge;
                        OutConflict.AgentA = ProposalA->MissionId;
                        OutConflict.AgentB = ProposalB->MissionId;
                        OutConflict.Cell = ProposalA->ProposedCell;
                        return true;
                    }

                    if (PlannerType == EPlannerType::LaCAMUTM)
                    {
                        const FDroneMissionConfig* MissionConfigA = ExecutionMissionConfigsByMissionId.Find(ProposalA->MissionId);
                        const FDroneMissionConfig* MissionConfigB = ExecutionMissionConfigsByMissionId.Find(ProposalB->MissionId);
                        if (MissionConfigA && MissionConfigB)
                        {
                            const EStaticUTMConflictType UTMConflictType = GetStaticUTMConfigConflictType(
                                ProposalA->ProposedCell,
                                *MissionConfigA,
                                ProposalB->ProposedCell,
                                *MissionConfigB);

                            if (UTMConflictType != EStaticUTMConflictType::None)
                            {
                                OutConflict.Type = (UTMConflictType == EStaticUTMConflictType::ProtectionFootprint)
                                    ? EPredictedExecutionConflictType::ProtectionFootprint
                                    : EPredictedExecutionConflictType::Downwash;
                                OutConflict.AgentA = ProposalA->MissionId;
                                OutConflict.AgentB = ProposalB->MissionId;
                                OutConflict.Cell = ProposalA->ProposedCell;
                                return true;
                            }
                        }
                    }
                }
            }

            return false;
        };

    auto ChooseYieldingMissionId =
        [&](const FExecutionStepProposal& ProposalA,
            const FExecutionStepProposal& ProposalB) -> int32
        {
            const bool bAStays = (ProposalA.ProposedCell == ProposalA.ObservedCell);
            const bool bBStays = (ProposalB.ProposedCell == ProposalB.ObservedCell);
            if (bAStays != bBStays)
            {
                return bAStays ? ProposalB.MissionId : ProposalA.MissionId;
            }

            const FExecutionAgentState* StateA = ExecutionStates.Find(ProposalA.MissionId);
            const FExecutionAgentState* StateB = ExecutionStates.Find(ProposalB.MissionId);
            const bool bAGoalHold = StateA && (StateA->bFinished || ProposalA.FinalAction == EDiscreteAlignmentAction::GoalHold);
            const bool bBGoalHold = StateB && (StateB->bFinished || ProposalB.FinalAction == EDiscreteAlignmentAction::GoalHold);
            if (bAGoalHold != bBGoalHold)
            {
                return bAGoalHold ? ProposalB.MissionId : ProposalA.MissionId;
            }

            return ProposalA.MissionId > ProposalB.MissionId
                ? ProposalA.MissionId
                : ProposalB.MissionId;
        };

    if (bEnableConflictAwareAlignment && StepProposals.Num() > 1)
    {
        const int32 MaxPasses = FMath::Max(1, AlignmentConflictResolutionPasses);
        bool bNeedsAnotherPass = true;

        for (int32 Pass = 0; Pass < MaxPasses && bNeedsAnotherPass; ++Pass)
        {
            bNeedsAnotherPass = false;

            FPredictedExecutionConflict Conflict;
            if (!FindFirstPredictedConflict(Conflict))
            {
                break;

            }

            FExecutionStepProposal* ProposalA = StepProposals.Find(Conflict.AgentA);
            FExecutionStepProposal* ProposalB = StepProposals.Find(Conflict.AgentB);
            if (!ProposalA || !ProposalB)
            {
                RequestedReplanMissionIds.Add(Conflict.AgentA);
                RequestedReplanMissionIds.Add(Conflict.AgentB);
                break;
            }

            const int32 YieldMissionId = ChooseYieldingMissionId(*ProposalA, *ProposalB);
            FExecutionStepProposal* YieldProposal = StepProposals.Find(YieldMissionId);
            const int32 KeepMissionId = (YieldMissionId == Conflict.AgentA) ? Conflict.AgentB : Conflict.AgentA;

            if (!YieldProposal)
            {
                RequestedReplanMissionIds.Add(Conflict.AgentA);
                RequestedReplanMissionIds.Add(Conflict.AgentB);
                break;

            }

            const bool bAlreadyHolding =
                (YieldProposal->ProposedCell == YieldProposal->ObservedCell) &&
                (YieldProposal->FinalAction == EDiscreteAlignmentAction::HoldForPredictedConflict);

            if (bAlreadyHolding)
            {
                RequestedReplanMissionIds.Add(Conflict.AgentA);
                RequestedReplanMissionIds.Add(Conflict.AgentB);

                if (bLogConflictPredictionEvents)
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("[AlignmentConflictPrediction] t=%d unresolved predicted %s conflict between Mission %d and Mission %d, escalate to replan"),
                        CurrentExecutionTimeStep,
                        LexToString(Conflict.Type),
                        Conflict.AgentA,
                        Conflict.AgentB);
                }
                break;
            }

            YieldProposal->ProposedCell = YieldProposal->ObservedCell;
            YieldProposal->ProposedPlanIndex = YieldProposal->ReferencePlanIndex;
            YieldProposal->bHeldForPredictedConflict = true;
            YieldProposal->FinalAction = EDiscreteAlignmentAction::HoldForPredictedConflict;
            YieldProposal->ResolutionReason = FString::Printf(
                TEXT("yield to Mission %d for predicted %s conflict"),
                KeepMissionId,
                LexToString(Conflict.Type));

            if (bLogConflictPredictionEvents)
            {
                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("[AlignmentConflictPrediction] t=%d Mission=%d hold for predicted %s conflict with Mission=%d at Cell=(%d,%d,%d)"),
                    CurrentExecutionTimeStep,
                    YieldProposal->MissionId,
                    LexToString(Conflict.Type),
                    KeepMissionId,
                    Conflict.Cell.X,
                    Conflict.Cell.Y,
                    Conflict.Cell.Z);










            }

            bNeedsAnotherPass = true;
        }

        FPredictedExecutionConflict RemainingConflict;
        if (FindFirstPredictedConflict(RemainingConflict))
        {
            RequestedReplanMissionIds.Add(RemainingConflict.AgentA);
            RequestedReplanMissionIds.Add(RemainingConflict.AgentB);



            if (bLogConflictPredictionEvents)
            {
                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("[AlignmentConflictPrediction] t=%d remaining predicted %s conflict between Mission %d and Mission %d after arbitration"),
                    CurrentExecutionTimeStep,
                    LexToString(RemainingConflict.Type),
                    RemainingConflict.AgentA,
                    RemainingConflict.AgentB);


            }
        }

        const int32 ConflictHoldBudget = FMath::Max(1, AlignmentConflictHoldThresholdForReplan);
        for (const int32 MissionId : MissionIds)
        {
            const FExecutionStepProposal* Proposal = StepProposals.Find(MissionId);
            const FExecutionAgentState* State = ExecutionStates.Find(MissionId);
            if (!Proposal || !State || !Proposal->bHeldForPredictedConflict)
            {
                continue;
            }

            if (State->ConsecutiveConflictHoldCount + 1 >= ConflictHoldBudget)
            {
                RequestedReplanMissionIds.Add(MissionId);
            }
        }
    }

    TSet<int32> SuccessfulReplanMissionIds;
    bool bReplanSucceeded = false;
    if (RequestedReplanMissionIds.Num() > 0 && ExecutionReplanMode != EExecutionReplanMode::Disabled)
    {
        const bool bUseGlobalReplan = (ExecutionReplanMode == EExecutionReplanMode::GlobalUnfinished);
        bReplanSucceeded = TryExecutionReplan(RequestedReplanMissionIds, bUseGlobalReplan, SuccessfulReplanMissionIds);

        if (!bReplanSucceeded && ExecutionReplanMode == EExecutionReplanMode::LocalConflictSet)
        {
            bReplanSucceeded = TryExecutionReplan(RequestedReplanMissionIds, true, SuccessfulReplanMissionIds);
        }

        if (bReplanSucceeded)
        {
            for (const int32 MissionId : MissionIds)
            {
                FExecutionStepProposal* Proposal = StepProposals.Find(MissionId);
                FExecutionAgentState* State = ExecutionStates.Find(MissionId);
                if (!Proposal || !State || State->PlannedCells.Num() <= 0)
                {
                    continue;
                }

                Proposal->bValid = true;
                Proposal->bHeldForReplan = true;
                Proposal->bHeldForPredictedConflict = false;
                Proposal->bRequiresReplan = false;
                Proposal->FinalAction = EDiscreteAlignmentAction::HoldForReplan;
                Proposal->ReferencePlanIndex = FMath::Clamp(State->ExecutedPlanIndex, 0, State->PlannedCells.Num() - 1);
                Proposal->ProposedPlanIndex = SuccessfulReplanMissionIds.Contains(MissionId)
                    ? FMath::Min(Proposal->ReferencePlanIndex + 1, State->PlannedCells.Num() - 1)
                    : Proposal->ReferencePlanIndex;
                Proposal->ProposedCell = Proposal->ObservedCell;
                Proposal->ResolutionReason = SuccessfulReplanMissionIds.Contains(MissionId)
                    ? TEXT("hold while applying replanned trajectory")
                    : TEXT("hold to synchronize with replanned agents");
            }
        }
    }

    bool bStopExecutionForSafetyGate = false;

    if (bEnableFinalSafetyGate && StepProposals.Num() > 1)
    {
        auto FindProposalPairConflict = [&](const FExecutionStepProposal& ProposalA, const FExecutionStepProposal& ProposalB, FPredictedExecutionConflict& OutConflict) -> bool
            {
                if (ProposalA.ProposedCell == ProposalB.ProposedCell)
                {
                    OutConflict.Type = EPredictedExecutionConflictType::Vertex;
                    OutConflict.AgentA = ProposalA.MissionId;
                    OutConflict.AgentB = ProposalB.MissionId;
                    OutConflict.Cell = ProposalA.ProposedCell;
                    return true;
                }

                const bool bEdgeConflict =
                    (ProposalA.ObservedCell == ProposalB.ProposedCell) &&
                    (ProposalB.ObservedCell == ProposalA.ProposedCell) &&
                    (ProposalA.ProposedCell != ProposalB.ProposedCell);

                if (bEdgeConflict)
                {
                    OutConflict.Type = EPredictedExecutionConflictType::Edge;
                    OutConflict.AgentA = ProposalA.MissionId;
                    OutConflict.AgentB = ProposalB.MissionId;
                    OutConflict.Cell = ProposalA.ProposedCell;
                    return true;
                }

                if (PlannerType == EPlannerType::LaCAMUTM)
                {
                    const FDroneMissionConfig* MissionConfigA = ExecutionMissionConfigsByMissionId.Find(ProposalA.MissionId);
                    const FDroneMissionConfig* MissionConfigB = ExecutionMissionConfigsByMissionId.Find(ProposalB.MissionId);
                    if (MissionConfigA && MissionConfigB)
                    {
                        const EStaticUTMConflictType UTMConflictType = GetStaticUTMConfigConflictType(
                            ProposalA.ProposedCell,
                            *MissionConfigA,
                            ProposalB.ProposedCell,
                            *MissionConfigB);

                        if (UTMConflictType != EStaticUTMConflictType::None)
                        {
                            OutConflict.Type = (UTMConflictType == EStaticUTMConflictType::ProtectionFootprint)
                                ? EPredictedExecutionConflictType::ProtectionFootprint
                                : EPredictedExecutionConflictType::Downwash;
                            OutConflict.AgentA = ProposalA.MissionId;
                            OutConflict.AgentB = ProposalB.MissionId;
                            OutConflict.Cell = ProposalA.ProposedCell;
                            return true;
                        }
                    }
                }

                return false;
            };

        auto CollectProposalConflictEndpoints = [&](TSet<int32>& OutMissionIds, FPredictedExecutionConflict& OutFirstConflict) -> bool
            {
                bool bFoundConflict = false;
                for (int32 I = 0; I < MissionIds.Num(); ++I)
                {
                    const FExecutionStepProposal* ProposalA = StepProposals.Find(MissionIds[I]);
                    if (!ProposalA)
                    {
                        continue;
                    }

                    for (int32 J = I + 1; J < MissionIds.Num(); ++J)
                    {
                        const FExecutionStepProposal* ProposalB = StepProposals.Find(MissionIds[J]);
                        if (!ProposalB)
                        {
                            continue;
                        }

                        FPredictedExecutionConflict Conflict;
                        if (!FindProposalPairConflict(*ProposalA, *ProposalB, Conflict))
                        {
                            continue;
                        }

                        if (!bFoundConflict)
                        {
                            OutFirstConflict = Conflict;
                            bFoundConflict = true;
                        }

                        OutMissionIds.Add(Conflict.AgentA);
                        OutMissionIds.Add(Conflict.AgentB);
                    }
                }

                return bFoundConflict;
            };

        auto ApplySafetyGateHold = [&](const TSet<int32>& HoldMissionIds)
            {
                for (const int32 MissionId : HoldMissionIds)
                {
                    FExecutionStepProposal* Proposal = StepProposals.Find(MissionId);
                    if (!Proposal)
                    {
                        continue;
                    }

                    Proposal->bValid = true;
                    Proposal->bHeldForPredictedConflict = false;
                    Proposal->bHeldForReplan = false;
                    Proposal->bRequiresReplan = true;
                    Proposal->FinalAction = EDiscreteAlignmentAction::HoldForSafetyGate;
                    Proposal->ProposedPlanIndex = Proposal->ReferencePlanIndex;
                    Proposal->ProposedCell = Proposal->ObservedCell;
                    Proposal->ResolutionReason = TEXT("final safety gate hold before replanning");
                }
            };

        FPredictedExecutionConflict GateConflict;
        TSet<int32> SafetyGateMissionIds;
        if (CollectProposalConflictEndpoints(SafetyGateMissionIds, GateConflict))
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[FinalSafetyGate] t=%d unsafe proposed %s conflict between Mission %d and Mission %d at Cell=(%d,%d,%d); forcing %d missions to hold"),
                CurrentExecutionTimeStep,
                LexToString(GateConflict.Type),
                GateConflict.AgentA,
                GateConflict.AgentB,
                GateConflict.Cell.X,
                GateConflict.Cell.Y,
                GateConflict.Cell.Z,
                SafetyGateMissionIds.Num());

            bool bHoldConfigurationSafe = false;
            FPredictedExecutionConflict HoldConflict;
            for (int32 Pass = 0; Pass < MissionIds.Num(); ++Pass)
            {
                ApplySafetyGateHold(SafetyGateMissionIds);

                TSet<int32> RemainingConflictMissionIds;
                FPredictedExecutionConflict RemainingConflict;
                if (!CollectProposalConflictEndpoints(RemainingConflictMissionIds, RemainingConflict))
                {
                    bHoldConfigurationSafe = true;
                    break;
                }

                bool bExpandedHoldSet = false;
                const int32 PreviousHoldCount = SafetyGateMissionIds.Num();
                for (const int32 MissionId : RemainingConflictMissionIds)
                {
                    if (!SafetyGateMissionIds.Contains(MissionId))
                    {
                        SafetyGateMissionIds.Add(MissionId);
                        bExpandedHoldSet = true;
                    }
                }

                if (bExpandedHoldSet)
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("[FinalSafetyGate] t=%d hold set expanded from %d to %d missions due to remaining %s conflict between Mission %d and Mission %d"),
                        CurrentExecutionTimeStep,
                        PreviousHoldCount,
                        SafetyGateMissionIds.Num(),
                        LexToString(RemainingConflict.Type),
                        RemainingConflict.AgentA,
                        RemainingConflict.AgentB);
                    continue;
                }

                HoldConflict = RemainingConflict;
                break;
            }

            if (bHoldConfigurationSafe)
            {
                ApplySafetyGateHold(SafetyGateMissionIds);
                for (const int32 MissionId : SafetyGateMissionIds)
                {
                    RequestedReplanMissionIds.Add(MissionId);
                }

                bool bForceGlobalSafetyGateReplan = false;
                const int32 SafetyGateHoldBudget = FMath::Max(1, FinalSafetyGateMaxHoldSteps);
                for (const int32 MissionId : SafetyGateMissionIds)
                {
                    const FExecutionAgentState* State = ExecutionStates.Find(MissionId);
                    if (State && State->ConsecutiveSafetyGateHoldCount + 1 >= SafetyGateHoldBudget)
                    {
                        bForceGlobalSafetyGateReplan = true;
                        UE_LOG(
                            LogTemp,
                            Warning,
                            TEXT("[FinalSafetyGate] t=%d Mission %d reached safety-gate hold limit %d; upgrade to global replan"),
                            CurrentExecutionTimeStep,
                            MissionId,
                            SafetyGateHoldBudget);
                        break;
                    }
                }

                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("[FinalSafetyGate] t=%d hold fallback is safe for %d missions; trigger %s execution replan"),
                    CurrentExecutionTimeStep,
                    SafetyGateMissionIds.Num(),
                    bForceGlobalSafetyGateReplan ? TEXT("global") : TEXT("configured"));

                bool bSafetyGateReplanSucceeded = false;
                TSet<int32> SafetyGateSuccessfulReplanMissionIds;
                if (ExecutionReplanMode != EExecutionReplanMode::Disabled)
                {
                    const bool bUseGlobalReplan = bForceGlobalSafetyGateReplan || (ExecutionReplanMode == EExecutionReplanMode::GlobalUnfinished);
                    bSafetyGateReplanSucceeded = TryExecutionReplan(SafetyGateMissionIds, bUseGlobalReplan, SafetyGateSuccessfulReplanMissionIds);

                    if (!bSafetyGateReplanSucceeded && !bForceGlobalSafetyGateReplan && ExecutionReplanMode == EExecutionReplanMode::LocalConflictSet)
                    {
                        UE_LOG(
                            LogTemp,
                            Warning,
                            TEXT("[FinalSafetyGate] t=%d local safety-gate replan failed; upgrade to global replan"),
                            CurrentExecutionTimeStep);
                        bSafetyGateReplanSucceeded = TryExecutionReplan(SafetyGateMissionIds, true, SafetyGateSuccessfulReplanMissionIds);
                    }
                }
                else
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("[FinalSafetyGate] t=%d execution replan disabled; committing safe hold only"),
                        CurrentExecutionTimeStep);
                }

                if (bSafetyGateReplanSucceeded)
                {
                    bReplanSucceeded = true;
                    for (const int32 MissionId : SafetyGateSuccessfulReplanMissionIds)
                    {
                        SuccessfulReplanMissionIds.Add(MissionId);
                    }

                    for (const int32 MissionId : MissionIds)
                    {
                        FExecutionStepProposal* Proposal = StepProposals.Find(MissionId);
                        FExecutionAgentState* State = ExecutionStates.Find(MissionId);
                        if (!Proposal || !State || State->PlannedCells.Num() <= 0)
                        {
                            continue;
                        }

                        if (!SafetyGateMissionIds.Contains(MissionId) && !SafetyGateSuccessfulReplanMissionIds.Contains(MissionId))
                        {
                            continue;
                        }

                        Proposal->bValid = true;
                        Proposal->bHeldForReplan = true;
                        Proposal->bHeldForPredictedConflict = false;
                        Proposal->bRequiresReplan = false;
                        Proposal->FinalAction = EDiscreteAlignmentAction::HoldForReplan;
                        Proposal->ReferencePlanIndex = FMath::Clamp(State->ExecutedPlanIndex, 0, State->PlannedCells.Num() - 1);
                        Proposal->ProposedPlanIndex = SafetyGateSuccessfulReplanMissionIds.Contains(MissionId)
                            ? FMath::Min(Proposal->ReferencePlanIndex + 1, State->PlannedCells.Num() - 1)
                            : Proposal->ReferencePlanIndex;
                        Proposal->ProposedCell = Proposal->ObservedCell;
                        Proposal->ResolutionReason = SafetyGateSuccessfulReplanMissionIds.Contains(MissionId)
                            ? TEXT("hold while applying safety-gate replanned trajectory")
                            : TEXT("hold to synchronize with safety-gate replanned agents");
                    }
                    TSet<int32> FinalConflictMissionIds;
                    FPredictedExecutionConflict FinalConflict;
                    if (CollectProposalConflictEndpoints(FinalConflictMissionIds, FinalConflict))
                    {
                        UE_LOG(
                            LogTemp,
                            Error,
                            TEXT("[FinalSafetyGate] t=%d final proposal remains unsafe after safety-gate replan: %s conflict between Mission %d and Mission %d at Cell=(%d,%d,%d); mark execution failed instead of committing unsafe state"),
                            CurrentExecutionTimeStep,
                            LexToString(FinalConflict.Type),
                            FinalConflict.AgentA,
                            FinalConflict.AgentB,
                            FinalConflict.Cell.X,
                            FinalConflict.Cell.Y,
                            FinalConflict.Cell.Z);
                        bStopExecutionForSafetyGate = true;
                    }

                }
                else if (bForceGlobalSafetyGateReplan)
                {
                    UE_LOG(
                        LogTemp,
                        Error,
                        TEXT("[FinalSafetyGate] t=%d global replan failed after safety-gate hold limit; mark execution failed instead of committing unsafe state"),
                        CurrentExecutionTimeStep);
                    bStopExecutionForSafetyGate = true;
                }
            }
            else
            {
                UE_LOG(
                    LogTemp,
                    Error,
                    TEXT("[FinalSafetyGate] t=%d hold fallback remains unsafe: %s conflict between Mission %d and Mission %d at Cell=(%d,%d,%d); dirty-start recovery required but unavailable, mark execution failed"),
                    CurrentExecutionTimeStep,
                    LexToString(HoldConflict.Type),
                    HoldConflict.AgentA,
                    HoldConflict.AgentB,
                    HoldConflict.Cell.X,
                    HoldConflict.Cell.Y,
                    HoldConflict.Cell.Z);
                bStopExecutionForSafetyGate = true;
            }
        }
    }

    if (bStopExecutionForSafetyGate)
    {
        bExecutionRunning = false;
        BuildExecutionSummary();
        if (bLogExecutionSummary)
        {
            LogExecutionSummary();
        }
        return;
    }

    bool bAnyActive = false;

    for (const int32 MissionId : MissionIds)
    {
        FExecutionAgentState* State = ExecutionStates.Find(MissionId);
        FExecutionStepProposal* Proposal = StepProposals.Find(MissionId);
        if (!State || State->PlannedCells.Num() <= 0 || !Proposal)
        {
            continue;
        }

        const bool bReplanRequestedForState =
            RequestedReplanMissionIds.Contains(MissionId) ||
            SuccessfulReplanMissionIds.Contains(MissionId);
        const bool bReplannedForState = SuccessfulReplanMissionIds.Contains(MissionId);

        State->ExecutedPlanIndex = FMath::Clamp(
            Proposal->ProposedPlanIndex,
            0,
            State->PlannedCells.Num() - 1);
        State->DisplayToCell = Proposal->ProposedCell;
        State->LastAlignmentAction = FDiscreteAlignmentManager::LexToString(Proposal->FinalAction);
        State->MaxAlignmentSpatialError = FMath::Max(
            State->MaxAlignmentSpatialError,
            Proposal->AlignmentResult.SpatialErrorCells);
        State->MaxAlignmentTemporalError = FMath::Max(
            State->MaxAlignmentTemporalError,
            FMath::Abs(Proposal->AlignmentResult.TemporalErrorSteps));

        if (Proposal->FinalAction == EDiscreteAlignmentAction::SnapToPlanIndex)
        {
            State->AlignmentSnapCount++;
        }
        else if (Proposal->FinalAction == EDiscreteAlignmentAction::RecoverTowardPlan)
        {
            State->AlignmentCorrectionCount++;
        }
        else if (Proposal->FinalAction == EDiscreteAlignmentAction::HoldForAlignment ||
            Proposal->FinalAction == EDiscreteAlignmentAction::HoldForPredictedConflict ||
            Proposal->FinalAction == EDiscreteAlignmentAction::HoldForSafetyGate ||
            Proposal->FinalAction == EDiscreteAlignmentAction::HoldForReplan)
        {
            State->AlignmentHoldCount++;
        }

        if (Proposal->bHeldForPredictedConflict)
        {
            State->AlignmentConflictHoldCount++;
            State->ConsecutiveConflictHoldCount++;
        }
        else if (Proposal->FinalAction != EDiscreteAlignmentAction::HoldForReplan)
        {
            State->ConsecutiveConflictHoldCount = 0;
        }

        if (Proposal->FinalAction == EDiscreteAlignmentAction::HoldForSafetyGate)
        {
            State->ConsecutiveSafetyGateHoldCount++;
        }
        else if (Proposal->FinalAction != EDiscreteAlignmentAction::HoldForReplan)
        {
            State->ConsecutiveSafetyGateHoldCount = 0;
        }

        if (bReplanRequestedForState)
        {
            State->AlignmentReplanRequestCount++;
        }

        if (bReplannedForState)
        {
            State->AlignmentSuccessfulReplanCount++;
            State->bAlignmentLost = false;
            State->ConsecutiveConflictHoldCount = 0;
            State->ConsecutiveSafetyGateHoldCount = 0;
        }
        else if ((Proposal->bRequiresReplan || Proposal->bInitialAlignmentInvalid || RequestedReplanMissionIds.Contains(MissionId)) &&
            !bReplanSucceeded)
        {
            State->bAlignmentLost = true;
        }

        if (Proposal->bDelayRequested)
        {
            State->TotalDelaySteps++;

            if (bLogExecutionDelay)
            {
                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("[ExecutionDelay] t=%d Mission=%d stay at Cell=(%d,%d,%d)"),
                    CurrentExecutionTimeStep,
                    State->MissionId,
                    Proposal->ObservedCell.X,
                    Proposal->ObservedCell.Y,
                    Proposal->ObservedCell.Z
                );
            }
        }

        if (bLogAlignmentEvents && Proposal->FinalAction != EDiscreteAlignmentAction::FollowPlan)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[Alignment] t=%d Mission=%d Action=%s Observed=(%d,%d,%d) RefIndex=%d RefCell=(%d,%d,%d) NextCell=(%d,%d,%d) SpatialError=%d TemporalError=%d Replan=%s Reason=%s"),
                CurrentExecutionTimeStep,
                State->MissionId,
                FDiscreteAlignmentManager::LexToString(Proposal->FinalAction),
                Proposal->AlignmentResult.ObservedCell.X,
                Proposal->AlignmentResult.ObservedCell.Y,
                Proposal->AlignmentResult.ObservedCell.Z,
                Proposal->ReferencePlanIndex,
                GetCellAtTime(State->PlannedCells, Proposal->ReferencePlanIndex).X,
                GetCellAtTime(State->PlannedCells, Proposal->ReferencePlanIndex).Y,
                GetCellAtTime(State->PlannedCells, Proposal->ReferencePlanIndex).Z,
                Proposal->ProposedCell.X,
                Proposal->ProposedCell.Y,
                Proposal->ProposedCell.Z,
                Proposal->AlignmentResult.SpatialErrorCells,
                Proposal->AlignmentResult.TemporalErrorSteps,
                bReplanRequestedForState ? TEXT("true") : TEXT("false"),
                *Proposal->ResolutionReason);
        }

        State->ActualCells.Add(Proposal->ProposedCell);
        State->bFinished =
            (State->ExecutedPlanIndex >= State->PlannedCells.Num() - 1) &&
            (Proposal->ProposedCell == State->PlannedCells.Last());

        if (!State->bFinished)
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
        Item.AlignmentCorrectionCount = State.AlignmentCorrectionCount;
        Item.AlignmentHoldCount = State.AlignmentHoldCount;
        Item.AlignmentConflictHoldCount = State.AlignmentConflictHoldCount;
        Item.AlignmentSnapCount = State.AlignmentSnapCount;
        Item.AlignmentReplanRequestCount = State.AlignmentReplanRequestCount;
        Item.AlignmentSuccessfulReplanCount = State.AlignmentSuccessfulReplanCount;
        Item.MaxAlignmentSpatialError = State.MaxAlignmentSpatialError;
        Item.MaxAlignmentTemporalError = State.MaxAlignmentTemporalError;
        Item.bAlignmentLost = State.bAlignmentLost;

        if (Item.bReachedGoal)
        {
            LastExecutionSummary.CompletedAgentCount++;
        }

        LastExecutionSummary.PlannedMakespan =
            FMath::Max(LastExecutionSummary.PlannedMakespan, Item.PlannedMakespan);

        LastExecutionSummary.ActualMakespan =
            FMath::Max(LastExecutionSummary.ActualMakespan, Item.ActualMakespan);

        LastExecutionSummary.TotalDelaySteps += Item.TotalDelaySteps;
        LastExecutionSummary.AlignmentCorrectionCount += Item.AlignmentCorrectionCount;
        LastExecutionSummary.AlignmentHoldCount += Item.AlignmentHoldCount;
        LastExecutionSummary.AlignmentConflictHoldCount += Item.AlignmentConflictHoldCount;
        LastExecutionSummary.AlignmentSnapCount += Item.AlignmentSnapCount;
        LastExecutionSummary.AlignmentReplanRequestCount += Item.AlignmentReplanRequestCount;
        LastExecutionSummary.AlignmentSuccessfulReplanCount += Item.AlignmentSuccessfulReplanCount;
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

    TArray<int32> ExecutionMissionIds;
    ExecutionStates.GetKeys(ExecutionMissionIds);
    ExecutionMissionIds.Sort();

    for (int32 TimeStep = 0; TimeStep <= LastExecutionSummary.ActualMakespan; ++TimeStep)
    {
        for (int32 I = 0; I < ExecutionMissionIds.Num(); ++I)
        {
            const int32 MissionIdA = ExecutionMissionIds[I];
            const FExecutionAgentState* StateA = ExecutionStates.Find(MissionIdA);
            const FDroneMissionConfig* ConfigA = ExecutionMissionConfigsByMissionId.Find(MissionIdA);
            if (!StateA || !ConfigA || StateA->ActualCells.Num() <= 0)
            {
                continue;
            }

            const FIntVector CellA = GetCellAtTime(StateA->ActualCells, TimeStep);

            for (int32 J = I + 1; J < ExecutionMissionIds.Num(); ++J)
            {
                const int32 MissionIdB = ExecutionMissionIds[J];
                const FExecutionAgentState* StateB = ExecutionStates.Find(MissionIdB);
                const FDroneMissionConfig* ConfigB = ExecutionMissionConfigsByMissionId.Find(MissionIdB);
                if (!StateB || !ConfigB || StateB->ActualCells.Num() <= 0)
                {
                    continue;
                }

                const FIntVector CellB = GetCellAtTime(StateB->ActualCells, TimeStep);

                const EStaticUTMConflictType UTMConflictType =
                    GetStaticUTMConfigConflictType(CellA, *ConfigA, CellB, *ConfigB);

                if (UTMConflictType == EStaticUTMConflictType::None)
                {
                    continue;
                }

                LastExecutionSummary.UTMStaticConflictCount++;

                if (UTMConflictType == EStaticUTMConflictType::ProtectionFootprint)
                {
                    LastExecutionSummary.UTMProtectionConflictCount++;
                }
                else if (UTMConflictType == EStaticUTMConflictType::Downwash)
                {
                    LastExecutionSummary.UTMDownwashConflictCount++;
                }

                if (LastExecutionSummary.FirstUTMConflictTime < 0 ||
                    TimeStep < LastExecutionSummary.FirstUTMConflictTime)
                {
                    LastExecutionSummary.FirstUTMConflictTime = TimeStep;
                }

                if (LastExecutionSummary.FirstConflictTime < 0 ||
                    TimeStep < LastExecutionSummary.FirstConflictTime)
                {
                    LastExecutionSummary.FirstConflictTime = TimeStep;
                }
            }
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
    UE_LOG(LogTemp, Warning, TEXT("AlignmentEnabled = %s, SearchRadius = %d, MaxSpatialError = %d, MaxSnapAhead = %d, RecoveryMoves = %s, HoldOnFailure = %s"),
        bEnableDiscreteAlignment ? TEXT("true") : TEXT("false"),
        AlignmentSearchRadiusSteps,
        AlignmentMaxSpatialErrorCells,
        AlignmentMaxSnapAheadSteps,
        bAlignmentAllowRecoveryMoves ? TEXT("true") : TEXT("false"),
        bAlignmentHoldPositionOnFailure ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Warning, TEXT("ConflictAwareAlignment = %s, ReplanMode = %s, ResolutionPasses = %d, ConflictHoldThreshold = %d, MaxExecutionReplans = %d"),
        bEnableConflictAwareAlignment ? TEXT("true") : TEXT("false"),
        *UEnum::GetValueAsString(ExecutionReplanMode),
        AlignmentConflictResolutionPasses,
        AlignmentConflictHoldThresholdForReplan,
        MaxExecutionReplanCount);
    UE_LOG(LogTemp, Warning, TEXT("AppliedExecutionReplans = %d"), TotalExecutionReplanCount);

    UE_LOG(LogTemp, Warning, TEXT("AgentCount = %d, CompletedAgentCount = %d"),
        LastExecutionSummary.AgentCount,
        LastExecutionSummary.CompletedAgentCount);

    UE_LOG(LogTemp, Warning, TEXT("PlannedMakespan = %d, ActualMakespan = %d, Expansion = %d"),
        LastExecutionSummary.PlannedMakespan,
        LastExecutionSummary.ActualMakespan,
        LastExecutionSummary.ActualMakespan - LastExecutionSummary.PlannedMakespan);

    UE_LOG(LogTemp, Warning, TEXT("TotalDelaySteps = %d"),
        LastExecutionSummary.TotalDelaySteps);

    UE_LOG(LogTemp, Warning, TEXT("AlignmentCorrectionCount = %d, AlignmentHoldCount = %d, AlignmentConflictHoldCount = %d, AlignmentSnapCount = %d, AlignmentReplanRequestCount = %d, AlignmentSuccessfulReplanCount = %d"),
        LastExecutionSummary.AlignmentCorrectionCount,
        LastExecutionSummary.AlignmentHoldCount,
        LastExecutionSummary.AlignmentConflictHoldCount,
        LastExecutionSummary.AlignmentSnapCount,
        LastExecutionSummary.AlignmentReplanRequestCount,
        LastExecutionSummary.AlignmentSuccessfulReplanCount);

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("VertexConflictCount = %d, EdgeConflictCount = %d, UTMStaticConflictCount = %d "
            "(Protection = %d, Downwash = %d), FirstConflictTime = %d, FirstUTMConflictTime = %d"),
        LastExecutionSummary.VertexConflictCount,
        LastExecutionSummary.EdgeConflictCount,
        LastExecutionSummary.UTMStaticConflictCount,
        LastExecutionSummary.UTMProtectionConflictCount,
        LastExecutionSummary.UTMDownwashConflictCount,
        LastExecutionSummary.FirstConflictTime,
        LastExecutionSummary.FirstUTMConflictTime);

    for (const FExecutionAgentSummary& Item : LastExecutionSummary.AgentSummaries)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("Mission %d | PlannedCells=%d ActualCells=%d | PlannedMakespan=%d ActualMakespan=%d | Delay=%d | FirstMismatch=%d | ReachedGoal=%s | AlignCorrection=%d Hold=%d ConflictHold=%d Snap=%d Replan=%d ReplanSuccess=%d | MaxSpatialError=%d MaxTemporalError=%d | AlignmentLost=%s"),
            Item.MissionId,
            Item.PlannedCellCount,
            Item.ActualCellCount,
            Item.PlannedMakespan,
            Item.ActualMakespan,
            Item.TotalDelaySteps,
            Item.FirstMismatchTime,
            Item.bReachedGoal ? TEXT("true") : TEXT("false"),
            Item.AlignmentCorrectionCount,
            Item.AlignmentHoldCount,
            Item.AlignmentConflictHoldCount,
            Item.AlignmentSnapCount,
            Item.AlignmentReplanRequestCount,
            Item.AlignmentSuccessfulReplanCount,
            Item.MaxAlignmentSpatialError,
            Item.MaxAlignmentTemporalError,
            Item.bAlignmentLost ? TEXT("true") : TEXT("false"));
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

    if (bLogStructuredExperimentJson)
    {
        LogStructuredExperimentSummaryJson();
    }
}

int32 APathPlanningDemoActor::GetEnabledNoFlyZoneCount() const
{
    int32 EnabledZoneCount = 0;

    for (const FTemporalNoFlyZoneConfig& ZoneConfig : NoFlyZoneConfigs)
    {
        if (ZoneConfig.bEnabled)
        {
            EnabledZoneCount++;
        }
    }

    return EnabledZoneCount;
}

FString APathPlanningDemoActor::GetEffectiveExperimentScenarioName() const
{
    const FString TrimmedScenarioName = ExperimentScenarioName.TrimStartAndEnd();
    return TrimmedScenarioName.IsEmpty() ? GetCityLayoutTypeName() : TrimmedScenarioName;
}

FString APathPlanningDemoActor::BuildFallbackExperimentRunId(
    const FString& InPhase,
    const FString& InGroupId,
    const FString& InScenarioName) const
{
    const int32 EffectiveAgentCount =
        LastExecutionSummary.AgentCount > 0 ? LastExecutionSummary.AgentCount : LastPlanningStats.MissionCount;
    const int32 DelayPercent = FMath::Clamp(FMath::RoundToInt(StepDelayProbability * 100.0f), 0, 999);
    const FString SafePhase = InPhase.IsEmpty() ? TEXT("PhaseA") : InPhase;
    const FString SafeGroupId = InGroupId.IsEmpty() ? TEXT("G1") : InGroupId;
    const FString SafeScenarioName = InScenarioName.IsEmpty() ? GetEffectiveExperimentScenarioName() : InScenarioName;

    return FString::Printf(
        TEXT("%s_%s_%s_%s_N%d_P%03d_S%02d"),
        *SanitizeExperimentToken(SafePhase),
        *SanitizeExperimentToken(SafeGroupId),
        *SanitizeExperimentToken(SafeScenarioName),
        *SanitizeExperimentToken(GetPlannerTypeName()),
        EffectiveAgentCount,
        DelayPercent,
        ExecutionRandomSeed);
}


void APathPlanningDemoActor::ResolveExperimentMetadata(
    FString& OutRunId,
    FString& OutPhase,
    FString& OutGroupId,
    FString& OutGroupName,
    FString& OutScenarioName) const
{
    OutRunId = ExperimentRunId.TrimStartAndEnd();
    OutPhase = ExperimentPhase.TrimStartAndEnd();
    OutGroupId = ExperimentGroupId.TrimStartAndEnd();
    OutGroupName = ExperimentGroupName.TrimStartAndEnd();
    OutScenarioName = GetEffectiveExperimentScenarioName();

    TArray<FString> Tokens;
    if (!OutRunId.IsEmpty())
    {
        OutRunId.ParseIntoArray(Tokens, TEXT("_"), true);
    }

	// 1. 优先从 run_id 解析显式信息，一般为空或不规范，但如果符合约定格式则优先使用
    if ((OutPhase.IsEmpty() || !IsKnownExperimentPhase(OutPhase)) &&
        Tokens.Num() > 0 &&
        IsKnownExperimentPhase(Tokens[0]))
    {
        OutPhase = Tokens[0];
    }

    if ((OutGroupId.IsEmpty() || !IsKnownExperimentGroupId(OutGroupId)) &&
        Tokens.Num() > 1 &&
        IsKnownExperimentGroupId(Tokens[1]))
    {
        OutGroupId = Tokens[1];
    }

    // 2. 如果 phase 还不明确，先用 seed 决定 phase
    // 当前实验协议约定：
    //   seed == 1 -> PhaseA
    //   seed == 3 -> PhaseB
    if (OutPhase.IsEmpty() || !IsKnownExperimentPhase(OutPhase))
    {
        const FString SeedPhase = GetExperimentPhaseBySeed(ExecutionRandomSeed);
        if (!SeedPhase.IsEmpty())
        {
            OutPhase = SeedPhase;
        }
    }

    // 3. phase 确定后，再根据配置推 group_id
    if (OutGroupId.IsEmpty() || !IsKnownExperimentGroupId(OutGroupId))
    {
        if (!bEnableDiscreteAlignment && !bEnableConflictAwareAlignment)
        {
            OutGroupId = TEXT("G1");
        }
        else if (bEnableDiscreteAlignment && !bEnableConflictAwareAlignment)
        {
            OutGroupId = TEXT("G2");
        }
        else if (bEnableDiscreteAlignment && bEnableConflictAwareAlignment)
        {
            switch (ExecutionReplanMode)
            {
            case EExecutionReplanMode::Disabled:
                OutGroupId = TEXT("R1");
                break;

            case EExecutionReplanMode::LocalConflictSet:
                OutGroupId = TEXT("R2");
                break;

            case EExecutionReplanMode::GlobalUnfinished:
                OutGroupId = (OutPhase == TEXT("PhaseB")) ? TEXT("R3") : TEXT("G3");
                break;

            default:
                break;
            }
        }
    }

    // 4. 如果 phase 还没定下来，再根据 group_id 兜底
    if (OutPhase.IsEmpty() || !IsKnownExperimentPhase(OutPhase))
    {
        OutPhase = GetDefaultExperimentPhaseByGroupId(OutGroupId);
    }

    // 5. 归一化 G3 / R3 与 phase 的对应关系
    if (OutGroupId == TEXT("G3") && OutPhase == TEXT("PhaseB"))
    {
        OutGroupId = TEXT("R3");
    }
    else if (OutGroupId == TEXT("R3") && OutPhase == TEXT("PhaseA"))
    {
        OutGroupId = TEXT("G3");
    }

    // 6. group_name 按 group_id 统一生成
    if (OutGroupName.IsEmpty() || GetDefaultExperimentGroupNameById(OutGroupId) != OutGroupName)
    {
        OutGroupName = GetDefaultExperimentGroupNameById(OutGroupId);
    }

    // 7. 如果 run_id 为空，自动生成
    if (OutRunId.IsEmpty())
    {
        OutRunId = BuildFallbackExperimentRunId(OutPhase, OutGroupId, OutScenarioName);
    }

    // 8. 可选的一致性告警
    if (OutPhase == TEXT("PhaseA") && ExecutionRandomSeed != 1)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("Experiment metadata mismatch: PhaseA usually expects execution_random_seed=1, got %d"),
            ExecutionRandomSeed);
    }
    else if (OutPhase == TEXT("PhaseB") && ExecutionRandomSeed != 3)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("Experiment metadata mismatch: PhaseB usually expects execution_random_seed=3, got %d"),
            ExecutionRandomSeed);
    }
}

FString APathPlanningDemoActor::BuildStructuredExperimentSummaryJson() const
{
    FString RunId;
    FString Phase;
    FString GroupId;
    FString GroupName;
    FString ScenarioName;
    ResolveExperimentMetadata(RunId, Phase, GroupId, GroupName, ScenarioName);

    const bool bHasExecutionSummary =
        LastExecutionSummary.AgentCount > 0 ||
        LastExecutionSummary.AgentSummaries.Num() > 0 ||
        LastExecutionSummary.CompletedAgentCount > 0 ||
        LastExecutionSummary.PlannedMakespan > 0 ||
        LastExecutionSummary.ActualMakespan > 0 ||
        ExecutionConflicts.Num() > 0;

    const int32 EffectiveAgentCount =
        bHasExecutionSummary ? LastExecutionSummary.AgentCount : LastPlanningStats.MissionCount;
    const int32 Expansion =
        bHasExecutionSummary ? (LastExecutionSummary.ActualMakespan - LastExecutionSummary.PlannedMakespan) : 0;
    const FString PlannerName =
        LastPlanningStats.PlannerName.IsEmpty() ? GetPlannerTypeName() : LastPlanningStats.PlannerName;
    const FString DelayModeName = GetEnumNameString(DelayMode);
    const FString ReplanModeName = GetEnumNameString(ExecutionReplanMode);
    const FString MapTypeName = GetCityLayoutTypeName();
    const bool bNoFlyValidationClear =
        !bValidatePathsAgainstNoFlyZones || LastNoFlyZonePathValidation.TotalViolationCount <= 0;
    const int32 ExecutionReplanAttemptCount =
        ExecutionReplanTimingStats.LocalAttemptCount + ExecutionReplanTimingStats.GlobalAttemptCount;
    const double ExecutionReplanTotalTimeMs =
        ExecutionReplanTimingStats.LocalTotalTimeMs + ExecutionReplanTimingStats.GlobalTotalTimeMs;
    const double ExecutionReplanMaxTimeMs = FMath::Max(
        ExecutionReplanTimingStats.LocalMaxTimeMs,
        ExecutionReplanTimingStats.GlobalMaxTimeMs);

    TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetStringField(TEXT("run_id"), RunId);
    Root->SetStringField(TEXT("phase"), Phase);
    Root->SetStringField(TEXT("group_id"), GroupId);
    Root->SetStringField(TEXT("group_name"), GroupName);
    Root->SetStringField(TEXT("scenario_name"), ScenarioName);
    Root->SetStringField(TEXT("map_type"), MapTypeName);
    Root->SetStringField(TEXT("planner_name"), PlannerName);
    Root->SetStringField(TEXT("planner_type"), PlannerName);
    Root->SetStringField(TEXT("delay_mode"), DelayModeName);
    Root->SetStringField(TEXT("replan_mode"), ReplanModeName);
    Root->SetStringField(TEXT("execution_replan_mode"), ReplanModeName);
    Root->SetStringField(TEXT("notes"), ExperimentNotes);

    Root->SetBoolField(TEXT("planning_success"), LastPlanningStats.bSuccess);
    Root->SetBoolField(TEXT("planning_multi_agent"), LastPlanningStats.bMultiAgent);
    Root->SetBoolField(TEXT("execution_summary_available"), bHasExecutionSummary);
    Root->SetBoolField(TEXT("alignment_enabled"), bEnableDiscreteAlignment);
    Root->SetBoolField(TEXT("b_enable_discrete_alignment"), bEnableDiscreteAlignment);
    Root->SetBoolField(TEXT("conflict_aware_alignment"), bEnableConflictAwareAlignment);
    Root->SetBoolField(TEXT("b_enable_conflict_aware_alignment"), bEnableConflictAwareAlignment);
    Root->SetBoolField(TEXT("b_alignment_allow_recovery_moves"), bAlignmentAllowRecoveryMoves);
    Root->SetBoolField(TEXT("b_alignment_hold_position_on_failure"), bAlignmentHoldPositionOnFailure);
    Root->SetBoolField(TEXT("b_validate_paths_against_no_fly_zones"), bValidatePathsAgainstNoFlyZones);
    Root->SetBoolField(TEXT("no_fly_validation_clear"), bNoFlyValidationClear);

    Root->SetNumberField(TEXT("mission_count"), LastPlanningStats.MissionCount);
    Root->SetNumberField(TEXT("agent_count"), EffectiveAgentCount);
    Root->SetNumberField(TEXT("city_seed"), CitySeed);
    Root->SetNumberField(TEXT("random_seed"), RandomSeed);
    Root->SetNumberField(TEXT("execution_random_seed"), ExecutionRandomSeed);
    Root->SetNumberField(TEXT("step_delay_probability"), StepDelayProbability);
    Root->SetNumberField(TEXT("alignment_search_radius_steps"), AlignmentSearchRadiusSteps);
    Root->SetNumberField(TEXT("alignment_max_spatial_error_cells"), AlignmentMaxSpatialErrorCells);
    Root->SetNumberField(TEXT("alignment_max_snap_ahead_steps"), AlignmentMaxSnapAheadSteps);
    Root->SetNumberField(TEXT("alignment_conflict_resolution_passes"), AlignmentConflictResolutionPasses);
    Root->SetNumberField(TEXT("alignment_conflict_hold_threshold_for_replan"), AlignmentConflictHoldThresholdForReplan);
    Root->SetNumberField(TEXT("max_execution_replans"), MaxExecutionReplanCount);

    Root->SetNumberField(TEXT("planning_build_grid_time_ms"), LastPlanningStats.BuildGridTimeMs);
    Root->SetNumberField(TEXT("planning_input_preparation_time_ms"), LastPlanningStats.InputPreparationTimeMs);
    Root->SetNumberField(TEXT("planning_solve_time_ms"), LastPlanningStats.SolveTimeMs);
    Root->SetNumberField(TEXT("planning_post_process_time_ms"), LastPlanningStats.PostProcessTimeMs);
    Root->SetNumberField(TEXT("initial_planning_wall_time_ms"), LastPlanningStats.TotalTimeMs);

    Root->SetNumberField(TEXT("no_fly_enabled_zone_count"), GetEnabledNoFlyZoneCount());
    Root->SetNumberField(TEXT("no_fly_checked_mission_count"), LastNoFlyZonePathValidation.CheckedMissionCount);
    Root->SetNumberField(TEXT("no_fly_checked_point_count"), LastNoFlyZonePathValidation.CheckedPointCount);
    Root->SetNumberField(TEXT("no_fly_violating_mission_count"), LastNoFlyZonePathValidation.ViolatingMissionCount);
    Root->SetNumberField(TEXT("no_fly_total_violation_count"), LastNoFlyZonePathValidation.TotalViolationCount);

    Root->SetNumberField(TEXT("completed_agent_count"), LastExecutionSummary.CompletedAgentCount);
    Root->SetNumberField(TEXT("planned_makespan"), LastExecutionSummary.PlannedMakespan);
    Root->SetNumberField(TEXT("actual_makespan"), LastExecutionSummary.ActualMakespan);
    Root->SetNumberField(TEXT("expansion"), Expansion);
    Root->SetNumberField(TEXT("total_delay_steps"), LastExecutionSummary.TotalDelaySteps);

    Root->SetNumberField(TEXT("vertex_conflict_count"), LastExecutionSummary.VertexConflictCount);
    Root->SetNumberField(TEXT("edge_conflict_count"), LastExecutionSummary.EdgeConflictCount);
    Root->SetNumberField(TEXT("first_conflict_time"), LastExecutionSummary.FirstConflictTime);

    //utm_static_conflict_count = utm_protection_conflict_count + utm_downwash_conflict_count
    Root->SetNumberField(TEXT("utm_static_conflict_count"), LastExecutionSummary.UTMStaticConflictCount);
    Root->SetNumberField(TEXT("utm_protection_conflict_count"), LastExecutionSummary.UTMProtectionConflictCount);
    Root->SetNumberField(TEXT("utm_downwash_conflict_count"), LastExecutionSummary.UTMDownwashConflictCount);
    Root->SetNumberField(TEXT("first_utm_conflict_time"), LastExecutionSummary.FirstUTMConflictTime);

    Root->SetNumberField(TEXT("alignment_correction_count"), LastExecutionSummary.AlignmentCorrectionCount);
    Root->SetNumberField(TEXT("alignment_hold_count"), LastExecutionSummary.AlignmentHoldCount);
    Root->SetNumberField(TEXT("alignment_conflict_hold_count"), LastExecutionSummary.AlignmentConflictHoldCount);
    Root->SetNumberField(TEXT("alignment_snap_count"), LastExecutionSummary.AlignmentSnapCount);
    Root->SetNumberField(TEXT("alignment_replan_request_count"), LastExecutionSummary.AlignmentReplanRequestCount);
    Root->SetNumberField(TEXT("alignment_successful_replan_count"), LastExecutionSummary.AlignmentSuccessfulReplanCount);
    Root->SetNumberField(TEXT("applied_execution_replans"), TotalExecutionReplanCount);
    Root->SetNumberField(TEXT("execution_replan_attempt_count"), ExecutionReplanAttemptCount);
    Root->SetNumberField(TEXT("execution_replan_total_time_ms"), ExecutionReplanTotalTimeMs);
    Root->SetNumberField(TEXT("execution_replan_max_time_ms"), ExecutionReplanMaxTimeMs);
    Root->SetNumberField(TEXT("execution_replan_local_attempt_count"), ExecutionReplanTimingStats.LocalAttemptCount);
    Root->SetNumberField(TEXT("execution_replan_local_total_time_ms"), ExecutionReplanTimingStats.LocalTotalTimeMs);
    Root->SetNumberField(TEXT("execution_replan_local_max_time_ms"), ExecutionReplanTimingStats.LocalMaxTimeMs);
    Root->SetNumberField(TEXT("execution_replan_global_attempt_count"), ExecutionReplanTimingStats.GlobalAttemptCount);
    Root->SetNumberField(TEXT("execution_replan_global_total_time_ms"), ExecutionReplanTimingStats.GlobalTotalTimeMs);
    Root->SetNumberField(TEXT("execution_replan_global_max_time_ms"), ExecutionReplanTimingStats.GlobalMaxTimeMs);

    FString JsonString;
    const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
        TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&JsonString);

    if (!FJsonSerializer::Serialize(Root, Writer))
    {
        return FString();
    }

    return JsonString;
}

void APathPlanningDemoActor::LogStructuredExperimentSummaryJson() const
{
    const FString JsonString = BuildStructuredExperimentSummaryJson();
    if (JsonString.IsEmpty())
    {
        UE_LOG(LogTemp, Error, TEXT("[StructuredExperimentJSON] Failed to serialize experiment summary JSON"));
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("[StructuredExperimentJSON] %s"), *JsonString);
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
            TEXT("M%d  t=%d\nPlanned=(%d,%d,%d)\nActual=(%d,%d,%d)\nDelay=%d\nAlign=%s"),
            State.MissionId,
            TimeStep,
            PlannedCell.X, PlannedCell.Y, PlannedCell.Z,
            ActualCell.X, ActualCell.Y, ActualCell.Z,
            State.TotalDelaySteps,
            *State.LastAlignmentAction
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

    CacheExecutionMissionConfigs(Missions);

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

    CacheExecutionMissionConfigs(MissionConfigs);

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
    return PlanMultiAgentMissionsOnGrid(GridMap, Missions, OutPaths);
}

bool APathPlanningDemoActor::PlanMultiAgentMissionsOnGrid(
    const FGridMap3D& PlanningGrid,
    const TArray<FDroneMissionConfig>& Missions,
    TMap<int32, TArray<FVector>>& OutPaths) const
{
    const FPlannerRuntimeConfig Config = BuildPlannerRuntimeConfig();
    return FPlannerRegistry::PlanMultiAgentMissions(PlannerType, Config, PlanningGrid, Missions, OutPaths);
}

bool APathPlanningDemoActor::TryExecutionReplan(
    const TSet<int32>& RequestedMissionIds,
    bool bGlobalReplan,
    TSet<int32>& OutReplannedMissionIds)
{
    OutReplannedMissionIds.Reset();

    if (RequestedMissionIds.Num() <= 0)
    {
        return false;
    }

    if (ExecutionReplanMode == EExecutionReplanMode::Disabled)
    {
        return false;
    }

    if (MaxExecutionReplanCount >= 0 && TotalExecutionReplanCount >= MaxExecutionReplanCount)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[AlignmentReplan] skipped because total replan count %d reached limit %d"),
            TotalExecutionReplanCount,
            MaxExecutionReplanCount);
        return false;
    }

    auto IsActiveMission = [&](int32 MissionId) -> bool
        {
            const FExecutionAgentState* State = ExecutionStates.Find(MissionId);
            return State != nullptr && !State->bFinished;
        };

    int32 ActiveRequestedMissionCount = 0;
    for (const int32 MissionId : RequestedMissionIds)
    {
        if (IsActiveMission(MissionId))
        {
            ActiveRequestedMissionCount++;
        }
    }

    auto GetPredictedCellAtOffset = [&](const FExecutionAgentState& State, int32 Offset) -> FIntVector
        {
            if (Offset <= 0 || State.PlannedCells.Num() <= 0)
            {
                return State.LastObservedCell;
            }

            const int32 BaseIndex = FMath::Clamp(State.ExecutedPlanIndex, 0, State.PlannedCells.Num() - 1);
            return GetCellAtTime(State.PlannedCells, BaseIndex + Offset);
        };

    auto GetCellDistance = [](const FIntVector& A, const FIntVector& B) -> int32
        {
            return FMath::Max3(
                FMath::Abs(A.X - B.X),
                FMath::Abs(A.Y - B.Y),
                FMath::Abs(A.Z - B.Z));
        };

    auto HaveStaticUTMCoupling = [&](const FIntVector& CellA, int32 MissionIdA, const FIntVector& CellB, int32 MissionIdB) -> bool
        {
            if (PlannerType != EPlannerType::LaCAMUTM)
            {
                return false;
            }

            const FDroneMissionConfig* MissionConfigA = ExecutionMissionConfigsByMissionId.Find(MissionIdA);
            const FDroneMissionConfig* MissionConfigB = ExecutionMissionConfigsByMissionId.Find(MissionIdB);
            if (!MissionConfigA || !MissionConfigB)
            {
                return false;
            }

            return HasStaticUTMConfigConflict(CellA, *MissionConfigA, CellB, *MissionConfigB);
        };

    auto AreCurrentStartsCoupled = [&](int32 MissionIdA, int32 MissionIdB) -> bool
        {
            if (MissionIdA == MissionIdB)
            {
                return false;
            }

            const FExecutionAgentState* StateA = ExecutionStates.Find(MissionIdA);
            const FExecutionAgentState* StateB = ExecutionStates.Find(MissionIdB);
            if (!StateA || !StateB || StateA->bFinished || StateB->bFinished)
            {
                return false;
            }

            return StateA->LastObservedCell == StateB->LastObservedCell
                || HaveStaticUTMCoupling(StateA->LastObservedCell, MissionIdA, StateB->LastObservedCell, MissionIdB);
        };

    auto AreWithinSpatialExpansion = [&](int32 MissionIdA, int32 MissionIdB, int32 SpatialRadiusCells) -> bool
        {
            if (SpatialRadiusCells <= 0 || MissionIdA == MissionIdB)
            {
                return false;
            }

            const FExecutionAgentState* StateA = ExecutionStates.Find(MissionIdA);
            const FExecutionAgentState* StateB = ExecutionStates.Find(MissionIdB);
            if (!StateA || !StateB || StateA->bFinished || StateB->bFinished)
            {
                return false;
            }

            return GetCellDistance(StateA->LastObservedCell, StateB->LastObservedCell) <= SpatialRadiusCells;
        };

    auto HaveFutureWindowCoupling = [&](int32 MissionIdA, int32 MissionIdB, int32 LookaheadSteps) -> bool
        {
            if (LookaheadSteps <= 0 || MissionIdA == MissionIdB)
            {
                return false;
            }

            const FExecutionAgentState* StateA = ExecutionStates.Find(MissionIdA);
            const FExecutionAgentState* StateB = ExecutionStates.Find(MissionIdB);
            if (!StateA || !StateB || StateA->bFinished || StateB->bFinished)
            {
                return false;
            }

            for (int32 Offset = 0; Offset <= LookaheadSteps; ++Offset)
            {
                const FIntVector CellA = GetPredictedCellAtOffset(*StateA, Offset);
                const FIntVector CellB = GetPredictedCellAtOffset(*StateB, Offset);

                if (CellA == CellB || HaveStaticUTMCoupling(CellA, MissionIdA, CellB, MissionIdB))
                {
                    return true;
                }

                if (Offset > 0)
                {
                    const FIntVector PrevA = GetPredictedCellAtOffset(*StateA, Offset - 1);
                    const FIntVector PrevB = GetPredictedCellAtOffset(*StateB, Offset - 1);
                    if (PrevA == CellB && PrevB == CellA && CellA != CellB)
                    {
                        return true;
                    }
                }
            }

            return false;
        };

    auto BuildCandidateSet = [&](int32 SpatialRadiusCells, int32 LookaheadSteps) -> TSet<int32>
        {
            TSet<int32> CandidateMissionIdSet;

            for (const TPair<int32, FExecutionAgentState>& KVP : ExecutionStates)
            {
                const FExecutionAgentState& State = KVP.Value;
                if (State.bFinished)
                {
                    continue;
                }

                if (bGlobalReplan || RequestedMissionIds.Contains(State.MissionId))
                {
                    CandidateMissionIdSet.Add(State.MissionId);
                }
            }

            if (bGlobalReplan)
            {
                return CandidateMissionIdSet;
            }

            bool bExpandedLocalComponent = true;
            while (bExpandedLocalComponent)
            {
                bExpandedLocalComponent = false;

                for (const TPair<int32, FExecutionAgentState>& KVP : ExecutionStates)
                {
                    const FExecutionAgentState& State = KVP.Value;
                    if (State.bFinished || CandidateMissionIdSet.Contains(State.MissionId))
                    {
                        continue;
                    }

                    bool bCoupledWithCandidate = false;
                    for (const int32 CandidateMissionId : CandidateMissionIdSet)
                    {
                        if (AreCurrentStartsCoupled(CandidateMissionId, State.MissionId)
                            || AreWithinSpatialExpansion(CandidateMissionId, State.MissionId, SpatialRadiusCells)
                            || HaveFutureWindowCoupling(CandidateMissionId, State.MissionId, LookaheadSteps))
                        {
                            bCoupledWithCandidate = true;
                            break;
                        }
                    }

                    if (bCoupledWithCandidate)
                    {
                        CandidateMissionIdSet.Add(State.MissionId);
                        bExpandedLocalComponent = true;
                    }
                }
            }

            return CandidateMissionIdSet;
        };

    auto TryPlanCandidateSet = [&](TSet<int32> CandidateMissionIdSet, int32 AttemptIndex, int32 AttemptCount, int32 SpatialRadiusCells, int32 LookaheadSteps) -> bool
        {
            const double AttemptStartSeconds = FPlatformTime::Seconds();
            ON_SCOPE_EXIT
            {
                const double AttemptTimeMs = (FPlatformTime::Seconds() - AttemptStartSeconds) * 1000.0;
                if (bGlobalReplan)
                {
                    ExecutionReplanTimingStats.GlobalAttemptCount++;
                    ExecutionReplanTimingStats.GlobalTotalTimeMs += AttemptTimeMs;
                    ExecutionReplanTimingStats.GlobalMaxTimeMs = FMath::Max(ExecutionReplanTimingStats.GlobalMaxTimeMs, AttemptTimeMs);
                }
                else
                {
                    ExecutionReplanTimingStats.LocalAttemptCount++;
                    ExecutionReplanTimingStats.LocalTotalTimeMs += AttemptTimeMs;
                    ExecutionReplanTimingStats.LocalMaxTimeMs = FMath::Max(ExecutionReplanTimingStats.LocalMaxTimeMs, AttemptTimeMs);
                }
            };

            const int32 MaxPostCheckTargetedRetries = bGlobalReplan ? 0 : 1;
            int32 PostCheckTargetedRetryCount = 0;
            TSet<int32> ForcedAnchorMissionIdSet;

            while (true)
            {
            TArray<int32> CandidateMissionIds;
            CandidateMissionIds.Reserve(CandidateMissionIdSet.Num());
            for (const int32 CandidateMissionId : CandidateMissionIdSet)
            {
                CandidateMissionIds.Add(CandidateMissionId);
            }

            CandidateMissionIds.Sort();

            if (CandidateMissionIds.Num() <= 0)
            {
                return false;
            }

            const int32 AnchorLookaheadSteps = FMath::Max(0, LookaheadSteps);
            const int32 AnchorSpatialRadiusCells = FMath::Max(0, SpatialRadiusCells);

            auto GetMissionInfluenceRadiusCells = [](const FDroneMissionConfig& Mission) -> int32
                {
                    return FMath::Max3(
                        FMath::Max(Mission.ProtectionXYRadiusCells, Mission.DownwashXYRadiusCells),
                        FMath::Max(Mission.ProtectionZUpCells, Mission.ProtectionZDownCells),
                        Mission.DownwashZBelowCells);
                };

            auto IsAnchorRelevantToCandidates = [&](const FExecutionAgentState& AnchorState, const FDroneMissionConfig& AnchorConfig) -> bool
                {
                    const FIntVector AnchorCell = AnchorState.LastObservedCell;
                    const int32 AnchorInfluenceRadius = GetMissionInfluenceRadiusCells(AnchorConfig);

                    for (const int32 CandidateMissionId : CandidateMissionIds)
                    {
                        const FExecutionAgentState* CandidateState = ExecutionStates.Find(CandidateMissionId);
                        const FDroneMissionConfig* CandidateConfig = ExecutionMissionConfigsByMissionId.Find(CandidateMissionId);
                        if (!CandidateState || !CandidateConfig)
                        {
                            continue;
                        }

                        const int32 EffectiveRadiusCells = AnchorSpatialRadiusCells
                            + AnchorInfluenceRadius
                            + GetMissionInfluenceRadiusCells(*CandidateConfig);

                        auto IsCandidateCellRelevant = [&](const FIntVector& CandidateCell) -> bool
                            {
                                return CandidateCell == AnchorCell
                                    || HaveStaticUTMCoupling(CandidateCell, CandidateMissionId, AnchorCell, AnchorState.MissionId)
                                    || (EffectiveRadiusCells > 0 && GetCellDistance(CandidateCell, AnchorCell) <= EffectiveRadiusCells);
                            };

                        for (int32 Offset = 0; Offset <= AnchorLookaheadSteps; ++Offset)
                        {
                            if (IsCandidateCellRelevant(GetPredictedCellAtOffset(*CandidateState, Offset)))
                            {
                                return true;
                            }
                        }

                        if (IsCandidateCellRelevant(CandidateState->GoalCell))
                        {
                            return true;
                        }
                    }

                    return false;
                };

            TArray<int32> AnchorMissionIds;
            TSet<int32> AnchorMissionIdSet;
            if (PlannerType == EPlannerType::LaCAMUTM)
            {
                AnchorMissionIds.Reserve(ExecutionStates.Num());
                for (const TPair<int32, FExecutionAgentState>& KVP : ExecutionStates)
                {
                    const FExecutionAgentState& State = KVP.Value;
                    if (!State.bFinished || CandidateMissionIdSet.Contains(State.MissionId))
                    {
                        continue;
                    }

                    const FDroneMissionConfig* MissionConfig = ExecutionMissionConfigsByMissionId.Find(State.MissionId);
                    const bool bForcedAnchor = ForcedAnchorMissionIdSet.Contains(State.MissionId);
                    if (!MissionConfig || (!bForcedAnchor && !IsAnchorRelevantToCandidates(State, *MissionConfig)))
                    {
                        continue;
                    }

                    AnchorMissionIds.Add(State.MissionId);
                    AnchorMissionIdSet.Add(State.MissionId);
                }
            }

            AnchorMissionIds.Sort();

            FGridMap3D ReplanGrid = GridMap;

            auto MarkBlockedCell = [&](const FIntVector& Cell)
                {
                    if (!ReplanGrid.IsInside(Cell.X, Cell.Y, Cell.Z) || ReplanGrid.Occupancy.Num() <= 0)
                    {
                        return;
                    }

                    const int32 Index = ReplanGrid.ToIndex(Cell.X, Cell.Y, Cell.Z);
                    if (ReplanGrid.Occupancy.IsValidIndex(Index))
                    {
                        ReplanGrid.Occupancy[Index] = 1;
                    }
                };

            if (!bGlobalReplan)
            {
                for (const TPair<int32, FExecutionAgentState>& KVP : ExecutionStates)
                {
                    const FExecutionAgentState& State = KVP.Value;
                    if (State.bFinished || CandidateMissionIdSet.Contains(State.MissionId))
                    {
                        continue;
                    }

                    MarkBlockedCell(State.LastObservedCell);
                }
            }

            int32 StaticAnchorBlockedCellCount = 0;
            TSet<FIntVector> StaticAnchorBlockedCells;
            auto MarkStaticAnchorFootprint = [&](const FExecutionAgentState& AnchorState, const FDroneMissionConfig& AnchorConfig)
                {
                    const FIntVector AnchorCell = AnchorState.LastObservedCell;
                    const int32 AnchorInfluenceRadius = GetMissionInfluenceRadiusCells(AnchorConfig);

                    for (const int32 CandidateMissionId : CandidateMissionIds)
                    {
                        const FDroneMissionConfig* CandidateConfig = ExecutionMissionConfigsByMissionId.Find(CandidateMissionId);
                        if (!CandidateConfig)
                        {
                            continue;
                        }

                        const int32 SearchRadiusCells = FMath::Max(0, AnchorInfluenceRadius + GetMissionInfluenceRadiusCells(*CandidateConfig));
                        for (int32 Z = AnchorCell.Z - SearchRadiusCells; Z <= AnchorCell.Z + SearchRadiusCells; ++Z)
                        {
                            for (int32 Y = AnchorCell.Y - SearchRadiusCells; Y <= AnchorCell.Y + SearchRadiusCells; ++Y)
                            {
                                for (int32 X = AnchorCell.X - SearchRadiusCells; X <= AnchorCell.X + SearchRadiusCells; ++X)
                                {
                                    const FIntVector CandidateCell(X, Y, Z);
                                    if (!ReplanGrid.IsInside(CandidateCell.X, CandidateCell.Y, CandidateCell.Z))
                                    {
                                        continue;
                                    }

                                    if (!HasStaticUTMConfigConflict(CandidateCell, *CandidateConfig, AnchorCell, AnchorConfig))
                                    {
                                        continue;
                                    }

                                    if (!StaticAnchorBlockedCells.Contains(CandidateCell))
                                    {
                                        StaticAnchorBlockedCells.Add(CandidateCell);
                                        StaticAnchorBlockedCellCount++;
                                    }
                                    MarkBlockedCell(CandidateCell);
                                }
                            }
                        }
                    }
                };

            for (const int32 AnchorMissionId : AnchorMissionIds)
            {
                const FExecutionAgentState* AnchorState = ExecutionStates.Find(AnchorMissionId);
                const FDroneMissionConfig* AnchorConfig = ExecutionMissionConfigsByMissionId.Find(AnchorMissionId);
                if (!AnchorState || !AnchorConfig)
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("[AlignmentReplan] missing static-anchor state or mission config for Mission %d"),
                        AnchorMissionId);
                    return false;
                }

                MarkStaticAnchorFootprint(*AnchorState, *AnchorConfig);
            }

            TArray<FDroneMissionConfig> ReplanMissions;
            ReplanMissions.Reserve(CandidateMissionIds.Num());

            for (const int32 MissionId : CandidateMissionIds)
            {
                const FExecutionAgentState* State = ExecutionStates.Find(MissionId);
                const FDroneMissionConfig* MissionConfig = ExecutionMissionConfigsByMissionId.Find(MissionId);
                if (!State || !MissionConfig)
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("[AlignmentReplan] missing state or mission config for Mission %d"),
                        MissionId);
                    return false;
                }

                FDroneMissionConfig ReplanMission = *MissionConfig;
                ReplanMission.StartWorld = GridMap.CellToWorld(State->LastObservedCell);
                ReplanMission.bStationaryAnchor = false;
                ReplanMissions.Add(ReplanMission);
            }

            TMap<int32, TArray<FVector>> ReplannedWorldPaths;
            const bool bSuccess = PlanMultiAgentMissionsOnGrid(ReplanGrid, ReplanMissions, ReplannedWorldPaths);
            if (!bSuccess)
            {
                if (bGlobalReplan)
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("[AlignmentReplan] global replan failed for %d movable missions (Anchors=%d StaticBlocked=%d)"),
                        CandidateMissionIds.Num(),
                        AnchorMissionIds.Num(),
                        StaticAnchorBlockedCellCount);
                }
                else
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("[AlignmentReplan] local replan attempt %d/%d failed for %d movable missions (Anchors=%d StaticBlocked=%d K=%d W=%d)"),
                        AttemptIndex + 1,
                        AttemptCount,
                        CandidateMissionIds.Num(),
                        AnchorMissionIds.Num(),
                        StaticAnchorBlockedCellCount,
                        SpatialRadiusCells,
                        LookaheadSteps);
                }
                return false;
            }

            TMap<int32, TArray<FIntVector>> ReplannedCellPathsByMission;
            ReplannedCellPathsByMission.Reserve(ReplanMissions.Num());

            for (const FDroneMissionConfig& Mission : ReplanMissions)
            {
                FExecutionAgentState* State = ExecutionStates.Find(Mission.MissionId);
                const bool bStationaryAnchor = AnchorMissionIdSet.Contains(Mission.MissionId);
                const TArray<FVector>* ReplannedWorldPath = ReplannedWorldPaths.Find(Mission.MissionId);
                if (!State || (!bStationaryAnchor && (!ReplannedWorldPath || ReplannedWorldPath->Num() <= 0)))
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("[AlignmentReplan] invalid replanned path for Mission %d"),
                        Mission.MissionId);
                    return false;
                }

                TArray<FIntVector> ReplannedCellPath;
                if (bStationaryAnchor)
                {
                    ReplannedCellPath.Add(State->LastObservedCell);
                }
                else
                {
                    ReplannedCellPath = BuildCellPathFromWorldPath(*ReplannedWorldPath);
                    if (ReplannedCellPath.Num() <= 0)
                    {
                        return false;
                    }

                    if (ReplannedCellPath[0] != State->LastObservedCell)
                    {
                        ReplannedCellPath.Insert(State->LastObservedCell, 0);
                    }
                }

                ReplannedCellPathsByMission.Add(Mission.MissionId, MoveTemp(ReplannedCellPath));
            }

            auto ClampPostCheckCell = [&](const FIntVector& Cell) -> FIntVector
                {
                    return FIntVector(
                        FMath::Clamp(Cell.X, 0, FMath::Max(0, GridMap.GridDim.X - 1)),
                        FMath::Clamp(Cell.Y, 0, FMath::Max(0, GridMap.GridDim.Y - 1)),
                        FMath::Clamp(Cell.Z, 0, FMath::Max(0, GridMap.GridDim.Z - 1)));
                };

            auto GetPostCheckCell = [&](const FExecutionAgentState& State, int32 Offset) -> FIntVector
                {
                    if (const TArray<FIntVector>* ReplannedCellPath = ReplannedCellPathsByMission.Find(State.MissionId))
                    {
                        return ClampPostCheckCell(GetCellAtTime(*ReplannedCellPath, Offset));
                    }

                    if (Offset <= 0 || State.PlannedCells.Num() <= 0)
                    {
                        return State.LastObservedCell;
                    }

                    const int32 BaseIndex = FMath::Clamp(State.ExecutedPlanIndex, 0, State.PlannedCells.Num() - 1);
                    return ClampPostCheckCell(GetCellAtTime(State.PlannedCells, BaseIndex + Offset));
                };

            auto FindPostReplanConflict = [&](FPredictedExecutionConflict& OutConflict, int32& OutOffset) -> bool
                {
                    TArray<int32> ValidationMissionIds;
                    ExecutionStates.GetKeys(ValidationMissionIds);
                    ValidationMissionIds.Sort();

                    const int32 PostCheckLookaheadSteps = FMath::Max(0, LookaheadSteps);
                    for (int32 Offset = 0; Offset <= PostCheckLookaheadSteps; ++Offset)
                    {
                        for (int32 I = 0; I < ValidationMissionIds.Num(); ++I)
                        {
                            const FExecutionAgentState* StateA = ExecutionStates.Find(ValidationMissionIds[I]);
                            if (!StateA || StateA->PlannedCells.Num() <= 0)
                            {
                                continue;
                            }

                            for (int32 J = I + 1; J < ValidationMissionIds.Num(); ++J)
                            {
                                const FExecutionAgentState* StateB = ExecutionStates.Find(ValidationMissionIds[J]);
                                if (!StateB || StateB->PlannedCells.Num() <= 0)
                                {
                                    continue;
                                }

                                if (!CandidateMissionIdSet.Contains(StateA->MissionId)
                                    && !CandidateMissionIdSet.Contains(StateB->MissionId))
                                {
                                    continue;
                                }

                                const FIntVector CellA = GetPostCheckCell(*StateA, Offset);
                                const FIntVector CellB = GetPostCheckCell(*StateB, Offset);

                                if (CellA == CellB)
                                {
                                    OutConflict.Type = EPredictedExecutionConflictType::Vertex;
                                    OutConflict.AgentA = StateA->MissionId;
                                    OutConflict.AgentB = StateB->MissionId;
                                    OutConflict.Cell = CellA;
                                    OutOffset = Offset;
                                    return true;
                                }

                                if (Offset > 0)
                                {
                                    const FIntVector PrevA = GetPostCheckCell(*StateA, Offset - 1);
                                    const FIntVector PrevB = GetPostCheckCell(*StateB, Offset - 1);
                                    const bool bEdgeConflict =
                                        (PrevA == CellB) &&
                                        (PrevB == CellA) &&
                                        (CellA != CellB);

                                    if (bEdgeConflict)
                                    {
                                        OutConflict.Type = EPredictedExecutionConflictType::Edge;
                                        OutConflict.AgentA = StateA->MissionId;
                                        OutConflict.AgentB = StateB->MissionId;
                                        OutConflict.Cell = CellA;
                                        OutOffset = Offset;
                                        return true;
                                    }
                                }

                                if (PlannerType == EPlannerType::LaCAMUTM)
                                {
                                    const FDroneMissionConfig* MissionConfigA = ExecutionMissionConfigsByMissionId.Find(StateA->MissionId);
                                    const FDroneMissionConfig* MissionConfigB = ExecutionMissionConfigsByMissionId.Find(StateB->MissionId);
                                    if (MissionConfigA && MissionConfigB)
                                    {
                                        const EStaticUTMConflictType UTMConflictType = GetStaticUTMConfigConflictType(
                                            CellA,
                                            *MissionConfigA,
                                            CellB,
                                            *MissionConfigB);

                                        if (UTMConflictType != EStaticUTMConflictType::None)
                                        {
                                            OutConflict.Type = (UTMConflictType == EStaticUTMConflictType::ProtectionFootprint)
                                                ? EPredictedExecutionConflictType::ProtectionFootprint
                                                : EPredictedExecutionConflictType::Downwash;
                                            OutConflict.AgentA = StateA->MissionId;
                                            OutConflict.AgentB = StateB->MissionId;
                                            OutConflict.Cell = CellA;
                                            OutOffset = Offset;
                                            return true;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    return false;
                };

            FPredictedExecutionConflict PostCheckConflict;
            int32 PostCheckOffset = 0;
            if (FindPostReplanConflict(PostCheckConflict, PostCheckOffset))
            {
                if (bGlobalReplan)
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("[AlignmentReplan] global replan post-check failed: predicted %s conflict between Mission %d and Mission %d at +%d Cell=(%d,%d,%d) (Movable=%d Anchors=%d StaticBlocked=%d)"),
                        LexToString(PostCheckConflict.Type),
                        PostCheckConflict.AgentA,
                        PostCheckConflict.AgentB,
                        PostCheckOffset,
                        PostCheckConflict.Cell.X,
                        PostCheckConflict.Cell.Y,
                        PostCheckConflict.Cell.Z,
                        CandidateMissionIds.Num(),
                        AnchorMissionIds.Num(),
                        StaticAnchorBlockedCellCount);
                }
                else
                {
                    UE_LOG(
                        LogTemp,
                        Warning,
                        TEXT("[AlignmentReplan] local replan attempt %d/%d post-check failed: predicted %s conflict between Mission %d and Mission %d at +%d Cell=(%d,%d,%d) (Movable=%d Anchors=%d StaticBlocked=%d K=%d W=%d)"),
                        AttemptIndex + 1,
                        AttemptCount,
                        LexToString(PostCheckConflict.Type),
                        PostCheckConflict.AgentA,
                        PostCheckConflict.AgentB,
                        PostCheckOffset,
                        PostCheckConflict.Cell.X,
                        PostCheckConflict.Cell.Y,
                        PostCheckConflict.Cell.Z,
                        CandidateMissionIds.Num(),
                        AnchorMissionIds.Num(),
                        StaticAnchorBlockedCellCount,
                        SpatialRadiusCells,
                        LookaheadSteps);
                }
                const int32 PreviousMovableCount = CandidateMissionIds.Num();
                const int32 PreviousForcedAnchorCount = ForcedAnchorMissionIdSet.Num();
                bool bTargetedRetryExpanded = false;

                auto AddPostCheckRetryMission = [&](int32 MissionId)
                    {
                        if (IsActiveMission(MissionId))
                        {
                            if (!CandidateMissionIdSet.Contains(MissionId))
                            {
                                CandidateMissionIdSet.Add(MissionId);
                                bTargetedRetryExpanded = true;
                            }
                            return;
                        }

                        const FExecutionAgentState* State = ExecutionStates.Find(MissionId);
                        if (PlannerType == EPlannerType::LaCAMUTM
                            && State
                            && State->bFinished
                            && !ForcedAnchorMissionIdSet.Contains(MissionId))
                        {
                            ForcedAnchorMissionIdSet.Add(MissionId);
                            bTargetedRetryExpanded = true;
                        }
                    };

                if (PostCheckTargetedRetryCount < MaxPostCheckTargetedRetries)
                {
                    AddPostCheckRetryMission(PostCheckConflict.AgentA);
                    AddPostCheckRetryMission(PostCheckConflict.AgentB);

                    bool bExpandedTargetedComponent = true;
                    while (bTargetedRetryExpanded && bExpandedTargetedComponent)
                    {
                        bExpandedTargetedComponent = false;
                        for (const TPair<int32, FExecutionAgentState>& KVP : ExecutionStates)
                        {
                            const FExecutionAgentState& State = KVP.Value;
                            if (State.bFinished || CandidateMissionIdSet.Contains(State.MissionId))
                            {
                                continue;
                            }

                            bool bCoupledWithCandidate = false;
                            for (const int32 CandidateMissionId : CandidateMissionIdSet)
                            {
                                if (AreCurrentStartsCoupled(CandidateMissionId, State.MissionId)
                                    || AreWithinSpatialExpansion(CandidateMissionId, State.MissionId, SpatialRadiusCells)
                                    || HaveFutureWindowCoupling(CandidateMissionId, State.MissionId, LookaheadSteps))
                                {
                                    bCoupledWithCandidate = true;
                                    break;
                                }
                            }

                            if (bCoupledWithCandidate)
                            {
                                CandidateMissionIdSet.Add(State.MissionId);
                                bExpandedTargetedComponent = true;
                            }
                        }
                    }

                    if (bTargetedRetryExpanded)
                    {
                        PostCheckTargetedRetryCount++;
                        UE_LOG(
                            LogTemp,
                            Warning,
                            TEXT("[AlignmentReplan] local post-check targeted retry %d/%d after %s conflict between Mission %d and Mission %d: movable %d->%d forced anchors %d->%d (K=%d W=%d)"),
                            PostCheckTargetedRetryCount,
                            MaxPostCheckTargetedRetries,
                            LexToString(PostCheckConflict.Type),
                            PostCheckConflict.AgentA,
                            PostCheckConflict.AgentB,
                            PreviousMovableCount,
                            CandidateMissionIdSet.Num(),
                            PreviousForcedAnchorCount,
                            ForcedAnchorMissionIdSet.Num(),
                            SpatialRadiusCells,
                            LookaheadSteps);
                        continue;
                    }
                }

                return false;
            }

            for (const int32 MissionId : CandidateMissionIds)
            {
                FExecutionAgentState* State = ExecutionStates.Find(MissionId);
                const FDroneMissionConfig* MissionConfig = ExecutionMissionConfigsByMissionId.Find(MissionId);
                const TArray<FIntVector>* ReplannedCellPath = ReplannedCellPathsByMission.Find(MissionId);
                if (!State || !MissionConfig || !ReplannedCellPath || ReplannedCellPath->Num() <= 0)
                {
                    return false;
                }

                TArray<FIntVector> TimelineCells = State->ActualCells;
                if (TimelineCells.Num() <= 0 || TimelineCells.Last() != State->LastObservedCell)
                {
                    TimelineCells.Add(State->LastObservedCell);
                }

                // Reserve the current execution step for the replan hold itself.
                TimelineCells.Add(State->LastObservedCell);

                for (int32 Index = 1; Index < ReplannedCellPath->Num(); ++Index)
                {
                    TimelineCells.Add((*ReplannedCellPath)[Index]);
                }

                TArray<FVector> TimelineWorld;
                TimelineWorld.Reserve(TimelineCells.Num());
                for (const FIntVector& Cell : TimelineCells)
                {
                    TimelineWorld.Add(GridMap.CellToWorld(Cell));
                }

                State->PlannedCells = TimelineCells;
                State->ExecutedPlanIndex = FMath::Max(0, State->ActualCells.Num() - 1);
                State->GoalCell = ReplannedCellPath->Last();
                State->GoalWorld = MissionConfig->GoalWorld;
                State->ConsecutiveConflictHoldCount = 0;
                State->bAlignmentLost = false;

                PlannedCellPathsByMission.Add(MissionId, TimelineCells);
                LastPlannedPathsByMission.Add(MissionId, TimelineWorld);
                OutReplannedMissionIds.Add(MissionId);
            }

            TotalExecutionReplanCount++;

            if (bGlobalReplan)
            {
                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("[AlignmentReplan] global replan succeeded for %d movable missions at t=%d (Total=%d, Anchors=%d StaticBlocked=%d)"),
                    OutReplannedMissionIds.Num(),
                    CurrentExecutionTimeStep,
                    TotalExecutionReplanCount,
                    AnchorMissionIds.Num(),
                    StaticAnchorBlockedCellCount);
            }
            else
            {
                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT("[AlignmentReplan] local replan succeeded for %d movable missions at t=%d (Total=%d, Attempt=%d/%d, Anchors=%d StaticBlocked=%d K=%d W=%d)"),
                    OutReplannedMissionIds.Num(),
                    CurrentExecutionTimeStep,
                    TotalExecutionReplanCount,
                    AttemptIndex + 1,
                    AttemptCount,
                    AnchorMissionIds.Num(),
                    StaticAnchorBlockedCellCount,
                    SpatialRadiusCells,
                    LookaheadSteps);
            }

            return OutReplannedMissionIds.Num() > 0;
            }
        };

    const int32 AttemptCount = bGlobalReplan
        ? 1
        : FMath::Max(1, LocalReplanMaxExpansionRounds);
    const int32 BaseSpatialRadiusCells = FMath::Max(0, LocalReplanSpatialExpansionRadiusCells);
    const int32 BaseLookaheadSteps = FMath::Max(0, LocalReplanLookaheadSteps);

    for (int32 AttemptIndex = 0; AttemptIndex < AttemptCount; ++AttemptIndex)
    {
        const int32 SpatialRadiusCells = bGlobalReplan
            ? BaseSpatialRadiusCells
            : BaseSpatialRadiusCells * (AttemptIndex + 1);
        const int32 LookaheadSteps = bGlobalReplan
            ? BaseLookaheadSteps
            : BaseLookaheadSteps * (AttemptIndex + 1);

        TSet<int32> CandidateMissionIdSet = BuildCandidateSet(SpatialRadiusCells, LookaheadSteps);

        if (!bGlobalReplan && CandidateMissionIdSet.Num() > ActiveRequestedMissionCount)
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[AlignmentReplan] local conflict component attempt %d/%d expanded from %d to %d missions (K=%d W=%d)"),
                AttemptIndex + 1,
                AttemptCount,
                ActiveRequestedMissionCount,
                CandidateMissionIdSet.Num(),
                SpatialRadiusCells,
                LookaheadSteps);
        }

        if (TryPlanCandidateSet(CandidateMissionIdSet, AttemptIndex, AttemptCount, SpatialRadiusCells, LookaheadSteps))
        {
            return true;
        }

        OutReplannedMissionIds.Reset();
    }

    return false;
}
bool APathPlanningDemoActor::IsMultiAgentPlannerType() const
{
    return FPlannerRegistry::IsMultiAgentPlannerType(PlannerType);
}

FPlannerRuntimeConfig APathPlanningDemoActor::BuildPlannerRuntimeConfig() const
{
    FPlannerRuntimeConfig Config;
    Config.ECBSSuboptimalityBound = ECBSSuboptimalityBound;
    Config.LaCAMTimeLimitMs = LaCAMTimeLimitMs;
    Config.LaCAMRandomSeed = LaCAMRandomSeed;
    Config.bLaCAMAnytime = bLaCAMAnytime;
    Config.LaCAMVerboseLevel = LaCAMVerboseLevel;
    Config.NoFlyZoneConfigs = NoFlyZoneConfigs;
    return Config;
}

TUniquePtr<IPathPlannerBase> APathPlanningDemoActor::CreatePlannerByType() const
{
    return FPlannerRegistry::CreateSingleAgentPlanner(PlannerType, BuildPlannerRuntimeConfig());
}

FString APathPlanningDemoActor::GetPlannerTypeName() const
{
    return FPlannerRegistry::GetPlannerTypeName(PlannerType);
}

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

    const auto HasConflictWithExistingStarts =
        [this](const FIntVector& CandidateStartCell, const FDroneMissionConfig& CandidateMission)
        {
            for (const FDroneMissionConfig& ExistingMission : MissionConfigs)
            {
                const FIntVector ExistingStartCell = GridMap.WorldToCell(ExistingMission.StartWorld);
                if (HasStaticUTMConfigConflict(CandidateStartCell, CandidateMission, ExistingStartCell, ExistingMission))
                {
                    return true;
                }
            }
            return false;
        };

    const auto HasConflictWithExistingGoals =
        [this](const FIntVector& CandidateGoalCell, const FDroneMissionConfig& CandidateMission)
        {
            for (const FDroneMissionConfig& ExistingMission : MissionConfigs)
            {
                const FIntVector ExistingGoalCell = GridMap.WorldToCell(ExistingMission.GoalWorld);
                if (HasStaticUTMConfigConflict(CandidateGoalCell, CandidateMission, ExistingGoalCell, ExistingMission))
                {
                    return true;
                }
            }
            return false;
        };

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

            FDroneMissionConfig CandidateMission = OutMission;
            CandidateMission.MissionId = MissionId;
            CandidateMission.StartWorld = GridMap.CellToWorld(StartCell);
            CandidateMission.GoalWorld = GridMap.CellToWorld(GoalCell);

            if (HasConflictWithExistingStarts(StartCell, CandidateMission))
            {
                continue;
            }

            if (HasConflictWithExistingGoals(GoalCell, CandidateMission))
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
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("Failed to generate mission %d under current random-space and UTM safety constraints. Consider reducing RandomMissionCount or shrinking mission footprints."),
                MissionId);
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

    CityLayoutType = ECityLayoutType::Manhattan;

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

    CityLayoutType = ECityLayoutType::Residential;

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

    CityLayoutType = ECityLayoutType::Industrial;

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

    CityLayoutType = ECityLayoutType::Mixed;

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

