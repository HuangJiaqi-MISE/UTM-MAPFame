#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Planning/GridMap3D.h"
#include "Planning/AStarPlanner.h"
#include "Planning/DStarLitePlanner.h"
#include "Planning/PathPlannerBase.h"
#include "Planning/PlanningInputValidator.h"
#include "Planning/DroneMissionTypes.h"
#include "Planning/TemporalNoFlyZoneTypes.h"
#include "PathPlanningDemoActor.generated.h"

class USceneComponent;
class ADroneActor;
class AMissionMarkerActor;
class ANoFlyZoneMarkerActor;
class AStaticMeshActor;

UENUM(BlueprintType)
enum class EPlannerType : uint8
{
    AStar      UMETA(DisplayName = "A*"),
    SIPP       UMETA(DisplayName = "SIPP"),
    DStarLite  UMETA(DisplayName = "D* Lite"),
    JPS        UMETA(DisplayName = "JPS"),
    CBS        UMETA(DisplayName = "CBS"),
    ECBS       UMETA(DisplayName = "ECBS"),
    PBS        UMETA(DisplayName = "PBS"),
    LaCAM      UMETA(DisplayName = "LaCAM"),
    LaCAMUTM   UMETA(DisplayName = "LaCAM-UTM")
};

/*
模式 A：全局随机延迟,适合快速看效果。
模式 B：指定 agent 延迟,比如只让 Mission 2 延迟,能明确观察单个执行异常如何破坏整体 schedule。
模式 C：指定 timestep 延迟,比如让 Agent 2 在 t=4, 7, 8 原地等待,让实验可复现、可解释。
*/
UENUM(BlueprintType)
enum class EExecutionDelayMode : uint8
{
    RandomGlobal        UMETA(DisplayName = "Random Global"),
    PerAgentProbability UMETA(DisplayName = "Per-Agent Probability"),
    ScriptedTimesteps   UMETA(DisplayName = "Scripted Timesteps")
};

UENUM(BlueprintType)
enum class ECityLayoutType : uint8
{
    Manhattan,
    Residential,
    Industrial,
    Mixed
};

// 用于记录单个任务的规划统计数据
USTRUCT(BlueprintType)
struct FSingleMissionTimingStats
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    int32 MissionId = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    bool bSuccess = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    int32 PathPointCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double SolveTimeMs = 0.0;
};

// 用于记录整体规划过程的统计数据，包括每个任务的统计
USTRUCT(BlueprintType)
struct FPlanningTimingStats
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    FString PlannerName;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    bool bMultiAgent = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    bool bSuccess = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    int32 MissionCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double TotalTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double BuildGridTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double InputPreparationTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double SolveTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    double PostProcessTimeMs = 0.0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
    TArray<FSingleMissionTimingStats> MissionStats;
};

USTRUCT(BlueprintType)
struct FNoFlyZonePathViolation
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "No-Fly Zone Validation")
    int32 MissionId = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "No-Fly Zone Validation")
    int32 ZoneId = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "No-Fly Zone Validation")
    int32 TimeStep = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "No-Fly Zone Validation")
    FIntVector Cell = FIntVector::ZeroValue;
};

USTRUCT(BlueprintType)
struct FNoFlyZonePathValidationSummary
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "No-Fly Zone Validation")
    int32 CheckedMissionCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "No-Fly Zone Validation")
    int32 CheckedPointCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "No-Fly Zone Validation")
    int32 ViolatingMissionCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "No-Fly Zone Validation")
    int32 TotalViolationCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "No-Fly Zone Validation")
    TArray<FNoFlyZonePathViolation> Violations;
};

USTRUCT(BlueprintType)
struct FExecutionAgentState
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 MissionId = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    TObjectPtr<ADroneActor> Drone = nullptr;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    TArray<FIntVector> PlannedCells;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    TArray<FIntVector> ActualCells;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 ExecutedPlanIndex = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 TotalDelaySteps = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    bool bFinished = false;

    FIntVector DisplayFromCell = FIntVector::ZeroValue;
    FIntVector DisplayToCell = FIntVector::ZeroValue;
};

USTRUCT(BlueprintType)
struct FExecutionConflict
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 TimeStep = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 AgentA = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    int32 AgentB = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    bool bIsEdgeConflict = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    FIntVector Cell = FIntVector::ZeroValue;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    FIntVector FromA = FIntVector::ZeroValue;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    FIntVector ToA = FIntVector::ZeroValue;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    FIntVector FromB = FIntVector::ZeroValue;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution")
    FIntVector ToB = FIntVector::ZeroValue;
};

USTRUCT(BlueprintType)
struct FAgentDelayConfig
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution")
    int32 MissionId = INDEX_NONE;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float DelayProbability = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution")
    TArray<int32> ForcedDelaySteps;
};

USTRUCT(BlueprintType)
struct FExecutionAgentSummary
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 MissionId = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 PlannedCellCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 ActualCellCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 PlannedMakespan = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 ActualMakespan = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 TotalDelaySteps = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 FirstMismatchTime = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    bool bReachedGoal = false;
};

USTRUCT(BlueprintType)
struct FExecutionSummary
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 AgentCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 CompletedAgentCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 PlannedMakespan = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 ActualMakespan = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 TotalDelaySteps = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 VertexConflictCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 EdgeConflictCount = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    int32 FirstConflictTime = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    TArray<FExecutionAgentSummary> AgentSummaries;
};


UCLASS()
class UTM_API APathPlanningDemoActor : public AActor
{
    GENERATED_BODY()

public:
    APathPlanningDemoActor();

protected:
    virtual void BeginPlay() override;

public:
    virtual void Tick(float DeltaTime) override;

private:
    bool ParseTaggedId(AActor* Actor, const FString& Prefix, int32& OutId) const;

    void CollectStartGoalPairs(
        TArray<int32>& OutIds,
        TMap<int32, TObjectPtr<AActor>>& OutStarts,
        TMap<int32, TObjectPtr<AActor>>& OutGoals) const;

    void LogPathCoordinates(const TArray<FVector>& InPath, int32 Id, const TCHAR* Label) const;
    void DrawPathDebug(const TArray<FVector>& InPath, const FColor& Color) const;
    FColor GetDebugColorById(int32 Id) const;
    bool IsCellInsideNoFlyZoneAtTime(const FIntVector& Cell, int32 TimeStep, const FTemporalNoFlyZoneConfig& ZoneConfig) const;
    void ValidatePathAgainstNoFlyZones(int32 MissionId, const TArray<FVector>& PathPoints, FNoFlyZonePathValidationSummary& InOutSummary) const;
    void LogNoFlyZonePathValidationSummary(const FNoFlyZonePathValidationSummary& Summary) const;
    void CachePlannedPath(int32 MissionId, const TArray<FVector>& PathPoints);
    void ResetPathValidationCache();

private:
    TArray<FIntVector> BuildCellPathFromWorldPath(const TArray<FVector>& PathPoints) const;
    FIntVector GetCellAtTime(const TArray<FIntVector>& Cells, int32 TimeStep) const;

    void ResetExecutionCache();
    void InitializeExecutionStates();
    void AdvanceExecutionOneStep();
    void UpdateExecutionVisuals(float Alpha);
    bool ShouldDelayThisStep(const FExecutionAgentState& State, int32 TimeStep);
    void DetectExecutionConflictsAtStep(int32 TimeStep);
    void DrawExecutionDebugForState(const FExecutionAgentState& State, int32 TimeStep) const;
    const FAgentDelayConfig* FindAgentDelayConfig(int32 MissionId) const;
    bool IsForcedDelayStep(const FExecutionAgentState& State, int32 TimeStep) const;
    int32 ComputeFirstMismatchTime(const FExecutionAgentState& State) const;
    void BuildExecutionSummary();
    void LogExecutionSummary() const;

private:
    FGridMap3D GridMap;
    FPlanningInputValidator InputValidator;
    TMap<int32, TArray<FVector>> LastPlannedPathsByMission;

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planning")
    bool bAutoRunPlanningOnBeginPlay = false;

    UFUNCTION(BlueprintCallable, Category = "Planning")
    void RunPlanning();

    UFUNCTION(BlueprintCallable, Category = "No-Fly Zone Validation")
    void ValidateLastPlannedPathsAgainstNoFlyZones();



    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
    FVector GridOrigin = FVector(0.f, 0.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
    FIntVector GridDim = FIntVector(100, 100, 10);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
    float CellSize = 100.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid Debug")
    bool bDrawOccupiedCells = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid Debug")
    bool bDrawFreeCells = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid Debug")
    float DebugDrawTime = 10.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Debug")
    bool bDrawPaths = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Debug")
    bool bDrawPathPoints = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Debug")
    float PathDrawTime = 20.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Debug")
    float PathLineThickness = 5.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Debug")
    bool bLogPathCoordinates = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Validation")
    bool bValidatePathsAgainstNoFlyZones = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Validation", meta = (ClampMin = "0"))
    int32 MaxLoggedNoFlyZoneViolations = 20;

    // 无人机蓝图自动挂载
protected:
    ADroneActor* SpawnDroneForPath(const TArray<FVector>& PathPoints, int32 PairId);

private:
    TArray<TObjectPtr<ADroneActor>> SpawnedDrones;

    TMap<int32, TObjectPtr<ADroneActor>> SpawnedDroneByMissionId;
    TMap<int32, TArray<FIntVector>> PlannedCellPathsByMission;
    TMap<int32, FExecutionAgentState> ExecutionStates;
    TArray<FExecutionConflict> ExecutionConflicts;

    FRandomStream ExecutionRandom;
    bool bExecutionRunning = false;
    int32 CurrentExecutionTimeStep = 0;
    float ExecutionAccumulator = 0.f;

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Spawn")
    TSubclassOf<ADroneActor> DroneClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Spawn")
    bool bAutoSpawnDrones = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Spawn", meta = (ClampMin = "0.01"))
    float CBSStepDuration = 0.333f;

    // 执行器构建
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution")
    bool bUseCentralizedExecution = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution")
    EExecutionDelayMode DelayMode = EExecutionDelayMode::RandomGlobal;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution")
    TArray<FAgentDelayConfig> AgentDelayConfigs;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution")
    bool bLogExecutionSummary = true;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Execution Summary")
    FExecutionSummary LastExecutionSummary;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float StepDelayProbability = 0.20f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution")
    int32 ExecutionRandomSeed = 12345;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution Debug")
    bool bDrawExecutionCells = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution Debug")
    bool bDrawExecutionText = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Execution Debug")
    float ExecutionDebugDrawTime = 0.4f;

    // 无人机数量+序号配置
    // 如果不使用 MissionConfigs，则从场景里手动配置 Start_i / Goal_i 来指定任务数量和起终点
protected:
    bool ProcessMissionConfigs();
    bool ProcessStartGoalPairsMultiAgent();
    bool ProcessMissionConfigsMultiAgent();
    bool PlanMultiAgentMissions(const TArray<FDroneMissionConfig>& Missions, TMap<int32, TArray<FVector>>& OutPaths) const;
    bool IsMultiAgentPlannerType() const;
public:
    /*
    false：继续用场景里手动配置的 Start_i / Goal_i
    true：改用 MissionConfigs 从界面直接配置
    */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission Config")
    bool bUseMissionConfigs = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission Config")
    TArray<FDroneMissionConfig> MissionConfigs;

    // 算法选择
private:
    FString GetPlannerTypeName() const;
    TUniquePtr<IPathPlannerBase> CreatePlannerByType() const;
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planner")
    EPlannerType PlannerType = EPlannerType::AStar;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planner", meta = (ClampMin = "1.0"))
    float ECBSSuboptimalityBound = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planner", meta = (ClampMin = "1"))
    int32 LaCAMTimeLimitMs = 8000;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planner")
    int32 LaCAMRandomSeed = 12345;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planner")
    bool bLaCAMAnytime = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planner", meta = (ClampMin = "0"))
    int32 LaCAMVerboseLevel = 0;


    // UI导入起点、终点配置
private:
    bool TryGenerateSingleMission(
        FRandomStream& RandomStream,
        int32 MissionId,
        TSet<FIntVector>& UsedStarts,
        TSet<FIntVector>& UsedGoals,
        FDroneMissionConfig& OutMission) const;

    void GetMissionMarkerActors(TArray<AMissionMarkerActor*>& OutMarkers) const;
    void GetNoFlyZoneMarkerActors(TArray<ANoFlyZoneMarkerActor*>& OutMarkers) const;
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission Editor")
    TSubclassOf<AMissionMarkerActor> MissionMarkerClass;

    // 是纯显示/编辑用途的高度偏移,marker 会浮得更高，更好看见
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission Editor")
    float MarkerZOffset = 30.f;

    // Generate Random Missions 生成数量
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission Editor")
    int32 RandomMissionCount = 10;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission Editor")
    int32 RandomSeed = 12345;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission Editor")
    int32 MinStartGoalCellDistance = 3;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission Editor")
    bool bAllowDuplicateStartCells = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission Editor")
    bool bAllowDuplicateGoalCells = false;

    UFUNCTION(BlueprintCallable, Category = "Mission Editor")
    void EditorBuildGridForMissionEditing();

    UFUNCTION(BlueprintCallable, Category = "Mission Editor")
    void EditorGenerateRandomMissionConfigs();

    UFUNCTION(BlueprintCallable, Category = "Mission Editor")
    void EditorSpawnMissionMarkers();

    UFUNCTION(BlueprintCallable, Category = "Mission Editor")
    void EditorClearMissionMarkers();

    UFUNCTION(BlueprintCallable, Category = "Mission Editor")
    void EditorValidateMissionConfigs();

    UFUNCTION(BlueprintCallable, Category = "Mission Editor")
    void EditorReadMissionMarkersToConfigs();

	// UI编辑临时禁飞区配置
    // 保存当前所有禁飞区配置
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor")
    TArray<FTemporalNoFlyZoneConfig> NoFlyZoneConfigs;

    // 指定场景里生成禁飞区盒子时用哪个蓝图类
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor")
    TSubclassOf<ANoFlyZoneMarkerActor> NoFlyZoneMarkerClass;

    // 默认生成多大的禁飞区,单位是 cell 栅格化坐标，不是世界坐标
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor", meta = (ClampMin = "1"))
    int32 DefaultNoFlyZoneSizeCells = 3;

    // 默认时间窗持续多久，例如默认 10，就会生成类似：StartTimeStep = 0    EndTimeStep = 9
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor", meta = (ClampMin = "1"))
    int32 DefaultNoFlyZoneDuration = 10;

    // 一次随机生成多少个禁飞区
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor", meta = (ClampMin = "0"))
    int32 RandomNoFlyZoneCount = 5;

    // 同样的种子 + 同样的参数，理论上会生成同一批禁飞区配置适合做可复现实验
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor")
    int32 NoFlyZoneRandomSeed = 12345;

    // 随机禁飞区的最小平面尺寸。这里主要控制 XY 尺寸下界。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor", meta = (ClampMin = "1"))
    int32 RandomNoFlyZoneMinSizeCells = 2;

    // 随机禁飞区的最大平面尺寸。这里主要控制 XY 尺寸上界。举例：MinSize = 2   MaxSize = 6  表示禁飞区的 X / Y 尺寸会在 2 到 6 个 cell 之间随机。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor", meta = (ClampMin = "1"))
    int32 RandomNoFlyZoneMaxSizeCells = 6;

    // 禁飞区最早从哪个时间步开始生效
	// 举例：MinStartTimeStep = 0   MaxStartTimeStep = 50  表示禁飞区的开始时间会在 0 到 50 个时间步之间随机。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor", meta = (ClampMin = "0"))
    int32 RandomNoFlyZoneMinStartTimeStep = 0;

    // 禁飞区最晚从哪个时间步开始生效
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor", meta = (ClampMin = "0"))
    int32 RandomNoFlyZoneMaxStartTimeStep = 5;

    // 禁飞区时间窗的最短持续时间
    // 举例： MinDuration = 10   MaxDuration = 30  表示每个禁飞区会持续 10 到 30 个时间步。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor", meta = (ClampMin = "1"))
    int32 RandomNoFlyZoneMinDuration = 5;

    // 禁飞区时间窗的最长持续时间
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "No-Fly Zone Editor", meta = (ClampMin = "1"))
    int32 RandomNoFlyZoneMaxDuration = 15;

    UFUNCTION(BlueprintCallable, Category = "No-Fly Zone Editor")
    void EditorGenerateRandomNoFlyZoneConfigs();

    UFUNCTION(BlueprintCallable, Category = "No-Fly Zone Editor")
    void EditorAddNoFlyZoneConfig();

    UFUNCTION(BlueprintCallable, Category = "No-Fly Zone Editor")
    void EditorSpawnNoFlyZoneMarkers();

    UFUNCTION(BlueprintCallable, Category = "No-Fly Zone Editor")
    void EditorClearNoFlyZoneMarkers();

    UFUNCTION(BlueprintCallable, Category = "No-Fly Zone Editor")
    void EditorReadNoFlyZoneMarkersToConfigs();

    UFUNCTION(BlueprintCallable, Category = "No-Fly Zone Editor")
    void EditorValidateNoFlyZones();





// 静态障碍物生成与编辑
private:
    void SpawnCityBuilding(const FVector& Center, const FVector& Extent);
    void GenerateCityLayout_Manhattan(FRandomStream& RandomStream);
    void GenerateCityLayout_Residential(FRandomStream& RandomStream);
    void GenerateCityLayout_Industrial(FRandomStream& RandomStream);
    void GenerateCityLayout_Mixed(FRandomStream& RandomStream);


public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator")
    int32 CitySeed = 12345;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator", meta = (ClampMin = "1"))
    int32 CityBlocksX = 10;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator", meta = (ClampMin = "1"))
    int32 CityBlocksY = 10;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator", meta = (ClampMin = "10.0"))
    float CityBlockSize = 800.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator", meta = (ClampMin = "0.0"))
    float CityRoadWidth = 200.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator", meta = (ClampMin = "10.0"))
    float BuildingWidthMin = 200.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator", meta = (ClampMin = "10.0"))
    float BuildingWidthMax = 500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator", meta = (ClampMin = "10.0"))
    float BuildingDepthMin = 200.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator", meta = (ClampMin = "10.0"))
    float BuildingDepthMax = 500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator", meta = (ClampMin = "10.0"))
    float BuildingHeightMin = 600.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator", meta = (ClampMin = "10.0"))
    float BuildingHeightMax = 1200.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City Generator")
    bool bAutoRebuildGridAfterCityGenerate = true;

    UFUNCTION(BlueprintCallable, Category = "City Generator")
    void EditorGenerateManhattanCity();

    UFUNCTION(BlueprintCallable, Category = "City Generator")
    void EditorGenerateResidentialDistrict();

    UFUNCTION(BlueprintCallable, Category = "City Generator")
    void EditorGenerateIndustrialPark();

    UFUNCTION(BlueprintCallable, Category = "City Generator")
    void EditorGenerateMixedUrbanArea();


    UFUNCTION(BlueprintCallable, Category = "City Generator")
    void EditorClearCityEnvironment();

//  统计求解时间等数据
public:
        UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Timing")
        FPlanningTimingStats LastPlanningStats;

        UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "No-Fly Zone Validation")
        FNoFlyZonePathValidationSummary LastNoFlyZonePathValidation;

        UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Timing")
        bool bEnablePlanningTimeLog = true;

        UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "City")
        ECityLayoutType CityLayoutType = ECityLayoutType::Manhattan;

        bool ProcessStartGoalPairsSingleAgent(
            const TArray<int32>& Ids,
            const TMap<int32, TObjectPtr<AActor>>& Starts,
            const TMap<int32, TObjectPtr<AActor>>& Goals);

private:
    void ResetPlanningStats();
    void LogPlanningStatsSummary() const;
    FString GetCityLayoutTypeName() const;
};









