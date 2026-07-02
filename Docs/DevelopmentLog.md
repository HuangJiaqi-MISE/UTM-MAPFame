# UTM-MAPFame 开发日志

本文档用于持续记录 UTM-MAPFame 的架构优化、接口调整、实验日志字段变化和已验证结果。后续每次较大的代码修改，都建议在这里追加一节，方便回溯平台已经实现了哪些能力。

## 2026-06-26 组件化重构一期：Planner 与 Task/Mission Scheduler

### 背景

对比 MAPF 竞赛开始套件后，我们确认 UTM-MAPFame 早期代码存在一个主要问题：`APathPlanningDemoActor` 同时负责场景输入、任务配置、规划器选择、任务规划、执行仿真、重规划、可视化和实验日志。这样适合快速 demo，但不利于其他研究者替换自己的算法。

本阶段目标是先做低风险的模块化拆分：

- 保持现有 EUW 任务生成与 Mission Marker 工作流可用。
- 保持 `MissionConfigs` 作为默认任务来源。
- 先抽出 Planner Registry 和 Task/Mission Scheduler Registry。
- 不改变旧实验默认行为。

### Planner Registry 抽象

新增或整理的核心文件：

- `Source/UTM/Public/Planning/PlannerTypes.h`
- `Source/UTM/Public/Planning/PlannerRegistry.h`
- `Source/UTM/Private/Planning/PlannerRegistry.cpp`

主要变化：

- 将 `EPlannerType` 从 `PathPlanningDemoActor` 中抽出，作为规划器类型枚举。
- 新增 `FPlannerRuntimeConfig`，统一承载规划器运行参数，例如 ECBS 次优界、LaCAM 时间限制、随机种子和禁飞区配置。
- 新增 `FPlannerRegistry`，负责：
  - 判断规划器是否是多智能体规划器。
  - 返回规划器显示名。
  - 创建单智能体规划器。
  - 创建多智能体规划器。
  - 统一调用多智能体任务规划入口。
- `APathPlanningDemoActor` 不再直接包含大量具体规划器头文件，也不再直接负责 planner factory 逻辑。

当前支持的规划器类型包括：

- `AStar`
- `SIPP`
- `DStarLite`
- `JPS`，当前仍回退到 A*
- `CBS`
- `ECBS`
- `PBS`
- `LaCAM`
- `LaCAM-UTM`

行为变化：

- 默认规划结果应保持不变。
- 主要变化是代码边界更清楚，后续新增规划器只需要实现接口并在 registry 中注册。

### Task/Mission Scheduler 抽象

新增核心文件：

- `Source/UTM/Public/Planning/MissionSchedulerTypes.h`
- `Source/UTM/Public/Planning/MissionSchedulerRegistry.h`
- `Source/UTM/Private/Planning/MissionSchedulerRegistry.cpp`

主要变化：

- 新增 `EMissionSchedulerType`，目前包含：
  - `Static Scheduler`
  - `Nearest-First Scheduler`
- 新增 `FMissionSchedulerContext`，用于向 Scheduler 提供运行上下文。目前包含：
  - `GridMap`
  - `RequestedMissionCount`
- 新增 `IMissionScheduler` 接口：
  - 输入 raw missions。
  - 输出 scheduled missions。
- 新增 `FMissionSchedulerRegistry`，负责根据 `EMissionSchedulerType` 创建和调用具体 Scheduler。

`APathPlanningDemoActor` 中新增：

- `BuildScheduledMissionConfigs(...)`
- `MissionSchedulerType`

`MissionSchedulerType` 只暴露到 Details 面板，不暴露 Blueprint API。Details 面板显示名为：

```text
Mission Scheduler -> Task Assignment Strategy
```

### 当前任务分配策略

#### Static Scheduler

作用：

- 不改变任务。
- 原样传递 `MissionConfigs`。
- 用于保持旧 EUW 和旧实验流程兼容。

数据流：

```text
MissionConfigs -> Static Scheduler -> ScheduledMissions
```

其中 `ScheduledMissions` 与 `MissionConfigs` 内容一致。

#### Nearest-First Scheduler

作用：

- 根据距离做一个简单的任务重新分配。
- 用于验证平台已经支持可替换的任务分配策略。

当前规则：

```text
把每个 MissionConfig 的 StartWorld 看成无人机初始位置。
把每个 MissionConfig 的 GoalWorld 看成待分配目标点。
按 MissionId 从小到大遍历无人机。
每次给当前无人机分配最近的未分配目标点。
```

注意：

- `Nearest-First Scheduler` 不会改写 `MissionConfigs` 本身。
- 它只在规划前生成一份临时的 `ScheduledMissions`。
- 它保留当前无人机的 `MissionId` 和 `StartWorld`，但可能把 `GoalWorld` 换成另一个 Mission 的目标点。
- 因此它表达的是“无人机起点集合 + 待服务目标集合”的任务分配问题。
- 如果未来语义是“某个 MissionId 绑定固定任务，不能换目标”，则需要实现任务排序策略，而不是目标重分配策略。

### EUW 兼容性

本阶段刻意没有破坏 EUW 任务生成流程。

保留的关键字段和函数：

- `bUseMissionConfigs`
- `MissionConfigs`
- `EditorBuildGridForMissionEditing()`
- `EditorGenerateRandomMissionConfigs()`
- `EditorSpawnMissionMarkers()`
- `EditorClearMissionMarkers()`
- `EditorValidateMissionConfigs()`
- `EditorReadMissionMarkersToConfigs()`
- `RunPlanning()`

当前流程变为：

```text
EUW / Mission Marker
    -> MissionConfigs
    -> Mission Scheduler
    -> ScheduledMissions
    -> Planner Registry
    -> Concrete Planner
    -> Execution / Visualization
```

### 结构化实验日志

在结构化 JSON 日志中新增字段：

```json
"scheduler_type": "Static Scheduler"
```

或：

```json
"scheduler_type": "Nearest-First Scheduler"
```

该字段位于 `planner_type` 后面，方便后续实验表格按 scheduler 分组。

相关位置：

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`
- `BuildStructuredExperimentSummaryJson()`

### 编译中暴露并修复的问题

新增 cpp 文件后，Unreal unity build 的分组发生变化，暴露了两个既有的匿名命名空间符号重名问题。

已修复：

- `Source/UTM/Private/Planning/DStarLitePlanner.cpp`
  - 将文件内部常量 `INF_COST` 改名为 `DStarLiteInfCost`。
  - 行为不变，只避免 unity build 中与其他 cpp 的同名符号冲突。
- `Source/UTM/Private/Planning/LaCAMUTM.cpp`
  - 将文件内部函数 `ValidateMissionIdsUniqueLaCAM` 改名为 `ValidateMissionIdsUniqueLaCAMUTM`。
  - 行为不变，只避免与 `LaCAMPlanner.cpp` 中同名函数冲突。

### 已验证结果

已通过 Unreal 编译：

```powershell
& 'D:\Program Files\Epic Games\UE_5.5\Engine\Build\BatchFiles\Build.bat' UTMEditor Win64 Development '-Project=D:\UTM-MAPFame\UTM.uproject' -WaitMutex -FromMsBuild -architecture=x64
```

验证结论：

- UHT 通过。
- C++ 编译通过。
- `UnrealEditor-UTM.dll` 链接通过。
- 用户已测试 EUW 原流程，没有发现破坏性问题。

### 初步实验观察

用户在 Residential 地图上比较了两种任务分配策略。

实验配置摘要：

- `PhaseB`
- `R3 / V2Global`
- `Residential`
- `LaCAM-UTM`
- `N=100`
- `step_delay_probability=0.2`
- `execution_replan_mode=GlobalUnfinished`
- `execution_random_seed=3`

结果对比：

| 指标 | Static Scheduler | Nearest-First Scheduler | 变化 |
|---|---:|---:|---:|
| `execution_replan_total_time_ms` | 6743.88 ms | 3203.52 ms | 下降约 52.49% |
| `execution_replan_attempt_count` | 23 | 11 | 下降约 52.17% |
| `execution_replan_global_attempt_count` | 23 | 11 | 下降约 52.17% |
| `execution_replan_local_attempt_count` | 0 | 0 | 无变化 |
| `planned_makespan` | 230 | 183 | 下降约 20.43% |
| `actual_makespan` | 240 | 198 | 下降约 17.50% |
| `total_delay_steps` | 4515 | 3601 | 下降约 20.24% |
| `alignment_hold_count` | 2347 | 1129 | 下降约 51.90% |

初步解释：

- `Nearest-First Scheduler` 的执行期重规划总耗时下降，主要来自 global replan 次数减少。
- 单次 global replan 的平均耗时基本相近。
- 这说明任务分配策略改变了初始任务结构，使执行阶段更少触发全局重规划。

### 当前限制

- Scheduler 还没有独立的 agent state 和 task pool 数据结构。
- `Nearest-First Scheduler` 目前基于 `FDroneMissionConfig` 复用语义实现，适合做第一版任务分配验证，但还不是最终的任务调度框架。
- JSON 中目前只记录 `scheduler_type`，还没有记录每个 agent 的 assignment mapping。
- EUW 蓝图资源没有修改，本阶段只通过 Details 面板暴露策略选择。

### 后续建议

优先级较高：

- 用多个 `random_seed` 和 `execution_random_seed` 批量验证 `Nearest-First Scheduler` 的稳定性。
- 将 Scheduler 输入从 `MissionConfigs` 扩展为更清晰的：
  - agent initial states
  - open task pool
  - scheduler constraints

中期目标：

- 增加更多任务分配策略，例如：
  - Round-Robin Scheduler
  - Priority Scheduler
  - Auction-based Scheduler
- 将 execution coordinator 从 `APathPlanningDemoActor` 中继续抽出。
- 将实验 JSON 生成逻辑抽成 reporter，减少 Actor 内职责。

暂缓项：

- 暂不在主 JSON 中增加完整 `scheduler_assignment` 明细，避免结构化日志过大。
- 暂不增加 scheduler 自身统计指标，优先推进模块化拆分。

## 2026-06-30 组件化重构二期：Execution 模块骨架

### 背景

在 Planner 和 Task/Mission Scheduler 初步模块化之后，`APathPlanningDemoActor` 中最复杂的剩余职责是执行期逻辑，包括执行期对齐、冲突预测、重规划触发、重规划任务构造、路径合并和 UE 场景应用。

本阶段先建立独立 `Execution` 目录和 policy 类骨架，不迁移现有主流程，目标是降低后续拆分风险。

### 新增目录

新增独立执行期模块目录：

```text
Source/UTM/Public/Execution/
Source/UTM/Private/Execution/
```

没有继续放入 `Planning` 目录，原因是：

- `Planning` 负责初始路径生成。
- `Scheduling` 负责任务分配。
- `Execution` 负责执行过程中对齐、冲突预测、重规划和路径更新。

### 新增文件

公共头文件：

- `Source/UTM/Public/Execution/ExecutionTypes.h`
- `Source/UTM/Public/Execution/ExecutionAlignmentPolicy.h`
- `Source/UTM/Public/Execution/ConflictPredictionPolicy.h`
- `Source/UTM/Public/Execution/ExecutionReplanPolicy.h`
- `Source/UTM/Public/Execution/ReplanMissionBuilder.h`
- `Source/UTM/Public/Execution/PathMergePolicy.h`

私有实现文件：

- `Source/UTM/Private/Execution/ExecutionAlignmentPolicy.cpp`
- `Source/UTM/Private/Execution/ConflictPredictionPolicy.cpp`
- `Source/UTM/Private/Execution/ExecutionReplanPolicy.cpp`
- `Source/UTM/Private/Execution/ReplanMissionBuilder.cpp`
- `Source/UTM/Private/Execution/PathMergePolicy.cpp`

### ExecutionTypes.h

新增执行期纯数据结构和枚举：

- `EExecutionPolicyAction`
- `EExecutionPolicyReplanMode`
- `EExecutionPredictedConflictType`
- `FExecutionAgentSnapshot`
- `FExecutionSnapshot`
- `FExecutionStepDecision`
- `FExecutionPredictedConflict`
- `FExecutionReplanPolicySettings`
- `FExecutionReplanRequest`

这些类型不使用 `USTRUCT`、`UCLASS`、`UPROPERTY`，用于后续把执行期算法从 UE Actor 生命周期中分离出来。

### ExecutionAlignmentPolicy

当前作用：

- 包装现有 `FDiscreteAlignmentManager`。
- 输入 `FExecutionAgentSnapshot` 和 `FGridMap3D`。
- 输出 `FExecutionStepDecision`。

后续目标：

- 将 `AdvanceExecutionOneStep()` 中的 alignment decision 逻辑逐步迁移到该 policy。

### ConflictPredictionPolicy

当前作用：

- 根据多个 `FExecutionStepDecision` 预测下一步冲突。
- 当前已支持：
  - vertex conflict
  - edge conflict

后续目标：

- 将 protection footprint 和 downwash conflict 预测逐步迁移进来。
- 将当前 Actor 内部的 predicted conflict arbitration 逻辑拆分出来。

### ExecutionReplanPolicy

当前作用：

- 根据执行快照、step decision 和 predicted conflicts 构造 `FExecutionReplanRequest`。
- 支持模式：
  - `Disabled`
  - `LocalConflictSet`
  - `GlobalUnfinished`

后续目标：

- 接管当前 `AdvanceExecutionOneStep()` 中关于是否触发执行期重规划的判断。

### ReplanMissionBuilder

当前作用：

- 将 `FExecutionAgentSnapshot`、原始 `FDroneMissionConfig` 和 replan request 转换为 `ReplanMissions`。

后续目标：

- 接管 `TryExecutionReplan()` 中构造重规划任务的逻辑。
- 后续需要补充 stationary anchor、局部重规划窗口和全局未完成任务语义。

### PathMergePolicy

当前作用：

- 提供按 `MissionId` 替换路径的基础合并逻辑。

后续目标：

- 接管重规划成功后旧路径与新路径的合并逻辑。
- 后续需要支持保留已执行前缀、从当前 timestep 拼接、未受影响路径保持不变等策略。

### Actor 中保留的执行入口

当前保留原来的稳定执行主流程：

```text
AdvanceExecutionOneStep()
UpdateExecutionVisuals()
DrawExecutionDebugForState()
TryExecutionReplan()
```

`APathPlanningDemoActor` 中保留 `CaptureExecutionSnapshot()`，它可以从现有 `ExecutionStates` 中生成纯数据快照，作为未来 Execution policy 接入时的状态采集入口。

已移除以下空占位函数：

- `ApplyExecutionCommands(...)`
- `DrawExecutionDebug(...)`

移除原因：

- 两个函数尚未接入主流程，只是预留接口。
- 当前阶段继续深拆 execution 状态机会带来较大工作量和实验指标变化风险。
- 因此先保留已验证的旧主流程，把新 `Execution` 目录作为未来执行期算法模块骨架。

设计边界仍然是：

```text
CaptureExecutionSnapshot()
```

保留 UE 状态采集职责，例如读取 `ADroneActor` 状态并生成纯数据快照。

```text
ExecutionAlignmentPolicy
ConflictPredictionPolicy
ExecutionReplanPolicy
ReplanMissionBuilder
PathMergePolicy
```

保持纯逻辑，不直接依赖 `UWorld`、`AActor`、`DrawDebug`、`UPROPERTY` 或 `UFUNCTION`。

### 行为变化

本阶段没有把新 Execution policy 接入现有执行主流程，因此实验行为理论上不应改变。

这是刻意设计的：先建立可编译的模块边界，再逐步迁移内部逻辑，避免一次性大改导致执行期指标不可比。

### 已验证结果

已通过 Unreal 编译：

```powershell
& 'D:\Program Files\Epic Games\UE_5.5\Engine\Build\BatchFiles\Build.bat' UTMEditor Win64 Development '-Project=D:\UTM-MAPFame\UTM.uproject' -WaitMutex -FromMsBuild -architecture=x64
```

验证结论：

- UHT 通过。
- 新增 `Execution` 目录下的 cpp 文件已被 UBT 编译。
- `UnrealEditor-UTM.dll` 链接通过。
- 编译中仅保留既有 `PBSPlanner.cpp` 的 UE API deprecation warning。

## 2026-07-03 Mission Source 抽象

### 背景

继续推进模块化拆分时，我们把任务来源层从 `APathPlanningDemoActor` 中进一步分离。此前系统已经有 Planner Registry 和 Mission Scheduler Registry，但任务进入 Scheduler 之前仍然分散在 Actor 里：

- `MissionConfigs` 模式直接使用 Details / EUW 写入的数组。
- Start/Goal Actor 模式扫描场景里的 `Start_i` / `Goal_i`，再在 Actor 中手动拼成 `FDroneMissionConfig`。

这一步的目标是先建立统一的 Mission Source 边界，让后续新增任务来源时不需要继续扩大 `APathPlanningDemoActor`。

### 新增文件

- `Source/UTM/Public/Missions/MissionSourceTypes.h`
- `Source/UTM/Public/Missions/MissionSourceBuilder.h`
- `Source/UTM/Private/Missions/MissionSourceBuilder.cpp`

### 当前职责划分

`APathPlanningDemoActor` 仍然负责 UE 依赖部分：

- 扫描场景中的 Start/Goal Actor。
- 读取 Actor 世界坐标。
- 调用 `InputValidator` 结合当前 `GridMap` 做输入校验。
- 构建障碍物忽略列表。
- 继续调用 Scheduler、Planner、可视化和执行期流程。

`FMissionSourceBuilder` 负责纯任务来源标准化：

- `BuildFromMissionConfigs(...)`：把 Details / EUW 中已有的 `MissionConfigs` 作为 raw missions 传入后续流程。
- `BuildFromStartGoalWorldPairs(...)`：把已经采集好的 mission id、起点坐标、终点坐标转换成统一的 `TArray<FDroneMissionConfig>`。

### 当前数据流

```text
EUW / Details
    -> MissionConfigs
    -> Mission Source Builder
    -> Mission Scheduler
    -> Planner Registry
    -> Concrete Planner

Scene Start_i / Goal_i Actors
    -> APathPlanningDemoActor scans UE world
    -> APathPlanningDemoActor validates against GridMap
    -> Mission Source Builder
    -> Mission Scheduler
    -> Planner Registry
    -> Concrete Planner
```

### 保守性说明

- 没有修改 `bUseMissionConfigs`。
- 没有修改 `MissionConfigs` 的 UPROPERTY 暴露方式。
- 没有修改 EUW 任务生成、Mission Marker 生成和读取流程。
- 没有修改 Scheduler 下拉框和 JSON 中的 `scheduler_type`。
- 没有修改具体 Planner 算法。
- 单智能体 Start/Goal 模式也通过同一个 Mission Source 转换入口，但保留无效任务的失败统计。

## 2026-07-03 Experiment Reporter 抽象

### 背景

结构化实验 JSON 字段已经很多，继续把字段组装和 JSON 序列化留在 `APathPlanningDemoActor` 中会让 Actor 更难维护。为了让后续实验日志字段、CSV/JSONL 输出、文件保存或批量实验汇总更容易扩展，本阶段先抽出一个独立的 Experiment Reporter。

### 新增文件

- `Source/UTM/Public/Reporting/ExperimentReportTypes.h`
- `Source/UTM/Public/Reporting/ExperimentReporter.h`
- `Source/UTM/Private/Reporting/ExperimentReporter.cpp`

### 当前职责划分

`APathPlanningDemoActor` 仍然负责收集 UE 运行时状态和实验上下文：

- 解析或推断 `run_id`、`phase`、`group_id`、`group_name`、`scenario_name`。
- 读取 Planner、Scheduler、地图、随机种子、执行期参数。
- 读取规划统计、执行统计、禁飞区校验统计、重规划耗时统计。
- 调用 `UE_LOG` 输出 `[StructuredExperimentJSON]`。

`FExperimentReporter` 负责报告序列化：

- 接收 `FExperimentReportContext`。
- 按既有字段名生成结构化 JSON。
- 使用 condensed JSON 输出，保持日志仍为单行 JSON。

### 保守性说明

- 没有删除或重命名现有 JSON 字段。
- 没有修改 `scheduler_type`、`planner_type`、`execution_replan_total_time_ms` 等已有字段含义。
- 没有把实验元数据推断规则一次性搬走，降低对现有实验命名和分组逻辑的影响。
- `APathPlanningDemoActor::BuildStructuredExperimentSummaryJson()` 仍作为兼容入口保留，但内部改为构造 `FExperimentReportContext` 并调用 Reporter。
