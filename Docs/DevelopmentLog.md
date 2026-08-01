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

## 2026-07-12 多智能体 Planning Pipeline 抽象

### 背景

在 Mission Source、Mission Scheduler、Planner Registry 已经拆分后，`APathPlanningDemoActor` 里仍然直接串联多智能体规划流程。尤其是 `ProcessStartGoalPairsMultiAgent()` 和 `ProcessMissionConfigsMultiAgent()` 中存在重复逻辑：调用 Scheduler、调用多智能体 Planner、统计 solve time、生成每个 mission 的规划统计。

本阶段只抽多智能体 Planning Pipeline，暂不统一单智能体规划流程，降低对旧流程和 EUW 使用方式的影响。

### 新增文件

- `Source/UTM/Public/Planning/PlanningStatsTypes.h`
- `Source/UTM/Public/Planning/PlanningPipelineTypes.h`
- `Source/UTM/Public/Planning/PlanningPipeline.h`
- `Source/UTM/Private/Planning/PlanningPipeline.cpp`

### 当前职责划分

`FPlanningPipeline::RunMultiAgent(...)` 负责：

- 接收 raw missions。
- 调用 `FMissionSchedulerRegistry` 生成 scheduled missions。
- 调用 `FPlannerRegistry::PlanMultiAgentMissions(...)`。
- 记录多智能体规划 solve time。
- 使用 `FSingleMissionTimingStats` 为每个 scheduled mission 生成路径统计。
- 返回 `FMultiAgentPlanningPipelineResult`，包含 scheduled missions、paths、mission stats 和失败阶段。

`PlanningStatsTypes.h` 负责：

- 从 `APathPlanningDemoActor.h` 中移出 `FSingleMissionTimingStats`。
- 从 `APathPlanningDemoActor.h` 中移出 `FPlanningTimingStats`。
- 让 Actor、Planning Pipeline 和后续实验报告逻辑共享同一套规划统计类型。

`APathPlanningDemoActor` 仍然负责 UE 相关后处理：

- 从 EUW / Details 或 Start/Goal Actor 得到 raw missions。
- 构建 `GridMap`。
- 把 Pipeline 结果写回 `LastPlanningStats`。
- `CachePlannedPath(...)`。
- `DrawPathDebug(...)`。
- `SpawnDroneForPath(...)`。
- `CacheExecutionMissionConfigs(...)`。

同时新增 Actor 内部辅助函数：

- `ApplyMultiAgentPlanningResult(...)`

该函数只负责把 `FMultiAgentPlanningPipelineResult` 应用回 UE 场景和 Actor 状态，包括：

- 写回 `LastPlanningStats`。
- 处理 Pipeline 失败日志。
- 缓存执行期任务配置。
- 遍历每个 mission 的路径。
- 缓存路径、绘制路径、生成无人机。
- 累加 post-process time。

它仍然保留在 `APathPlanningDemoActor` 中，没有放进 `PlanningPipeline`，因为这些步骤依赖 Actor 状态、Debug 绘制和无人机生成，不属于纯规划流程。

### 保守性说明

- `ProcessMissionConfigs()` 单智能体 MissionConfigs 流程没有统一进 Pipeline。
- `ProcessStartGoalPairsSingleAgent()` 单智能体 Start/Goal 流程没有统一进 Pipeline。
- `PlanMultiAgentMissionsOnGrid(...)` 保留，因为执行期重规划仍然需要基于指定 grid 调用多智能体规划器。
- 移除了无调用的 `PlanMultiAgentMissions(...)` Actor wrapper。
- Pipeline 不接触 `UWorld`、`AActor`、无人机生成、Debug 绘制和执行期状态。
- `FSingleMissionTimingStats` 和 `FPlanningTimingStats` 字段名与类型保持不变，只移动定义位置。
- `ApplyMultiAgentPlanningResult(...)` 只是 Actor 内部去重，不改变多智能体规划、Scheduler 或执行期行为。

## 2026-07-12 Experiment Metadata Resolver 抽象

### 背景

在 `Experiment Reporter` 抽象完成后，结构化实验 JSON 的字段序列化已经从 `APathPlanningDemoActor` 中移出，但 `run_id`、`phase`、`group_id`、`group_name`、`scenario_name` 的解析和兜底推断逻辑仍然留在 Actor 内部。

这些逻辑本质上不是 UE 场景控制逻辑，而是实验命名和分组规则。继续留在 Actor 中会让 `BuildStructuredExperimentSummaryJson()` 同时承担运行状态收集、实验元数据推断和 JSON 报告组装三类职责。

### 新增文件

- `Source/UTM/Public/Reporting/ExperimentMetadataResolver.h`
- `Source/UTM/Private/Reporting/ExperimentMetadataResolver.cpp`

### 当前职责划分

`FExperimentMetadataResolver` 负责：

- 接收 `FExperimentMetadataResolverInput`。
- 清理 Details 面板中手动填写的 `ExperimentRunId`、`ExperimentPhase`、`ExperimentGroupId`、`ExperimentGroupName`、`ExperimentScenarioName`。
- 从已有 `run_id` 中解析 `phase` 和 `group_id`。
- 在缺失字段时根据执行配置推断默认实验分组。
- 根据 `ExecutionRandomSeed` 推断默认实验阶段。
- 根据地图类型提供默认 `scenario_name`。
- 在 `run_id` 缺失时生成兼容旧规则的 fallback run id。

`APathPlanningDemoActor` 现在负责：

- 收集 UE 运行时状态、规划统计、执行统计、地图类型、Planner 类型和执行期配置。
- 构造 `FExperimentMetadataResolverInput`。
- 调用 `FExperimentMetadataResolver::Resolve(...)` 获得标准化后的实验元数据。
- 将解析结果填入 `FExperimentReportContext`，再交给 `FExperimentReporter` 生成 JSON。

`FExperimentReporter` 继续负责：

- 接收 `FExperimentReportContext`。
- 按现有字段名生成结构化实验 JSON。
- 保持 `[StructuredExperimentJSON]` 单行日志输出格式。

### 保守性说明

- 没有新增、删除或重命名 JSON 字段。
- 没有修改 `scheduler_type`、`planner_type`、`scenario_name`、`run_id`、`phase`、`group_id`、`group_name` 等字段含义。
- 保留原有 PhaseA/PhaseB、G1/G2/G3、R1/R2/R3 的推断规则。
- 保留原有随机种子与 Phase 不匹配时的 `UE_LOG` 警告。
- `ExperimentMetadataResolver` 不接触 `UWorld`、`AActor`、无人机生成、Debug 绘制和执行期状态推进。
- `APathPlanningDemoActor::BuildStructuredExperimentSummaryJson()` 仍然是实验报告入口，但内部不再直接维护实验元数据推断细节。

## 2026-07-14 Execution Replan 第一轮拆分

### 背景

`APathPlanningDemoActor::TryExecutionReplan(...)` 仍然承担了执行期重规划的大量算法职责，包括局部候选任务扩展、UTM 静态安全约束判断、replan mission 构造、replan grid / anchor 构造、Planner 调用、post-check 验证和执行状态写回。

本阶段只做第一轮低风险拆分：先把可复用、无 UE Actor 指针依赖的逻辑移出 Actor，不改 post-check、anchor/grid 构造和执行状态写回。

### 新增文件

- `Source/UTM/Public/Planning/UTMSafetyModel.h`
- `Source/UTM/Private/Planning/UTMSafetyModel.cpp`
- `Source/UTM/Public/Execution/ExecutionReplanCandidateSelector.h`
- `Source/UTM/Private/Execution/ExecutionReplanCandidateSelector.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`
- `Source/UTM/Private/Execution/ReplanMissionBuilder.cpp`

### 当前职责划分

`FUTMSafetyModel` 负责：

- 判断两个 mission 在指定 cell 上是否存在静态 UTM 冲突。
- 区分 `ProtectionFootprint` 和 `Downwash`。
- 计算 mission 的安全影响半径，供 anchor relevance 和 footprint blocking 复用。

`FExecutionReplanCandidateSelector` 负责：

- 根据 `FExecutionSnapshot`、mission config、请求重规划的 mission 集合、空间扩展半径和 lookahead window 选择本轮 replan 候选集合。
- 支持 local conflict component 扩展。
- 在 LaCAM-UTM 场景下复用 `FUTMSafetyModel` 判断静态 UTM 耦合。

`FReplanMissionBuilder` 现在负责：

- 根据执行快照和候选 mission id 构造 `ReplanMissions`。
- 使用当前观测位置作为新的 `StartWorld`。
- 保留原 mission config 中的 `GoalWorld` 和 UTM safety 参数。
- 按 MissionId 排序输出，保持旧流程中 Planner 输入顺序稳定。

`APathPlanningDemoActor::TryExecutionReplan(...)` 仍然负责：

- 入口条件检查和 replan 计时统计。
- static anchor 选择和 replan grid 构造。
- 调用 `PlanMultiAgentMissionsOnGrid(...)`。
- post-check 验证和 targeted retry 控制。
- 将重规划结果写回 `ExecutionStates`、`PlannedCellPathsByMission` 和 `LastPlannedPathsByMission`。
- 更新 `TotalExecutionReplanCount` 和日志。

### 保守性说明

- 没有修改 `ExecutionReplanMode`、`MaxExecutionReplanCount`、local/global replan 的外部配置。
- 没有修改 post-check 冲突类型和检查窗口。
- 没有修改 replan attempt 的计时范围。
- 没有把 `FExecutionAgentState` 传入新模块，因为它包含 `TObjectPtr<ADroneActor>`。
- `TryExecutionReplan(...)` 内部构造 replan snapshot 时显式使用 `LastObservedCell`，保持旧候选选择和 replan start 位置语义。
- anchor/grid 构造、post-check 和状态写回暂时保留在 Actor，作为下一轮拆分对象。

## 2026-07-14 Execution Replan 第二轮拆分

### 背景

第一轮拆分后，`APathPlanningDemoActor::TryExecutionReplan(...)` 已经不再直接维护 UTM 静态安全判断、局部候选任务扩展和 replan mission 构造，但 static anchor 选择和 `ReplanGrid` 构造仍然保留在 Actor 内部。

这部分逻辑仍然属于执行期重规划算法：它根据当前执行快照、候选重规划集合、forced anchor 集合、UTM 安全约束、空间扩展半径和 lookahead window，构造真正交给 Planner 的重规划网格。因此本阶段将其继续抽入 Execution 模块。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionReplanGridBuilder.h`
- `Source/UTM/Private/Execution/ExecutionReplanGridBuilder.cpp`

### 当前职责划分

`FExecutionReplanGridBuilder` 负责：

- 接收 `FExecutionReplanGridBuildInput`。
- 复制基础 `FGridMap3D`，生成本次 attempt 使用的 `ReplanGrid`。
- 在 local replan 模式下，把未参与重规划的 active agent 当前 cell 标记为 blocked。
- 在 LaCAM-UTM 场景下选择 static anchor mission。
- 处理 post-check targeted retry 传入的 forced anchors。
- 根据 `FUTMSafetyModel` 标记 static anchor footprint。
- 返回 `AnchorMissionIds`、`AnchorMissionIdSet` 和 `StaticAnchorBlockedCellCount`，供 Actor 原有日志和后续路径处理继续使用。

`APathPlanningDemoActor::TryExecutionReplan(...)` 现在仍然负责：

- replan attempt 计时统计。
- 调用 `FExecutionReplanGridBuilder::Build(...)`。
- 调用 `FReplanMissionBuilder::Build(...)`。
- 调用 `PlanMultiAgentMissionsOnGrid(...)`。
- 将重规划世界路径转换为 cell path。
- post-check 验证和 targeted retry 控制。
- 将重规划结果写回执行状态。

### 保守性说明

- 没有修改 static anchor 的选择条件。
- 没有修改 local replan 下非候选 active agent 的 blocking 规则。
- 没有修改 `StaticAnchorBlockedCellCount` 的计数语义。
- 没有修改 global/local replan 的日志字段。
- `ExecutionReplanGridBuilder` 不接触 `AActor`、`ADroneActor`、`UWorld` 或 Debug 绘制。
- post-check 验证和执行状态写回仍留在 Actor，作为下一轮可选拆分对象。

## 2026-07-14 Execution Replan 第三轮拆分

### 背景

第二轮拆分后，`APathPlanningDemoActor::TryExecutionReplan(...)` 已经不再直接维护 static anchor 选择和 `ReplanGrid` 构造，但重规划完成后的 post-check lookahead 验证仍然保留在 Actor 内部。

post-check 验证用于检查重规划后的路径与未重规划 agent 的未来轨迹是否仍然存在 vertex、edge、ProtectionFootprint 或 Downwash 冲突。它本身只需要执行快照、候选 mission 集合、重规划后的 cell path、mission config 和 grid 维度，不需要访问 `ADroneActor` 或 `UWorld`，因此适合继续抽入 Execution 模块。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionReplanPostCheckPolicy.h`
- `Source/UTM/Private/Execution/ExecutionReplanPostCheckPolicy.cpp`

### 当前职责划分

`FExecutionReplanPostCheckPolicy` 负责：

- 接收 `FExecutionReplanPostCheckInput`。
- 将重规划后的 cell path 与原执行快照中的计划 path 合并为验证视角。
- 在 lookahead window 内检查 vertex conflict。
- 在 lookahead window 内检查 edge swap conflict。
- 在 LaCAM-UTM 场景下通过 `FUTMSafetyModel` 检查 `ProtectionFootprint` 和 `Downwash`。
- 返回 `FExecutionReplanPostCheckResult`，包括是否存在冲突、冲突类型、冲突双方、冲突 cell 和 offset。

`APathPlanningDemoActor::TryExecutionReplan(...)` 现在仍然负责：

- 构造 `FExecutionReplanPostCheckInput`。
- 输出原有 post-check 失败日志。
- 控制 local post-check targeted retry。
- 将成功的重规划结果写回执行状态。

### 保守性说明

- 没有修改 post-check 的 lookahead 范围。
- 没有修改 vertex、edge、ProtectionFootprint、Downwash 的判定顺序。
- 没有修改 post-check 失败后的日志字段。
- 没有修改 local targeted retry 触发条件。
- 没有把 `FExecutionAgentState` 传入新模块，post-check 只读取 `FExecutionSnapshot`。
- 执行状态写回仍保留在 Actor，作为下一轮可选拆分对象。

## 2026-07-14 Execution Conflict Prediction Policy 统一

### 背景

执行期冲突预测逻辑此前分散在多个位置：`APathPlanningDemoActor::AdvanceExecutionOneStep()` 内部有用于 conflict-aware alignment 的预测检查，Final Safety Gate 内部又维护了一套相似的 pair conflict 检查，而 Execution 模块中已有的 `FConflictPredictionPolicy` 只覆盖了较基础的 vertex / edge 检查。

这会导致后续维护 UTM 安全规则时出现风险：执行推进、最终安全门和重规划后复查可能逐渐使用不同的冲突判定语义。

### 修改文件

- `Source/UTM/Public/Execution/ConflictPredictionPolicy.h`
- `Source/UTM/Private/Execution/ConflictPredictionPolicy.cpp`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 当前职责划分

`FConflictPredictionPolicy` 现在负责：

- 接收 one-step conflict prediction 输入。
- 检查 vertex conflict。
- 检查 edge swap conflict。
- 在启用 UTM 静态安全检查时，调用 `FUTMSafetyModel` 检查 `ProtectionFootprint` 和 `Downwash`。
- 返回统一的 `FExecutionPredictedConflict` 结果。

`APathPlanningDemoActor::AdvanceExecutionOneStep()` 现在负责：

- 将内部 `FExecutionStepProposal` 转换成 `FExecutionConflictPredictionInput`。
- 决定是否启用 UTM 静态安全检查。
- 根据 `FConflictPredictionPolicy` 的结果继续执行原有 hold、replan、Final Safety Gate 和失败处理流程。

### 保守性说明

- 没有修改 conflict-aware alignment 的让行策略。
- 没有修改 Final Safety Gate 的 hold set 扩展逻辑。
- 没有修改 local / global replan 的触发条件。
- 没有修改原有日志文本和执行失败处理语义。
- `PlannerType` 仍然只在 Actor 中判断，Execution 模块不反向依赖具体 planner 枚举。
- Actor 内部的临时 `FPredictedExecutionConflict` 已移除，执行期冲突结果统一使用 `FExecutionPredictedConflict`。

## 2026-07-14 Replan PostCheck 复用 ConflictPredictionPolicy

### 背景

`FConflictPredictionPolicy` 已经统一了承担 one-step 执行期冲突预测的 vertex、edge、`ProtectionFootprint` 和 `Downwash` 判断，但 `FExecutionReplanPostCheckPolicy` 内部仍然保留了一套相似的 post-check pair conflict 判断。

为了避免执行推进、Final Safety Gate 和重规划后复查使用不同的安全语义，本轮将 post-check 的 pair conflict 判断接入统一的 `FConflictPredictionPolicy`。

### 修改文件

- `Source/UTM/Public/Execution/ConflictPredictionPolicy.h`
- `Source/UTM/Private/Execution/ConflictPredictionPolicy.cpp`
- `Source/UTM/Private/Execution/ExecutionReplanPostCheckPolicy.cpp`

### 当前职责划分

`FConflictPredictionPolicy` 现在额外提供：

- `FindPairConflict(...)`，用于直接检查两个 execution conflict item 是否存在冲突。
- 内部 `FindFirstConflict(...)` 和 `FindConflicts(...)` 也复用同一个 pair conflict 入口。

`FExecutionReplanPostCheckPolicy` 现在负责：

- 保留原有 lookahead offset 遍历。
- 保留 candidate mission pair 过滤。
- 将 offset 下的 previous / current cell 转换成 `FExecutionConflictCheckItem`。
- 调用 `FConflictPredictionPolicy::FindPairConflict(...)` 做 vertex、edge、UTM 静态安全判断。

### 保守性说明

- 没有修改 post-check 的 lookahead 范围。
- 没有修改 candidate mission pair 过滤规则。
- 没有修改 vertex、edge、`ProtectionFootprint`、`Downwash` 的判定优先级。
- `Offset == 0` 时仍然不启用 edge swap 检查。
- `FExecutionReplanPostCheckPolicy` 不再直接依赖 `FUTMSafetyModel`，统一通过 `FConflictPredictionPolicy` 间接使用安全模型。

## 2026-07-14 Execution Replan Attempt 第一轮整理

### 背景

在前几轮拆分后，`APathPlanningDemoActor::TryExecutionReplan(...)` 已经不再直接维护 candidate selection、replan grid 构造、post-check 内部冲突判断等细节，但单次 replan attempt 的 grid 构造、mission 构造、planner 调用、cell path 转换和 post-check 调用仍然堆在同一个内部 lambda 中。

本轮先做低风险整理：抽出一次 attempt 的输入/输出结构和 Actor 内部 helper，并将成功后的执行状态写回整理为单独的 Actor 内部 helper。

### 修改文件

- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 新增内部结构

`APathPlanningDemoActor` 新增以下私有结构：

- `FExecutionReplanAttemptSpec`
- `FExecutionReplanAttemptInput`
- `EExecutionReplanAttemptStatus`
- `FExecutionReplanAttemptResult`

这些结构只用于 Actor 内部，不暴露给 Blueprint。

### 当前职责划分

`RunExecutionReplanAttempt(...)` 现在负责：

- 将 candidate mission set 排序为本轮 movable mission 列表。
- 调用 `FExecutionReplanGridBuilder::Build(...)`。
- 调用 `FReplanMissionBuilder::Build(...)`。
- 调用 `PlanMultiAgentMissionsOnGrid(...)`。
- 将 replanned world path 转换为 cell path。
- 调用 `FExecutionReplanPostCheckPolicy::Validate(...)`。
- 返回 attempt 状态、anchor 信息、static blocked cell 数量、replanned cell paths 和 post-check conflict。

`TryExecutionReplan(...)` 继续负责：

- 入口条件检查。
- 捕获执行期 snapshot。
- local/global attempt 计时和统计。
- local expansion attempt 循环。
- post-check targeted retry 控制。
- 成功后调用 `ApplyExecutionReplanAttemptResult(...)` 写回执行状态。
- 更新 `TotalExecutionReplanCount` 和原有日志。

`ApplyExecutionReplanAttemptResult(...)` 现在负责：

- 根据成功 attempt 产物更新 `ExecutionStates`。
- 重建 timeline cell path 和 world path。
- 更新 `PlannedCellPathsByMission`。
- 更新 `LastPlannedPathsByMission`。
- 填充 `OutReplannedMissionIds`。

### 保守性说明

- 没有修改 local/global replan attempt 计时作用域。
- 没有把 post-check targeted retry 计为额外 attempt。
- 没有修改 planner 调用参数。
- 没有修改 post-check 触发条件。
- 没有修改成功后的执行状态写回语义。
- `RunExecutionReplanAttempt(...)` 仍然留在 Actor 中，因为它需要访问 `PlanMultiAgentMissionsOnGrid(...)`、`BuildCellPathFromWorldPath(...)`、`ExecutionStates` 和 `GridMap`。
- `ApplyExecutionReplanAttemptResult(...)` 也留在 Actor 中，因为它直接写入 `FExecutionAgentState` 和 Actor 内部路径缓存。

## 2026-07-15 Execution Replan Attempt 类型外移

### 背景

上一轮已经用四个数据类型描述单次 execution replan attempt 的配置、输入、状态和结果，但这些类型仍然嵌套在 `APathPlanningDemoActor` 内部，使后续独立的 Execution Replan Pipeline 无法在不依赖 Actor 类型的情况下复用这些输入输出协议。

### 修改文件

- 新增 `Source/UTM/Public/Execution/ExecutionReplanAttemptTypes.h`
- 修改 `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- 修改 `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 当前职责划分

以下类型已从 `APathPlanningDemoActor` 私有定义外移到 `Execution` 公共目录：

- `FExecutionReplanAttemptSpec`
- `FExecutionReplanAttemptInput`
- `EExecutionReplanAttemptStatus`
- `FExecutionReplanAttemptResult`

`APathPlanningDemoActor` 继续负责 `RunExecutionReplanAttempt(...)` 和 `ApplyExecutionReplanAttemptResult(...)` 的具体实现，只是通过新的公共头文件使用 attempt 数据结构。

### 保守性说明

- 四个类型的字段、默认值和状态枚举顺序均未修改。
- 没有修改 local/global replan attempt 循环。
- 没有修改 planner 调用、post-check 或 targeted retry 行为。
- 没有修改执行状态和路径缓存的写回逻辑。
- 本轮只建立独立 Execution Replan Pipeline 后续可使用的公共数据协议。

## 2026-07-15 Discrete Alignment 目录归属整理

### 背景

`FDiscreteAlignmentManager` 负责根据计划路径、实际执行位置、当前执行时刻和延迟请求计算单步对齐动作，属于执行期控制逻辑，但此前仍存放在 `Planning` 目录。该目录归属会让路径规划与执行对齐的模块边界不清晰。

### 修改文件

- `Source/UTM/Public/Planning/DiscreteAlignmentManager.h` 移动到 `Source/UTM/Public/Execution/DiscreteAlignmentManager.h`
- `Source/UTM/Private/Planning/DiscreteAlignmentManager.cpp` 移动到 `Source/UTM/Private/Execution/DiscreteAlignmentManager.cpp`
- 更新 `APathPlanningDemoActor` 和 `FExecutionAlignmentPolicy` 的 include 路径

### 当前职责划分

- `FDiscreteAlignmentManager` 继续负责原有离散路径对齐计算。
- `FExecutionAlignmentPolicy` 继续作为未来可替换的执行对齐策略入口。
- `APathPlanningDemoActor` 本轮仍直接调用 `FDiscreteAlignmentManager`，下一阶段再单独接入 `FExecutionAlignmentPolicy`。

### 保守性说明

- 没有修改 `FDiscreteAlignmentSettings`、`FDiscreteAlignmentResult` 或 `EDiscreteAlignmentAction`。
- 没有修改 `AlignStep(...)` 的实现。
- 没有修改 Actor 的 Alignment 调用顺序、随机延迟顺序、冲突消解或重规划逻辑。
- 本轮仅调整文件目录和 include 路径，便于独立验证目录迁移不会改变 N200 行为。

## 2026-07-15 ExecutionAlignmentPolicy 接入主流程

### 背景

`FExecutionAlignmentPolicy` 已经能够把执行快照转换成统一的 `FExecutionStepDecision`，但此前 Actor 仍然直接创建 `FDiscreteAlignmentManager` 并调用 `AlignStep(...)`，因此该 Execution Policy 只是预留实现，没有实际参与执行流程。

### 修改文件

- `Source/UTM/Public/Execution/ExecutionAlignmentPolicy.h`
- `Source/UTM/Private/Execution/ExecutionAlignmentPolicy.cpp`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 当前职责划分

- `APathPlanningDemoActor` 继续采集无人机实际位置、生成随机延迟请求并提供执行状态。
- `FExecutionAlignmentPolicy::Decide(...)` 根据 `FExecutionAgentSnapshot` 和网格生成 `FExecutionStepDecision`。
- Actor 内部后续冲突仲裁、Safety Gate、重规划和状态写回统一使用 `EExecutionPolicyAction` 和 `FExecutionStepDecision`。
- `FExecutionAlignmentPolicy::LexToString(...)` 保留原有动作名称，用于现有状态文本和 Alignment 日志。

### 保守性说明

- `FExecutionAlignmentPolicy` 内部仍调用原有 `FDiscreteAlignmentManager::AlignStep(...)`。
- Agent Snapshot 使用与旧调用完全相同的计划路径、执行索引、时间步、实际单元格和延迟请求。
- 动作转换保持一一对应，动作日志字符串未改变。
- 没有修改随机数调用位置和次数。
- 没有修改冲突仲裁、Final Safety Gate、重规划请求或状态统计规则。

## 2026-07-15 ExecutionReplanAttemptRunner 抽取

### 背景

单次 execution replan attempt 的输入、配置、状态和结果类型已经外移，但具体执行流程仍然由 `APathPlanningDemoActor::RunExecutionReplanAttempt(...)` 直接串联 Grid Builder、Mission Builder、Planner Registry、路径转换和 PostCheck。该实现仍然需要读取 Actor 成员，其他执行器无法独立复用。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionReplanAttemptRunner.h`
- `Source/UTM/Private/Execution/ExecutionReplanAttemptRunner.cpp`

### 修改文件

- `Source/UTM/Public/Execution/ExecutionReplanAttemptTypes.h`
- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 当前职责划分

`FExecutionReplanAttemptRunner::Run(...)` 负责：

- 检查 attempt context、snapshot 和 candidate mission 集合。
- 调用 `FExecutionReplanGridBuilder::Build(...)`。
- 调用 `FReplanMissionBuilder::Build(...)`。
- 通过 `FPlannerRegistry::PlanMultiAgentMissions(...)` 调用多智能体规划器。
- 将 replanned world path 转换为 cell path。
- 调用 `FExecutionReplanPostCheckPolicy::Validate(...)`。
- 返回 attempt 状态、路径、anchor、失败原因和 post-check conflict。

`APathPlanningDemoActor::RunExecutionReplanAttempt(...)` 现在只负责：

- 用 Actor 当前的 GridMap、Mission 配置、Planner 类型和 Planner Runtime Config 构造 context。
- 调用 `FExecutionReplanAttemptRunner::Run(...)`。
- 根据 attempt result 输出原有 UE 日志。

Actor 中原有的 `PlanMultiAgentMissionsOnGrid(...)` 已移除。成功后的 `ApplyExecutionReplanAttemptResult(...)` 继续留在 Actor，因为它仍然需要写入 `ExecutionStates`、`PlannedCellPathsByMission` 和 `LastPlannedPathsByMission`。

### 保守性说明

- Runner 不依赖 `APathPlanningDemoActor`、`AActor`、`UWorld`、`DrawDebug` 或 Details 反射属性。
- Planner 类型和 Runtime Config 仍由 Actor 使用原有 `BuildPlannerRuntimeConfig()` 构造。
- world path 到 cell path 仍使用同一个基础 `GridMap.WorldToCell(...)`。
- stationary anchor 和路径起点仍使用 Snapshot 中由 Actor 写入的 `LastObservedCell`。
- local/global attempt 计时、扩张轮次、targeted retry、成功写回和计数逻辑未修改。
- 原有失败日志文本和触发状态保持不变。

## 2026-07-15 ExecutionReplanCoordinator 抽取

### 背景

`FExecutionReplanAttemptRunner` 已经负责一次具体重规划尝试，但 `APathPlanningDemoActor::TryExecutionReplan(...)` 仍然维护 local/global 扩张轮次、candidate selection、PostCheck targeted retry、attempt 计时和成功控制。该部分属于执行期重规划编排，不需要直接依赖 UE 场景对象。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionReplanCoordinator.h`
- `Source/UTM/Private/Execution/ExecutionReplanCoordinator.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 当前职责划分

`FExecutionReplanCoordinator::Run(...)` 现在负责：

- 根据 local/global 模式确定 outer attempt 数量。
- 按 attempt index 扩大 spatial radius 和 lookahead。
- 调用 `FExecutionReplanCandidateSelector::Select(...)` 构造 candidate mission 集合。
- 在一次 outer attempt 内执行最多一次 local PostCheck targeted retry。
- 根据冲突双方状态扩充 movable mission 或 LaCAM-UTM stationary anchor。
- 统计每个 outer attempt 的次数、总耗时和最大耗时。
- 通过同步回调执行单次 attempt、成功结果写回和事件通知。

`APathPlanningDemoActor::TryExecutionReplan(...)` 现在负责：

- 保留空请求、Disabled 模式和最大重规划次数检查。
- 捕获执行快照，并将 `LastObservedCell` 写入 Snapshot。
- 构造 Coordinator Request 和同步回调。
- 将 Coordinator timing result 合并到原有 local/global 统计字段。
- 更新 `TotalExecutionReplanCount` 并返回 replanned mission IDs。

Actor 继续通过回调负责：

- 调用 `RunExecutionReplanAttempt(...)` Actor 适配层。
- 调用 `ApplyExecutionReplanAttemptResult(...)` 写入执行状态和路径缓存。
- 根据 `FExecutionReplanCoordinatorEvent` 输出原有 UE 日志。

### 保守性说明

- global replan 仍固定为一次 outer attempt。
- local replan outer attempt 数量仍为 `Max(1, LocalReplanMaxExpansionRounds)`。
- spatial radius 和 lookahead 仍按 `AttemptIndex + 1` 线性扩大。
- global 模式 targeted retry 上限仍为 0，local 模式仍为 1。
- targeted retry 仍包含在当前 outer attempt 的计时中，不额外增加 attempt count。
- candidate selection 前的 local expansion 日志仍不计入 attempt 时间。
- Runner、PostCheck 日志、targeted retry、成功写回和成功日志仍位于 attempt 计时作用域内。
- 原有 `[AlignmentReplan]` 日志模板逐项比对一致。
- 没有修改 JSON 字段或 execution replan timing 指标定义。

## 2026-07-15 Execution Conflict Resolution Policy 抽取

### 背景

执行对齐已经通过 `FExecutionAlignmentPolicy` 生成单步 proposal，但 `APathPlanningDemoActor::AdvanceExecutionOneStep()` 仍然直接负责预测冲突后的让行选择、多轮冲突消解和连续 Hold 触发重规划。这部分属于可替换的执行期冲突仲裁策略，不需要 UE 场景对象。

本轮只拆冲突仲裁，Final Safety Gate 继续保留在 Actor 中。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionStepTypes.h`
- `Source/UTM/Public/Execution/ExecutionConflictResolutionPolicy.h`
- `Source/UTM/Private/Execution/ExecutionConflictResolutionPolicy.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 当前职责划分

`FExecutionStepProposal` 已从 Actor `.cpp` 的匿名命名空间外移到 `ExecutionStepTypes.h`，作为 Alignment、冲突仲裁、重规划和 Safety Gate 之间共享的单步 proposal 数据结构。

`FExecutionConflictResolutionPolicy::Resolve(...)` 现在负责：

- 将当前 proposals 转换为统一的 conflict prediction input。
- 复用 `FConflictPredictionPolicy` 检查 vertex、edge 和可选 UTM 静态安全冲突。
- 按原规则选择 yielding mission：优先让移动方避让静止方，再保护 finished/GoalHold，最后由较大 Mission ID 让行。
- 在配置的 resolution passes 内重复执行冲突消解。
- 将 yielding proposal 改成 `HoldForPredictedConflict`。
- 无法继续消解或仍有剩余冲突时返回需要重规划的 Mission ID。
- 根据连续 conflict hold 次数和阈值触发重规划请求。
- 返回结构化事件，由 Actor 输出原有日志。

`APathPlanningDemoActor::AdvanceExecutionOneStep()` 现在负责：

- 从 `ExecutionStates` 构造轻量 agent 完成状态和连续 Hold 计数。
- 调用 `FExecutionConflictResolutionPolicy::Resolve(...)`。
- 将 Policy 返回的 Mission ID 合并到已有 alignment replan 请求。
- 根据返回事件输出 `[AlignmentConflictPrediction]` 日志。

### Final Safety Gate 保持不变

- `CollectProposalConflictEndpoints(...)` 仍然留在 Actor。
- Safety Gate Hold 集合扩张逻辑未移动。
- Safety Gate local/global replan 和失败停止逻辑未移动。
- Safety Gate 仍会在普通冲突仲裁和可能的重规划之后重新检查最终 proposals。

### 保守性说明

- proposal conflict item 仍按排序后的 Mission ID 构造。
- 冲突预测时仍将 proposal item 标记为 valid。
- yielding mission 的选择顺序和 Mission ID tie-break 未修改。
- resolution pass 上限仍使用 `Max(1, AlignmentConflictResolutionPasses)`。
- conflict hold 阈值仍使用 `State.ConsecutiveConflictHoldCount + 1` 判断。
- `ResolutionReason` 文本和三种 `[AlignmentConflictPrediction]` 日志模板保持不变。
- 没有修改随机延迟、Alignment Policy、Execution Replan Coordinator 或 Final Safety Gate。

## 2026-07-15 Execution State Transition 抽取

### 背景

`APathPlanningDemoActor::AdvanceExecutionOneStep()` 在 Alignment、冲突仲裁、执行期重规划和 Final Safety Gate 得到最终 proposal 后，仍直接维护路径进度、完成状态、连续 Hold 计数、Alignment 状态和各类执行统计。这些确定性的状态转移规则属于执行模块，不需要依赖 UE 场景对象。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionStateTransitionTypes.h`
- `Source/UTM/Public/Execution/ExecutionStateTransition.h`
- `Source/UTM/Private/Execution/ExecutionStateTransition.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 当前职责划分

`FExecutionStateTransition::Compute(...)` 现在负责：

- 将 proposal 的目标路径下标限制在当前路径有效范围内。
- 根据最终执行动作更新 Snap、Correction 和 Hold 统计。
- 更新 conflict hold 与 safety-gate hold 的连续计数。
- 更新重规划请求次数、成功次数和 Alignment Lost 状态。
- 更新延迟总步数以及最大空间、时间对齐误差。
- 根据最终路径下标和提交位置判断任务是否完成。
- 返回提交位置和新的轻量执行状态，不直接访问 `FExecutionAgentState`。

Actor 中的适配层现在负责：

- 从 `FExecutionAgentState` 捕获纯状态转移输入。
- 将 `FExecutionStateTransitionResult` 提交回 UE 反射状态。
- 追加 `ActualCells`，更新 `DisplayToCell` 和动作文本。
- 继续输出原有 Delay 与 Alignment 日志。
- 继续执行冲突统计、执行结束判断、Summary 生成和可视化更新。

### 模块边界

- State Transition 模块不依赖 `APathPlanningDemoActor`、`ADroneActor`、`UWorld`、`DrawDebug` 或 `UPROPERTY`。
- `FExecutionAgentState` 继续保留在 Actor 层，因为它包含 `TObjectPtr<ADroneActor>` 和供 Details/Blueprint 读取的反射字段。
- 算法模块通过统一的 `FExecutionStepProposal` 表达本时间步决定，平台通过统一 State Transition 规则提交该决定。
- 本模块是执行平台的统一状态语义，不作为任意修改统计口径的算法扩展点。

### 保守性说明

- 随机延迟生成位置、调用次数和顺序未修改。
- `FExecutionStepProposal` 的生成、冲突仲裁和重规划后修正未修改。
- Final Safety Gate 的 Hold 集合扩张、local/global replan 和失败停止逻辑未修改，仍完整保留在 Actor。
- `HoldForReplan` 期间不清零连续 conflict/safety-gate hold 的规则保持不变。
- 成功重规划后清零连续 Hold、恢复 Alignment 状态的规则保持不变。
- 原有 Delay 和 Alignment 日志模板及触发条件保持不变。
- `ActualCells`、完成判定、`bAnyActive` 和执行 Summary 的统计口径保持不变。

## 2026-07-15 Final Safety Gate 第一阶段抽取

### 背景

`APathPlanningDemoActor::AdvanceExecutionOneStep()` 在 Alignment、冲突仲裁和普通执行期重规划之后，仍直接维护 Final Safety Gate 的冲突检查、Hold 集合扩张和连续 Hold 升级全局重规划判断。这些规则只依赖单步 proposals、Mission 配置和轻量计数状态，不需要访问 UE 场景对象或直接调用 Planner。

本轮只完成 Final Safety Gate 第一阶段抽取。重规划调用、日志、失败停止和 Summary 继续保留在 Actor。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionFinalSafetyGateTypes.h`
- `Source/UTM/Public/Execution/ExecutionFinalSafetyGatePolicy.h`
- `Source/UTM/Private/Execution/ExecutionFinalSafetyGatePolicy.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Policy 当前职责

`FExecutionFinalSafetyGatePolicy::EvaluateAndApplyHold(...)` 现在负责：

- 按 Actor 提供的排序后 Mission ID 构造统一 conflict prediction input。
- 复用 `FConflictPredictionPolicy` 检查最终 proposals 的 vertex、edge 和可选 UTM 静态安全冲突。
- 收集所有冲突两端的 Mission ID，形成初始 Hold 集合。
- 将 Hold proposal 改为 `HoldForSafetyGate`，并反复检查 Hold 后是否仍有冲突。
- 在最多 `OrderedMissionIds.Num()` 轮内扩张 Hold 集合。
- 记录每轮扩张前后的 Mission 数量和导致扩张的首个剩余冲突。
- 使用 `ConsecutiveSafetyGateHoldCount + 1 >= MaxHoldSteps` 判断是否强制升级全局重规划。
- 返回初始冲突、最终 Hold 集合、Hold 是否安全、未解决冲突和全局升级决定。

`FExecutionFinalSafetyGatePolicy::CheckConflicts(...)` 提供独立的 proposal 冲突检查，供 Actor 在 safety-gate replan 成功并同步 proposals 后执行最终复检。

### Actor 继续保留的职责

- 调用 `TryExecutionReplan(...)`。
- 根据配置选择 local/global safety-gate replan。
- local safety-gate replan 失败后升级 global。
- 重规划成功后同步 `FExecutionStepProposal` 和成功 Mission ID。
- 输出全部 `[FinalSafetyGate]` 日志。
- 根据 Policy 结果决定是否停止执行。
- 停止后调用 `BuildExecutionSummary()` 和 `LogExecutionSummary()`。

### 保守性说明

- Final Safety Gate 仍位于普通冲突仲裁和普通 execution replan 之后。
- proposal conflict item 仍按排序后的 Mission ID 构造，并继续统一标记为 valid。
- 初始 Hold 集合仍包含所有检测到的冲突端点，而不仅是首个冲突。
- Hold proposal 的字段、`FinalAction` 和 `ResolutionReason` 文本保持不变。
- Hold 集合扩张轮数、扩张条件和首个剩余冲突选择保持不变。
- Safety Gate Hold 阈值仍使用当前连续计数加一后与 `Max(1, FinalSafetyGateMaxHoldSteps)` 比较。
- `TryExecutionReplan(...)` 调用顺序、local/global 升级条件和执行期重规划计时范围未修改。
- safety-gate replan 后的 proposal 同步和最终冲突复检仍由 Actor 控制。
- 9 条 `[FinalSafetyGate]` 日志模板及其触发顺序保持不变。
- Final Safety Gate 失败停止、Summary 生成、状态提交、随机延迟和 JSON 指标定义未修改。

## 2026-07-15 Final Safety Gate 第二阶段编排抽取

### 背景

第一阶段已经将冲突检查、Hold 集合扩张和全局升级判断移入 `FExecutionFinalSafetyGatePolicy`，但 `APathPlanningDemoActor::AdvanceExecutionOneStep()` 仍直接编排 safety-gate local/global replan、失败升级、proposal 同步和重规划后复检。该控制流不需要直接依赖 UE 生命周期，可以通过同步回调与 Actor 适配层连接。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionFinalSafetyGateCoordinator.h`
- `Source/UTM/Private/Execution/ExecutionFinalSafetyGateCoordinator.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Coordinator 当前职责

`FExecutionFinalSafetyGateCoordinator::Run(...)` 现在负责：

- 调用 `FExecutionFinalSafetyGatePolicy::EvaluateAndApplyHold(...)`。
- 按原顺序发出初始冲突、Hold 扩张、Hold 阈值和安全 Hold 事件。
- 根据 Execution replan mode 选择 configured local/global replan。
- local safety-gate replan 失败后发出升级事件并调用 global replan。
- replan disabled 时保留安全 Hold，不调用重规划。
- 重规划成功后调用 Actor 提供的 proposal 同步回调。
- 复用 Policy 对同步后的全部 proposals 执行最终冲突复检。
- 返回 Safety Gate 新增的 requested/successful Mission ID、重规划成功状态和停止决定。

### Actor 适配层继续负责

- 通过 `RunReplan` 回调调用原有 `TryExecutionReplan(...)`。
- 通过 `ApplyReplanResult` 回调读取 `ExecutionStates`，按原规则同步 safety-gate replanned proposals。
- 通过 `OnEvent` 回调输出原有 9 条 `[FinalSafetyGate]` 日志。
- 将 Coordinator 返回的 Mission ID 合并到当前执行步的 requested/successful 集合。
- 根据 `bStopExecution` 停止执行并生成 Summary。

### 保守性说明

- 本轮只移动 Final Safety Gate 编排，普通 alignment/conflict execution replan 流程未移动。
- Actor 的 `TryExecutionReplan(...)` 实现、调用参数和内部计时未修改。
- configured global、configured local、local 失败升级 global 和 disabled 分支顺序保持不变。
- 强制 global replan 失败后的停止条件保持不变。
- 非强制 local/global replan 都失败时继续提交安全 Hold 的规则保持不变。
- safety-gate replan 成功后的 proposal 字段、路径下标和 `ResolutionReason` 保持不变。
- 最终冲突复检仍发生在 proposal 同步之后，并检查全部 Mission proposals。
- 日志仍由 Actor 输出，9 条日志模板、级别和触发顺序保持不变。
- Summary、状态提交、随机延迟、replan timing 和 JSON 指标定义未修改。

## 2026-07-15 Execution Replan Path Integrator 抽取

### 背景

`APathPlanningDemoActor::ApplyExecutionReplanAttemptResult(...)` 在收到成功的 replanned cell path 后，仍直接拼接实际执行轨迹、当前观测位置、replan Hold 时间步和新的规划路径。该时间线拼接规则属于执行期重规划语义，不需要访问 UE 场景对象或 Actor 路径缓存。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionReplanPathIntegrator.h`
- `Source/UTM/Private/Execution/ExecutionReplanPathIntegrator.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Integrator 当前职责

`FExecutionReplanPathIntegrator::Integrate(...)` 现在负责：

- 复制当前 `ActualCells` 作为新的执行时间线前缀。
- 当实际轨迹末尾不是 `LastObservedCell` 时补入当前观测位置。
- 再加入一次 `LastObservedCell`，保留当前 execution replan Hold 时间步。
- 从下标 1 开始追加 replanned cell path，避免重复加入其起点。
- 按旧规则使用 `Max(0, ActualCells.Num() - 1)` 计算新的 `ExecutedPlanIndex`。
- 使用 replanned cell path 的最后一个单元格作为 `GoalCell`。
- 对空 replanned path 返回失败，不产生可应用结果。

### Actor 继续保留的职责

- 校验 Mission 对应的 `FExecutionAgentState`、Mission 配置和 replanned path 是否存在。
- 调用 Integrator 并处理失败结果。
- 使用 `GridMap.CellToWorld(...)` 生成 world timeline。
- 更新 `FExecutionAgentState`、`PlannedCellPathsByMission` 和 `LastPlannedPathsByMission`。
- 重置 conflict Hold 和 Alignment Lost 状态，并记录成功重规划的 Mission ID。

### 保守性说明

- 仍按 `Result.CandidateMissionIds` 原顺序逐 Mission 计算并立即写回，没有改成批量事务提交。
- `LastObservedCell` 的条件补入规则保持不变。
- replan Hold 使用的重复观测单元格是时间轴等待动作，没有使用 `AddUnique()` 去重。
- replanned cell path 仍从下标 1 开始追加。
- `ExecutedPlanIndex` 没有改为根据新 timeline 长度重新计算。
- world 坐标转换、Actor 状态字段和路径缓存写入顺序保持不变。
- `PathMergePolicy` 本轮未修改、未删除，也没有接入执行期时间线整合流程。
- Replan Runner、Coordinator、attempt 计时、状态统计和 JSON 指标定义未修改。

## 2026-07-15 Execution Step Proposal Builder 抽取

### 背景

`APathPlanningDemoActor::AdvanceExecutionOneStep()` 在调用 `FExecutionAlignmentPolicy::Decide(...)` 后，仍直接将 `FExecutionStepDecision` 转换为 `FExecutionStepProposal`，并在 Actor 中维护默认动作、路径下标钳制、无效 Alignment fallback 和初始重规划请求条件。这些规则只依赖轻量输入和 Alignment Decision，不需要访问 UE 场景对象。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionStepProposalBuilder.h`
- `Source/UTM/Private/Execution/ExecutionStepProposalBuilder.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Builder 当前职责

`FExecutionStepProposalBuilder::Build(...)` 现在负责：

- 写入 Mission ID、Observed Cell、Delay 请求和完整 Alignment Decision。
- 将当前执行路径下标限制在有效路径范围内。
- 初始化 Proposed Index、Observed Cell、`HoldForAlignment` 和 Alignment Reason。
- 对有效 Alignment 写入 valid、requires replan、reference/target index、target cell 和最终动作。
- 对无效 Alignment 设置 `bInitialAlignmentInvalid` 和 `bRequiresReplan`。
- 在无效 Alignment 没有 reason 时继续使用 `invalid alignment result`。
- 返回是否需要加入初始 execution replan Mission 集合。
- 对空规划路径返回失败，不生成可提交 Proposal。

### Actor 继续保留的职责

- 读取 Drone 实际位置并更新 `LastObservedCell` 和 `DisplayFromCell`。
- 按原顺序调用 `ShouldDelayThisStep(...)` 生成随机或脚本延迟请求。
- 构造 `FExecutionAgentSnapshot` 并调用 `FExecutionAlignmentPolicy::Decide(...)`。
- 调用 Builder，将结果加入 `StepProposals` 和初始 replan Mission 集合。

### 保守性说明

- Mission ID 仍按排序后的顺序逐个处理。
- `ShouldDelayThisStep(...)` 的调用位置、次数和 Mission 顺序未修改。
- `FRandomStream` 仍由 Actor 持有和消费，随机数序列未移入 Builder。
- Alignment Snapshot 的字段、Policy 设置和调用顺序未修改。
- Proposal 默认字段、有效/无效分支、索引钳制范围和 fallback reason 保持不变。
- 初始 replan 条件仍为 `bRequiresReplan || bInitialAlignmentInvalid`。
- Conflict Resolution、普通 Execution Replan、Final Safety Gate 和 State Transition 未修改。
- 日志、replan timing、状态统计和 JSON 指标定义未修改。

## 2026-07-15 Execution Step Replan Coordinator 抽取

### 背景

`APathPlanningDemoActor::AdvanceExecutionOneStep()` 在收集 Alignment 和 Conflict Resolution 产生的重规划请求后，仍直接维护普通 execution replan 的模式分派、local 失败后的 global 回退，以及成功后所有 Proposal 的同步暂停。该控制流属于单个执行时间步的重规划编排，不需要直接依赖 UE 场景对象。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionStepReplanCoordinator.h`
- `Source/UTM/Private/Execution/ExecutionStepReplanCoordinator.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Coordinator 当前职责

`FExecutionStepReplanCoordinator::Run(...)` 现在负责：

- 没有重规划请求或 replan mode 为 `Disabled` 时直接跳过。
- `GlobalUnfinished` 模式下直接执行一次 global replan。
- `LocalConflictSet` 模式下先执行 local replan；失败后使用同一请求集合升级为 global replan。
- 重规划成功后，通过同步回调将成功结果应用到本时间步 Proposal。
- 返回本时间步普通重规划是否成功以及实际重规划的 Mission ID 集合。

### 两层 Coordinator 的职责区别

- `FExecutionStepReplanCoordinator` 是外层的“单步分派器”，只决定本时间步调用 local 还是 global replan，以及 local 失败后是否升级。
- 已有 `FExecutionReplanCoordinator` 是 `TryExecutionReplan(...)` 内部的“重规划尝试编排器”，继续负责 candidate selection、local 扩张、多轮 attempt、PostCheck targeted retry 和 attempt timing。
- 两者不是重复实现：前者决定一次执行步如何调用重规划服务，后者负责一次重规划服务调用内部如何求解。

### Actor 继续保留的职责

- 通过 `RunReplan` 回调调用原有 `TryExecutionReplan(...)`。
- 通过 `ApplyReplanResult` 回调读取 `ExecutionStates`，同步更新本步全部有效 Proposal。
- 继续维护 Final Safety Gate、执行状态提交、日志、Summary 和可视化。

### 保守性说明

- Alignment 和 Conflict Resolution 产生的请求集合及合并顺序未修改。
- `Disabled`、configured global、configured local 和 local 失败升级 global 的分支顺序保持不变。
- `TryExecutionReplan(...)` 的实现、调用参数、内部 attempt 次数和 timing 范围未修改。
- 普通重规划成功后仍暂停本步全部有效 Proposal，使未重规划的 Agent 与新轨迹同步。
- 实际重规划 Mission 的 Proposed Index 仍前进一格，其他 Mission 仍保持当前 Reference Index。
- Proposal 标志、最终动作和两种 `ResolutionReason` 文本保持不变。
- Final Safety Gate 仍在普通 execution replan 之后执行，其 Coordinator 和复检逻辑未修改。
- 日志、状态统计、Summary 和 StructuredExperimentJSON 指标定义未修改。

## 2026-07-15 Execution Replan Proposal Synchronizer 与 Step Pipeline 抽取

### 背景

普通 execution replan 和 Final Safety Gate 在成功重规划后，都需要根据最新执行路径修改本时间步 `FExecutionStepProposal`。两处逻辑原本分别位于 `APathPlanningDemoActor::AdvanceExecutionOneStep()`，字段写入高度相似，但同步范围和原因文本不同。同时，Actor 仍直接串联 Conflict Resolution、普通 Step Replan 和 Final Safety Gate，导致已经独立的 Execution 组件仍缺少统一的中层调用入口。

本轮在一次优化中完成两个相互依赖的阶段：先统一重规划后的 Proposal 同步规则，再建立第一阶段 Execution Step Pipeline。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionReplanProposalSynchronizer.h`
- `Source/UTM/Private/Execution/ExecutionReplanProposalSynchronizer.cpp`
- `Source/UTM/Public/Execution/ExecutionStepPipeline.h`
- `Source/UTM/Private/Execution/ExecutionStepPipeline.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Proposal Synchronizer 当前职责

`FExecutionReplanProposalSynchronizer::Apply(...)` 现在负责：

- 按排序后的 Mission ID 顺序处理指定同步集合。
- 使用重规划完成后重新捕获的 `ExecutedPlanIndex` 和路径长度计算 Reference/Proposed Index。
- 将目标 Proposal 标记为 valid、`HoldForReplan`，并清除 predicted-conflict hold 和 replan request 标志。
- 实际重规划 Mission 的 Proposed Index 前进一格，纯同步 Mission 保持当前 Reference Index。
- 根据 Mission 是否实际重规划写入调用方提供的两种 `ResolutionReason`。

普通重规划将全部 Mission 作为同步目标；Final Safety Gate 只将 Hold Mission 与 Replanned Mission 的并集作为同步目标。该差异由请求数据表达，不再维护两套字段写入代码。

### Step Pipeline 当前职责

`FExecutionStepPipeline::Run(...)` 当前统一编排：

1. 调用 `FExecutionConflictResolutionPolicy::Resolve(...)`，合并冲突仲裁产生的重规划请求并转发事件。
2. 调用 `FExecutionStepReplanCoordinator::Run(...)` 执行普通 local/global step replan。
3. 在普通重规划完成后，通过回调重新捕获最新执行状态并调用 Proposal Synchronizer。
4. 在普通重规划及其状态写回完成后，通过回调构造最新的 Final Safety Gate 输入。
5. 调用 `FExecutionFinalSafetyGateCoordinator::Run(...)`，并在成功时复用 Proposal Synchronizer。
6. 合并普通重规划和 Safety Gate 的 requested/successful Mission ID，返回总体重规划成功状态和停止执行决定。

### Actor 当前职责

`APathPlanningDemoActor::AdvanceExecutionOneStep()` 继续负责：

- 更新时间步并读取 Drone 实际位置。
- 按原顺序生成随机或脚本延迟。
- 调用 Alignment Policy 和 Proposal Builder 产生初始 Proposal。
- 将 UE/Actor 状态适配为 Conflict Resolution、Proposal Synchronizer 和 Final Safety Gate 的轻量输入。
- 通过回调调用原有 `TryExecutionReplan(...)`，并输出 Conflict Resolution 与 Final Safety Gate 日志。
- 根据 Pipeline Result 执行停止/Summary，随后计算并提交 State Transition。
- 执行冲突统计、结束判断和可视化。

`AdvanceExecutionOneStep()` 从约 368 行降至约 269 行。减少的主要是组件调用、结果合并和重复 Proposal 写回，不是简单移动 UE 日志或可视化代码。

### Pipeline 当前边界

当前第一阶段 Pipeline 的范围是：

`Prebuilt Proposals -> Conflict Resolution -> Ordinary Step Replan -> Final Safety Gate -> Pipeline Result`

以下内容尚未进入 Pipeline：

- Drone 位置读取和 `CaptureExecutionSnapshot()`。
- Delay Policy、Alignment Policy 调用和初始 Proposal 构建。
- `TryExecutionReplan(...)` 内部实现与路径缓存写回。
- State Transition 计算结果写回 `FExecutionAgentState`。
- UE 日志、停止执行、Summary、冲突统计和可视化。

### 保守性说明

- Mission 排序、Delay 随机数消费位置和 Alignment 调用顺序未修改。
- Conflict Resolution 仍发生在普通 Step Replan 之前，事件输出顺序未修改。
- 普通 replan 的 Disabled、configured local、configured global 和 local 失败升级 global 语义未修改。
- 普通重规划成功后仍暂停全部有效 Proposal；Safety Gate 仍只同步 Hold/Replanned 集合。
- Proposal 字段、索引规则和四种 `ResolutionReason` 文本保持不变。
- Final Safety Gate 输入在普通重规划之后构造，避免读取重规划前的旧路径下标或状态。
- Final Safety Gate 的 Hold 扩张、local/global 升级、最终复检和停止执行规则未修改。
- `TryExecutionReplan(...)`、attempt timing、路径整合、状态统计、Summary 和 StructuredExperimentJSON 指标定义未修改。

## 2026-07-15 Execution Step Pipeline 第二阶段

### 背景

第一阶段 `FExecutionStepPipeline` 已经统一编排 Conflict Resolution、普通 Step Replan 和 Final Safety Gate，但调用方仍需自行调用 Alignment Policy 与 Proposal Builder，并将预构建的 `StepProposals` 作为可变参数传入。这意味着其他研究者使用执行 Pipeline 时仍需了解内部 Proposal 构造规则，尚未形成完整的执行决策入口。

第二阶段将 Pipeline 左边界扩展到有序的 `FExecutionAgentSnapshot`。Actor 只捕获 UE 观察和 Delay 结果，Alignment 与初始 Proposal 生成移入 Pipeline。

### 修改文件

- `Source/UTM/Public/Execution/ExecutionStepPipeline.h`
- `Source/UTM/Private/Execution/ExecutionStepPipeline.cpp`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Pipeline Request 变化

`FExecutionStepPipelineRequest` 新增：

- `OrderedAgentSnapshots`：Actor 按排序 Mission ID 捕获的执行 Agent 快照。
- `GridMap`：供 Alignment Policy 执行离散对齐。
- `AlignmentSettings`：由 Actor 的 Details 配置转换得到的纯 Alignment 设置。

原有 `InitialRequestedReplanMissionIds` 已移除。初始重规划请求现在由 Pipeline 根据 Alignment Decision 和 Proposal Builder 结果自行产生。

### Pipeline Result 变化

`FExecutionStepPipelineResult` 新增 `StepProposals`。Pipeline 不再要求调用方提供可变 Proposal Map，而是完整返回经过以下阶段处理的最终 Proposal：

`Ordered Agent Snapshots -> Alignment Policy -> Proposal Builder -> Conflict Resolution -> Ordinary Step Replan -> Final Safety Gate -> Pipeline Result`

### Pipeline 新增职责

`FExecutionStepPipeline::Run(...)` 在原有第一阶段职责之前新增：

1. 按 `OrderedAgentSnapshots` 的顺序逐个调用 `FExecutionAlignmentPolicy::Decide(...)`。
2. 在 Alignment 前跳过空规划路径，与旧 Actor 的入口保护一致。
3. 直接从 Agent Snapshot 构造 `FExecutionStepProposalBuildInput`。
4. 调用 `FExecutionStepProposalBuilder::Build(...)` 生成初始 Proposal。
5. 跳过 Proposal Builder 返回失败的 Agent，与旧 Actor 流程一致。
6. 合并 `bRequiresReplan || bInitialAlignmentInvalid` 产生的初始重规划请求。
7. 将初始 Proposal Map 继续传入 Conflict Resolution、普通 Replan 和 Final Safety Gate。

### Actor 当前职责

`APathPlanningDemoActor::AdvanceExecutionOneStep()` 现在继续负责：

- 更新时间步并按 Mission ID 排序。
- 从 Drone 或执行状态读取 Observed Cell，并更新 `LastObservedCell` 和 `DisplayFromCell`。
- 保留 `bCanAdvance && ShouldDelayThisStep(...)` 的原有短路调用，产生 Delay 结果。
- 将 Actor 状态复制为有序 `FExecutionAgentSnapshot`。
- 构造 Conflict Resolution 输入和 Pipeline 回调。
- 调用 Pipeline，并使用其返回的 Proposal Map、重规划集合和停止决定。
- 计算并提交 State Transition，输出日志、Summary、冲突统计和可视化。

Actor 已不再直接调用 `FExecutionAlignmentPolicy::Decide(...)` 或 `FExecutionStepProposalBuilder::Build(...)`。`AdvanceExecutionOneStep()` 从上一阶段约 269 行进一步降至约 246 行。

### 保守性说明

- Observed Cell 的读取、`LastObservedCell` 和 `DisplayFromCell` 写入位置未修改。
- Mission ID 排序及 Snapshot 添加顺序保持不变。
- `ShouldDelayThisStep(...)` 仍只在 Agent 可以前进时调用，调用次数和随机流消费顺序未修改。
- Alignment Policy 仍按同一 Mission 顺序执行，Settings、GridMap 和 Snapshot 字段保持不变。
- Proposal Builder 的输入字段、失败跳过条件和初始 replan 条件未修改。
- Alignment 和 Proposal Builder 均不修改 Actor 状态或 `FRandomStream`；统一移入 Pipeline 不改变后续 Conflict/Safety 输入。
- Conflict Resolution、普通 Step Replan、Proposal Synchronizer 和 Final Safety Gate 的调用顺序与实现未修改。
- Final Safety Gate 输入仍在普通重规划及路径写回完成后通过回调重新捕获。
- State Transition、UE 状态写回、日志、Summary、replan timing 和 StructuredExperimentJSON 指标定义未修改。

## 2026-07-15 Execution Runtime Config 与 Controller Registry

### 背景

`FExecutionStepPipeline` 已经统一编排 Alignment、Conflict Resolution、普通 Step Replan 和 Final Safety Gate，但运行参数仍由 `APathPlanningDemoActor` 分散组装，Actor 也直接依赖具体 Pipeline 类型。其他研究者若要替换整个执行控制算法，需要修改 Actor 主流程，扩展边界仍不够清晰。

本轮引入统一运行配置和 Controller Registry：Actor 负责把 Details 参数转换成轻量配置并捕获 UE 运行状态，Controller 负责执行一次完整的执行期决策。默认 Controller 继续委托现有 Pipeline，不改变算法行为。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionRuntimeConfig.h`
- `Source/UTM/Public/Execution/ExecutionControllerTypes.h`
- `Source/UTM/Public/Execution/ExecutionControllerRegistry.h`
- `Source/UTM/Private/Execution/ExecutionControllerRegistry.cpp`

### 修改文件

- `Source/UTM/Public/Execution/ExecutionStepPipeline.h`
- `Source/UTM/Private/Execution/ExecutionStepPipeline.cpp`
- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Execution Runtime Config

新增 `FExecutionRuntimeConfig`，集中保存一次执行控制所需的算法配置：

- `Alignment`：离散对齐参数。
- `ConflictResolution`：冲突仲裁开关、处理轮数和触发重规划的 Hold 阈值。
- `FinalSafetyGate`：最终安全检查开关和最大 Hold 扩张轮数。
- `ReplanService`：最大重规划次数、局部空间扩张半径、局部前瞻步数和最大扩张轮数。
- `ReplanMode`：Disabled、LocalConflictSet 或 GlobalUnfinished。
- `bCheckStaticUTMSafety`：是否启用 LaCAM-UTM 静态安全检查。

Actor 新增 `BuildExecutionRuntimeConfig()`，只在 UE 配置边界读取 Details 属性。`AdvanceExecutionOneStep()` 和 `TryExecutionReplan()` 现在使用同一份配置语义，避免不同阶段重复拼装参数。Delay 随机流、日志开关和可视化参数没有放入 Runtime Config，因为它们仍属于 UE 运行适配层，不是可替换执行算法的输入。

### Controller 扩展契约

新增以下轻量类型：

- `FExecutionControllerStepRequest`：有序 Mission、Agent Snapshot、GridMap、Runtime Config 和动态冲突输入。
- `FExecutionControllerStepCallbacks`：重规划服务、重规划后状态捕获、Final Safety 输入捕获和事件回调。
- `FExecutionControllerStepResult`：最终 Proposal、重规划请求/成功集合、重规划结果和停止执行决定。
- `IExecutionController`：所有执行 Controller 的统一 `RunStep(...)` 接口。

`FExecutionControllerRegistry` 根据 `EExecutionControllerType` 创建并调用 Controller。Details 面板新增 `Execution Controller` 选择项，当前只有 `Default Execution Pipeline`，未暴露给 Blueprint。

默认 `FDefaultExecutionController` 的调用链为：

`APathPlanningDemoActor -> FExecutionControllerRegistry -> FDefaultExecutionController -> FExecutionStepPipeline`

以后新增执行算法时，可以增加枚举值、实现 `IExecutionController` 并在 Registry 注册，不需要改写 Actor 的执行控制主流程。

### Pipeline 配置边界

原 Pipeline 专用 Request、Callbacks 和 Result 已替换为 Controller 公共契约。Pipeline 从 `RuntimeConfig` 读取 Alignment、Conflict Resolution、Replan 和 Final Safety 参数；Actor 回调只提供当前时间步的动态状态。Final Safety Gate 输入仍在普通重规划完成后重新捕获，避免读取重规划前的旧路径和索引。

### 保守性说明

- 默认 Controller 只委托现有 Pipeline，执行阶段顺序未修改。
- Mission 排序、Snapshot 捕获、Delay 判断及随机数消费顺序未修改。
- Alignment、Conflict Resolution、普通 Replan 和 Final Safety Gate 的参数值保持原定义。
- `TryExecutionReplan()` 的 local/global 尝试、扩张、targeted retry、计时和日志语义未修改。
- State Transition、UE 对象写回、停止执行、Summary、冲突统计和可视化流程未修改。
- 本轮没有新增或修改 StructuredExperimentJSON 字段。
- UTMEditor Win64 Development 编译通过，后续使用原有 N200 参数实验进行行为回归。

## 2026-07-15 Execution Step Result Applier 抽取

### 背景

Execution Controller 已经统一输出本时间步的 Proposal、重规划请求/成功集合和停止决定，但 `APathPlanningDemoActor::AdvanceExecutionOneStep()` 仍逐 Mission 解释这些集合、构造 `FExecutionStateTransitionInput`、调用单 Agent State Transition，并汇总是否仍有 Agent 未完成。这些规则属于 Benchmark 固定的结果提交语义，不应隐藏在 UE Actor 中，也不应由不同 Controller 各自实现。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionStepResultApplier.h`
- `Source/UTM/Private/Execution/ExecutionStepResultApplier.cpp`

### 修改文件

- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Applier 当前职责

`FExecutionStepResultApplier::Apply(...)` 现在负责：

- 按调用方提供的有序 Mission ID 逐 Agent 处理，保持确定性顺序。
- 跳过缺少轻量状态、规划路径为空或缺少 Proposal 的 Mission。
- 统一计算 `RequestedReplanMissionIds` 与 `SuccessfulReplanMissionIds` 对单 Agent 状态的含义。
- 构造标准 `FExecutionStateTransitionInput`。
- 调用 `FExecutionStateTransition::Compute(...)` 计算每个 Agent 的下一状态。
- 返回有序的 `FExecutionStepAppliedAgent` 结果，其中包含 Mission ID、重规划标志和 Transition Result。
- 根据所有成功应用的下一状态统一计算 `bAnyActive`。

因此模块层级现在为：

`Execution Controller -> Execution Step Result Applier -> Single-Agent State Transition`

Controller 决定本步动作，Applier 固定整个时间步的批量提交规则，State Transition 固定单 Agent 的状态更新规则。

### Actor 当前职责

Actor 在 Controller 完成及可能的重规划路径写回后，捕获最新的轻量状态、路径长度和最终路径格子，再调用 Applier。Applier 返回后，Actor 继续负责：

- 将纯 Transition Result 写回 `FExecutionAgentState`。
- 更新 `DisplayToCell`、`ActualCells` 和 `LastAlignmentAction`。
- 输出原有 Execution Delay 与 Alignment 日志。
- 执行冲突统计、结束判断、Summary 和可视化。

`FExecutionAgentState` 仍保留在 Actor 层，因为它包含 `TObjectPtr<ADroneActor>` 和 UE 反射字段。Applier 不依赖 `APathPlanningDemoActor`、`ADroneActor`、`UWorld`、`DrawDebug` 或 `UPROPERTY`。

### Benchmark 边界

`ExecutionStepResultApplier` 是固定的 Benchmark Host 组件，不是新的执行算法，也不是 Controller Registry 中的可替换策略。其他研究者只需让 Controller 输出统一 Proposal 和重规划集合；平台始终使用同一个 Applier 更新路径进度、完成状态和统计字段，保证不同执行算法使用相同实验口径。

### 保守性说明

- Controller、Conflict Resolution、普通 Replan 和 Final Safety Gate 的执行顺序未修改。
- Safety Gate 要求停止执行时仍在应用任何 Transition 前生成 Summary 并返回。
- Applier 的轻量状态在 Controller 和重规划完成后捕获，继续使用可能已更新的路径长度、终点和执行下标。
- Mission 处理顺序、空路径/缺少 Proposal 的跳过条件保持不变。
- 请求重规划标志仍等于 requested 与 successful 集合的并集；originally requested 和 replanned 标志保持原定义。
- 单 Agent 状态计算仍复用原 `FExecutionStateTransition::Compute(...)`，字段更新和完成判断未修改。
- Delay 与 Alignment 日志仍在 UE 状态提交后按相同条件和顺序输出。
- 冲突统计、执行结束处理、Summary、可视化和 StructuredExperimentJSON 指标定义未修改。
- UTMEditor Win64 Development 编译通过，后续使用原有 N200 参数实验进行行为回归。

## 2026-07-15 Execution Summary Builder 抽取

### 背景

`APathPlanningDemoActor::BuildExecutionSummary()` 原本直接生成单 Agent 摘要、聚合 makespan/Delay/Alignment 指标、统计离散冲突，并重新扫描实际轨迹计算静态 UTM 冲突。该过程约 140 行，全部是确定性数据计算，不需要访问 UE 场景对象，但长期隐藏在 Actor 中。

本轮按 Summary Builder 与 Editor Services 的分阶段计划，先完成 Summary Builder 稳定节点，暂未修改任何 EUW Editor Service 入口。

### 新增文件

- `Source/UTM/Public/Reporting/ExecutionSummaryTypes.h`
- `Source/UTM/Public/Reporting/ExecutionSummaryBuilder.h`
- `Source/UTM/Private/Reporting/ExecutionSummaryBuilder.cpp`

### 修改文件

- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Summary 类型外移

`FExecutionAgentSummary` 和 `FExecutionSummary` 从 `PathPlanningDemoActor.h` 移到 `Reporting/ExecutionSummaryTypes.h`。两个类型的名称、`USTRUCT(BlueprintType)`、所有 `UPROPERTY` 字段、默认值和 Category 均保持不变。Actor 继续通过原 `LastExecutionSummary` 属性向 Details/Blueprint 暴露相同类型。

### Builder 输入

`FExecutionSummaryBuildRequest` 使用轻量数据表达 Summary 所需的完整输入：

- `AgentStatesByMissionId`：路径、实际轨迹、Delay、Alignment、Replan 和误差统计。
- `Conflicts`：冲突时间步及 vertex/edge 类型。
- `MissionConfigsByMissionId`：静态 UTM 安全检查所需的 Mission 配置。

Actor 新增本地适配函数，在执行结束时将 `FExecutionAgentState`、`FExecutionConflict` 和 Mission Config 缓存转换为上述输入。该转换不访问或复制 Drone Actor。

### Builder 当前职责

`FExecutionSummaryBuilder::Build(...)` 现在负责：

- 计算每个 Agent 的 planned/actual cell count 和 makespan。
- 使用原时间轴补齐规则计算 first mismatch time。
- 根据规划终点与实际轨迹终点判断是否到达目标。
- 汇总完成数量、Delay、Alignment 和 Replan 指标。
- 统计已有 vertex/edge execution conflict 及首次冲突时间。
- 按排序后的 Execution State Map key 扫描所有实际轨迹时间步。
- 复用 `FUTMSafetyModel` 统计静态、Protection Footprint 和 Downwash 冲突。
- 更新首次 UTM 冲突时间和总体首次冲突时间。
- 按 Mission ID 排序最终 Agent Summary。

`APathPlanningDemoActor::BuildExecutionSummary()` 现在只捕获请求并调用 Builder，将返回值写入 `LastExecutionSummary`。

### 模块边界

- Builder 不依赖 `APathPlanningDemoActor`、`FExecutionAgentState`、`FExecutionConflict`、`ADroneActor`、`UWorld`、`DrawDebug` 或 `UE_LOG`。
- `LogExecutionSummary()` 和 StructuredExperimentJSON 组装继续保留在现有 Actor/Reporter 边界。
- Summary 类型仍是 UE 反射数据类型，但 Summary 计算过程只依赖轻量值数据和 `FUTMSafetyModel`。

### 保守性说明

- AgentCount 仍等于 Execution State Map 的元素数量。
- TMap key 与 `State.MissionId` 的原有区别被保留：UTM 扫描使用 Map key，Agent Summary 使用 State 中的 Mission ID。
- 空路径、轨迹超出长度后停留在最后格子、first mismatch 和 reached-goal 规则未修改。
- execution conflict 计数、UTM 冲突扫描时间范围、Mission 配对顺序及首次冲突更新时间条件未修改。
- Summary 字段、日志模板、StructuredExperimentJSON 字段和 Reporter 映射未修改。
- 所有 Mission/No-Fly Zone/City EUW `UFUNCTION` 名称、签名、Category 和 Actor 所有权未修改。
- UTMEditor Win64 Development 编译通过；UHT 成功生成外移 Summary 类型的反射代码，后续使用原有 N200 参数实验核对 JSON 结果。

## 2026-07-15 Editor Grid Service 与 Mission Editor Service 拆分

### 背景

`APathPlanningDemoActor` 原本直接实现 Mission 编辑网格构建、随机任务生成、Marker 查找/生成/清理、任务校验和 Marker 回读。EUW 依赖 Actor 上的 6 个 `BlueprintCallable` 函数，因此不能通过删除、改名或改变签名的方式直接迁移接口。

本轮采用稳定门面方式：EUW 继续调用原 Actor 函数，Actor 将 Details/World 状态组装为明确 Request，再委托 Editor Service 执行。

### 新增文件

- `Source/UTM/Public/EditorServices/EditorGridService.h`
- `Source/UTM/Private/EditorServices/EditorGridService.cpp`
- `Source/UTM/Public/EditorServices/MissionEditorService.h`
- `Source/UTM/Private/EditorServices/MissionEditorService.cpp`

### 修改文件

- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Editor Grid Service

`FEditorGridService::BuildGridForMissionEditing(...)` 现在负责：

- 将 Grid Origin、Grid Dimension 和 Cell Size 写入目标 `FGridMap3D`。
- 使用调用方提供的 World、Ignore Actor 集合和 Debug 参数构建 Occupancy Grid。
- 保留原 `EditorBuildGridForMissionEditing done` 日志。

Actor 继续负责把自身加入 Ignore Actor 集合；`FMissionEditorService::AppendMissionMarkerActors(...)` 将所有带 `MissionMarker` Tag 的 Marker 追加到该集合。因此网格构建仍忽略 Demo Actor 和已有 Mission Marker。

### Mission Editor Service

`FMissionEditorService` 现在负责：

- 按原 Tag 规则查找 Mission Marker。
- 使用原随机种子、Grid 范围和重复 Start/Goal 设置生成 Mission Config。
- 使用 `FUTMSafetyModel` 检查新任务与已生成任务的 Start/Goal 静态安全冲突。
- 清理已有 Mission Marker。
- 按 MissionConfigs 生成 Start/Goal Marker，设置 Owner、Tag、Mission ID、Marker Type 和 Cell。
- 使用原 `FPlanningInputValidator` 校验 Mission Config，并报告重复 ID/Start/Goal。
- 将 Marker 世界坐标扣除显示偏移、对齐到 Grid，并回写按 Mission ID 排序的 MissionConfigs。

服务使用按操作划分的 Request，只接收所需的 World、Grid、Validator、MarkerClass、MissionConfigs 和 Details 参数，不接收或包含 `APathPlanningDemoActor`。

### EUW 稳定门面

以下 6 个 Actor `UFUNCTION(BlueprintCallable, Category = "Mission Editor")` 名称、签名、Category 和所有权保持不变：

- `EditorBuildGridForMissionEditing()`
- `EditorGenerateRandomMissionConfigs()`
- `EditorSpawnMissionMarkers()`
- `EditorClearMissionMarkers()`
- `EditorValidateMissionConfigs()`
- `EditorReadMissionMarkersToConfigs()`

EUW Blueprint 仍以原方式持有并调用 `APathPlanningDemoActor`。`MissionConfigs`、`MissionMarkerClass`、`MarkerZOffset`、随机种子及所有生成设置仍是 Actor 的原 `UPROPERTY`，无需修改 EUW 资源。

### 保守性说明

- `EditorGenerateRandomMissionConfigs()` 仍先构建 Grid，再清空并生成 MissionConfigs。
- 每个 Mission 的 Start/Goal 仍分别最多尝试 500 次，Mission ID 仍从 1 开始。
- `FRandomStream` 构造、RandRange 调用位置和失败后继续处理下一 Mission 的顺序未修改。
- 静态 UTM Start/Goal 冲突检查仍按当前 MissionConfigs 顺序执行。
- MarkerClass 为空时仍在清理旧 Marker 前返回。
- Spawn 仍先清理旧 Marker，再检查 World，并为 Start 后 Goal 的顺序生成 Marker。
- Marker Tag、Owner、位置偏移、Collision Handling 和 `UpdateVisual()` 调用未修改。
- Marker 回读仍将非 Start 类型视为 Goal，仍跳过缺少 Goal 的 Mission，最终仍按 Mission ID 排序。
- 原 Mission Editor 日志文本和输出时机保持不变。
- No-Fly Zone 与 City EUW 接口尚未迁移；它们调用 `EditorBuildGridForMissionEditing()` 时会通过新的 Grid Service 使用相同参数。
- UTMEditor Win64 Development 编译通过，UHT 报告 0 个反射代码改动。编译中的 `PBSPlanner.cpp` `RemoveAt` 弃用警告为既有问题，与本轮修改无关。

### EUW 手动回归清单

1. 在原 EUW 中执行 Build Grid，确认完成日志和占用网格显示正常。
2. 使用固定 RandomSeed 生成随机 Mission，确认数量以及 Start/Goal 配置可重复。
3. Spawn Mission Markers，确认每个 Mission 生成一对 Start/Goal Marker。
4. 移动 Marker 后执行 Read Markers，确认 MissionConfigs 坐标对齐到 Grid 且按 Mission ID 排序。
5. 执行 Validate Missions，确认合法任务、重复 ID/Start/Goal 日志正常。
6. 执行 Clear Mission Markers，确认 Marker 全部删除且删除数量日志正确。
7. 再运行一次原 N200 参数实验，确认 Editor Service 拆分没有影响运行期规划和执行流程。

## 2026-07-18 No-Fly Zone Editor Service 拆分

### 背景

`APathPlanningDemoActor` 原本直接实现 No-Fly Zone Config 新增与随机生成、Marker 查找/生成/清理、Marker 回读以及配置校验。为继续减轻 Actor 职责，本轮只拆分 No-Fly Zone 编辑逻辑；City Editor 保持原状，等待本轮 6 个 EUW 接口回归通过后再单独迁移。

### 新增文件

- `Source/UTM/Public/EditorServices/NoFlyZoneEditorService.h`
- `Source/UTM/Private/EditorServices/NoFlyZoneEditorService.cpp`

### 修改文件

- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### No-Fly Zone Editor Service

`FNoFlyZoneEditorService` 现在负责：

- 根据 Grid 中心、默认尺寸和默认持续时间新增 No-Fly Zone Config。
- 按 `NoFlyZoneMarker` Tag 查找并清理 Marker。
- 规范化并限制 Min/Max Cell 后，按 Config 生成 Marker、设置 Owner、Tag、Box Extent 和显示状态。
- 将 Marker 的位置与 Box Extent 转换回 Grid Cell，回写并按 Zone ID 排序 NoFlyZoneConfigs。
- 校验重复 Zone ID、Grid 边界、时间窗口以及区域中的阻塞 Cell 数量。
- 使用固定随机种子和原有尝试次数生成包含至少一个可用 Cell 的随机 No-Fly Zone Config。

服务通过按操作划分的 Request 接收 World、Grid、Marker Class、NoFlyZoneConfigs 和 Details 参数，不包含或反向依赖 `APathPlanningDemoActor`。服务本身仍是 UE-aware Editor Service，可以使用 `UWorld`、Actor、Component 和 Spawn API。

### EUW 稳定门面

以下 6 个 Actor `UFUNCTION(BlueprintCallable, Category = "No-Fly Zone Editor")` 的名称、签名、Category 和所有权保持不变：

- `EditorGenerateRandomNoFlyZoneConfigs()`
- `EditorAddNoFlyZoneConfig()`
- `EditorSpawnNoFlyZoneMarkers()`
- `EditorClearNoFlyZoneMarkers()`
- `EditorReadNoFlyZoneMarkersToConfigs()`
- `EditorValidateNoFlyZones()`

EUW 仍调用原 Actor 接口。Actor 负责读取自身 `UPROPERTY`、在需要时先调用 `EditorBuildGridForMissionEditing()`，然后组装 Request 并转发给 Service；因此无需修改 EUW Blueprint 资源。

### 保守性说明

- Add 操作仍使用当前最大 Zone ID 加 1，并保留原 Grid 中心、默认尺寸和时间窗口计算。
- Spawn 操作仍先检查 Marker Class 和 World，再清理旧 Marker；Config 遍历顺序、Cell 规范化、位置偏移、Collision Handling 和 `UpdateVisual()` 调用未修改。
- Clear 和 Read 仍只处理带 `NoFlyZoneMarker` Tag 的 Marker。
- Read 仍扣除 `MarkerZOffset`、按半 Cell 修正 Box 边界、限制到 Grid，并按 Zone ID 排序。
- Validate 和随机生成仍先通过原 Actor 接口构建 Grid。
- 随机生成仍使用相同 `FRandomStream` 种子、RandRange 调用顺序、每个 Zone 最多 200 次尝试以及至少一个 Free Cell 的接受条件。
- 原 No-Fly Zone 日志文本和主要输出时机保持不变。
- City Editor 的函数、属性和实现未修改。

### EUW 手动回归清单

1. 执行 Generate Random No-Fly Zones，使用固定种子确认数量、Cell 范围和时间窗口可重复。
2. 执行 Add No-Fly Zone，确认新增 Zone ID、默认区域和时间窗口正确。
3. 执行 Spawn No-Fly Zone Markers，确认每个 Config 生成对应 Marker，位置与 Box 尺寸正确。
4. 移动或缩放 Marker 后执行 Read No-Fly Zone Markers，确认 Config 回写、Grid 对齐和 Zone ID 排序正确。
5. 执行 Validate No-Fly Zones，确认正常配置及重复 ID、越界、无效时间窗口日志与原版本一致。
6. 执行 Clear No-Fly Zone Markers，确认带目标 Tag 的 Marker 全部清理且删除数量日志正确。
7. 六个接口通过后，再运行原 N200 参数实验，确认规划和执行流程未受 Editor Service 拆分影响。

### 回归结果

- 2026-07-18：上述 6 个 No-Fly Zone EUW 接口已依次完成人工测试，未发现异常。在该稳定基础上继续拆分 City Editor Service。

## 2026-07-18 City Editor Service 拆分

### 背景

`APathPlanningDemoActor` 原本直接负责清理城市建筑、加载 Cube Mesh、设置 Static Mesh/碰撞/缩放，并实现 Manhattan、Residential、Industrial 和 Mixed 四种布局的随机生成循环。为继续减轻 Benchmark Host 的 UE 编辑器职责，本轮将这些实现迁入独立的 UE-aware City Editor Service，同时保留 Actor 上的原 EUW 门面和实验元数据状态。

### 新增文件

- `Source/UTM/Public/EditorServices/CityEditorService.h`
- `Source/UTM/Private/EditorServices/CityEditorService.cpp`

### 修改文件

- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### City Editor Service

`FCityEditorGenerateRequest` 显式保存一次城市生成所需的输入：World、Grid Origin、City Seed、Block 数量与尺寸、Road Width，以及建筑 Width/Depth/Height 范围。它不包含或反向依赖 `APathPlanningDemoActor`。

`FCityEditorService` 现在负责：

- 按 `CityBuilding` Tag 遍历并清理城市建筑 Actor。
- Spawn `AStaticMeshActor`，加载原 Engine Cube Mesh，设置 Static Mobility、BlockAll 碰撞和 Actor Scale。
- 使用固定 City Seed 生成 Manhattan 布局。
- 使用固定 City Seed 生成 Residential 布局。
- 使用固定 City Seed 生成 Industrial 布局。
- 使用固定 City Seed 为每个 Block 选择子区域类型并生成 Mixed 布局。

Service 可以直接依赖 `UWorld`、`AStaticMeshActor`、`UStaticMeshComponent` 和 UE Spawn/LoadObject API，因为它属于 Editor Services，而不是纯算法模块；但它不持有 Actor 状态，所有生成输入都由 Request 提供。

### Actor 保留职责

Actor 继续负责：

- 保留 Details 面板中的全部 City Generator `UPROPERTY`。
- 保留各 EUW 入口原有的 World、Block Count 和 Manhattan Block Size 校验。
- 在生成前写回 `CityLayoutType`，确保后续 Planning Stats 和 StructuredExperimentJSON 的 `map_type` 行为不变。
- 根据 `bAutoRebuildGridAfterCityGenerate` 调用原 `EditorBuildGridForMissionEditing()`。
- 保留各布局生成完成的原 EUW 日志。

Actor 内原 `SpawnCityBuilding(...)` 和四个 `GenerateCityLayout_*()` 私有函数已删除。Actor 的城市生成入口现在只负责构造 Request、维持 Host 状态和编排 Service。

### EUW 稳定门面

以下 5 个 Actor `UFUNCTION(BlueprintCallable, Category = "City Generator")` 的名称、签名、Category 和所有权保持不变：

- `EditorGenerateManhattanCity()`
- `EditorGenerateResidentialDistrict()`
- `EditorGenerateIndustrialPark()`
- `EditorGenerateMixedUrbanArea()`
- `EditorClearCityEnvironment()`

EUW Blueprint 仍以原方式调用 `APathPlanningDemoActor`，无需修改现有 EUW 资源。

### 保守性说明

- 每种布局仍在生成前清理带 `CityBuilding` Tag 的旧建筑。
- `FRandomStream` 仍在清理后使用相同 `CitySeed` 构造。
- 四种布局的 Block 遍历顺序、BuildingCount 范围、尺寸缩放系数、Margin、位置随机顺序和 Z 中心计算未修改。
- 原实现与新 Service 中的随机调用结构均为 7 处 `RandRange` 和 20 处 `FRandRange`，各布局内部的调用顺序保持一致。
- Manhattan 仍额外检查 `CityBlockSize > 0`；其他三种布局仍只检查 World 和 Block Count，未借重构改变既有校验规则。
- 建筑 Tag、Cube Mesh 路径、Mobility、Collision Profile、位置和缩放规则未修改。
- `CityLayoutType`、自动 Grid 重建时机、完成日志和 Map Type 实验元数据来源未修改。

### EUW 手动回归清单

1. 使用固定 CitySeed 执行 Generate Manhattan City，确认建筑数量日志、位置、高度和重复生成结果与原版本一致。
2. 执行 Generate Residential District，确认建筑较矮、数量与分布正常，并替换旧城市建筑。
3. 执行 Generate Industrial Park，确认建筑数量较少、体积较大，并替换旧城市建筑。
4. 执行 Generate Mixed Urban Area，确认三种子区域风格能够混合生成，并替换旧城市建筑。
5. 检查每次生成后的 `CityLayoutType` 和自动 Build Grid 行为，确认日志及 Occupancy Grid 正常。
6. 执行 Clear City Environment，确认所有带 `CityBuilding` Tag 的建筑被清理，删除数量日志正确。
7. 再运行原 N200 参数实验，确认 `map_type`、规划和执行结果未受 Editor Service 拆分影响。

### 回归结果

- 2026-07-18：上述 City Generator EUW 接口已完成人工测试，原 N200 参数回归实验也已完成，均未发现异常。

## 2026-07-18 Execution Runtime Session 第一阶段：Runtime State Types 外移

### 背景

建立独立 `Execution Runtime Session` 前，执行期配置和状态类型仍直接定义在 `PathPlanningDemoActor.h`，导致 Execution 模块需要通过 Actor Header 才能了解完整运行状态。本阶段先整理类型所有权，不迁移状态数据和执行逻辑。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionRuntimeStateTypes.h`

### 外移类型

以下类型从 `PathPlanningDemoActor.h` 原样移动到 `ExecutionRuntimeStateTypes.h`：

- `EExecutionDelayMode`
- `EExecutionReplanMode`
- `FExecutionReplanTimingStats`
- `FExecutionAgentState`
- `FExecutionConflict`
- `FAgentDelayConfig`

### Actor 调整

`PathPlanningDemoActor.h` 改为包含 `Execution/ExecutionRuntimeStateTypes.h`。Actor 中的以下成员、函数签名和 Details 属性继续使用原类型名称：

- `ExecutionStates`
- `ExecutionConflicts`
- `ExecutionReplanTimingStats`
- `DelayMode`
- `AgentDelayConfigs`
- `ExecutionReplanMode`

### 保守性说明

- 所有 `UENUM`/`USTRUCT` 名称、`BlueprintType`、`UPROPERTY`、Category、Meta、字段顺序和默认值保持不变。
- `FExecutionAgentState::Drone` 本阶段继续保留，尚未拆分纯 Runtime State 与 UE View Binding。
- `ResetExecutionCache()`、`InitializeExecutionStates()`、`AdvanceExecutionOneStep()` 和 `TryExecutionReplan()` 均未修改。
- Execution 状态仍由 `APathPlanningDemoActor` 持有，本阶段只建立后续 Runtime Session 所需的独立类型位置。
- Delay 随机流、Mission ID 顺序、冲突检测、重规划、Summary 和 StructuredExperimentJSON 行为均未修改。

### 后续计划

下一阶段可在该类型边界上抽取 Session 初始化职责，包括 Reset、Mission Config 缓存、初始 Agent State 构造和 Snapshot 构造；每个阶段继续使用原 N200 参数实验回归。

### 回归结果

- 2026-07-18：Runtime State Types 外移后，原 N200 参数实验回归通过，未发现异常。

## 2026-07-18 Execution Runtime Session 第二阶段：Session Builder

### 背景

第一阶段只移动了类型定义，Mission Config Map、初始 Agent State 和 Execution Snapshot 仍由 `APathPlanningDemoActor` 逐项构造。本阶段新增无 `UWorld` 依赖的 Session Builder，先迁移确定性的纯数据构造，Actor 继续持有状态和执行生命周期。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionRuntimeSessionBuilder.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeSessionBuilder.cpp`

### Builder 职责

`FExecutionRuntimeSessionBuilder` 现在负责：

- 将 `TArray<FDroneMissionConfig>` 构造为按 Mission ID 索引的 Map。
- 根据 Planned Cell Paths、Mission Config Map 和 Grid 构造初始 `FExecutionAgentState`。
- 保留空路径跳过、单点路径立即完成、起点 Actual Cell、Goal Cell/World 和 Alignment 初始状态规则。
- 根据当前 Agent States、时间步、重规划次数和观察 Cell 回调构造 `FExecutionSnapshot`。

### Actor 保留职责

- 初始化 `ExecutionRandom`、时间步、重规划统计和 Running 状态。
- 将 Builder 返回的 Agent State 与 `SpawnedDroneByMissionId` 绑定。
- 将 Drone Actor 放置到初始 Cell。
- 提供 `GetObservedExecutionCell()` 回调，从 UE Actor 世界位置获取观察 Cell。
- 执行初始冲突检测、Summary、日志和后续 `AdvanceExecutionOneStep()`。

### 保守性说明

- `ResetExecutionCache()`、Delay 随机判定和 `AdvanceExecutionOneStep()` 本阶段未迁移。
- `FExecutionAgentState::Drone` 仍由 Actor 写入，Builder 不访问 `ADroneActor`、`UWorld` 或 DrawDebug。
- 初始状态字段、Mission Config Goal 覆盖规则和 Snapshot 字段逐项保持不变。
- Planned Path Map 和 Agent State Map 的原 `TMap` 遍历顺序保持不变。
- 初始 Drone 定位仍发生在冲突检测和执行开始之前。
- Replan、Summary、StructuredExperimentJSON 和所有指标字段未修改。

### 回归建议

继续运行原 N200 参数实验，重点核对 completed count、makespan、delay、alignment、replan timing、conflict count 和 StructuredExperimentJSON。

### 回归结果

- 2026-07-18：Session Builder 接入后，原 N200 参数实验回归通过，未发现异常。

## 2026-07-18 Execution Runtime Session 第三阶段：Step Processor

### 背景

Session Builder 已负责初始状态和 Snapshot 构造，但 `AdvanceExecutionOneStep()` 仍在 Actor 内逐 Mission 完成排序、观察位置采样、Delay 判定、Controller 输入构造以及标准状态转移写回。本阶段把这些无 `UWorld` 依赖的 Runtime State 操作集中到独立的 Step Processor，继续保留 Actor 作为 UE Benchmark Host。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionRuntimeSessionStepProcessor.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeSessionStepProcessor.cpp`

### Step Processor 职责

`FExecutionRuntimeSessionStepProcessor` 现在负责：

- 按 Mission ID 排序当前 Agent States。
- 通过 Actor 提供的观察 Cell 回调更新 `LastObservedCell` 和 `DisplayFromCell`。
- 通过 Actor 提供的 Delay 回调生成 `FExecutionAgentSnapshot`，保持原随机流调用条件和顺序。
- 从 Runtime State 构造 `FExecutionConflictResolutionInput`。
- 从 Runtime State 构造重规划 Proposal 同步输入和 Final Safety Gate 输入。
- 复用原 `FExecutionStepResultApplier` 计算标准状态转移，并将结果批量写回 `FExecutionAgentState`。

### Actor 保留职责

- 在每个执行步开始时调用 `UpdateExecutionVisuals()` 并推进时间步。
- 通过 `GetObservedExecutionCell()` 读取 Drone Actor 的世界位置。
- 使用 `ExecutionRandom` 和 Details 配置执行 Delay 判定。
- 构造 Runtime Config、选择 Execution Controller，并提供 `TryExecutionReplan()` 回调。
- 输出 Delay、Alignment、Conflict Resolution 和 Final Safety Gate 日志。
- 执行实际冲突记录、停止执行、更新显示以及构造 Execution Summary。

### 保守性说明

- Mission ID 排序、Snapshot 字段、Delay 回调调用条件和随机数调用顺序保持不变。
- Conflict Resolution 和 Final Safety Gate 的输入字段逐项保持不变。
- `FExecutionStateTransition` 和 `FExecutionStepResultApplier` 未修改，只迁移其输入捕获和结果提交位置。
- 状态提交仍发生在 Controller 完成之后、Delay/Alignment 日志和实际冲突检测之前。
- Step Processor 不访问 `ADroneActor`、`UWorld`、Actor Transform、DrawDebug 或 `UE_LOG`。
- `TryExecutionReplan()`、Final Safety Gate 策略、冲突检测、Summary 和 StructuredExperimentJSON 均未修改。

### 回归建议

继续运行原 N200 参数实验，重点核对 Delay 随机序列、completed count、actual makespan、alignment、Final Safety Gate、replan timing、conflict count 和 StructuredExperimentJSON。

### 回归结果

- 2026-07-18：Session Step Processor 接入后，原 N200 参数实验回归通过，未发现异常。

## 2026-07-18 Execution Runtime Session 第四阶段：Replan Committer

### 背景

执行期重规划的候选选择、Planner 调用、PostCheck 和多轮 Coordinator 已经位于 Execution 模块，但重规划成功后的路径整合字段写回，以及 local/global attempt 耗时和 applied replan 次数累计仍直接写在 Actor 中。本阶段把这些 Runtime Session 写操作集中到独立 Replan Committer。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionRuntimeSessionReplanCommitter.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeSessionReplanCommitter.cpp`

### Replan Committer 职责

`FExecutionRuntimeSessionReplanCommitter` 现在负责：

- 复用 `FExecutionReplanPathIntegrator` 将新的 Cell Path 合并到当前执行时间线。
- 写回 `PlannedCells`、`ExecutedPlanIndex`、Goal、Conflict Hold 和 Alignment Lost 状态。
- 更新按 Mission ID 缓存的 Planned Cell Paths。
- 返回实际完成写回的 Mission ID 集合。
- 按 local/global 类型累计 attempt count、total time 和 max time。
- 累计 `TotalExecutionReplanCount`，继续使用 Coordinator 返回的 `AppliedReplanCount`。

### Actor 保留职责

- 构造 `FExecutionReplanCoordinatorRequest` 和 Planner/Apply/Event 回调。
- 调用 `TryExecutionReplan()` 并维持原成功、失败和最大重规划次数判断。
- 将 Committer 已提交的 Cell Path 转为 World Path，更新用于 UE 路径显示的 `LastPlannedPathsByMission`。
- 输出 Coordinator、Planner 失败和重规划上限日志。

### 保守性说明

- 路径整合继续调用原 `FExecutionReplanPathIntegrator::Integrate()`，时间线 Hold 和 Executed Plan Index 规则未修改。
- Candidate Mission 遍历顺序、缺失 State/Config/Path 时立即失败以及前序 Mission 部分写回行为保持不变。
- GoalWorld、Conflict Hold 清零和 Alignment Lost 清除字段保持不变。
- World Path 仍由 Actor 使用原 `GridMap.CellToWorld()` 逐 Cell 构造。
- local/global attempt 指标的加法和 max 规则逐项保持不变。
- Coordinator、Planner、PostCheck、Final Safety Gate、Summary 和 StructuredExperimentJSON 未修改。
- Replan Committer 不访问 `ADroneActor`、`UWorld`、Actor Transform、DrawDebug 或 `UE_LOG`。

### 回归建议

继续运行原 N200 参数实验，重点核对 applied replan count、local/global attempt count、total/max time、planned/actual makespan、alignment replan success、conflict count 和 StructuredExperimentJSON。

### 回归结果

- 2026-07-18：Session Replan Committer 接入后，原 N200 参数实验回归通过，未发现异常。

## 2026-07-18 Execution Runtime Session 第五阶段：Session 状态所有权集中

### 背景

前四个阶段已经外移 Runtime 类型、初始化构造、标准 Step 状态处理和 Replan 写回，但 Mission Config Map、Agent States、Conflict 历史、随机流、时间步和重规划统计仍是 `APathPlanningDemoActor` 的多个独立成员。本阶段建立真正的 `FExecutionRuntimeSession` 状态对象，统一这些执行期数据的所有权和生命周期。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionRuntimeSession.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeSession.cpp`

### Session 持有状态

`FExecutionRuntimeSession` 现在集中持有：

- `MissionConfigsById`
- `AgentStatesByMissionId`
- `Conflicts`
- `Random`
- `bRunning`
- `TimeStep`
- `TotalReplanCount`
- `ReplanTimingStats`

Actor 中原来对应的八个独立成员已删除，替换为单一 `ExecutionSession` 成员。

### Session 生命周期

- `Reset()` 清理 Mission Config、Agent State 和 Conflict，并复位 Running、时间步和重规划统计。
- `PrepareForExecution()` 保留 Mission Config，清理 Agent State 和 Conflict，按 `ExecutionRandomSeed` 初始化随机流，并复位 Running、时间步和重规划统计。
- Actor 的 `ResetExecutionCache()` 和 `InitializeExecutionStates()` 分别调用上述两个生命周期入口。

### Actor 保留状态和职责

- `PlannedCellPathsByMission` 和 `LastPlannedPathsByMission` 继续作为规划结果及 UE World Path 显示缓存。
- `SpawnedDrones` 和 `SpawnedDroneByMissionId` 继续持有 UE 场景对象绑定。
- `ExecutionAccumulator` 继续由 Tick Host 持有，因为它表示 UE 帧时间累计，而不是离散执行算法状态。
- `LastExecutionSummary`、Details 配置、日志和 StructuredExperimentJSON 继续留在 Actor/Reporter 边界。
- Tick、Drone Transform、DrawDebug、Planner 回调和 Summary 输出时机均未修改。

### 保守性说明

- 所有 Runtime 字段类型、默认值和每个执行阶段的读写顺序保持不变。
- `Reset()` 与旧 `ResetExecutionCache()` 一致，不额外初始化或消耗随机流。
- `PrepareForExecution()` 与旧 `InitializeExecutionStates()` 一致，在生成执行步 Delay 前使用相同 Seed 初始化随机流。
- Mission Config 在初始化 Agent State 前保持可用，Reset 与重新缓存顺序未改变。
- Tick 的 Running 判断、时间步推进、Conflict 记录、Replan 上限判断和统计读取均只改为通过 Session 访问。
- Summary 和 StructuredExperimentJSON 使用同一组状态和统计值，字段名称及计算公式未修改。
- `FExecutionRuntimeSession` 不访问 `UWorld`、Drone Transform、DrawDebug 或 `UE_LOG`。
- `FExecutionAgentState::Drone` 本阶段仍保留，Session 状态与 UE View Binding 的进一步拆分留待后续阶段。

### 回归建议

继续运行原 N200 参数实验，重点核对 Delay 随机序列、completed count、planned/actual makespan、Conflict、Alignment、applied replan count、local/global replan timing 和 StructuredExperimentJSON。

### 回归结果

- 2026-07-18：`FExecutionRuntimeSession` 状态所有权集中后，原 N200 参数实验回归通过，未发现异常。

## 2026-07-31 Runtime State 与 Drone View Binding 第一阶段：主流程旁路

### 背景

`FExecutionRuntimeSession` 已集中持有执行期逻辑状态，但 `FExecutionAgentState` 仍通过 `Drone` 字段保存 `ADroneActor` 指针。同时，Actor 已经通过 `SpawnedDroneByMissionId` 保存同一组 Mission-to-Drone 绑定，形成重复所有权入口。本阶段先让 C++ 执行主流程统一使用 Actor 绑定表，同时保留原反射字段，降低后续删除字段时的 Blueprint 兼容风险。

### 修改文件

- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Actor View Binding 入口

新增 `FindExecutionDrone(int32 MissionId)`，统一从 `SpawnedDroneByMissionId` 查询 Drone。以下流程不再读取 `FExecutionAgentState::Drone`：

- 初始化执行状态后的 Drone 起点定位。
- `GetObservedExecutionCell()` 中的世界位置读取。
- `UpdateExecutionVisuals()` 中的 Transform 插值写入。
- `DrawExecutionDebugForState()` 中的 Debug Text 定位。

### 兼容策略

- `FExecutionAgentState::Drone` 本阶段继续保留。
- 初始化时继续执行一次兼容赋值，避免已有 Blueprint 或调试逻辑立即失去该字段内容。
- Execution Runtime Session、Step Processor、Replan Committer 和 Controller 不再通过该字段访问场景对象。
- 下一阶段在完成 N200 和 Blueprint 编译检查后，再删除 `Drone` 反射字段及 `ADroneActor` 前置声明。

### 保守性说明

- `SpawnedDroneByMissionId` 原本就是 Drone Spawn 后的绑定来源，本阶段没有新增第二张映射表。
- Drone 缺失时仍按原逻辑回退到 `ActualCells` 或 Planned Cell。
- 初始化位置、观察位置、视觉插值和 Debug Text 使用的仍是同一个 Mission ID 对应 Drone。
- Planner、Scheduler、Execution Controller、Delay、Alignment、Conflict Resolution、Replan、Summary 和 StructuredExperimentJSON 均未修改。
- EUW 和 Details 接口未修改。

### 回归建议

运行原 N200 参数实验，并额外检查 Drone 动画、Execution Debug Text、`bAutoSpawnDrones=false` 回退流程以及 Blueprint 编译状态。

### 回归结果

- 2026-07-31：Runtime State 与 Drone View Binding 第一阶段接入后，原 N200 参数实验回归通过，未发现异常。
- 2026-07-31：使用 UE 5.5 `CompileAllBlueprints -ProjectOnly` 完成删除 `Drone` 字段前的全项目 Blueprint 编译基线检查；命令退出码为 0，结果为 0 errors、0 warnings、0 blueprints failed to load。

## 2026-07-31 Runtime State 与 Drone View Binding 第二阶段：删除 Drone 状态字段

### 背景

第一阶段已经让初始化定位、Observed Cell、视觉插值和 Debug Text 全部通过 Actor 的 `SpawnedDroneByMissionId` 查询 Drone，并完成 N200 与全项目 Blueprint 编译基线验证。本阶段正式删除 Runtime State 中不再使用的 UE 场景对象引用。

### 修改文件

- `Source/UTM/Public/Execution/ExecutionRuntimeStateTypes.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 删除内容

- 删除 `FExecutionAgentState::Drone` 的 `UPROPERTY` 和 `TObjectPtr<ADroneActor>` 字段。
- 删除 `ExecutionRuntimeStateTypes.h` 中的 `ADroneActor` 前置声明。
- 删除初始化 Runtime State 时的兼容 `State.Drone = Drone` 赋值。

### 新边界

- `FExecutionRuntimeSession` 只保存 Mission、Cell Path、执行索引、Delay、Alignment、Conflict 和 Replan 等逻辑状态。
- `APathPlanningDemoActor` 通过 `SpawnedDroneByMissionId` 独占 Mission-to-Drone View Binding。
- UE 世界位置通过 `GetObservedExecutionCell()` 转换为 Cell 后再进入 Execution 模块。
- Execution 计算产生的 Display Cell 由 Actor 转换为 World Transform 并应用到 Drone。

### 保守性说明

- 第一阶段建立的 `FindExecutionDrone()` 和所有 Drone 访问路径未修改。
- Drone Spawn、销毁、初始定位、动画插值、Debug Text 和无 Drone 回退逻辑未修改。
- Planner、Scheduler、Execution Controller、Delay、Alignment、Conflict Resolution、Replan、Summary 和 StructuredExperimentJSON 未修改。
- `FExecutionAgentState` 仍是 UE `USTRUCT(BlueprintType)`，本阶段完成的是场景对象解耦，不是完全移除 CoreUObject 反射依赖。

### 验证计划

1. 编译 UTMEditor，确认 Unreal Header Tool 能正确更新结构体反射布局。
2. 重新运行 `CompileAllBlueprints -ProjectOnly`，与删除字段前的 0 errors、0 warnings、0 failed baseline 对比。
3. 运行原 N200 参数实验，检查执行结果、Drone 动画和 Debug Text。

### 验证结果

- 2026-07-31：UTMEditor Win64 Development 编译通过；Unreal Header Tool 使用 `-WarningsAsErrors` 成功更新反射代码，共写入 3 个生成文件，C++ 编译和链接均无错误。
- 2026-07-31：删除 `FExecutionAgentState::Drone` 后重新运行 UE 5.5 `CompileAllBlueprints -ProjectOnly`；命令退出码为 0，结果为 0 errors、0 warnings、0 blueprints failed to load，与删除字段前的基线一致。
- 2026-07-31：在删除 `FExecutionAgentState::Drone` 后运行原 N200 参数实验；StructuredExperimentJSON 检查正常，未发现执行结果异常。

## 2026-07-31 Execution Runtime Coordinator 第一阶段：Step 编排入口

### 背景

Execution 已经具备 Runtime Session、Step Processor、Controller Registry、Step Pipeline 和 Result Applier，但 `APathPlanningDemoActor::AdvanceExecutionOneStep()` 仍直接了解并连接上述内部组件。本阶段新增 Runtime Coordinator，先建立固定的执行步入口，不同时改变 Delay、Replan、冲突统计或结束语义。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinatorTypes.h`
- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinator.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeCoordinator.cpp`

### Coordinator 当前职责

- 校验 Runtime Session 和 Grid Map 输入。
- 按原位置推进 `ExecutionSession.TimeStep`。
- 构造 `FExecutionRuntimeStepPrepareRequest` 并调用 Session Step Processor。
- 构造标准 Controller Request 和 Controller Callbacks。
- 从同一个 Session 构造 Replan Proposal State 和 Final Safety Gate Input。
- 调用当前 `ExecutionControllerRegistry::RunStep()`。
- Controller 未要求停止时，调用 Session Step Processor 统一写回 Agent State。
- 返回 Controller Result、Apply Result、TimeStep 和失败原因。

### Actor 保留职责

- `Tick`、`ExecutionAccumulator` 和视觉段落结束对齐。
- 从 Drone World Transform 解析 Observed Cell。
- Delay 决策和随机流消费。
- `TryExecutionReplan()` 及 Planner 接入。
- Delay、Alignment、Conflict Resolution 和 Final Safety Gate 日志。
- 实际 vertex/edge conflict 记录。
- Drone 视觉更新、停止执行、Execution Summary 和 StructuredExperimentJSON 输出。

### 保守性说明

- `UpdateExecutionVisuals(1.f)` 仍在 timestep 推进前执行。
- timestep 仍在 Prepare Step 前只增加一次。
- Agent 排序、Observed Cell 写回、Delay 随机调用顺序、Controller Request 内容和 Apply 顺序未修改。
- Replan 仍通过同步 Actor 回调执行，并在同一个 Controller Step 内更新原 Session。
- Controller 要求停止时仍不会 Apply Step Result。
- 普通 Apply 后仍由 Actor 先输出事件，再记录实际冲突，最后判断是否全部完成。
- Controller 本阶段仍按原逻辑每个 timestep 创建一次；持久化 Controller 生命周期留待后续阶段。
- EUW、Details、Planner、Scheduler、JSON 字段和 Blueprint 接口未修改。

### 验证计划

1. 编译 UTMEditor Win64 Development。
2. 运行原 N200 参数实验，对比 StructuredExperimentJSON，重点检查 actual makespan、delay、alignment、conflict 和 local/global replan 指标。
3. 检查 Drone 动画、Execution Debug Text 以及执行结束时 Summary 的输出时机。

### 验证结果

- 2026-07-31：UTMEditor Win64 Development 编译通过；UnrealHeaderTool 使用 `-WarningsAsErrors` 成功生成反射代码，新 `ExecutionRuntimeCoordinator.cpp`、修改后的 `PathPlanningDemoActor.cpp` 以及 UTM 模块均完成编译和链接。
- 编译中仅保留既有 `PBSPlanner.cpp` 的 UE 5.5 `TArray::RemoveAt(bool)` 弃用警告，与本阶段修改无关。
- 2026-07-31：运行原 N200 参数实验，执行流程和 StructuredExperimentJSON 检查正常，未发现异常。

## 2026-07-31 Execution Runtime Coordinator 第二阶段：实际冲突记录

### 背景

第一阶段已经让 Runtime Coordinator 负责 `TimeStep -> Prepare -> Controller -> Apply`，但实际执行轨迹上的 vertex/edge conflict 仍由 Actor 遍历 Agent State、计算并写入 `ExecutionSession.Conflicts`。实际冲突属于标准执行结果，本阶段将判断公式和 Session 写入移入 Execution 模块，Actor 只保留 UE 日志输出。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionObservedConflictDetector.h`
- `Source/UTM/Private/Execution/ExecutionObservedConflictDetector.cpp`

### 修改文件

- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinatorTypes.h`
- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinator.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeCoordinator.cpp`
- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 新职责边界

- `FExecutionObservedConflictDetector` 根据 Agent `ActualCells` 和指定 timestep 检测实际 vertex conflict 与 edge swap conflict。
- `FExecutionRuntimeCoordinator::RecordObservedConflicts()` 统一调用 Detector，并将结果追加到 `ExecutionSession.Conflicts`。
- `Advance()` 在 Controller Result 成功 Apply 后记录当前 timestep 实际冲突，并通过 `ObservedConflicts` 返回本步新增项。
- 初始化 timestep 0 的冲突记录也通过同一个 Coordinator 入口完成。
- Actor 的 `LogObservedExecutionConflicts()` 只读取结构化冲突并保持原日志格式，不再包含冲突判断或 Session 写入。

### 保守性说明

- `GetCellAtTime()` 的空路径、timestep 0、路径范围内和超过路径长度时的取值语义保持不变。
- Mission ID 获取和两两遍历顺序未修改，不额外排序。
- vertex conflict、edge swap conflict 的判断公式以及 `FExecutionConflict` 字段赋值保持不变。
- timestep 0 仍在 Execution State 初始化后立即检测一次。
- 普通步骤仍在 Apply 后记录冲突，并在全部完成判断前输出日志。
- Controller 要求停止时仍不会 Apply，也不会记录该 timestep 实际冲突。
- Delay 随机序列、Alignment、Conflict Resolution、Final Safety Gate、Replan、Summary 和 JSON 字段未修改。
- EUW、Details 和 Blueprint 接口未修改。

### 验证计划

1. 编译 UTMEditor Win64 Development。
2. 运行原 N200 参数实验，重点核对 vertex/edge conflict count、first conflict time、actual makespan 和 Replan 指标。
3. 检查 `[ExecutionConflict][Vertex]`、`[ExecutionConflict][Edge]` 日志格式以及执行结束 Summary 输出时机。

### 验证结果

- 2026-07-31：UTMEditor Win64 Development 编译通过；UnrealHeaderTool 使用 `-WarningsAsErrors` 通过，新 `ExecutionObservedConflictDetector.cpp`、修改后的 Runtime Coordinator、Actor 和 UTM 模块均完成编译与链接，本轮无编译警告。
- 2026-07-31：运行原 N200 参数实验，执行流程和 StructuredExperimentJSON 检查正常，未发现实际冲突统计、makespan 或 Replan 指标异常。

## 2026-07-31 Execution Runtime Coordinator 第三阶段：Delay Policy

### 背景

前两阶段已经让 Runtime Coordinator 负责 Step 编排和实际冲突记录，但 `RandomGlobal`、`PerAgentProbability`、`ScriptedTimesteps` 三种 Delay 决策及 `ExecutionSession.Random` 消费仍由 Actor 回调执行。Delay 是 Benchmark 执行扰动策略，不依赖 UE World 或 Drone，本阶段将其移入独立 Execution Policy，并由 Coordinator 统一调用。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionDelayPolicyTypes.h`
- `Source/UTM/Public/Execution/ExecutionDelayPolicy.h`
- `Source/UTM/Private/Execution/ExecutionDelayPolicy.cpp`

### 修改文件

- `Source/UTM/Public/Execution/ExecutionRuntimeConfig.h`
- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinatorTypes.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeCoordinator.cpp`
- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 新职责边界

- `FExecutionDelayPolicySettings` 保存 Delay Mode、全局概率和 Per-Agent 配置快照。
- `FExecutionDelayPolicy::ShouldDelay()` 统一执行三种 Delay 策略。
- `FExecutionRuntimeConfig` 新增 `Delay` 设置，和 Alignment、Conflict Resolution、Safety Gate、Replan 设置一起构成执行配置快照。
- Runtime Coordinator 在 `PrepareStep()` 内通过同一个 `ExecutionSession.Random` 调用 Delay Policy。
- Actor 的 `BuildExecutionRuntimeConfig()` 只负责把 Details 属性复制到 Delay Settings。
- Actor 中原 `FindAgentDelayConfig()`、`IsForcedDelayStep()`、`ShouldDelayThisStep()` 以及 `ShouldDelay` Coordinator 回调已删除。

### 随机序列保守性

- Agent 仍由 Session Step Processor 按 Mission ID 排序后依次判断。
- Agent 无下一步可执行时，Step Processor 仍不会调用 Delay Policy。
- `bFinished=true` 时不消费随机数。
- `RandomGlobal` 概率小于等于 0 时不消费随机数。
- `PerAgentProbability` 找不到对应配置或配置概率小于等于 0 时不消费随机数。
- `ScriptedTimesteps` 只查询 `ForcedDelaySteps`，不消费随机数。
- 概率仍使用 `[0, 1]` Clamp，并通过同一次 `FRandomStream::FRand() < Probability` 判断。
- Session Random Seed 初始化位置和时机未修改。

### 保守性说明

- `DelayMode`、`StepDelayProbability` 和 `AgentDelayConfigs` 的 Details、Blueprint 和 Reporter 字段未修改。
- Delay 日志仍由 Actor 根据最终 Step Proposal 输出，格式和时机未修改。
- Alignment、Conflict Resolution、Final Safety Gate、Replan、实际冲突记录、Summary 和 StructuredExperimentJSON 字段未修改。
- Controller 仍按原逻辑每个 timestep 创建一次；本阶段不改变 Controller 生命周期。
- EUW 接口未修改。

### 验证计划

1. 编译 UTMEditor Win64 Development。
2. 使用原 N200 RandomGlobal 参数实验，重点核对 `total_delay_steps`、`actual_makespan`、Alignment 和 Replan 指标。
3. 后续可分别使用 Per-Agent Probability 与 Scripted Timesteps 做小规模定向检查，确认配置查找和强制 Delay timestep 行为不变。

### 验证结果

- 首次编译在 Unity Build 中发现 `ExecutionObservedConflictDetector.cpp` 与 `ExecutionReplanCandidateSelector.cpp` 的匿名命名空间辅助函数同名；将 Detector 内部函数重命名为唯一的 `GetObservedConflictCellAtTime()`，函数逻辑未修改。
- 2026-07-31：修正内部辅助函数名称后，UTMEditor Win64 Development 编译通过；Execution Delay Policy、Observed Conflict Detector、Runtime Coordinator、Actor、Unity translation unit 和 UTM 模块均完成编译与链接，本轮最终编译无警告。
- 2026-07-31：运行原 N200 RandomGlobal 参数实验，执行流程和 StructuredExperimentJSON 检查正常，未发现 Delay 随机序列、total delay steps、makespan、Alignment、Conflict 或 Replan 指标异常。

## 2026-07-31 Execution Runtime Coordinator 第四阶段 A：Controller 生命周期持久化

### 背景

此前 `FExecutionRuntimeCoordinator` 已经成为标准执行时间步入口，但 `FExecutionControllerRegistry::RunStep()` 仍在每个 timestep 创建并销毁一次 Controller。默认 Controller 当前没有内部状态，因此旧流程能够正常运行；但这种生命周期无法支持需要保存冲突历史、动态优先级、在线统计或学习参数的研究型 Controller。

### 修改文件

- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinator.h`
- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinatorTypes.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeCoordinator.cpp`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 新生命周期

- `FExecutionRuntimeCoordinator` 新增持久成员 `ActiveController`。
- 新增 `InitializeController()`：一次集中式执行会话开始时，根据 Details 中选择的 `ExecutionControllerType` 创建 Controller。
- `Advance()` 不再通过 Registry 每步重新创建 Controller，而是复用当前会话的 `ActiveController`。
- 新增 `ResetController()`：重新运行规划、重新初始化执行、执行异常、Controller 主动停止或所有 Agent 完成时释放 Controller。
- `FExecutionRuntimeCoordinatorRequest` 不再携带每步重复的 `ControllerType`；Controller 类型在执行初始化时确定，并在该次执行会话中保持不变。
- `FExecutionControllerRegistry::RunStep()` 暂时保留为兼容入口，其他潜在调用者的行为不变。

### 保守性说明

- `IExecutionController::RunStep()` 本阶段仍保持 `const`，没有引入可变状态接口；生命周期接口扩展留到后续阶段。
- `FDefaultExecutionController` 和 `FExecutionStepPipeline` 的算法、输入、回调顺序和输出均未修改。
- TimeStep、Delay 随机流、Alignment、Conflict Resolution、Replan、Final Safety Gate、状态写回和冲突记录顺序未修改。
- Controller 初始化发生在 Runtime State 构建成功且存在可执行 Agent 之后；无可执行 Agent 时不会创建无用实例。
- Controller 创建失败时，Actor 沿用执行失败路径生成 Summary，并输出明确错误日志。
- Details 中运行期间对 `ExecutionControllerType` 的修改从下一次执行初始化开始生效，不在当前执行会话中途替换实例。
- EUW、Details、Blueprint、Reporter 和 StructuredExperimentJSON 接口未修改。

### 验证计划

1. 编译 UTMEditor Win64 Development，检查持有不完整接口类型的析构边界和 Unreal Header Tool。
2. 运行原 N200 参数实验，核对 `total_delay_steps`、`actual_makespan`、Alignment、Conflict 和 local/global Replan 指标。
3. 确认一次执行结束后再次点击运行规划，Controller 能够释放并重新创建，第二次实验不会继承第一次会话状态。

### 验证结果

- 2026-07-31：UTMEditor Win64 Development 编译通过；持有 `TUniquePtr<IExecutionController>` 的 Coordinator 构造、析构边界，修改后的 Actor、Execution Runtime Coordinator 和 UTM 模块均完成编译与链接。
- 编译仅出现既有 `PBSPlanner.cpp` 的 UE 5.5 `TArray::RemoveAt(bool)` 弃用警告，与本阶段修改无关。
- 2026-07-31：运行原 N200 参数回归实验，执行流程和 StructuredExperimentJSON 检查正常，未发现 Controller 持久化对 makespan、Delay、Alignment、Conflict 或 Replan 指标造成异常影响。
- 后续仍可在同一编辑器会话内连续运行两次实验，定向确认 Controller 在运行间能够释放并重新初始化。

## 2026-07-31 Execution Runtime Coordinator 第四阶段 B：标准 Controller 生命周期接口

### 背景

第四阶段 A 已经让一次执行会话复用同一个 Controller 实例，但 `IExecutionController` 仍只有只读的 `RunStep() const`，没有标准初始化和清理钩子。Controller 虽然存活时间变长，研究者仍不能通过普通成员自然维护冲突历史、动态优先级、在线统计或学习参数。本阶段补齐 Controller 的标准生命周期契约。

### 修改文件

- `Source/UTM/Public/Execution/ExecutionControllerTypes.h`
- `Source/UTM/Public/Execution/ExecutionControllerRegistry.h`
- `Source/UTM/Private/Execution/ExecutionControllerRegistry.cpp`
- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinatorTypes.h`
- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinator.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeCoordinator.cpp`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 标准生命周期接口

- 新增 `FExecutionControllerInitializeRequest`，向 Controller 提供排序后的 Mission ID、Grid Map 和执行开始时的 Runtime Config 快照。
- `IExecutionController` 新增可选 `Initialize()`：默认实现清空失败原因并返回成功，已有无状态 Controller 不需要编写额外逻辑。
- `IExecutionController` 新增可选 `Reset()`：默认实现为空，供有状态 Controller 清理会话数据。
- `IExecutionController::RunStep()` 删除末尾 `const`，允许实现类在 timestep 之间更新普通成员状态。
- 新增 `FExecutionRuntimeCoordinatorInitializeRequest`，由 Benchmark Host 向 Coordinator 提供 Controller 类型、Runtime Session、Grid Map 和 Runtime Config。
- Coordinator 负责从 Session 提取并排序 Mission ID，Actor 不需要理解 Controller 内部初始化数据结构。
- Controller 初始化失败时不会进入 Active 状态；若实现未提供失败原因，Coordinator 会补充包含 Controller 名称的错误信息。
- `ResetController()` 会先调用 Controller 的 `Reset()`，再销毁实例；Coordinator 析构时也执行同一清理路径。

### Registry 兼容入口

- `FExecutionControllerRegistry::RunStep()` 继续保留。
- 该兼容入口现在执行一次完整的 `Create -> Initialize -> RunStep -> Reset -> Destroy` 生命周期。
- Runtime Coordinator 主流程仍不使用该临时入口，而是复用持久的 `ActiveController`。

### 保守性与兼容性说明

- 默认 Controller 继续直接调用原 `FExecutionStepPipeline::Run()`，没有新增内部状态。
- 初始化时的 Runtime Config 是会话开始快照；每个 `RunStep()` 仍接收当前 timestep 的 Runtime Config，因此除 Controller 类型外，原有运行期配置读取行为不变。
- Mission ID 使用和 Session Step Processor 相同的升序语义，不改变 Agent 处理顺序。
- TimeStep、Delay 随机流、Alignment、Conflict Resolution、Replan、Final Safety Gate、状态写回、Summary 和 JSON 均未修改。
- EUW、Details 和 Blueprint 反射接口未修改。
- C++ 接口存在一次有意的签名升级：外部自定义 Controller 若覆盖旧版 `RunStep(...) const`，需要删除实现末尾的 `const`，并可按需覆盖 `Initialize()` 和 `Reset()`。

### 验证计划

1. 编译 UTMEditor Win64 Development，检查 Unreal Header Tool、接口覆盖签名和 Coordinator 生命周期接线。
2. 运行原 N200 参数实验，重点核对 makespan、Delay、Alignment、Conflict 和 local/global Replan 指标。
3. 在同一编辑器会话中连续运行两次实验，确认 `Reset()` 后的新 Controller 不继承上一轮内部状态。

### 验证结果

- 2026-07-31：UTMEditor Win64 Development 编译通过；Unreal Header Tool 使用 `-WarningsAsErrors` 完成检查并报告 0 个反射文件改动，Controller Registry、Runtime Coordinator、Actor 和 UTM 模块均完成编译与链接，本轮无编译警告。
- 2026-07-31：运行原 N200 参数回归实验，执行流程和 StructuredExperimentJSON 检查正常，未发现标准 Controller 生命周期接口对 makespan、Delay、Alignment、Conflict 或 Replan 指标造成异常影响。
- 后续仍可在同一编辑器会话内连续运行两次实验，定向检查自定义有状态 Controller 的会话间 Reset；默认无状态 Controller 的本次 N200 回归已通过。

## 2026-08-01 Execution Replan Service 第一阶段：完整重规划请求编排

### 背景

执行期重规划的 Candidate 扩张、targeted retry、单次 Attempt 和路径提交此前已经分别模块化，但 `APathPlanningDemoActor::TryExecutionReplan()` 仍负责连接全部组件，并额外承担快照修正、重规划次数限制、计时统计提交和路径缓存同步。研究者若要从其他 Host 调用同一套重规划流程，仍需要复制 Actor 中的编排代码。本阶段新增应用层 Replan Service，为一次完整的执行期重规划请求提供统一入口。

### 新增文件

- `Source/UTM/Public/Execution/ExecutionReplanServiceTypes.h`
- `Source/UTM/Public/Execution/ExecutionReplanService.h`
- `Source/UTM/Private/Execution/ExecutionReplanService.cpp`

### 修改文件

- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### Service 职责

- 校验 Grid、Runtime Session 和 Cell Path Cache 等完整请求上下文。
- 统一处理空请求、Replan Disabled 和最大重规划次数限制。
- 从 Runtime Session 构造使用 `LastObservedCell` 的算法快照，不读取 `UWorld`、Drone Actor 或 World Transform。
- 构造并调用 `FExecutionReplanCoordinator`，继续复用已有 Candidate 扩张、local/global 多轮尝试和 targeted retry。
- 在 Coordinator 的计时范围内调用 `FExecutionReplanAttemptRunner`、失败事件回调、Session Committer 和 World Path Cache 同步。
- 将 Coordinator 的 local/global Attempt 耗时和成功应用次数写回 Runtime Session。
- 返回结构化状态、成功标记和实际重规划 Mission ID 集合。

### Actor 新边界

- `TryExecutionReplan()` 保留为 Benchmark Host 的薄适配层，只把 Grid、Session、Planner/Execution 配置和两类路径缓存放入标准 Request。
- Attempt 失败日志和 Coordinator 过程日志继续由 Actor 的 UE 日志函数输出，日志文本和触发位置不变。
- 达到最大重规划次数时仍由 Actor 输出原 `[AlignmentReplan] skipped...` 日志。
- 删除 Actor 中仅供重规划使用的 `CaptureExecutionSnapshot()`、`RunExecutionReplanAttempt()` 和 `ApplyExecutionReplanAttemptResult()`。
- `AdvanceExecutionOneStep()` 到 `TryExecutionReplan()` 的同步回调接口暂时不变，便于继续使用原 N200 实验验证。

### 保守性说明

- Planner Registry、Replan Coordinator、Attempt Runner、Candidate Selector、PostCheck 和 Path Integrator 算法均未修改。
- 快照继续使用 `LastObservedCell`；原流程先读取 observed cell、随后在 `TryExecutionReplan()` 中逐 Agent 覆盖为 `LastObservedCell`，本阶段直接构造最终等价快照。
- local/global Attempt 次数、总耗时、最大耗时和 `TotalReplanCount` 的提交位置不变。
- 单次 Attempt 的 Planner Config 复制、失败日志、结果提交和 World Path Cache 转换仍位于 Coordinator 的 Attempt 计时范围内。
- Mission 遍历顺序、Cell Path 写回顺序、成功日志时机和同步执行方式未修改。
- EUW、Details、Blueprint、Summary 和 StructuredExperimentJSON 字段未修改。
- 本阶段没有同时修正 Committer 在中途失败时可能产生部分写回的既有事务性问题，避免把行为修复与架构迁移混在同一次回归中。

### 验证计划

1. 编译 UTMEditor Win64 Development，检查新增 Service 类型、Unity Build 和模块链接。
2. 运行原 N200 参数实验，重点核对 `execution_replan_attempt_count`、local/global Attempt 数量与耗时、applied replans、makespan、Conflict 和 Alignment 指标。
3. 检查 `[AlignmentReplan]` 的 Attempt 失败、扩张、targeted retry、成功和次数上限日志格式。

### 验证结果

- 2026-08-01：UTMEditor Win64 Development 编译通过；UnrealHeaderTool 使用 `-WarningsAsErrors` 完成检查，新增 `ExecutionReplanService.cpp` 与修改后的 `PathPlanningDemoActor.cpp` 分别完成非 Unity 编译，UTM 模块成功链接，本轮无编译警告。
- 2026-08-01：运行原 N200 参数回归实验，执行流程和 StructuredExperimentJSON 检查正常，未发现 Execution Replan Service 抽取对 makespan、Alignment、Conflict、applied replans 或 local/global Replan 指标造成异常影响。

## 2026-08-01 Execution Replan Service 第二阶段：移除 Actor 中转

### 背景

第一阶段已经把一次完整重规划请求的编排移入 `FExecutionReplanService`，但 Runtime Coordinator 仍通过 `RunReplan` 回调进入 `APathPlanningDemoActor::TryExecutionReplan()`，再由 Actor 调用 Service。Actor 因而仍是 Execution Controller 与 Replan Service 之间的控制流中转站。本阶段移除该中转，使执行模块内部直接完成重规划请求，同时保留 Actor 的 UE 配置和日志边界。

### 修改文件

- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinatorTypes.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeCoordinator.cpp`
- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 新调用链

```text
APathPlanningDemoActor
    -> FExecutionRuntimeCoordinator::Advance()
        -> IExecutionController::RunStep()
            -> Step Replan / Final Safety Gate
                -> Coordinator 内部 RunReplan 适配
                    -> FExecutionReplanService::Run()
```

### Coordinator 新职责

- `FExecutionRuntimeCoordinatorRequest` 新增 `FExecutionRuntimeReplanContext`，统一携带 Planner Type、Cell Path Cache 和 World Path Cache。
- Coordinator 根据当前 Runtime Session、Grid Map、Runtime Config 和 Replan Context 构造标准 `FExecutionReplanServiceRequest`。
- Controller、Step Replan Coordinator 和 Final Safety Gate 继续使用通用 `RunReplan` 函数契约，不直接依赖具体 Service。
- Coordinator 将 Attempt 失败和 Replan Coordinator 事件转发给 Host 回调，并在 Service 返回后转发结构化 Result。
- Service 的成功结果和 Mission ID 集合由 Coordinator 原样返回给 Controller Pipeline。

### Actor 新边界

- 删除 `APathPlanningDemoActor::TryExecutionReplan()` 的声明与实现。
- `AdvanceExecutionOneStep()` 只提供 Planner Type、两类路径缓存和按需构造 Planner Runtime Config 的回调。
- `[AlignmentReplan]` Attempt、扩张、targeted retry、成功和次数上限日志继续由 Actor 输出。
- Actor 不再构造 Service Request、调用 Replan Service、接收成功结果或转发 Replanned Mission ID。

### 保守性说明

- Controller、Execution Step Pipeline、Step Replan Coordinator、Final Safety Gate、Replan Coordinator 和 Attempt Runner 的算法未修改。
- Planner Runtime Config 仍只在实际请求重规划时构造，不会在每个普通 timestep 重复复制 No-Fly Zone 配置。
- Replan Service 使用 Coordinator 当前 timestep 已构造的 Runtime Config 快照；同步调用期间配置值与旧 `TryExecutionReplan()` 再次读取的值一致。
- Attempt 失败事件和 Coordinator 事件仍在原 Replan Attempt 计时范围内触发。
- 达到次数上限后的日志文本和 local/global fallback 返回语义不变。
- Observed Cell、Delay 随机流、路径写回、Summary、StructuredExperimentJSON、EUW、Details 和 Blueprint 接口未修改。
- 本阶段仅移除 Actor 中转，尚未引入 `IExecutionReplanService` 或动态 Service Registry。

### 验证计划

1. 编译 UTMEditor Win64 Development，检查 Coordinator 公共类型、Lambda 捕获生命周期和模块链接。
2. 运行原 N200 参数实验，重点核对 makespan、Alignment、Conflict、applied replans 以及 local/global Replan Attempt 数量和耗时。
3. 检查 `[AlignmentReplan]` 日志格式，并确认 local replan 失败后的 global fallback 与 Final Safety Gate 重规划行为正常。

### 验证结果

- 2026-08-01：UTMEditor Win64 Development 编译通过；UnrealHeaderTool 使用 `-WarningsAsErrors` 完成检查并报告 0 个反射文件改动，修改后的 Runtime Coordinator、Actor 和 UTM 模块均完成编译与链接，本轮无编译警告。
- 2026-08-01：运行原 N200 参数回归实验，执行流程、StructuredExperimentJSON Replan 指标和 `[AlignmentReplan]` 日志检查正常，未发现 Runtime Coordinator 直接调用 Replan Service 对 makespan、Alignment、Conflict 或 local/global Replan 行为造成异常影响。

## 2026-08-01 Execution Replan Service 第三阶段：接口化与依赖注入

### 背景

第二阶段已经让 Runtime Coordinator 直接调用 Replan Service，但依赖目标仍是具体静态类，研究者若要替换完整的执行期重规划流程，仍需修改 Coordinator 源码。本阶段建立最小可替换接口，并采用 C++ 所有权注入；默认实验继续自动使用当前实现，不新增 Details、Blueprint 或固定枚举分支。

### 修改文件

- `Source/UTM/Public/Execution/ExecutionReplanService.h`
- `Source/UTM/Private/Execution/ExecutionReplanService.cpp`
- `Source/UTM/Public/Execution/ExecutionRuntimeCoordinator.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeCoordinator.cpp`
- `Source/UTM/Public/Actors/PathPlanningDemoActor.h`
- `Source/UTM/Private/Actors/PathPlanningDemoActor.cpp`

### 新接口

- 新增 `IExecutionReplanService`，公开 `Run(Request, Callbacks)` 作为完整执行期重规划入口。
- 接口提供可选 `Reset()` 生命周期钩子；无状态实现不需要覆盖，有状态研究实现可在新执行会话开始前清理历史数据。
- 原有实现改为 `FDefaultExecutionReplanService`，算法代码、Request/Result 类型和事件回调契约不变。
- `FExecutionRuntimeCoordinator` 默认创建并持有 `FDefaultExecutionReplanService`，现有 Actor 和实验配置无需额外设置。

### 注入与所有权

- `FExecutionRuntimeCoordinator::SetReplanService()` 接收 `TUniquePtr<IExecutionReplanService>&&`，Coordinator 获得自定义 Service 的唯一所有权。
- `APathPlanningDemoActor::SetExecutionReplanService()` 提供同名语义的公开 C++ 转发入口，但不是 `UFUNCTION`，因此不扩展 EUW 或 Blueprint 接口。
- 仅允许在没有 Active Controller 时替换 Service，避免一次执行会话中途切换实现或丢失有状态 Service 的内部历史。
- 传入空指针或执行期间尝试替换时返回 `false`，且 rvalue reference 在失败分支不会被移动，调用者仍保留原 Service。
- 成功替换前先调用旧 Service 的 `Reset()`，随后转移所有权。
- 每次 `InitializeController()` 开始新执行会话时调用当前 Service 的 `Reset()`。

### 新依赖方向

```text
APathPlanningDemoActor
    -> FExecutionRuntimeCoordinator
        -> IExecutionReplanService
            -> FDefaultExecutionReplanService
```

自定义研究实现只需实现 `IExecutionReplanService` 并在执行开始前注入，不需要修改 Actor、Runtime Coordinator、Controller、Step Pipeline 或 Final Safety Gate。

### 保守性说明

- 默认 Service 的函数主体仅从静态类成员迁移为实例成员，内部控制流未修改。
- Replan Request、Result、状态枚举、Attempt/Coordinator 回调和计时统计未修改。
- 默认 Service 仍由 Coordinator 自动创建，因此未注入自定义实现时调用链和实验行为不变。
- Planner、Candidate 扩张、targeted retry、PostCheck、路径整合、Session 写回和路径缓存同步未修改。
- `[AlignmentReplan]` 日志、Summary、StructuredExperimentJSON、EUW、Details 和 Blueprint 接口未修改。
- 本阶段有一处有意的 C++ 类型升级：原具体类名 `FExecutionReplanService` 改为 `FDefaultExecutionReplanService`；新的外部扩展应依赖 `IExecutionReplanService`。
- 暂不建立 Replan Service Registry；只有一个默认实现时，固定枚举和 `switch` 会增加扩展成本而没有用户选择价值。

### 验证计划

1. 编译 UTMEditor Win64 Development，检查接口虚函数、`TUniquePtr` 不完整类型析构边界和 UHT 对 Actor 非反射 C++ 方法的处理。
2. 运行原 N200 参数实验，核对 makespan、Alignment、Conflict、applied replans 以及 local/global Replan 数量和耗时。
3. 后续增加最小自定义 Service 编译测试，确认注入入口、所有权转移和执行期间拒绝替换的契约。

### 验证结果

- 2026-08-01：UTMEditor Win64 Development 编译通过；UnrealHeaderTool 使用 `-WarningsAsErrors` 成功处理 Actor 的非反射 C++ 注入方法并写入 1 个生成文件，Replan Service 接口与默认实现、Runtime Coordinator、Actor 和 UTM 模块均完成编译与链接，本轮无编译警告。
- 2026-08-01：运行原 N200 参数回归实验，默认 `FDefaultExecutionReplanService` 的执行流程和 StructuredExperimentJSON 检查正常，未发现接口化与依赖注入对 makespan、Alignment、Conflict、applied replans 或 local/global Replan 指标造成异常影响。

## 2026-08-01 Execution Replan Committer：原子提交

### 背景

`FExecutionRuntimeSessionReplanCommitter::CommitAttemptResult()` 原先在同一个 Candidate 循环中交错执行校验、路径整合和 Session 写回。如果前面的 Mission 已成功写回，而后面的 Mission 缺少 State、Mission Config、Replanned Path 或 Integration 失败，函数会返回失败，但前面 Mission 的 Runtime State 与 Cell Path Cache 已经改变。后续扩张或重试因而可能从部分新状态和部分旧状态组成的混合快照继续执行。本阶段将该过程改为两阶段事务。

### 修改文件

- `Source/UTM/Public/Execution/ExecutionRuntimeSessionReplanCommitter.h`
- `Source/UTM/Private/Execution/ExecutionRuntimeSessionReplanCommitter.cpp`
- `Source/UTM/Private/Execution/ExecutionReplanService.cpp`

### 两阶段提交

1. Prepare 阶段按原 Candidate 顺序完成全部 State、Mission Config 和 Replanned Path 查询。
2. Prepare 阶段调用 `FExecutionReplanPathIntegrator`，将每个 Mission 的 Timeline、索引、Goal 和目标状态指针保存在临时记录中。
3. Prepare 阶段不修改 Agent State、Cell Path Cache、World Path Cache 或 Commit Result Mission ID 集合。
4. 只有全部 Candidate Prepare 成功后才进入 Commit 阶段。
5. Commit 阶段按原 Candidate 顺序统一更新 Agent State 和 Cell Path Cache，并产生完整的 Replanned Mission ID 集合。
6. Service 仅在 Commit Result 整体成功时同步 World Path Cache。

### 结构化失败结果

新增 `EExecutionRuntimeReplanAttemptCommitStatus`：

- `Success`
- `InvalidRequest`
- `EmptyCandidateSet`
- `MissingAgentState`
- `MissingMissionConfig`
- `MissingReplannedPath`
- `PathIntegrationFailed`

Commit Result 同时新增 `FailedMissionId` 和 `FailureReason`，便于后续测试、诊断或自定义 Service 处理；本阶段不新增 UE 日志或 StructuredExperimentJSON 字段。

### 原子性边界

- 任意 Prepare 失败时，`ReplannedMissionIds` 保持为空。
- 任意 Prepare 失败时，不修改 `PlannedCells`、`ExecutedPlanIndex`、`GoalCell`、`GoalWorld`、Conflict Hold、Alignment Lost 或 Cell Path Cache。
- Service 使用 `CommitResult.bSuccess` 作为 World Path Cache 同步门槛，因此失败时 World Cache 同样不变。
- Commit 阶段只执行已经完整准备好的内存写回，不包含新的业务校验失败分支。

### 保守性说明

- 成功路径写回的字段、字段值、Candidate 顺序和 Replanned Mission ID 语义不变。
- Timeline 仍由原 `FExecutionReplanPathIntegrator` 生成，重规划 Hold timestep 和路径拼接规则未修改。
- Committer 仍由 Replan Coordinator 的 Apply 回调同步调用，因此 Prepare 和 Commit 都处于原 Replan Attempt 计时范围内。
- Planner、Candidate 扩张、targeted retry、PostCheck、local/global fallback 和总重规划次数统计未修改。
- Actor、EUW、Details、Blueprint、Summary 和 StructuredExperimentJSON 接口未修改。
- 正常 N200 路径通常已经由 Attempt Runner 完成前置校验；本阶段主要强化异常数据和自定义 Replan 实现下的失败契约。

### 验证计划

1. 编译 UTMEditor Win64 Development，检查新增状态类型、临时 Prepare 记录和 Service 成功门控。
2. 运行原 N200 参数实验，核对 makespan、Alignment、Conflict、applied replans 以及 local/global Replan 数量和耗时。
3. 后续通过自动化测试构造“前一个 Mission 有效、后一个 Mission 缺少路径”的输入，确认失败后所有 Agent State 和两类路径缓存保持不变。

### 验证结果

- 2026-08-01：UTMEditor Win64 Development 编译通过；修改后的 `ExecutionRuntimeSessionReplanCommitter.cpp` 与 `ExecutionReplanService.cpp` 分别完成非 Unity 编译，UTM 模块成功链接，本轮无编译警告。
- 2026-08-01：运行原 N200 参数回归实验，执行流程和 StructuredExperimentJSON 检查正常，未发现两阶段原子提交对 makespan、Alignment、Conflict、applied replans 或 local/global Replan 指标造成异常影响。
