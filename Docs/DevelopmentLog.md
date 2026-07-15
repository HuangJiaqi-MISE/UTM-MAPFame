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
