# Rule-Based Local Avoidance 与 MAPPO 安全降级策略说明

本文档用于说明 UE-MAPPO 应急恢复实验中的四类部署策略，重点解释 `Rule-Based Local Avoidance` 的作用，以及当前 `MAPPO + Safety Filter + Degradation` 中局部修补模块的工程含义。这里讨论的不是四种训练算法，而是在同一故障注入条件下，系统如何接管失联无人机并产生离散动作的四种部署模式。

## 四类应急恢复策略

| 模式 | 是否使用 MAPPO | 是否做安全过滤/降级 | 典型行为 |
|---|---:|---:|---|
| `No Emergency Recovery` | 否 | 否 | 故障后不接管、不发恢复动作，只作为“没有应急模块”的负对照基线 |
| `Rule-Based Local Avoidance` | 否 | 是，规则本身就是避障逻辑 | 用启发式局部规则选择安全动作，安全但容易卡死或等待过多 |
| `MAPPO Only` | 是 | 否 | 直接执行模型动作，能体现纯策略能力，但可能进入 blocked/no-fly 或其他非法状态 |
| `MAPPO + Safety Filter + Degradation` | 是 | 是 | 先用 MAPPO 给出动作偏好，再由 safety filter 和 UE fallback 修正不安全动作，是当前最实用的部署模式 |

### No Emergency Recovery

`No Emergency Recovery` 是最基础的负对照组。集中式 UTM 或 LaCAM-UTM 路径下发中断后，故障无人机不再获得新的恢复动作，也不调用 MAPPO 服务。它代表系统没有任何应急自主恢复模块时的表现。

这个模式的实际含义是：无人机仍处于通信故障状态，无法继续接收 UTM 中心的新路径；系统只记录故障触发后的状态，不主动规划局部动作。该模式用于评估“不恢复”的风险基线，例如无人机是否停留在危险区域、是否长期阻塞航路、是否无法脱离潜在冲突区域。

### Rule-Based Local Avoidance

`Rule-Based Local Avoidance` 是不使用神经网络的手写规则基线。它也通过 Python emergency service 返回动作，但不是 MAPPO 推理，而是按局部规则选择动作：优先找一个安全、能靠近目标的离散动作，避开 blocked cell、no-fly、RID 邻机占用、Vertex/Edge/Downwash 冲突等；如果找不到合适移动，就退化成 `WAIT`。

它的优点是可解释、部署简单、不需要神经网络推理。缺点是只看局部和短期收益，容易在建筑密集区、走廊狭窄区或多机相互阻塞时陷入等待或局部死锁。在实验中，如果 Rule-Based 也能恢复，说明简单局部规则已经足够；如果它卡死而 `MAPPO + Safety` 成功，则说明 MAPPO 提供了比局部贪心更好的方向选择。

### MAPPO Only

`MAPPO Only` 是学习策略的裸部署版本。故障无人机把本地观测输入 MAPPO actor，由模型直接输出 `{+X, -X, +Y, -Y, +Z, -Z, WAIT}` 中的动作，UE 侧不再额外修正或替代该动作。

这个模式用于检验模型自身是否已经学会了路径恢复和避障。它能反映神经策略的原始能力，但部署风险最大：如果模型输出动作进入建筑、禁飞区或冲突格，系统不会自动修正。因此 raw MAPPO 不能直接作为安全关键系统的唯一控制源，它更适合作为带安全过滤的候选动作生成器。

### MAPPO + Safety Filter + Degradation

`MAPPO + Safety Filter + Degradation` 是当前推荐的实际部署模式。MAPPO 先给出动作偏好或动作概率排序，系统再通过安全过滤器检查动作是否违反约束，例如 blocked cell、out of grid、no-fly、RID 邻机占用、Vertex/Edge/Downwash 冲突。如果首选动作不安全，系统按 MAPPO 的概率排序尝试下一个安全动作；如果策略候选仍不能给出可接受移动，则进入 UE 侧 fallback。

这个模式的实际意义是：MAPPO 负责“智能恢复方向”，安全过滤器负责“硬约束兜底”。这种结构更适合应急恢复，因为通信故障时无人机只能依赖本地 GPS、RID 邻机位置、本地障碍/禁飞信息和预置目标，不能假设中心还能实时纠错。

## 当前局部修补模块

在 `MAPPO + Safety Filter + Degradation` 模式中，我们加入了一个非常有限的 UE 侧局部修补模块。它不是完整路径规划器，也不是 LaCAM-UTM 的替代品，而是当 MAPPO 首选动作明显不安全时，尝试在本地范围内找一个更合理的下一步动作。

当前触发逻辑是保守的：如果 MAPPO 的首选动作安全且不是 `WAIT`，系统直接执行 MAPPO 的动作；如果首选动作是 `WAIT`，系统会把 `WAIT` 放到候选序列最后，优先尝试 MAPPO 概率排序中的安全移动动作；如果首选动作因为 `blocked_cell`、`out_of_grid` 或 `no_fly` 被拒绝，UE 侧才调用局部修补模块，尝试寻找一个安全的替代动作。无论替代动作来自 MAPPO 概率排序还是局部修补模块，最终都必须再次通过同一套 safety filter，因此不会绕过 Vertex、Edge、Downwash、blocked、no-fly 和 out-of-grid 约束。

局部修补模块当前只返回“下一步动作”，不生成完整路径片段。它会在当前 ghost cell 附近做短视距搜索，找一个能改善目标距离、且第一步安全的动作，然后把这个动作交给 ghost execution 消费。日志中如果出现 `repair_type=local_search`，说明这一步是由 UE 侧局部修补模块产生的；如果出现 `repair_type=policy_order`，说明系统只是按 MAPPO 动作概率顺序选择了后续安全动作。

## MaxLocalRepairRadius = 18 的含义

当前局部修补模块的本地范围参数写在 UE C++ 代码中：

```cpp
constexpr int32 MaxLocalRepairDepth = 18;
constexpr int32 MaxLocalRepairNodes = 1024;
constexpr int32 MaxLocalRepairRadius = 18;
```

其中 `MaxLocalRepairRadius = 18` 表示局部修补搜索不会访问距离当前 ghost cell 太远的格子。具体来说，它限制在当前 cell 周围的局部盒子中：

```text
|x - start_x| <= 18
|y - start_y| <= 18
|z - start_z| <= 18
```

这可以解释为应急恢复模块的“本地感知范围”或“本地修补范围”。在实际部署设定中，故障无人机并不能重新获得全局中心规划信息，也不能假设自己知道全图动态态势。它只能依靠自身定位、目标方向、有限局部栅格、局部 blocked/no-fly 信息，以及 RID 广播获得的邻机位置。因此 `MaxLocalRepairRadius = 18` 不应被理解为任意调大的性能参数，而应被理解为与本地 RID/感知范围相匹配的部署约束。

这个设定也帮助区分 MAPPO 应急恢复模块和集中式规划器。LaCAM-UTM 负责通信正常时的全局多机路径规划；MAPPO 和 UE safety fallback 只负责通信故障后的短时、局部、分布式恢复。换句话说，当前方法的目标不是在失联状态下重新解决完整 MAPF 问题，而是在局部失联窗口内让无人机尽快脱离危险状态、减少冲突、避免进入障碍或禁飞区，并尽量朝目标恢复。

## 能力边界

当前局部修补模块的能力边界也很明确。它最多搜索 `18` 个离散动作步，最多展开 `1024` 个本地节点，并且只执行搜索得到的第一步动作。因此它适合处理短距离局部阻挡，例如 MAPPO 首选动作撞到建筑边缘、越界或短暂进入禁飞区时，找一个安全的旁路动作。

如果场景需要长距离绕过大型建筑、需要先远离目标再绕回来，或者需要几十步以上的连续局部路径规划，当前模块可能会失败。例如某些 Manhattan 建筑场景中，ghost 已经到达高度上边界附近，MAPPO 仍倾向于继续 `+Z`，安全过滤会拒绝该动作，local search 则只能在本地范围内上下或左右修补，无法形成真正绕过建筑的长路径。这类失败不代表 safety filter 错误，而是说明该场景超出了当前局部应急恢复模块的设计能力。

因此，在报告中可以把当前方法表述为：MAPPO 提供分布式应急恢复策略，UE safety filter 提供硬安全约束，局部修补模块提供有限半径内的安全降级。它适用于短时、局部失联恢复，不保证解决需要全局重规划的复杂建筑绕行问题。这个限制与我们的部署假设是一致的，因为一旦需要全局绕行，合理的工程策略应当是等待通信恢复后交还给中心化 UTM/LaCAM-UTM，而不是让失联无人机在局部信息条件下尝试全图规划。
