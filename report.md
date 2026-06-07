# 当前进展汇报

## 项目目标

本项目的目标是复现 Stop-N-Go 论文中的核心思想，用于多机械臂共享工作空间下的时间协调规划。整体流程是：

- 先为每个机械臂单独生成初始轨迹
- 将所有轨迹同步到统一时间网格
- 检测机械臂之间的时间冲突
- 通过插入 pause 的方式解决冲突，而不改变原有空间路径

当前代码已经具备主要的数据抽象、MoveIt 桥接、冲突检测主链路，以及搜索框架的基础结构。

## 当前整体结构

当前实现大致分为三层：

- `stop_n_go::base`
- `stop_n_go::core`
- `stop_n_go::app`

其中：

- `base` 负责轻量数据结构和 MoveIt 桥接
- `core` 负责 Stop-N-Go 搜索相关核心逻辑
- `app` 负责面向 demo / 实验的应用层流程组织

## 已实现的核心模块

### 一、`stop_n_go::base`

这一层主要负责轻量抽象与 MoveIt 的桥接。

#### 1. `RobotState`

作用：

- 表示单个 planning group 在某一时刻的轻量状态
- 内部只存一组有序的 active variables 数值

特点：

- 不存时间
- 不存几何信息
- 不存 MoveIt 重对象
- 是纯值对象，便于拷贝和放入 trajectory

#### 2. `StateLayout`

作用：

- 说明 `RobotState` 中这组数值应该如何解释
- 将轻量状态与 MoveIt planning group 绑定起来

当前保存的信息包括：

- planning group 名称
- active variable 的顺序
- continuous joint 标记
- 对应的 `RobotModel` / `JointModelGroup`

#### 3. `Trajectory`

作用：

- 表示单个 planning group 的离散时间轨迹

当前包含：

- `StateLayout`
- 一串 `RobotState` waypoint
- 每个 waypoint 相对前一个 waypoint 的 duration

已实现能力：

- append
- appendHold
- resample
- 轨迹时长查询

#### 4. `ConversionContext`

作用：

- 负责自定义轻量抽象与 MoveIt / ROS 消息之间的转换

当前支持：

- MoveIt `RobotState` -> 轻量 `RobotState`
- 轻量 `RobotState` -> MoveIt full `RobotState`
- MoveIt `RobotTrajectory` -> 自定义 `Trajectory`
- 自定义 `Trajectory` -> `trajectory_msgs::msg::JointTrajectory`

#### 5. `RobotRegistry`

作用：

- 用系统内部的 `RobotId` 映射 MoveIt 侧语义

当前保存：

- `robot_id`
- `group_name`
- `end_effector_link`

#### 6. `InitialPlanner`

作用：

- 对外提供“以 end effector pose 为目标”的单机械臂初始规划接口

当前输入：

- `RobotId`
- `Pose` / `PoseStamped`

当前输出：

- 自定义 `Trajectory`

内部做法：

- 使用每个机器人对应的 `MoveGroupInterface`
- 调用 MoveIt 规划
- 再把结果转回 `stop_n_go::base::Trajectory`

---

### 二、`stop_n_go::core`

这一层主要负责 Stop-N-Go 搜索核心逻辑。

#### 1. `SearchNode`

作用：

- 表示 A* 搜索中的一个节点

当前包含：

- 所有机器人的轨迹集合
- `g`
- `h`
- `t_s`

其中：

- `t_s` 当前被实现为同步时间下标
- 表示下一次冲突检查从哪个时间步继续开始

#### 2. `ConflictResult`

作用：

- 表示一次检测到的成对冲突

当前包含：

- 第一个机器人 id
- 第二个机器人 id
- `time_index`

说明：

- `time_index` 表示冲突发生在哪个同步时间步
- 它和 `SearchNode::t_s` 不是同一个概念

#### 3. `ConflictChecker`

作用：

- 判断两个机器人在某一同步时间步是否冲突
- 或判断整个节点在某个时间步是否存在冲突

当前做法：

- 将两个 group-scoped 的轻量状态 overlay 到同一个 full MoveIt `RobotState`
- 使用 `PlanningScene` 做真实碰撞检测
- 只把这两个机器人之间的碰撞视为当前 pair conflict

#### 4. `ConflictResolver`

作用：

- 实现 Stop-N-Go 中最基础的冲突解决动作：向轨迹中插入 pause

当前已实现：

- `searchPauseStep()`
- `insertPause()`
- `resolve()` 的 basic 版本

当前理解和实现遵循论文中的语义：

- 每次重试都从当前节点重新构造候选节点
- 通过更大的 `step_back_count` 去搜索更早的停顿开始位置

#### 5. `StopNGoSearch`

作用：

- 作为 Stop-N-Go 的 A* 搜索驱动器

当前已实现的部分包括：

- root node 构造
- 轨迹同步 helper
- open set 中最优节点选择
- 冲突扫描框架
- successor expansion 框架

当前状态：

- 搜索主循环已经有基础骨架
- 但还需要继续完善和稳定，暂时不能认为已经最终完成

---

### 三、`stop_n_go::app`

这一层负责面向 demo / 实验的上层流程组织。

#### `MultiArmScenarioBuilder`

作用：

- 在共享区域内采样多机械臂目标
- 对每个机器人分别做初始规划
- 同步并补齐轨迹
- 检查这些初始轨迹是否存在冲突
- 记录每一次尝试的结果
- 生成 RViz 用的联合显示轨迹
- 生成搜索区域和 sampled goals 的 marker 可视化

当前输入包括：

- `RobotRegistry`
- `InitialPlanner`
- `ConflictChecker`
- `StateLayout` 集合
- `reference_state`

当前输出包括：

- 搜索结果
- 每次 trial 的完整记录
- 如果找到有效样本，则返回对应 scenario

## 当前 demo / 验证内容

### 1. `plan_resample_execute_demo`

作用：

- 验证单机械臂 MoveIt 规划
- 验证自定义 `Trajectory` 的转换与重采样
- 验证转换为控制器 `JointTrajectory` 消息

### 2. `conflict_checker_demo`

作用：

- 使用一组已知冲突的 arm1 / arm2 状态
- 验证 `ConflictChecker` 能正确判断冲突

### 3. `find_multi_arm_overlap_goals_demo`

作用：

- 在仿真环境中寻找 4 个机械臂共享空间内的 goal
- 对每次尝试做记录
- 对前 10 次尝试的 sampled goal 做 marker 可视化
- 对共享采样区域 box 做可视化
- 如果找到一组“初始轨迹确实有冲突”的目标，则在 RViz 中播放联合初始轨迹

当前这个 demo 的目标是：

- 证明“能够找到会产生初始冲突的 goal”
- 证明“联合初始轨迹在 RViz 中确实表现出冲突趋势”

## 当前的一些关键实现理解

### 1. 关于 `conflictCheck`

当前理解是：

- `conflictCheck` 更像“检查单个同步时间步是否存在冲突”
- 真正沿时间轴的扫描由外层循环完成

### 2. 关于 `SearchNode::t_s`

当前理解是：

- `t_s` 表示冲突检查继续开始的同步时间下标
- 它代表已处理前缀的边界

### 3. 关于轨迹同步

当前做法是：

- 对所有轨迹做固定时间步长的 resample
- 然后显式补齐到相同长度

这样更便于在统一时间网格上做冲突检测和搜索。

## 当前遇到的主要困难

当前最主要的困难是：

## 无法稳定选择有效的 goal

更具体地说：

- 我们希望 goal 位于 4 个机械臂的共享空间中
- 这样更容易诱发初始轨迹冲突
- 但共享区域中的很多 sampled pose 对某些机械臂来说并不可达
- 有些 sampled goal 在 IK 阶段就失败
- 有些 goal 通过 IK 后仍然可能在规划阶段失败

因此当前难点不是只有 Stop-N-Go 搜索本身，而是更前面的：

- 如何稳定地在共享空间中选出一组对 4 个机器人都有效、同时又足够容易产生冲突的 end effector goal

这个问题直接影响：

- demo 的稳定性
- 初始轨迹冲突样本的获得效率
- 后续 Stop-N-Go 搜索的验证效率

## 下一步建议

当前推荐的下一步包括：

1. 继续改进共享区域中的 goal 采样策略
2. 继续保留并分析每次 trial 的记录，理解 goal 无效的主要原因
3. 进一步完善 `StopNGoSearch::solve()`，使其稳定完成完整搜索流程
4. 在找到稳定冲突样本之后，再扩展到“初始冲突轨迹 -> Stop-N-Go 解 -> 执行”的完整多臂 demo
