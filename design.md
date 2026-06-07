### Intro

第一次写实验以外的项目，lot of emotions, some are positive, some are negative, but all of them are good, they are just part of this process

### Level 0

整体的行为流程:
- 为每个臂单独做规划
- 轨迹插值
- 搜索
    - 冲突检测
    - 冲突解决

为每个臂单独做规划:
- input: 每个机械臂的目标
- output: 每个机械臂的轨迹(将其它机械臂视为静态障碍进行单臂规划，不考虑其它机械臂的时间变化)

轨迹插值:
- input: 每个机械臂的轨迹(不保证同步)
- output: 同步后每个机械臂的轨迹

搜索:
- input: 同步后每个机械臂的轨迹
- output: 考虑相互避障的每个机械臂的轨迹(在不改变空间路径的前提下，通过插入 pause 得到无冲突时间化轨迹)

冲突检测:
- input: 当前轨迹集合，一个时间戳(含义不是太明白)
- output: 相互冲突的两个机器人编号, 一个时间戳(应该是冲突的时间节点)

冲突解决:
- input: 当前节点, 相互冲突的两个机器人编号, 要调整的机器人编号, 模式切换的indicator
- output: 调整后的节点

### Level 1

关键信息:
- 目标: end effector的目标pose(xyz, rpy)
- 轨迹: sequence of pair, each pair is (q, t), q: state in C-space, t: time
- 节点: 包含:
    - 当前的轨迹集合
    - cost(两部分构成)
    - t_s(含义尚不明确)
- 冲突: 包含:
    - 冲突的两个机器人的编号: i, j
    - 冲突的时刻

### Level 2

核心抽象

class:
- goal
- trajectory
- conflict result
- sreachtreenode

function:
- initial plan
    - input: goal
    - output: trajectory
- trajectory sync
    - input: set of trajectory
    - output: set of trajectory(synced)
- conflict check
    - input: searchtree node
    - output: conflict result
- conflict resolution
    - input: searchtreenode
    - output: modified searchtreenode
- stop-n-go search
    - input: searchtreenode
    - output: searchtreenode(without conflict)

### Level 3

trajectory:
- field:
    - sequence of (q, t) pair
- method:
    - interpolation
    - insert:
        - input:
            - q(state to be inserted)
            - duration(number of state to be inserted)
        - output:
            - modified trajectory
    - copy:
        - output:
            - new trajectory exactly same as origin


在完成插值后，index和t时有一个固定关系的

searchnode:
- field:
    - set of trajectory
    - cost
    - t_s
- method:
    - copy
    - g
    - h
    - f

conflict:
- field:
    - i
    - j
    - t_s

关于 conflict check 的理解更新:
- 论文里的 `CONFLICTCHECK(T, t)` 更像“检查单个时间步 `t` 是否存在冲突”，而不是一个函数内部自己从头扫完整条轨迹
- 真正沿时间轴从前往后扫描的是 Alg. 1 里的外层循环: `for t <- N.ts to max(tau)`
- 也就是说，`CONFLICTCHECK` 的核心 abstraction 更接近一个单时间步 predicate / query:
    - input: 轨迹集合, 一个同步时间下标 `t`
    - output: 该时间下标是否有冲突; 如果有，返回冲突的机器人对
- 论文伪代码里写成 `C, N.ts <- CONFLICTCHECK(T, t)`，可以理解为在时间步 `t` 做检查，并把当前发现冲突的位置继续记录在 `N.ts` 里
- 在 `CONFLICTRESOLUTION` 里，论文又使用 `CONFLICTCHECK({T.C[0], T.C[1]}, i)`，这里说明它还存在一个“只检查两条轨迹在单个时间步 `i` 是否冲突”的局部版本
- 因此，从实现 abstraction 的角度，更合理的设计是:
    - 外层搜索 / 冲突解决逻辑负责“从哪个时间下标开始、扫到哪个时间下标结束”
    - `conflict check` 本身负责“给定一个时间下标，判断这一时刻是否有冲突”
- 在当前系统已经把轨迹统一重采样后，`t` 最自然地实现为同步时间下标 `index`
- 这样 `SearchNode::t_s` 表示“下一次从哪个 index 继续扫描”，而 `ConflictResult::time_index` 表示“本次检测到的冲突发生在哪个 index”

关于 conflict resolution 的理解更新:
- 对论文 Alg. 2 的一个关键理解是: while 循环的每一轮开头都会重新执行 `T <- N.T, ts <- N.ts`
- 这意味着当前轮的 pause 搜索总是以“当前节点里的整套轨迹”为基准，而不是在某个临时局部变量上无限叠加改动后再继续往前找
- 与之配合的是参数 `n`:
    - `n = 1` 时，`SEARCHPAUSESTEP` 找的是距离 `ts` 最近的、且状态不同于冲突状态的那个更早配置
    - 如果插入 pause 后在 `[tp, ts]` 内仍然检测到冲突，则 `n` 递增
    - 下一轮重新从 `N.T` 出发，但这次要求找“第 `n` 个更早的不同配置”，也就是把停顿开始点继续往前推
- 因此，这里的搜索语义不是“在上一次已经选中的 pause_start 上再继续微调”，而是“不断尝试更远的停顿开始位置”
- 这样理解后，`searchPauseStep(trajectory, ts, n)` 的抽象就比较清楚:
    - input: 当前用于被修改的轨迹, 当前冲突时间下标 `ts`, 第几次向前回退 `n`
    - output: 第 `n` 个满足 `index < ts` 且状态不同于冲突状态的时间下标
- 也就是说，`n` 实际上是在控制“向前回退多远”，而 line 3 的重新赋值保证每一轮都是以当前节点轨迹集合为一致起点来重新尝试这个更远的 pause 起点


compute_initial_plan:
- input:
    - robot id(indicate planning for specific robot)
    - end effector goal
- output:
    - trajectory

trajectory sync:
- input:
    - set of trajectory
    - t_intv
- output:
    - set of synced trajectory(with shared time step and treated as same length)

trajectory interpolation:
- input:
    - trajectory
    - t_intv
- output:
    - modified trajectory, time step is t_intv

### Level 4

用int指代机器人(从0开始)

id_to_group

trajectory:
- 自己写还是用moveit的
    - 自己写，与moveit做桥接

### 成员指针语义

成员如果写成指针，语义必须从类型本身就能看出来，至少要回答三个问题:
- 这个对象归谁拥有
- 这个成员能不能为空
- 这个成员是否参与生命周期管理

建议规则:
- 默认优先值语义: `Goal`、`Trajectory`、`ConflictResult` 这类小型核心数据对象尽量直接作为成员或容器元素，不要先默认塞进堆里
- `std::unique_ptr<T>`: 表示唯一拥有者，适合延迟构造、运行时多态、或特别重而不希望拷贝的成员
- `std::shared_ptr<T>`: 表示共享生命周期，只有当多个对象确实共同拥有同一个实例时才使用，不要把它当作“通用指针”
- `std::weak_ptr<T>`: 表示观察者，不拥有对象，主要用于打破 `shared_ptr` 环
- `T*` / `const T*`: 表示非 owning 引用，可以为空；调用方或外部系统负责对象生命周期
- `T&` / `const T&`: 表示非 owning 且不能为空；如果构造后引用关系稳定，比裸指针更清晰

对当前这些抽象，倾向于这样处理:
- `Goal`、`Trajectory`、`ConflictResult`: 值对象，优先直接存值
- `SearchTreeNode`: 如果节点需要长期保留并形成搜索树，可以让 `parent` 成为非 owning 引用或显式的拥有关系；如果多个分支共享父节点生命周期，再考虑 `std::shared_ptr<const SearchTreeNode>`
- 轨迹集合: 如果节点修改时需要彼此隔离，优先让每个节点拥有自己的值拷贝；只有在确认共享不会引入隐式联动时才考虑共享指针

一个核心原则:
- 选择指针类型是在表达设计，不只是为了“方便传来传去”
- 看到成员声明时，应该能直接理解 ownership 和 lifetime，而不需要去读实现猜语义

例子:

```cpp
class SearchTreeNode
{
public:
  using Ptr = std::shared_ptr<SearchTreeNode>;
  using ConstPtr = std::shared_ptr<const SearchTreeNode>;

private:
  std::vector<Trajectory> trajectories_;                 // owned, non-null, value semantics
  SearchTreeNode::ConstPtr parent_;                      // shared lifetime only if tree nodes are shared
  const PlanningSceneMonitor* planning_scene_monitor_;   // non-owning, may be null
};
```

这里的语义分别是:
- `trajectories_`: 当前节点直接拥有
- `parent_`: 当前节点和别的节点可能共同引用同一个父节点
- `planning_scene_monitor_`: 只是借用外部对象，不负责释放

### MoveIt 中的 Link / Joint / Variable

可以把这三层理解成:
- `Link`: 刚体 / 几何层
- `Joint`: 运动学约束层
- `Variable`: 状态向量层

它们不是并列的三种“同级对象”，而是三层不同粒度的 abstraction。

整体关系:
- `Link` 是机器人上的刚体节点
- `Joint` 连接 `parent link` 和 `child link`
- 一个 `Joint` 决定了 0 个、1 个或多个 `Variable`
- `RobotState` 真正存储的是一整块按 `Variable` 排列的状态向量，不是直接存“joint 对象”

可以粗略画成:

```text
Link --Joint--> Link --Joint--> Link ...
          |
       Variable(s)
```

### MoveIt 中的 RobotModel / RobotState / RobotTrajectory

可以把这三个核心抽象理解成一条链:
- `RobotModel`: 机器人“是什么”
- `RobotState`: 机器人“现在在哪个状态”
- `RobotTrajectory`: 机器人“如何随时间从一个状态走到另一个状态”

`RobotModel` 是静态模型，主要来自 URDF + SRDF，描述机器人的 kinematic structure 和 semantic structure。它主要包含:
- 机器人整体元信息: model name、model frame、root joint、root link
- 全部 `Link` / `Joint` / `Variable` 的拓扑、名字、索引顺序
- active joint、fixed joint、mimic joint、continuous joint 等关节语义
- planning group、end effector、subgroup 等规划语义
- bounds、default values、random sampling、distance / interpolate 等和状态空间有关的规则

从 abstraction 的角度看，`RobotModel` 更像是“静态 schema”:
- 它定义状态空间长什么样
- 它定义机器人由哪些 link 和 joint 组成
- 它定义 group、eef、bounds、默认姿态这些高层语义

`RobotState` 是一个具体状态点，是建立在 `RobotModel` 之上的动态对象。它主要包含:
- 当前所有 `Variable` 的 position / velocity / acceleration / effort
- 由当前 joint values 派生出来的 joint transform、link transform、collision body transform
- dirty flag 和 lazy update 机制，用来避免重复做 FK
- attached body 等附着物体信息

从 abstraction 的角度看，`RobotState` 不只是“一组 joint angle”，而是:
- 一个 configuration-space state
- 一个可查询几何信息的状态容器
- 一个可以直接做 FK、IK、Jacobian、bounds check、interpolation 的操作对象

`RobotTrajectory` 是带时间信息的状态序列。它主要包含:
- 一个 `RobotModel` 引用
- 一个可选的 `JointModelGroup`
- 一串 waypoint；每个 waypoint 本质上是一个 `RobotState`
- 每个 waypoint 相对前一个 waypoint 的 duration

从 abstraction 的角度看，`RobotTrajectory` 表达的是:
- 一条离散的、时间化的路径
- 一个按时间顺序排列的 `RobotState` 序列
- 一个可以做 append、reverse、unwind、按时间采样和消息转换的轨迹容器

一句话总结:
- `RobotModel` 负责定义“状态空间和机器人结构”
- `RobotState` 负责表示“状态空间中的一个点”
- `RobotTrajectory` 负责表示“状态空间中的一条时间化离散路径”

### 轻量 RobotState 设计思路

当前准备自己实现 `Trajectory`，同时保留和 MoveIt trajectory 做转换的能力。为此，先实现一个轻量的 `RobotState`，但这个轻量版本不追求覆盖 MoveIt `RobotState` 的全部语义。

当前设计选择:
- 这个轻量 `RobotState` 只表示“单个 planning group 的 active joints / active variables”
- 之所以先做 group 级，而不是整机级，是因为当前方法本来就是先对单个机械臂分别规划

这个轻量 `RobotState` 先只回答一件事:
- 某个 planning group 在某一时刻的关节空间点是什么

明确不做的事情:
- 不表示整机状态
- 不表示时间
- 不直接保存几何状态、FK cache、collision info、attached body
- 不直接等价于 MoveIt 里那个较重的 `RobotState`

建议把“值”和“解释这些值的规则”拆开:
- 轻量 `RobotState`: 只存一组按固定顺序排列的数值，是纯值对象
- `GroupLayout` / `StateLayout`: 存 group name、joint / variable names 的顺序、bounds、continuous 语义、以及和 MoveIt group 的映射信息

这样拆分的好处:
- state 本身足够轻，适合大量拷贝和放进 trajectory
- 静态信息不需要在每个 state 里重复保存
- 与 MoveIt 的耦合可以集中在 layout / adapter 层，而不是污染核心数据结构

关于 state 中到底存什么，当前倾向于:
- 存一组 ordered values
- 这些 values 对应单个 planning group 的 active variables
- 在当前 UR 场景下，active joint 和 active variable 基本是一一对应的；概念上可以先说 joint values，但设计上保留 variable-level 的余地

state 中暂时不放:
- 时间戳
- `map<string, double>` 这种按名字存值的结构
- bounds / joint type / continuous flag 这类静态规则
- `moveit::core::RobotState` 指针或其他重对象

一个核心设计判断:
- 轻量 state 优先只存 active variables
- 不存 fixed joint
- 不直接存 mimic joint 的派生值

原因是:
- state 应该表达真正需要规划和控制的自由度
- mimic 更像规则，而不是独立状态维度；更适合放在 layout 或 MoveIt 转换层里处理

当前建议的 invariant:
- `values.size()` 必须和对应 layout 的 active variable count 一致
- 两个 state 只有在 layout 相同的前提下，才能做比较、插值、距离计算
- state 本身保持值语义，不持有时间和几何缓存

当前建议的能力边界:
- state 自身负责值语义、拷贝、按索引访问
- distance / interpolate 这类操作依赖 layout，因为 continuous joint 的最短角差等规则不应该硬编码在裸 state 里

当前更倾向的边界划分:
- `RobotState` 裸存 values
- `Trajectory` 持有 layout
- 所有解释 values 的操作显式依赖 layout

和 MoveIt 转换时的思路:
- 从 MoveIt group 提取时，按 group active variable 的顺序读出值，构造轻量 state
- 写回 MoveIt state 时，按 layout 顺序把值填回对应 group
- 从自定义 trajectory 转 MoveIt trajectory 时，除了 waypoint 和 duration，还需要一个 full reference state 来补齐非本 group 的部分

一句话总结:
- 这个轻量 `RobotState` 先被定义为“某个 planning group active variables 的有序值对象”

## Todo

非 owning 指针

- 写demo验证初始轨迹生成
- 验证SearchNode
