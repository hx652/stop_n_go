## Pose

要完整说清一个位姿，需要以下信息:
- 位置
- 朝向
- 参考坐标系
- 这个位姿对应的是哪个对象/坐标系

## Moveit中的处理

主要关注 `move_group.setPoseTarget()`

在 MoveIt 中，这 4 个信息不是放在同一个对象里的，而是分开表达:

- 位置: 放在 `Pose.position`
- 朝向: 放在 `Pose.orientation`
- 参考坐标系: 放在 `PoseStamped.header.frame_id`，或者由 `MoveGroupInterface` 中保存的当前 pose reference frame 提供
- 这个位姿对应的是谁: 由 `end_effector_link` 指定

其中，pose reference frame 可以理解为 `MoveGroupInterface` 自身维护的一项上下文设置。

- 如果传入的是 `PoseStamped`，则参考坐标系直接写在 `target.header.frame_id` 中
- 如果传入的是 `Pose` 或 `Eigen::Isometry3d`，则 `MoveGroupInterface` 会使用它当前保存的 pose reference frame 来补全参考坐标系

可以抽象地理解为:

`setPoseTarget()` 表达的是:

"让某个 end effector link，在某个参考坐标系下，到达某个位置和朝向。"

也就是说，MoveIt 中一个完整的 pose goal 由下面几部分共同组成:

- `position`
- `orientation`
- `frame_id`
- `end_effector_link`

## `setPoseTarget()` 的几种形式

MoveIt 中常见的 `setPoseTarget()` 有以下几种重载:

### 1. `setPoseTarget(const Eigen::Isometry3d& pose, const std::string& end_effector_link = "")`

含义:
- 使用 `Eigen::Isometry3d` 表达目标位姿
- 位置和朝向来自这个刚体变换
- 参考坐标系不在参数里显式给出，而是使用 `MoveGroupInterface` 当前保存的 pose reference frame
- 目标对象由 `end_effector_link` 指定；如果不写，则使用当前默认末端 link

适用场景:
- 代码里本来就使用 `Eigen` 变换表示位姿

### 2. `setPoseTarget(const geometry_msgs::msg::Pose& target, const std::string& end_effector_link = "")`

含义:
- 使用裸 `Pose` 表达目标位姿
- 参数中只包含位置和朝向
- 参考坐标系不在 `Pose` 内，需要由 `MoveGroupInterface` 当前保存的 pose reference frame 补充
- 目标对象由 `end_effector_link` 指定；如果不写，则使用当前默认末端 link

适用场景:
- 已经有一个 `Pose`，只想设置位置和朝向

注意:
- 这种形式最容易忽略参考坐标系，因为 `Pose` 本身不带 `frame_id`

### 3. `setPoseTarget(const geometry_msgs::msg::PoseStamped& target, const std::string& end_effector_link = "")`

含义:
- 使用 `PoseStamped` 表达目标位姿
- 位置和朝向在 `target.pose` 中
- 参考坐标系由 `target.header.frame_id` 显式给出
- 目标对象由 `end_effector_link` 指定；如果不写，则使用当前默认末端 link

适用场景:
- 想显式、完整地给出 pose goal 的语义
- 想避免参考坐标系来源不清楚的问题

## 总结

从抽象上看，MoveIt 中的 `setPoseTarget()` 并不是只接收一个 pose 数值，而是在组织 4 个信息:

- 到哪里: `position`
- 以什么姿态到那里: `orientation`
- 这些数值是相对于谁表达的: `frame_id`
- 要控制的是谁: `end_effector_link`

如果只是想把 pose goal 语义表达得最完整、最不容易混淆，`PoseStamped` 形式通常最直接。
