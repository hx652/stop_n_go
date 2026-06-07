#include <stop_n_go/base/initial_planner.hpp>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <stdexcept>
#include <utility>

namespace stop_n_go::base
{

namespace
{

moveit::core::RobotState referenceStateFromPlan(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const moveit::core::RobotState & fallback_state)
{
  moveit::core::RobotState reference_state(fallback_state);
  if (!moveit::core::robotStateMsgToRobotState(plan.start_state_, reference_state)) {
    reference_state = fallback_state;
    reference_state.update();
  }

  return reference_state;
}

}  // namespace

InitialPlanner::InitialPlanner(
  const RobotRegistry & robot_registry,
  moveit::core::RobotModelConstPtr robot_model,
  MoveGroupMap move_groups)
: robot_registry_(robot_registry),
  robot_model_(std::move(robot_model)),
  move_groups_(std::move(move_groups))
{
  validate();
}

Trajectory InitialPlanner::computeInitialTrajectory(
  RobotId robot_id,
  const geometry_msgs::msg::Pose & target_pose)
{
  const RobotDescriptor & robot = robot_registry_.descriptor(robot_id);
  moveit::planning_interface::MoveGroupInterface & move_group = moveGroup(robot_id);

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(5.0);
  if (current_state == nullptr) {
    throw std::runtime_error("Failed to get current robot state before planning");
  }

  move_group.setStartStateToCurrentState();
  if (!move_group.setPoseTarget(target_pose, robot.end_effector_link)) {
    throw std::runtime_error("MoveGroupInterface rejected the pose target");
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode result = move_group.plan(plan);
  move_group.clearPoseTargets();
  if (!static_cast<bool>(result)) {
    throw std::runtime_error("MoveGroupInterface failed to plan the initial trajectory");
  }

  const moveit::core::RobotState reference_state = referenceStateFromPlan(plan, *current_state);
  ConversionContext conversion_context = makeConversionContext(robot_id, reference_state);

  robot_trajectory::RobotTrajectory moveit_trajectory(robot_model_, robot.group_name);
  moveit_trajectory.setRobotTrajectoryMsg(reference_state, plan.trajectory_);
  return conversion_context.fromMoveItTrajectory(moveit_trajectory);
}

Trajectory InitialPlanner::computeInitialTrajectory(
  RobotId robot_id,
  const geometry_msgs::msg::PoseStamped & target_pose)
{
  const RobotDescriptor & robot = robot_registry_.descriptor(robot_id);
  moveit::planning_interface::MoveGroupInterface & move_group = moveGroup(robot_id);

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(5.0);
  if (current_state == nullptr) {
    throw std::runtime_error("Failed to get current robot state before planning");
  }

  move_group.setStartStateToCurrentState();
  if (!move_group.setPoseTarget(target_pose, robot.end_effector_link)) {
    throw std::runtime_error("MoveGroupInterface rejected the stamped pose target");
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode result = move_group.plan(plan);
  move_group.clearPoseTargets();
  if (!static_cast<bool>(result)) {
    throw std::runtime_error("MoveGroupInterface failed to plan the initial trajectory");
  }

  const moveit::core::RobotState reference_state = referenceStateFromPlan(plan, *current_state);
  ConversionContext conversion_context = makeConversionContext(robot_id, reference_state);

  robot_trajectory::RobotTrajectory moveit_trajectory(robot_model_, robot.group_name);
  moveit_trajectory.setRobotTrajectoryMsg(reference_state, plan.trajectory_);
  return conversion_context.fromMoveItTrajectory(moveit_trajectory);
}

Trajectory InitialPlanner::computeInitialTrajectoryToRandomJointGoal(RobotId robot_id)
{
  const RobotDescriptor & robot = robot_registry_.descriptor(robot_id);
  moveit::planning_interface::MoveGroupInterface & move_group = moveGroup(robot_id);

  // 1. 获取当前机器人状态
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(5.0);
  if (current_state == nullptr) {
    throw std::runtime_error(
      "Failed to get current robot state before random joint goal planning");
  }

  const moveit::core::JointModelGroup * jmg =
    current_state->getJointModelGroup(robot.group_name);
  if (jmg == nullptr) {
    throw std::runtime_error(
      "Failed to get JointModelGroup for group: " + robot.group_name);
  }

  // 2. 在关节限位内采样随机目标状态（最多重试 100 次）
  moveit::core::RobotState goal_state(*current_state);
  constexpr int kMaxSampleAttempts = 100;
  bool sampled = false;
  for (int attempt = 0; attempt < kMaxSampleAttempts; ++attempt) {
    goal_state.setToRandomPositions(jmg);
    goal_state.update();
    if (goal_state.satisfiesBounds(jmg)) {
      sampled = true;
      break;
    }
  }
  if (!sampled) {
    throw std::runtime_error(
      "Failed to sample a valid random joint goal within bounds for group: " + robot.group_name);
  }

  // 3. 提取该规划组的关节值，设为目标
  std::vector<double> joint_values;
  goal_state.copyJointGroupPositions(jmg, joint_values);

  move_group.setStartStateToCurrentState();
  if (!move_group.setJointValueTarget(joint_values)) {
    throw std::runtime_error(
      "MoveGroupInterface rejected the random joint value target for group: " + robot.group_name);
  }

  // 4. 规划
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode result = move_group.plan(plan);
  if (!static_cast<bool>(result)) {
    throw std::runtime_error(
      "MoveGroupInterface failed to plan to random joint goal for group: " + robot.group_name);
  }

  // 5. 转换为 stop_n_go::Trajectory
  const moveit::core::RobotState reference_state = referenceStateFromPlan(plan, *current_state);
  ConversionContext conversion_context = makeConversionContext(robot_id, reference_state);

  robot_trajectory::RobotTrajectory moveit_trajectory(robot_model_, robot.group_name);
  moveit_trajectory.setRobotTrajectoryMsg(reference_state, plan.trajectory_);
  return conversion_context.fromMoveItTrajectory(moveit_trajectory);
}

moveit::planning_interface::MoveGroupInterface & InitialPlanner::moveGroup(RobotId robot_id) const
{
  const auto iterator = move_groups_.find(robot_id);
  if (iterator == move_groups_.end() || iterator->second == nullptr) {
    throw std::out_of_range(
            "InitialPlanner does not contain a MoveGroupInterface for the requested robot id");
  }

  return *iterator->second;
}

ConversionContext InitialPlanner::makeConversionContext(
  RobotId robot_id,
  const moveit::core::RobotState & reference_state) const
{
  return ConversionContext(
    robot_model_,
    std::make_shared<StateLayout>(*robot_model_, robot_registry_.groupName(robot_id)),
    reference_state);
}

void InitialPlanner::validate() const
{
  if (robot_model_ == nullptr) {
    throw std::invalid_argument("InitialPlanner robot model must not be null");
  }

  if (move_groups_.size() != robot_registry_.size()) {
    throw std::invalid_argument(
            "InitialPlanner must receive exactly one MoveGroupInterface per registered robot");
  }

  for (const auto & [robot_id, move_group] : move_groups_) {
    if (!robot_registry_.hasRobot(robot_id)) {
      throw std::invalid_argument(
              "InitialPlanner received a MoveGroupInterface for an unknown robot id");
    }
    if (move_group == nullptr) {
      throw std::invalid_argument("InitialPlanner MoveGroupInterface pointers must not be null");
    }

    const RobotDescriptor & robot = robot_registry_.descriptor(robot_id);
    if (move_group->getName() != robot.group_name) {
      throw std::invalid_argument(
              "InitialPlanner MoveGroupInterface name does not match RobotRegistry group_name");
    }
    if (move_group->getRobotModel().get() != robot_model_.get()) {
      throw std::invalid_argument(
              "InitialPlanner MoveGroupInterface robot model does not match planner robot model");
    }
  }
}

}  // namespace stop_n_go::base
