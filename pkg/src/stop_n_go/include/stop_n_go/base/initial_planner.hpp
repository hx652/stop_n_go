#pragma once

#include <memory>
#include <unordered_map>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>

#include "stop_n_go/base/conversion.hpp"
#include "stop_n_go/base/robot_registry.hpp"

namespace stop_n_go::base
{

/// Planner that produces one stop_n_go trajectory per robot pose target.
class InitialPlanner
{
public:
  /// Map from stop_n_go robot ids to the MoveGroupInterface instance that plans for that robot.
  using MoveGroupMap =
    std::unordered_map<RobotId, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>;

  /// Construct the planner from robot metadata, a shared MoveIt model, and one move group per robot.
  InitialPlanner(
    const RobotRegistry & robot_registry,
    moveit::core::RobotModelConstPtr robot_model,
    MoveGroupMap move_groups);

  /// Plan from the current state to an end-effector pose target.
  Trajectory computeInitialTrajectory(
    RobotId robot_id,
    const geometry_msgs::msg::Pose & target_pose);

  /// Plan from the current state to a stamped end-effector pose target.
  Trajectory computeInitialTrajectory(
    RobotId robot_id,
    const geometry_msgs::msg::PoseStamped & target_pose);

  /// Plan from the current state to a random valid joint-space goal.
  /// Samples joint positions uniformly within the group's joint limits,
  /// then calls MoveGroupInterface::plan() toward that goal.
  /// Throws std::runtime_error if sampling or planning fails.
  Trajectory computeInitialTrajectoryToRandomJointGoal(RobotId robot_id);

private:
  moveit::planning_interface::MoveGroupInterface & moveGroup(RobotId robot_id) const;
  ConversionContext makeConversionContext(
    RobotId robot_id,
    const moveit::core::RobotState & reference_state) const;
  void validate() const;

  const RobotRegistry & robot_registry_;
  moveit::core::RobotModelConstPtr robot_model_;
  MoveGroupMap move_groups_;
};

}  // namespace stop_n_go::base
