#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "stop_n_go/base/r_state.hpp"
#include "stop_n_go/base/trajectory.hpp"

namespace stop_n_go::base
{

/// Group-scoped conversion context between MoveIt and stop_n_go abstractions.
///
/// Design choice: this context keeps the conversion logic outside the lightweight core types.
/// It owns the MoveIt model handle and a full reference RobotState so group-only custom states
/// can still be projected to and from MoveIt consistently. For command output we currently export
/// directly to `trajectory_msgs::msg::JointTrajectory` instead of reconstructing a full
/// `robot_trajectory::RobotTrajectory`, because the current stop_n_go trajectory is already a
/// single-group, collision-free command trajectory.
///
/// Note on stored MoveIt handles:
/// - `StateLayout` already names the planning group and stores raw pointers to the MoveIt model
///   and group as semantic references.
/// - `ConversionContext` stores `robot_model_owner_` in addition to `layout_` to own the
///   RobotModel lifetime explicitly, so those raw pointers remain valid.
/// - `joint_model_group_` is only a cached convenience alias to the same group referenced by
///   `layout_`; it does not represent a second source of truth.
class ConversionContext
{
public:
  /// Construct a conversion context for one planning group.
  ConversionContext(
    moveit::core::RobotModelConstPtr robot_model_owner,
    StateLayout::ConstPtr layout,
    const moveit::core::RobotState & reference_state);

  /// Return the owned MoveIt robot model handle.
  const moveit::core::RobotModelConstPtr & robotModelOwner() const;

  /// Return the shared immutable layout used by this context.
  const StateLayout::ConstPtr & layoutPtr() const;

  /// Return the layout referenced by this context.
  const StateLayout & layout() const;

  /// Return the MoveIt joint model group referenced by this context.
  const moveit::core::JointModelGroup & jointModelGroup() const;

  /// Return the stored full-state reference used to fill non-group variables.
  const moveit::core::RobotState & referenceState() const;

  /// Replace the stored full-state reference.
  void setReferenceState(const moveit::core::RobotState & reference_state);

  /// Project a full MoveIt RobotState to a group-scoped stop_n_go RobotState.
  RobotState fromMoveItState(const moveit::core::RobotState & moveit_state) const;

  /// Reconstruct a full MoveIt RobotState by overlaying a group-scoped state on the reference.
  moveit::core::RobotState toMoveItState(const RobotState & state) const;

  /// Project a MoveIt RobotTrajectory onto the context's planning group.
  Trajectory fromMoveItTrajectory(
    const robot_trajectory::RobotTrajectory & moveit_trajectory) const;

  /// Export a group-scoped trajectory directly to a ROS JointTrajectory command.
  trajectory_msgs::msg::JointTrajectory toJointTrajectoryMsg(const Trajectory & trajectory) const;

private:
  void validate() const;
  void validateReferenceState(const moveit::core::RobotState & reference_state) const;
  void validateTrajectory(const Trajectory & trajectory) const;
  void rebuildVariableIndexCache();

  /// Owns the RobotModel lifetime so the raw pointers stored in `layout_` stay valid.
  moveit::core::RobotModelConstPtr robot_model_owner_;

  /// Defines the single-group projection used by every conversion in this context.
  StateLayout::ConstPtr layout_;

  /// Cached alias of `layout_->jointModelGroup()` for repeated group-scoped MoveIt calls.
  const moveit::core::JointModelGroup * joint_model_group_ = nullptr;

  /// Maps layout-local variable order to full MoveIt RobotState variable indices.
  std::vector<std::size_t> variable_indices_in_full_state_;

  /// Full-state template used when reconstructing MoveIt states from group-only stop_n_go states.
  moveit::core::RobotState reference_state_;
};

}  // namespace stop_n_go::base
