#include <stop_n_go/base/conversion.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace stop_n_go::base
{

namespace
{

builtin_interfaces::msg::Duration toRosDuration(double seconds)
{
  if (seconds < 0.0) {
    throw std::invalid_argument("Cannot convert a negative duration to ROS Duration");
  }

  const double whole_seconds = std::floor(seconds);
  const double fractional_seconds = seconds - whole_seconds;

  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<std::int32_t>(whole_seconds);
  duration.nanosec = static_cast<std::uint32_t>(std::llround(fractional_seconds * 1.0e9));
  if (duration.nanosec == 1000000000U) {
    ++duration.sec;
    duration.nanosec = 0U;
  }

  return duration;
}

bool hasMatchingLayout(const StateLayout & lhs, const StateLayout & rhs)
{
  return lhs.groupName() == rhs.groupName() && lhs.variableNames() == rhs.variableNames();
}

}  // namespace

ConversionContext::ConversionContext(
  moveit::core::RobotModelConstPtr robot_model_owner,
  StateLayout::ConstPtr layout,
  const moveit::core::RobotState & reference_state)
: robot_model_owner_(std::move(robot_model_owner)),
  layout_(std::move(layout)),
  joint_model_group_(layout_ != nullptr ? layout_->jointModelGroup() : nullptr),
  reference_state_(reference_state)
{
  validateReferenceState(reference_state_);
  validate();
  rebuildVariableIndexCache();
}

const moveit::core::RobotModelConstPtr & ConversionContext::robotModelOwner() const
{
  return robot_model_owner_;
}

const StateLayout::ConstPtr & ConversionContext::layoutPtr() const
{
  return layout_;
}

const StateLayout & ConversionContext::layout() const
{
  return *layout_;
}

const moveit::core::JointModelGroup & ConversionContext::jointModelGroup() const
{
  return *joint_model_group_;
}

const moveit::core::RobotState & ConversionContext::referenceState() const
{
  return reference_state_;
}

void ConversionContext::setReferenceState(const moveit::core::RobotState & reference_state)
{
  validateReferenceState(reference_state);
  reference_state_ = reference_state;
}

RobotState ConversionContext::fromMoveItState(const moveit::core::RobotState & moveit_state) const
{
  validateReferenceState(moveit_state);

  RobotState state(layout_->size());
  for (std::size_t index = 0; index < variable_indices_in_full_state_.size(); ++index) {
    state[index] =
      moveit_state.getVariablePosition(static_cast<int>(variable_indices_in_full_state_[index]));
  }

  return state;
}

moveit::core::RobotState ConversionContext::toMoveItState(const RobotState & state) const
{
  if (!layout_->isCompatible(state)) {
    throw std::invalid_argument("RobotState is incompatible with ConversionContext layout");
  }

  moveit::core::RobotState moveit_state(reference_state_);
  moveit_state.setJointGroupActivePositions(joint_model_group_, state.values());
  moveit_state.update();
  return moveit_state;
}

Trajectory ConversionContext::fromMoveItTrajectory(
  const robot_trajectory::RobotTrajectory & moveit_trajectory) const
{
  if (moveit_trajectory.getRobotModel().get() != robot_model_owner_.get()) {
    throw std::invalid_argument("RobotTrajectory model does not match ConversionContext model");
  }

  Trajectory trajectory(layout_);
  for (std::size_t index = 0; index < moveit_trajectory.size(); ++index) {
    const double duration_from_previous = moveit_trajectory.getWayPointDurationFromPrevious(index);
    trajectory.append(
      fromMoveItState(moveit_trajectory.getWayPoint(index)),
      duration_from_previous);
  }

  return trajectory;
}

trajectory_msgs::msg::JointTrajectory ConversionContext::toJointTrajectoryMsg(
  const Trajectory & trajectory) const
{
  validateTrajectory(trajectory);

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.joint_names = layout_->variableNames();
  trajectory_msg.points.reserve(trajectory.size());

  double time_from_start = 0.0;
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = trajectory.waypoint(index).values();
    time_from_start += trajectory.durationFromPrevious(index);
    point.time_from_start = toRosDuration(time_from_start);
    trajectory_msg.points.push_back(std::move(point));
  }

  return trajectory_msg;
}

void ConversionContext::validate() const
{
  if (robot_model_owner_ == nullptr) {
    throw std::invalid_argument("ConversionContext robot model owner must not be null");
  }

  if (layout_ == nullptr) {
    throw std::invalid_argument("ConversionContext layout must not be null");
  }

  if (layout_->robotModel() != robot_model_owner_.get()) {
    throw std::invalid_argument("ConversionContext layout and robot model owner do not match");
  }

  if (joint_model_group_ == nullptr) {
    throw std::invalid_argument("ConversionContext layout does not reference a joint model group");
  }
}

void ConversionContext::validateReferenceState(const moveit::core::RobotState & reference_state)
const
{
  if (robot_model_owner_ != nullptr &&
    reference_state.getRobotModel().get() != robot_model_owner_.get())
  {
    throw std::invalid_argument("Reference RobotState model does not match ConversionContext model");
  }
}

void ConversionContext::validateTrajectory(const Trajectory & trajectory) const
{
  if (!hasMatchingLayout(trajectory.layout(), *layout_)) {
    throw std::invalid_argument("Trajectory layout does not match ConversionContext layout");
  }
}

void ConversionContext::rebuildVariableIndexCache()
{
  variable_indices_in_full_state_.clear();
  variable_indices_in_full_state_.reserve(layout_->size());

  for (const std::string & variable_name : layout_->variableNames()) {
    variable_indices_in_full_state_.push_back(robot_model_owner_->getVariableIndex(variable_name));
  }
}

}  // namespace stop_n_go::base
