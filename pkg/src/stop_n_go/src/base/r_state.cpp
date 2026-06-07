#include <stop_n_go/base/r_state.hpp>

#include <algorithm>
#include <cmath>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <stdexcept>
#include <utility>

namespace stop_n_go::base
{

namespace
{

std::vector<std::string> getActiveVariableNames(
  const moveit::core::JointModelGroup & joint_model_group)
{
  // Cache the active variable order once so all RobotState objects can share it via StateLayout.
  std::vector<std::string> variable_names;
  variable_names.reserve(joint_model_group.getActiveVariableCount());

  for (const moveit::core::JointModel * joint_model : joint_model_group.getActiveJointModels()) {
    const std::vector<std::string> & joint_variable_names = joint_model->getVariableNames();
    variable_names.insert(
      variable_names.end(),
      joint_variable_names.begin(), joint_variable_names.end());
  }

  return variable_names;
}

std::vector<bool> getActiveContinuousFlags(const moveit::core::JointModelGroup & joint_model_group)
{
  // Expand joint-level continuous semantics into a per-variable lookup table.
  std::vector<bool> continuous_flags;
  continuous_flags.reserve(joint_model_group.getActiveVariableCount());

  const std::vector<const moveit::core::JointModel *> & continuous_joint_models =
    joint_model_group.getContinuousJointModels();

  for (const moveit::core::JointModel * joint_model : joint_model_group.getActiveJointModels()) {
    const bool is_continuous = std::find(
      continuous_joint_models.begin(), continuous_joint_models.end(), joint_model) !=
      continuous_joint_models.end();
    const std::size_t variable_count = joint_model->getVariableCount();
    for (std::size_t index = 0; index < variable_count; ++index) {
      continuous_flags.push_back(is_continuous);
    }
  }

  return continuous_flags;
}

}  // namespace

RobotState interpolate(
  const RobotState & from,
  const RobotState & to,
  double ratio,
  const StateLayout & layout)
{
  if (ratio < 0.0 || ratio > 1.0) {
    throw std::invalid_argument("Interpolation ratio must be in [0, 1]");
  }

  if (!from.hasSameDimension(to)) {
    throw std::invalid_argument("Cannot interpolate RobotState objects with different dimensions");
  }

  if (!layout.isCompatible(from) || !layout.isCompatible(to)) {
    throw std::invalid_argument(
            "Cannot interpolate RobotState objects incompatible with StateLayout");
  }

  if (ratio == 0.0) {
    return from;
  }

  if (ratio == 1.0) {
    return to;
  }

  constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

  RobotState state(from.size());
  for (std::size_t index = 0; index < from.size(); ++index) {
    if (layout.isContinuous(index)) {
      // Continuous joints interpolate through the shortest angular difference.
      const double delta = std::remainder(to[index] - from[index], kTwoPi);
      state[index] = from[index] + ratio * delta;
      continue;
    }

    state[index] = from[index] + ratio * (to[index] - from[index]);
  }

  return state;
}

RobotState::RobotState() = default;

RobotState::RobotState(std::size_t dof)
: values_(dof, 0.0)
{
}

RobotState::RobotState(std::vector<double> values)
: values_(std::move(values))
{
}

std::size_t RobotState::size() const
{
  return values_.size();
}

bool RobotState::empty() const
{
  return values_.empty();
}

double RobotState::operator[](std::size_t index) const
{
  return values_[index];
}

double & RobotState::operator[](std::size_t index)
{
  return values_[index];
}

double RobotState::at(std::size_t index) const
{
  return values_.at(index);
}

double & RobotState::at(std::size_t index)
{
  return values_.at(index);
}

const std::vector<double> & RobotState::values() const
{
  return values_;
}

std::vector<double> & RobotState::values()
{
  return values_;
}

void RobotState::resize(std::size_t dof)
{
  values_.resize(dof);
}

void RobotState::assign(const std::vector<double> & values)
{
  values_ = values;
}

void RobotState::fill(double value)
{
  std::fill(values_.begin(), values_.end(), value);
}

bool RobotState::operator==(const RobotState & other) const
{
  return values_ == other.values_;
}

bool RobotState::operator!=(const RobotState & other) const
{
  return !(*this == other);
}

bool RobotState::hasSameDimension(const RobotState & other) const
{
  return size() == other.size();
}

bool RobotState::isApprox(const RobotState & other, double tolerance) const
{
  if (tolerance < 0.0) {
    throw std::invalid_argument("RobotState tolerance must be non-negative");
  }

  if (!hasSameDimension(other)) {
    return false;
  }

  for (std::size_t index = 0; index < size(); ++index) {
    if (std::abs(values_[index] - other.values_[index]) > tolerance) {
      return false;
    }
  }

  return true;
}

StateLayout::StateLayout(
  const moveit::core::RobotModel & robot_model,
  const std::string & group_name)
{
  const moveit::core::JointModelGroup * joint_model_group = robot_model.getJointModelGroup(
    group_name);
  if (joint_model_group == nullptr) {
    throw std::invalid_argument("StateLayout group not found in RobotModel: " + group_name);
  }

  robot_model_ = &robot_model;
  joint_model_group_ = joint_model_group;
  refreshCache();
}

StateLayout::StateLayout(const moveit::core::JointModelGroup & joint_model_group)
{
  robot_model_ = &joint_model_group.getParentModel();
  joint_model_group_ = &joint_model_group;
  refreshCache();
}

const moveit::core::RobotModel * StateLayout::robotModel() const
{
  return robot_model_;
}

const moveit::core::JointModelGroup * StateLayout::jointModelGroup() const
{
  return joint_model_group_;
}

const std::string & StateLayout::groupName() const
{
  return group_name_;
}

std::size_t StateLayout::size() const
{
  return variable_names_.size();
}

bool StateLayout::empty() const
{
  return variable_names_.empty();
}

const std::vector<std::string> & StateLayout::variableNames() const
{
  return variable_names_;
}

const std::string & StateLayout::variableNameAt(std::size_t index) const
{
  return variable_names_.at(index);
}

const std::vector<bool> & StateLayout::continuousFlags() const
{
  return continuous_flags_;
}

bool StateLayout::isContinuous(std::size_t index) const
{
  return continuous_flags_.at(index);
}

bool StateLayout::hasVariable(const std::string & variable_name) const
{
  return std::find(
    variable_names_.begin(), variable_names_.end(),
    variable_name) != variable_names_.end();
}

std::size_t StateLayout::variableIndex(const std::string & variable_name) const
{
  const auto iterator = std::find(variable_names_.begin(), variable_names_.end(), variable_name);
  if (iterator == variable_names_.end()) {
    throw std::out_of_range("StateLayout variable not found: " + variable_name);
  }

  return static_cast<std::size_t>(std::distance(variable_names_.begin(), iterator));
}

bool StateLayout::isCompatible(const RobotState & state) const
{
  return size() == state.size();
}

void StateLayout::refreshCache()
{
  if (joint_model_group_ == nullptr) {
    group_name_.clear();
    variable_names_.clear();
    continuous_flags_.clear();
    return;
  }

  group_name_ = joint_model_group_->getName();
  // Copy frequently used MoveIt metadata into lightweight caches for repeated trajectory/state operations.
  variable_names_ = getActiveVariableNames(*joint_model_group_);
  continuous_flags_ = getActiveContinuousFlags(*joint_model_group_);
}

}  // namespace stop_n_go::base
