#pragma once

#include <cstddef>
#include <memory>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <string>
#include <vector>


namespace stop_n_go::base
{

/// Separate state from layout metadata.
/// `RobotState` stores only ordered values.
/// `StateLayout` explains how to interpret those values for one planning group.

/// Lightweight state for one planning group's active variables.
class RobotState
{
public:
  /// Construct an empty state.
  RobotState();

  /// Construct a state with `dof` values initialized to zero.
  explicit RobotState(std::size_t dof);

  /// Construct a state from an ordered value vector.
  explicit RobotState(std::vector<double> values);

  /// Return the number of stored values.
  std::size_t size() const;

  /// Return true when the state stores no values.
  bool empty() const;

  /// Return the value at `index` without bounds checking.
  double operator[](std::size_t index) const;

  /// Return a mutable reference to the value at `index` without bounds checking.
  double & operator[](std::size_t index);

  /// Return the value at `index` with bounds checking.
  double at(std::size_t index) const;

  /// Return a mutable reference to the value at `index` with bounds checking.
  double & at(std::size_t index);

  /// Return the underlying ordered value vector.
  const std::vector<double> & values() const;

  /// Return mutable access to the underlying ordered value vector.
  std::vector<double> & values();

  /// Resize the state to `dof` values.
  void resize(std::size_t dof);

  /// Replace all stored values.
  void assign(const std::vector<double> & values);

  /// Fill every stored value with `value`.
  void fill(double value);

  /// Return true when both states contain exactly the same values.
  bool operator==(const RobotState & other) const;

  /// Return true when the states differ in at least one value.
  bool operator!=(const RobotState & other) const;

  /// Return true when both states have the same dimension.
  bool hasSameDimension(const RobotState & other) const;

  /**
   * @brief Return whether two states are approximately equal.
   * @param other State to compare against.
   * @param tolerance Non-negative absolute tolerance.
   * @return True if all values differ by no more than tolerance.
   * @throws std::invalid_argument If tolerance is negative.
   */
  bool isApprox(const RobotState & other, double tolerance) const;

private:
  std::vector<double> values_;
};

/// Immutable layout metadata for one MoveIt planning group.
class StateLayout
{
public:
  /// Mutable shared ownership alias.
  using Ptr = std::shared_ptr<StateLayout>;

  /// Shared immutable ownership alias.
  using ConstPtr = std::shared_ptr<const StateLayout>;

  /// Build layout metadata from a robot model and planning group name.
  StateLayout(const moveit::core::RobotModel & robot_model, const std::string & group_name);

  /// Build layout metadata directly from a MoveIt joint model group.
  explicit StateLayout(const moveit::core::JointModelGroup & joint_model_group);

  /// Return the backing MoveIt robot model.
  const moveit::core::RobotModel * robotModel() const;

  /// Return the backing MoveIt joint model group.
  const moveit::core::JointModelGroup * jointModelGroup() const;

  /// Return the planning group name.
  const std::string & groupName() const;

  /// Return the number of active variables in this layout.
  std::size_t size() const;

  /// Return true when the layout contains no variables.
  bool empty() const;

  /// Return the ordered active variable names.
  const std::vector<std::string> & variableNames() const;

  /// Return the active variable name at `index`.
  const std::string & variableNameAt(std::size_t index) const;

  /// Return per-variable continuous-joint flags.
  const std::vector<bool> & continuousFlags() const;

  /// Return true when the variable at `index` is continuous.
  bool isContinuous(std::size_t index) const;

  /// Return true when `variable_name` belongs to this layout.
  bool hasVariable(const std::string & variable_name) const;

  /// Return the index of `variable_name` or throw if it is absent.
  std::size_t variableIndex(const std::string & variable_name) const;

  /// Return true when `state` has the same dimension as this layout.
  bool isCompatible(const RobotState & state) const;

private:
  void refreshCache();

  const moveit::core::RobotModel * robot_model_ = nullptr;
  const moveit::core::JointModelGroup * joint_model_group_ = nullptr;
  std::string group_name_;
  std::vector<std::string> variable_names_;
  std::vector<bool> continuous_flags_;
};

/// Interpolate between two states using the layout's variable semantics.
/// Continuous variables use the shortest angular difference.
RobotState interpolate(
  const RobotState & from,
  const RobotState & to,
  double ratio,
  const StateLayout & layout);

}  // namespace stop_n_go::base
