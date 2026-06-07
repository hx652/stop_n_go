#pragma once

#include <cstddef>

#include <stop_n_go/base/robot_registry.hpp>

namespace stop_n_go::core
{

/// Pairwise conflict detected between two robots at one synchronized time step.
class ConflictResult
{
public:
  /// Construct a conflict result.
  ConflictResult(
    stop_n_go::base::RobotId first_robot,
    stop_n_go::base::RobotId second_robot,
    std::size_t time_index);

  /// Return the first robot involved in the conflict.
  stop_n_go::base::RobotId firstRobot() const;

  /// Return the second robot involved in the conflict.
  stop_n_go::base::RobotId secondRobot() const;

  /// Return the synchronized time-step index where the conflict occurs.
  std::size_t timeIndex() const;

private:
  /// One robot involved in the conflict.
  stop_n_go::base::RobotId first_robot_;

  /// The other robot involved in the conflict.
  stop_n_go::base::RobotId second_robot_;

  /// Synchronized time-step index where this specific conflict is detected.
  ///
  /// This is intentionally separate from `SearchNode::t_s`, which marks where conflict scanning
  /// should resume for a node. `time_index_` identifies when the detected conflict occurs.
  std::size_t time_index_;
};

}  // namespace stop_n_go::core
