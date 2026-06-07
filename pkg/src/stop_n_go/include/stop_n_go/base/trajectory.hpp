#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "stop_n_go/base/r_state.hpp"

namespace stop_n_go::base
{

/// Time-parameterized sequence of states for one planning group.
class Trajectory
{
public:
  /// Construct a trajectory that uses the shared immutable `layout`.
  explicit Trajectory(StateLayout::ConstPtr layout);

  /// Return the referenced layout.
  const StateLayout & layout() const;

  /// Return the shared layout pointer.
  const StateLayout::ConstPtr & layoutPtr() const;

  /// Return the number of waypoints.
  std::size_t size() const;

  /// Return true when the trajectory has no waypoints.
  bool empty() const;

  /// Return the stored waypoint sequence.
  const std::vector<RobotState> & waypoints() const;

  /// Return the stored per-segment durations.
  const std::vector<double> & durationsFromPrevious() const;

  /// Return the waypoint at `index`.
  const RobotState & waypoint(std::size_t index) const;

  /// Return mutable access to the waypoint at `index`.
  RobotState & waypoint(std::size_t index);

  /// Return the duration from waypoint `index - 1` to `index`.
  double durationFromPrevious(std::size_t index) const;

  /// Return the duration from the trajectory start to waypoint `index`.
  double durationFromStart(std::size_t index) const;

  /// Return the total trajectory duration.
  double totalDuration() const;

  /// Return true when `state` matches this trajectory's layout dimension.
  bool isCompatible(const RobotState & state) const;

  /// Remove all waypoints and durations while keeping the layout.
  void clear();

  /// Reserve storage for `size` waypoints and durations.
  void reserve(std::size_t size);

  /// Append a waypoint with its duration from the previous waypoint.
  void append(const RobotState & state, double duration_from_previous);

  /// Append a waypoint by moving it into the trajectory.
  void append(RobotState && state, double duration_from_previous);

  /// Append a pause by repeating the last waypoint for `duration` seconds.
  void appendHold(double duration);

  /// Resample onto a strict uniform clock with step `time_step`.
  /// The endpoint state is always preserved exactly. If the original total duration is not an
  /// integer multiple of `time_step`, the endpoint arrival time is delayed to the next sampling
  /// instant so that every non-zero duration in the resampled trajectory is exactly `time_step`.
  Trajectory resample(double time_step) const;

private:
  void validateLayout() const;
  void validateWaypoint(const RobotState & state, double duration_from_previous) const;

  // Shared immutable layout for this planning group.
  // Multiple trajectories may share the same layout instance.
  // The trajectory participates in shared ownership, but cannot mutate the layout through this pointer.
  StateLayout::ConstPtr layout_;
  std::vector<RobotState> waypoints_;
  std::vector<double> durations_from_previous_;
};

}  // namespace stop_n_go::base
