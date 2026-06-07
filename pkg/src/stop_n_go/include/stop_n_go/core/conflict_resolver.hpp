#pragma once

#include <memory>

#include <stop_n_go/core/conflict_check.hpp>
#include <stop_n_go/core/log_level.hpp>

namespace stop_n_go::core
{

/// Temporal conflict resolver used by Stop-N-Go search.
class ConflictResolver
{
public:
  enum class Mode
  {
    Basic,
    Jump,
  };

  /// Construct the resolver from a shared conflict checker and log level.
  explicit ConflictResolver(
    std::shared_ptr<const ConflictChecker> conflict_checker,
    LogLevel log_level = LogLevel::Summary);

  /// Update the log level used for pause-insertion messages.
  void setLogLevel(LogLevel log_level);

  /// Return the shared conflict checker used by this resolver.
  const std::shared_ptr<const ConflictChecker> & conflictChecker() const;

  /// Resolve one pairwise conflict by inserting a pause into the selected robot trajectory.
  ///
  /// Design choice: the current implementation follows the basic Stop-N-Go resolution pattern.
  /// For each retry it rebuilds a fresh successor candidate from the input node, then uses a
  /// larger `step_back_count` to search for an earlier pause-start index. This matches the paper's
  /// idea of searching farther back in the trajectory rather than accumulating local edits on top of
  /// an already failed pause placement.
  SearchNode resolve(
    const SearchNode & node,
    const ConflictResult & conflict,
    stop_n_go::base::RobotId robot_id,
    Mode mode) const;

  /// Find the pause-start index for one trajectory.
  ///
  /// Design choice: this helper is public for now so the basic Stop-N-Go pause mechanics can be
  /// implemented and tested before wiring the full `resolve()` entry point.
  ///
  /// The function returns the `step_back_count`-th largest index smaller than
  /// `conflict_time_index` whose state differs from the state at `conflict_time_index`.
  std::size_t searchPauseStep(
    const stop_n_go::base::Trajectory & trajectory,
    std::size_t conflict_time_index,
    std::size_t step_back_count) const;

  /// Insert a pause so the robot stays at `pause_start_index` until `conflict_time_index`.
  ///
  /// The current implementation assumes the trajectory has already been synchronized to a fixed
  /// time-step. It preserves that time grid by inserting duplicated waypoints with the synchronized
  /// step duration and shifting the rest of the trajectory forward.
  void insertPause(
    stop_n_go::base::Trajectory & trajectory,
    std::size_t pause_start_index,
    std::size_t conflict_time_index) const;

private:
  void validate() const;
  double synchronizedStepDuration(const stop_n_go::base::Trajectory & trajectory) const;

  std::shared_ptr<const ConflictChecker> conflict_checker_;
  LogLevel log_level_ = LogLevel::Summary;
};

}  // namespace stop_n_go::core
