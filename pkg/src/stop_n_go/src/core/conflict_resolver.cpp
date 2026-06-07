#include <stop_n_go/core/conflict_resolver.hpp>

#include <iostream>
#include <stdexcept>
#include <utility>

namespace stop_n_go::core
{

ConflictResolver::ConflictResolver(
  std::shared_ptr<const ConflictChecker> conflict_checker,
  LogLevel log_level)
: conflict_checker_(std::move(conflict_checker)),
  log_level_(log_level)
{
  validate();
}

void ConflictResolver::setLogLevel(LogLevel log_level)
{
  log_level_ = log_level;
}

const std::shared_ptr<const ConflictChecker> & ConflictResolver::conflictChecker() const
{
  return conflict_checker_;
}

SearchNode ConflictResolver::resolve(
  const SearchNode & node,
  const ConflictResult & conflict,
  stop_n_go::base::RobotId robot_id,
  Mode mode) const
{
  if (mode == Mode::Jump) {
    throw std::logic_error("ConflictResolver Jump mode is not implemented yet");
  }

  const stop_n_go::base::RobotId first_robot = conflict.firstRobot();
  const stop_n_go::base::RobotId second_robot = conflict.secondRobot();
  if (robot_id != first_robot && robot_id != second_robot) {
    throw std::invalid_argument("ConflictResolver robot_id must belong to the conflict pair");
  }

  const stop_n_go::base::RobotId other_robot = robot_id == first_robot ? second_robot : first_robot;
  const std::size_t conflict_time_index = conflict.timeIndex();

  std::size_t step_back_count = 1U;
  while (true) {
    SearchNode candidate = node;
    stop_n_go::base::Trajectory & trajectory = candidate.trajectory(robot_id);

    const std::size_t pause_start_index = searchPauseStep(
      trajectory,
      conflict_time_index,
      step_back_count);
    insertPause(trajectory, pause_start_index, conflict_time_index);

    bool still_in_conflict = false;
    for (std::size_t time_index = pause_start_index; time_index <= conflict_time_index;
      ++time_index)
    {
      try {
        if (conflict_checker_->conflictCheck(
            robot_id,
            candidate.trajectory(robot_id),
            other_robot,
            candidate.trajectory(other_robot),
            time_index)
          .has_value())
        {
          still_in_conflict = true;
          break;
        }
      } catch (const std::out_of_range &) {
        throw std::out_of_range(
                "ConflictResolver checked a time index outside the pair trajectory range");
      }
    }

    if (!still_in_conflict) {
      candidate.setTS(conflict_time_index);
      if (log_level_ == LogLevel::Debug) {
        std::cout << "[sng.pause] selected_robot=" << robot_id
                  << " ts=" << conflict_time_index
                  << " tp=" << pause_start_index
                  << " duration_steps=" << conflict_time_index - pause_start_index << '\n';
      }
      return candidate;
    }

    ++step_back_count;
  }
}

std::size_t ConflictResolver::searchPauseStep(
  const stop_n_go::base::Trajectory & trajectory,
  std::size_t conflict_time_index,
  std::size_t step_back_count) const
{
  if (step_back_count == 0U) {
    throw std::invalid_argument("ConflictResolver step_back_count must be positive");
  }
  if (conflict_time_index >= trajectory.size()) {
    throw std::out_of_range("ConflictResolver conflict_time_index is outside the trajectory range");
  }
  if (conflict_time_index == 0U) {
    throw std::out_of_range(
            "ConflictResolver cannot search for a pause before the first trajectory state");
  }

  const stop_n_go::base::RobotState & conflict_state = trajectory.waypoint(conflict_time_index);
  std::size_t found_count = 0U;
  for (std::size_t index = conflict_time_index; index-- > 0U; ) {
    if (trajectory.waypoint(index) == conflict_state) {
      continue;
    }

    ++found_count;
    if (found_count == step_back_count) {
      return index;
    }
  }

  throw std::out_of_range("ConflictResolver could not find a valid pause-start index");
}

void ConflictResolver::insertPause(
  stop_n_go::base::Trajectory & trajectory,
  std::size_t pause_start_index,
  std::size_t conflict_time_index) const
{
  if (pause_start_index >= trajectory.size() || conflict_time_index >= trajectory.size()) {
    throw std::out_of_range("ConflictResolver pause indices are outside the trajectory range");
  }
  if (pause_start_index >= conflict_time_index) {
    throw std::invalid_argument(
            "ConflictResolver requires pause_start_index to be smaller than conflict_time_index");
  }

  const std::size_t insert_count = conflict_time_index - pause_start_index;
  const double step_duration = synchronizedStepDuration(trajectory);
  const stop_n_go::base::RobotState hold_state = trajectory.waypoint(pause_start_index);

  stop_n_go::base::Trajectory shifted(trajectory.layoutPtr());
  shifted.reserve(trajectory.size() + insert_count);

  for (std::size_t index = 0; index <= pause_start_index; ++index) {
    shifted.append(trajectory.waypoint(index), trajectory.durationFromPrevious(index));
  }

  for (std::size_t copy_index = 0; copy_index < insert_count; ++copy_index) {
    shifted.append(hold_state, step_duration);
  }

  for (std::size_t index = pause_start_index + 1; index < trajectory.size(); ++index) {
    shifted.append(trajectory.waypoint(index), trajectory.durationFromPrevious(index));
  }

  trajectory = std::move(shifted);
}

void ConflictResolver::validate() const
{
  if (conflict_checker_ == nullptr) {
    throw std::invalid_argument("ConflictResolver requires a valid ConflictChecker");
  }
}

double ConflictResolver::synchronizedStepDuration(const stop_n_go::base::Trajectory & trajectory)
const
{
  if (trajectory.size() < 2U) {
    throw std::invalid_argument(
            "ConflictResolver requires at least two waypoints to infer a synchronized step duration");
  }

  const double step_duration = trajectory.durationFromPrevious(1);
  if (step_duration <= 0.0) {
    throw std::invalid_argument(
            "ConflictResolver synchronized trajectories must have a positive time step");
  }

  return step_duration;
}

}  // namespace stop_n_go::core
