#include <stop_n_go/core/conflict_result.hpp>

namespace stop_n_go::core
{

ConflictResult::ConflictResult(
  stop_n_go::base::RobotId first_robot,
  stop_n_go::base::RobotId second_robot,
  std::size_t time_index)
: first_robot_(first_robot),
  second_robot_(second_robot),
  time_index_(time_index)
{
}

stop_n_go::base::RobotId ConflictResult::firstRobot() const
{
  return first_robot_;
}

stop_n_go::base::RobotId ConflictResult::secondRobot() const
{
  return second_robot_;
}

std::size_t ConflictResult::timeIndex() const
{
  return time_index_;
}

}  // namespace stop_n_go::core
