#include <stop_n_go/base/robot_registry.hpp>

#include <stdexcept>

namespace stop_n_go::base
{

RobotRegistry::RobotRegistry(std::vector<RobotDescriptor> robots)
{
  for (RobotDescriptor & robot : robots) {
    if (robot.group_name.empty()) {
      throw std::invalid_argument("RobotDescriptor group_name must not be empty");
    }
    if (robot.end_effector_link.empty()) {
      throw std::invalid_argument("RobotDescriptor end_effector_link must not be empty");
    }

    const auto [iterator, inserted] = robots_.emplace(robot.id, std::move(robot));
    (void)iterator;
    if (!inserted) {
      throw std::invalid_argument("RobotRegistry contains duplicate robot ids");
    }
  }
}

std::size_t RobotRegistry::size() const
{
  return robots_.size();
}

bool RobotRegistry::empty() const
{
  return robots_.empty();
}

bool RobotRegistry::hasRobot(RobotId id) const
{
  return robots_.find(id) != robots_.end();
}

const RobotDescriptor & RobotRegistry::descriptor(RobotId id) const
{
  const auto iterator = robots_.find(id);
  if (iterator == robots_.end()) {
    throw std::out_of_range("RobotRegistry does not contain the requested robot id");
  }

  return iterator->second;
}

const std::string & RobotRegistry::groupName(RobotId id) const
{
  return descriptor(id).group_name;
}

const std::string & RobotRegistry::endEffectorLink(RobotId id) const
{
  return descriptor(id).end_effector_link;
}

}  // namespace stop_n_go::base
