#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace stop_n_go::base
{

/// Stable robot identifier used by the stop_n_go algorithm layer.
using RobotId = std::size_t;

/// Static metadata that maps one algorithm robot id to one MoveIt planning group.
struct RobotDescriptor
{
  RobotId id;
  std::string group_name;
  std::string end_effector_link;
};

/// Registry of robots known to the stop_n_go system.
class RobotRegistry
{
public:
  /// Construct the registry from all known robot descriptors.
  explicit RobotRegistry(std::vector<RobotDescriptor> robots);

  /// Return the number of registered robots.
  std::size_t size() const;

  /// Return true when no robots are registered.
  bool empty() const;

  /// Return true when `id` exists in the registry.
  bool hasRobot(RobotId id) const;

  /// Return the descriptor for `id` or throw if absent.
  const RobotDescriptor & descriptor(RobotId id) const;

  /// Return the MoveIt planning group name for `id`.
  const std::string & groupName(RobotId id) const;

  /// Return the end-effector link used for pose-target planning for `id`.
  const std::string & endEffectorLink(RobotId id) const;

private:
  std::unordered_map<RobotId, RobotDescriptor> robots_;
};

}  // namespace stop_n_go::base
