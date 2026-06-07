#pragma once

#include <cstddef>
#include <vector>

#include <stop_n_go/base/robot_registry.hpp>
#include <stop_n_go/base/trajectory.hpp>

namespace stop_n_go::core
{

/// A* search node for Stop-N-Go temporal conflict resolution.
class SearchNode
{
public:
  /// Construct an empty node.
  SearchNode();

  /// Construct a node from a full set of synchronized robot trajectories.
  explicit SearchNode(std::vector<stop_n_go::base::Trajectory> trajectories);

  /// Return the full trajectory set owned by this node.
  const std::vector<stop_n_go::base::Trajectory> & trajectories() const;

  /// Return mutable access to the full trajectory set owned by this node.
  std::vector<stop_n_go::base::Trajectory> & trajectories();

  /// Return the trajectory for one robot id.
  const stop_n_go::base::Trajectory & trajectory(stop_n_go::base::RobotId robot_id) const;

  /// Return mutable access to the trajectory for one robot id.
  stop_n_go::base::Trajectory & trajectory(stop_n_go::base::RobotId robot_id);

  /// Return the number of robots represented in this node.
  std::size_t robotCount() const;

  /// Return true when the node contains no trajectories.
  bool empty() const;

  /// Return the path cost accumulated up to the current processed conflict frontier.
  double g() const;

  /// Return the heuristic estimate of the remaining makespan.
  double h() const;

  /// Return the A* evaluation value `f = g + h`.
  double f() const;

  /// Set the path cost `g`.
  void setG(double g_cost);

  /// Set the heuristic cost `h`.
  void setH(double h_cost);

  /// Return the synchronized time-step index where conflict scanning should resume.
  std::size_t tS() const;

  /// Set the synchronized time-step index where conflict scanning should resume.
  void setTS(std::size_t t_s);

private:
  /// All robot trajectories for this node.
  ///
  /// The vector is indexed by `RobotId`, so `trajectories_[robot_id]` is the trajectory for that
  /// robot. Every trajectory is expected to already be synchronized to the same time-step before
  /// the search begins.
  std::vector<stop_n_go::base::Trajectory> trajectories_;

  /// Path cost already committed by this node.
  ///
  /// This corresponds to the execution time up to the current processed conflict frontier.
  double g_cost_ = 0.0;

  /// Heuristic estimate of the remaining cost.
  ///
  /// In the Stop-N-Go paper this is the remaining makespan estimate from the current node.
  double h_cost_ = 0.0;

  /// Synchronized time-step index where the next conflict scan should begin.
  ///
  /// Semantically, times before `t_s_` have already been processed for this node, so the search
  /// does not need to restart conflict checking from the beginning of the trajectories.
  std::size_t t_s_ = 0;
};

}  // namespace stop_n_go::core
