#include <stop_n_go/core/stop_n_go_search.hpp>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace stop_n_go::core
{

StopNGoSearchFailure::StopNGoSearchFailure(const std::string & message)
: std::runtime_error(message)
{
}

StopNGoSearch::StopNGoSearch(
  std::shared_ptr<const ConflictChecker> conflict_checker,
  std::shared_ptr<const ConflictResolver> conflict_resolver,
  double synchronization_time_step)
: conflict_checker_(std::move(conflict_checker)),
  conflict_resolver_(std::move(conflict_resolver)),
  options_(StopNGoSearchOptions{
    synchronization_time_step,
    StopNGoSearchOptions{}.max_expanded_nodes,
    StopNGoSearchOptions{}.log_level})
{
  validate();
}

StopNGoSearch::StopNGoSearch(
  std::shared_ptr<const ConflictChecker> conflict_checker,
  std::shared_ptr<const ConflictResolver> conflict_resolver,
  StopNGoSearchOptions options)
: conflict_checker_(std::move(conflict_checker)),
  conflict_resolver_(std::move(conflict_resolver)),
  options_(options)
{
  validate();
}

SearchNode StopNGoSearch::solve(
  const std::vector<stop_n_go::base::Trajectory> & initial_trajectories,
  ConflictResolver::Mode mode) const
{
  last_expanded_nodes_ = 0U;

  // 1. Build the synchronized root node and initialize the open set with it.
  std::vector<SearchNode> open_set;
  SearchNode root = makeRootNode(initial_trajectories);
  open_set.push_back(root);

  if (shouldLog(LogLevel::Summary)) {
    std::cout << "[sng.root] robots=" << root.robotCount()
              << " tS=" << root.tS()
              << " g=" << root.g()
              << " h=" << root.h()
              << " f=" << root.f() << '\n';
    for (stop_n_go::base::RobotId robot_id = 0; robot_id < root.robotCount(); ++robot_id) {
      std::cout << "[sng.root] robot=" << robot_id
                << " size=" << root.trajectory(robot_id).size()
                << " total_duration=" << root.trajectory(robot_id).totalDuration() << '\n';
    }
  }

  // 2. Repeatedly expand the current best node until a conflict-free solution is found.
  std::size_t iteration = 0U;
  while (!open_set.empty()) {
    ++iteration;
    if (iteration > options_.max_expanded_nodes) {
      throw StopNGoSearchFailure(
              "StopNGoSearch failed: max expanded nodes reached (max=" +
              std::to_string(options_.max_expanded_nodes) +
              ", open=" + std::to_string(open_set.size()) + ")");
    }

    // 2.1 Select and remove the best node currently available in the open set.
    const std::size_t best_index = selectBestNodeIndex(open_set);
    SearchNode node = open_set[best_index];
    open_set.erase(open_set.begin() + best_index);

    if (shouldLog(LogLevel::Debug)) {
      std::cout << "[sng.solve] iter=" << iteration
                << " open=" << open_set.size()
                << " pick=" << best_index
                << " tS=" << node.tS()
                << " g=" << node.g()
                << " h=" << node.h()
                << " f=" << node.f() << '\n';
    }

    // 2.2 Scan synchronized time-steps from the node's current frontier `t_s` to find the first
    // conflict. The scan is limited to the time range that is valid for every trajectory.
    std::size_t max_shared_time_index = node.trajectory(0).size() - 1U;
    for (stop_n_go::base::RobotId robot_id = 1; robot_id < node.robotCount(); ++robot_id) {
      max_shared_time_index =
        std::min(max_shared_time_index, node.trajectory(robot_id).size() - 1U);
    }
    if (shouldLog(LogLevel::Debug)) {
      std::cout << "[sng.solve] scan_range=" << node.tS() << "->" << max_shared_time_index << '\n';
    }

    std::optional<ConflictResult> conflict;
    for (std::size_t time_index = node.tS(); time_index <= max_shared_time_index; ++time_index) {
      conflict = conflict_checker_->conflictCheck(node, time_index);
      if (conflict.has_value()) {
        node.setTS(conflict->timeIndex());
        if (shouldLog(LogLevel::Debug)) {
          std::cout << "[sng.check] conflict t=" << conflict->timeIndex()
                    << " pair=(" << conflict->firstRobot() << ", " << conflict->secondRobot() << ")"
                    << '\n';
        }
        break;
      }
    }

    // 2.3 If no conflict is found, this node already represents a valid solution.
    if (!conflict.has_value()) {
      last_expanded_nodes_ = iteration;
      if (shouldLog(LogLevel::Summary)) {
        std::cout << "[sng.solve] success iter=" << iteration
                  << " tS=" << node.tS()
                  << " g=" << node.g()
                  << " h=" << node.h()
                  << " f=" << node.f() << '\n';
      }
      return node;
    }

    // 2.4 Otherwise expand the node by resolving the detected conflict in both possible ways and
    // push the resulting successor nodes back into the open set.
    std::vector<SearchNode> successors = expand(node, *conflict, mode);
    if (shouldLog(LogLevel::Debug)) {
      std::cout << "[sng.expand] successors=" << successors.size() << '\n';
    }
    for (SearchNode & successor : successors) {
      updateCost(successor);
      if (shouldLog(LogLevel::Debug)) {
        std::cout << "[sng.expand] push tS=" << successor.tS()
                  << " g=" << successor.g()
                  << " h=" << successor.h()
                  << " f=" << successor.f() << '\n';
      }
      open_set.push_back(std::move(successor));
    }
  }

  // 3. If the open set is exhausted, the current search implementation failed to find a solution.
  throw StopNGoSearchFailure(
          "StopNGoSearch failed: open set became empty before a solution was found");
}

std::size_t StopNGoSearch::lastExpandedNodes() const
{
  return last_expanded_nodes_;
}

SearchNode StopNGoSearch::makeRootNode(
  const std::vector<stop_n_go::base::Trajectory> & initial_trajectories) const
{
  SearchNode node(synchronizeTrajectories(initial_trajectories));
  equalizeTrajectoryLengths(node);
  node.setTS(0U);
  // Follow the paper's root initialization literally: the root node starts with zero cost.
  node.setG(0.0);
  node.setH(0.0);
  return node;
}

std::vector<stop_n_go::base::Trajectory> StopNGoSearch::synchronizeTrajectories(
  const std::vector<stop_n_go::base::Trajectory> & trajectories) const
{
  std::vector<stop_n_go::base::Trajectory> synchronized_trajectories;
  synchronized_trajectories.reserve(trajectories.size());
  for (const stop_n_go::base::Trajectory & trajectory : trajectories) {
    synchronized_trajectories.push_back(trajectory.resample(options_.synchronization_time_step));
  }
  return synchronized_trajectories;
}

void StopNGoSearch::equalizeTrajectoryLengths(SearchNode & node) const
{
  if (node.empty()) {
    return;
  }

  std::size_t max_size = 0U;
  for (stop_n_go::base::RobotId robot_id = 0; robot_id < node.robotCount(); ++robot_id) {
    max_size = std::max(max_size, node.trajectory(robot_id).size());
  }

  for (stop_n_go::base::RobotId robot_id = 0; robot_id < node.robotCount(); ++robot_id) {
    stop_n_go::base::Trajectory & trajectory = node.trajectory(robot_id);
    while (trajectory.size() < max_size) {
      trajectory.appendHold(options_.synchronization_time_step);
    }
  }
}

bool StopNGoSearch::isBetterNode(
  const SearchNode & lhs,
  const SearchNode & rhs,
  std::size_t lhs_index,
  std::size_t rhs_index) const
{
  // Primary A* ordering is by the evaluation value f = g + h.
  if (lhs.f() != rhs.f()) {
    return lhs.f() < rhs.f();
  }

  // Then prefer nodes with smaller heuristic value.
  if (lhs.h() != rhs.h()) {
    return lhs.h() < rhs.h();
  }

  // Then prefer nodes with smaller committed path cost.
  if (lhs.g() != rhs.g()) {
    return lhs.g() < rhs.g();
  }

  // Then prefer nodes whose conflict scan resumes earlier.
  if (lhs.tS() != rhs.tS()) {
    return lhs.tS() < rhs.tS();
  }

  // Finally keep the selection deterministic by preferring earlier insertion order.
  return lhs_index < rhs_index;
}

std::size_t StopNGoSearch::selectBestNodeIndex(const std::vector<SearchNode> & open_set) const
{
  if (open_set.empty()) {
    throw std::invalid_argument("StopNGoSearch cannot select a node from an empty open set");
  }

  std::size_t best_index = 0U;
  for (std::size_t index = 1; index < open_set.size(); ++index) {
    if (isBetterNode(open_set[index], open_set[best_index], index, best_index)) {
      best_index = index;
    }
  }

  return best_index;
}

std::vector<SearchNode> StopNGoSearch::expand(
  const SearchNode & node,
  const ConflictResult & conflict,
  ConflictResolver::Mode mode) const
{
  std::vector<SearchNode> successors;
  successors.reserve(2U);
  successors.push_back(conflict_resolver_->resolve(node, conflict, conflict.firstRobot(), mode));
  equalizeTrajectoryLengths(successors.back());
  successors.push_back(conflict_resolver_->resolve(node, conflict, conflict.secondRobot(), mode));
  equalizeTrajectoryLengths(successors.back());
  return successors;
}

void StopNGoSearch::updateCost(SearchNode & node) const
{
  node.setG(calculateG(node));
  node.setH(calculateH(node));
}

double StopNGoSearch::calculateG(const SearchNode & node) const
{
  return static_cast<double>(node.tS()) * options_.synchronization_time_step;
}

double StopNGoSearch::calculateH(const SearchNode & node) const
{
  double max_duration = 0.0;
  for (stop_n_go::base::RobotId robot_id = 0; robot_id < node.robotCount(); ++robot_id) {
    max_duration = std::max(max_duration, node.trajectory(robot_id).totalDuration());
  }

  const double remaining = max_duration - calculateG(node);
  return remaining > 0.0 ? remaining : 0.0;
}

std::string StopNGoSearch::nodeKey(const SearchNode & node) const
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  stream << node.tS() << '|';
  for (stop_n_go::base::RobotId robot_id = 0; robot_id < node.robotCount(); ++robot_id) {
    const stop_n_go::base::Trajectory & trajectory = node.trajectory(robot_id);
    stream << trajectory.size() << ':';
    for (std::size_t index = 0; index < trajectory.size(); ++index) {
      stream << trajectory.durationFromPrevious(index) << ':';
      for (double value : trajectory.waypoint(index).values()) {
        stream << value << ',';
      }
      stream << ';';
    }
    stream << '#';
  }
  return stream.str();
}

bool StopNGoSearch::shouldLog(LogLevel level) const
{
  return static_cast<int>(options_.log_level) >= static_cast<int>(level);
}

void StopNGoSearch::validate() const
{
  if (conflict_checker_ == nullptr) {
    throw std::invalid_argument("StopNGoSearch requires a valid ConflictChecker");
  }
  if (conflict_resolver_ == nullptr) {
    throw std::invalid_argument("StopNGoSearch requires a valid ConflictResolver");
  }
  if (options_.synchronization_time_step <= 0.0) {
    throw std::invalid_argument("StopNGoSearch synchronization_time_step must be positive");
  }
  if (options_.max_expanded_nodes == 0U) {
    throw std::invalid_argument("StopNGoSearch max_expanded_nodes must be positive");
  }
}

}  // namespace stop_n_go::core
