#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <stop_n_go/core/conflict_resolver.hpp>
#include <stop_n_go/core/log_level.hpp>

namespace stop_n_go::core
{

/// Search-level failure that represents an exhausted Stop-N-Go search budget or frontier.
///
/// Dataset runners can catch this separately from data/model errors and continue with the next
/// trial.
class StopNGoSearchFailure : public std::runtime_error
{
public:
  explicit StopNGoSearchFailure(const std::string & message);
};

/// Runtime options for Stop-N-Go A* search.
struct StopNGoSearchOptions
{
  double synchronization_time_step = 0.2;
  std::size_t max_expanded_nodes = 10000U;
  LogLevel log_level = LogLevel::Summary;
};

/// A* search driver for Stop-N-Go temporal conflict resolution.
class StopNGoSearch
{
public:
  /// Construct the search object from the shared checker, resolver, and synchronization step.
  StopNGoSearch(
    std::shared_ptr<const ConflictChecker> conflict_checker,
    std::shared_ptr<const ConflictResolver> conflict_resolver,
    double synchronization_time_step);

  /// Construct the search object from the shared checker, resolver, and search options.
  StopNGoSearch(
    std::shared_ptr<const ConflictChecker> conflict_checker,
    std::shared_ptr<const ConflictResolver> conflict_resolver,
    StopNGoSearchOptions options);

  /// Solve the Stop-N-Go search problem starting from individually planned trajectories.
  SearchNode solve(
    const std::vector<stop_n_go::base::Trajectory> & initial_trajectories,
    ConflictResolver::Mode mode) const;

  /// Return the number of nodes expanded by the most recent successful solve call.
  std::size_t lastExpandedNodes() const;

private:
  SearchNode makeRootNode(
    const std::vector<stop_n_go::base::Trajectory> & initial_trajectories) const;
  std::vector<stop_n_go::base::Trajectory> synchronizeTrajectories(
    const std::vector<stop_n_go::base::Trajectory> & trajectories) const;

  /// Pad shorter synchronized trajectories by holding their final state.
  ///
  /// Design choice: the current conflict-check abstraction queries one shared synchronized
  /// time-step across all robots. To keep that abstraction simple, every node stores trajectory
  /// sets that are explicitly equalized to the same length, rather than treating "time after the end
  /// of a trajectory" as an implicit final-state hold.
  void equalizeTrajectoryLengths(SearchNode & node) const;
  /// Return true when `lhs` should be expanded before `rhs`.
  ///
  /// Design choice: nodes are ordered primarily by A* value `f = g + h`. Ties are then broken by
  /// smaller `h`, smaller `g`, smaller `t_s`, and finally by earlier insertion order in the current
  /// open set. The open-set indices are therefore part of the comparison.
  bool isBetterNode(
    const SearchNode & lhs,
    const SearchNode & rhs,
    std::size_t lhs_index,
    std::size_t rhs_index) const;

  /// Return the index of the best node currently stored in the vector-based open set.
  ///
  /// Design choice: the search currently uses a simple `std::vector` open set instead of a heap,
  /// so selecting the best node is implemented as a linear scan with the tie-breaking policy above.
  std::size_t selectBestNodeIndex(const std::vector<SearchNode> & open_set) const;
  std::vector<SearchNode> expand(
    const SearchNode & node,
    const ConflictResult & conflict,
    ConflictResolver::Mode mode) const;
  void updateCost(SearchNode & node) const;
  double calculateG(const SearchNode & node) const;
  double calculateH(const SearchNode & node) const;
  std::string nodeKey(const SearchNode & node) const;
  bool shouldLog(LogLevel level) const;
  void validate() const;

  std::shared_ptr<const ConflictChecker> conflict_checker_;
  std::shared_ptr<const ConflictResolver> conflict_resolver_;
  StopNGoSearchOptions options_;
  mutable std::size_t last_expanded_nodes_ = 0U;
};

}  // namespace stop_n_go::core
