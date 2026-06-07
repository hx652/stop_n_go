#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <moveit/robot_model/robot_model.h>

#include <stop_n_go/app/scenario_search.hpp>

namespace stop_n_go::app
{

/// Reader for conflict-oriented scenario JSON datasets.
///
/// This class loads a scenario-search JSON file and reconstructs per-robot initial trajectories for
/// one recorded trial. It mirrors InitialPlanner's role at a different input boundary:
/// - InitialPlanner: goals -> initial trajectories
/// - ConflictScenarioDataset: dataset trial -> initial trajectories
class ConflictScenarioDataset
{
public:
  /// Load the dataset from a JSON file and build one StateLayout per registered robot.
  ConflictScenarioDataset(
    const stop_n_go::base::RobotRegistry & registry,
    moveit::core::RobotModelConstPtr robot_model,
    const std::string & json_path);

  /// Return the experiment-level request metadata parsed from the dataset.
  const MultiArmScenarioSearchRequest & request() const;

  /// Return trial indices marked as all-success conflict trials.
  const std::vector<std::size_t> & conflictTrialIndices() const;

  /// Return all parsed trial indices in ascending order.
  std::vector<std::size_t> availableTrialIndices() const;

  /// Build one initial trajectory per robot for the specified trial index.
  ///
  /// The selected trial must be an all-success conflict trial and must contain
  /// `initial_trajectory` data for every registered robot.
  std::vector<stop_n_go::base::Trajectory> buildInitialTrajectories(std::size_t trial_index) const;

private:
  struct RobotTrajectoryRecord
  {
    stop_n_go::base::RobotId robot_id = 0U;
    bool ik_succeeded = false;
    bool planning_succeeded = false;
    std::vector<std::vector<double>> positions;
    std::vector<double> duration_from_previous;
  };

  struct TrialRecord
  {
    std::size_t trial_index = 0U;
    bool all_planning_succeeded = false;
    bool has_conflict = false;
    std::unordered_map<stop_n_go::base::RobotId, RobotTrajectoryRecord> robot_records;
  };

  void loadFromJson(const std::string & json_path);
  void validate() const;
  const TrialRecord & trial(std::size_t trial_index) const;

  const stop_n_go::base::RobotRegistry & registry_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::vector<stop_n_go::base::StateLayout::ConstPtr> layouts_;
  MultiArmScenarioSearchRequest request_;
  std::vector<std::size_t> conflict_trial_indices_;
  std::unordered_map<std::size_t, TrialRecord> trials_;
};

}  // namespace stop_n_go::app
