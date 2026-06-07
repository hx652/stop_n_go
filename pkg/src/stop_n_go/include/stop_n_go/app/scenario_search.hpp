#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <stop_n_go/base/initial_planner.hpp>
#include <stop_n_go/core/conflict_check.hpp>

namespace stop_n_go::app
{

/// Shared world-frame box used to sample candidate goals for all robots.
///
/// Default range:
/// - x in [-0.35, 0.35]
/// - y in [-0.35, 0.35]
/// - z in [0.45, 0.85]
struct SharedGoalRegion
{
  std::string frame_id = "world";
  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.65;
  double half_extent_x = 0.35;
  double half_extent_y = 0.35;
  double half_extent_z = 0.20;
};

/// One sub-region assigned to one robot for world-frame goal sampling.
struct RegionPartition
{
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
};

/// Robot-to-region mapping used to bias goal sampling toward reachable nearby quadrants.
///
/// Default partitioning rule in the implementation:
/// - split the shared region by the planes x = 0 and y = 0
/// - arm1: x <= 0, y <= 0
/// - arm2: x >= 0, y <= 0
/// - arm3: x <= 0, y >= 0
/// - arm4: x >= 0, y >= 0
struct RobotRegionAssignment
{
  std::unordered_map<stop_n_go::base::RobotId, RegionPartition> partitions;
};

/// Request parameters for sampling a multi-arm scenario.
struct MultiArmScenarioSearchRequest
{
  SharedGoalRegion region;
  geometry_msgs::msg::Quaternion fixed_orientation;
  double synchronization_time_step = 0.2;
  std::size_t max_attempts = 100;
  std::uint32_t random_seed = 7U;
};

/// Per-robot planning result for one sampled goal set.
struct RobotPlanningAttempt
{
  stop_n_go::base::RobotId robot_id;
  geometry_msgs::msg::PoseStamped goal;
  bool ik_succeeded = false;
  bool planning_succeeded = false;
  std::optional<stop_n_go::base::Trajectory> initial_trajectory;
  std::string message;
};

/// Complete record of one sampled multi-arm attempt.
struct MultiArmScenarioTrialRecord
{
  std::size_t attempt_index = 0U;
  std::vector<RobotPlanningAttempt> robot_attempts;
  bool all_planning_succeeded = false;
  std::optional<stop_n_go::core::ConflictResult> first_conflict;
};

/// Runtime options for search-result recording behavior.
///
/// If json_output_path is set, all trial records are streamed to disk as they are produced.
/// If conflict_only_json_output_path is set, only all-success conflict trials are streamed into
/// that additional JSON file while still including full experiment metadata.
/// max_trial_records_in_memory controls how many latest trial records remain in memory.
struct MultiArmScenarioRecordOptions
{
  std::string json_output_path;
  std::string conflict_only_json_output_path;
  std::size_t max_trial_records_in_memory = std::numeric_limits<std::size_t>::max();
};

/// Full search result including every recorded attempt still retained in memory.
struct MultiArmScenarioSearchResult
{
  std::vector<MultiArmScenarioTrialRecord> trial_records;
  std::vector<std::size_t> conflict_trial_indices;
};

/// App-layer builder for sampling and evaluating multi-arm initial-planning scenarios.
class MultiArmScenarioBuilder
{
public:
  using MoveGroupMap = stop_n_go::base::InitialPlanner::MoveGroupMap;

  MultiArmScenarioBuilder(
    const stop_n_go::base::RobotRegistry & registry,
    stop_n_go::base::InitialPlanner & initial_planner,
    const stop_n_go::core::ConflictChecker & conflict_checker,
    std::vector<stop_n_go::base::StateLayout::ConstPtr> layouts,
    moveit::core::RobotState reference_state);

  /// Search for max_attempts sampled scenarios and record every trial outcome.
  ///
  /// Every sampled attempt is evaluated and recorded, regardless of whether planning fails,
  /// succeeds without conflict, or succeeds with a detected conflict.
  ///
  /// This overload keeps all trial records in memory and performs no streaming output.
  MultiArmScenarioSearchResult findScenario(
    const MultiArmScenarioSearchRequest & request,
    const MoveGroupMap & move_groups) const;

  /// Search for max_attempts sampled scenarios with configurable recording behavior.
  ///
  /// When json_output_path is set, the method writes a complete JSON file incrementally to avoid
  /// unbounded memory growth from large trial logs.
  MultiArmScenarioSearchResult findScenario(
    const MultiArmScenarioSearchRequest & request,
    const MoveGroupMap & move_groups,
    const MultiArmScenarioRecordOptions & record_options) const;

  /// Synchronize trajectories to a shared time step and explicitly equalize their lengths.
  std::vector<stop_n_go::base::Trajectory> synchronizeAndEqualizeTrajectories(
    const std::vector<stop_n_go::base::Trajectory> & trajectories,
    double time_step) const;

  /// Build one joint display trajectory for RViz from synchronized per-robot trajectories.
  moveit_msgs::msg::DisplayTrajectory buildDisplayTrajectory(
    const std::vector<stop_n_go::base::Trajectory> & trajectories) const;

  /// Build RViz markers for the request region and the first sampled goal attempts.
  visualization_msgs::msg::MarkerArray buildSearchMarkers(
    const MultiArmScenarioSearchRequest & request,
    const MultiArmScenarioSearchResult & result,
    std::size_t max_trials_to_visualize = 10U) const;

  /// Serialize experiment metadata and all trial records into a JSON document.
  std::string buildSearchResultJson(
    const MultiArmScenarioSearchRequest & request,
    const MultiArmScenarioSearchResult & result) const;

  /// Persist the JSON document produced by buildSearchResultJson() to disk.
  void writeSearchResultJson(
    const std::string & file_path,
    const MultiArmScenarioSearchRequest & request,
    const MultiArmScenarioSearchResult & result) const;

private:
  bool ikFeasible(
    const moveit::planning_interface::MoveGroupInterface & move_group,
    stop_n_go::base::RobotId robot_id,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /// Build the default robot-to-region mapping by splitting the shared world-frame box along the
  /// planes x = 0 and y = 0.
  ///
  /// Default partition ranges:
  /// - arm1: x in [-0.35, 0.00], y in [-0.35, 0.00], z in [0.45, 0.85]
  /// - arm2: x in [0.00, 0.35], y in [-0.35, 0.00], z in [0.45, 0.85]
  /// - arm3: x in [-0.35, 0.00], y in [0.00, 0.35], z in [0.45, 0.85]
  /// - arm4: x in [0.00, 0.35], y in [0.00, 0.35], z in [0.45, 0.85]
  RobotRegionAssignment defaultAssignment(const SharedGoalRegion & region) const;

  /// Sample one world-frame PoseStamped goal per robot from that robot's assigned partition.
  std::vector<geometry_msgs::msg::PoseStamped> sampleGoals(
    const MultiArmScenarioSearchRequest & request,
    std::mt19937 & generator) const;

  std::optional<stop_n_go::core::ConflictResult> findFirstConflict(
    const std::vector<stop_n_go::base::Trajectory> & synchronized_trajectories) const;

  void validate() const;

  const stop_n_go::base::RobotRegistry & registry_;
  stop_n_go::base::InitialPlanner & initial_planner_;
  const stop_n_go::core::ConflictChecker & conflict_checker_;
  std::vector<stop_n_go::base::StateLayout::ConstPtr> layouts_;
  moveit::core::RobotState reference_state_;
};

}  // namespace stop_n_go::app
