#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <stop_n_go/base/r_state.hpp>
#include <stop_n_go/base/robot_registry.hpp>
#include <stop_n_go/core/conflict_result.hpp>
#include <stop_n_go/core/search_node.hpp>

namespace stop_n_go::core
{

/// Collision-aware checker used by Stop-N-Go search.
///
/// Design choice:
/// - `conflictCheck` checks exactly one synchronized time-step at a time.
/// - The outer search / conflict-resolution loops are responsible for scanning over time.
/// - Pairwise collision judgment is implemented here, not in `SearchNode` or `Trajectory`, because
///   it needs MoveIt collision infrastructure.
/// - To evaluate one robot pair, the checker reconstructs a single full MoveIt RobotState by
///   overlaying both group-scoped stop_n_go states on top of a shared full-state reference.
class ConflictChecker
{
public:
  using LayoutMap =
    std::unordered_map<stop_n_go::base::RobotId, stop_n_go::base::StateLayout::ConstPtr>;

  /// Construct the checker from robot metadata, per-robot layouts, and a MoveIt planning scene.
  ConflictChecker(
    const stop_n_go::base::RobotRegistry & robot_registry,
    moveit::core::RobotModelConstPtr robot_model,
    LayoutMap layouts,
    std::shared_ptr<const planning_scene::PlanningScene> planning_scene,
    moveit::core::RobotState reference_state);

  /// Return the stored full-state reference used to fill non-queried robots.
  const moveit::core::RobotState & referenceState() const;

  /// Replace the stored full-state reference.
  void setReferenceState(const moveit::core::RobotState & reference_state);

  /// Return true when two robot states are in collision at the same synchronized time-step.
  bool inConflict(
    stop_n_go::base::RobotId first_robot,
    const stop_n_go::base::RobotState & first_state,
    stop_n_go::base::RobotId second_robot,
    const stop_n_go::base::RobotState & second_state) const;

  /// Check whether two robot trajectories conflict at one synchronized time-step.
  std::optional<ConflictResult> conflictCheck(
    stop_n_go::base::RobotId first_robot,
    const stop_n_go::base::Trajectory & first_trajectory,
    stop_n_go::base::RobotId second_robot,
    const stop_n_go::base::Trajectory & second_trajectory,
    std::size_t time_index) const;

  /// Check whether any robot pair in a search node conflicts at one synchronized time-step.
  std::optional<ConflictResult> conflictCheck(
    const SearchNode & node,
    std::size_t time_index) const;

private:
  const stop_n_go::base::StateLayout & layout(stop_n_go::base::RobotId robot_id) const;
  const std::unordered_set<std::string> & groupLinks(stop_n_go::base::RobotId robot_id) const;

  void validate() const;
  void validateReferenceState(const moveit::core::RobotState & reference_state) const;
  void validateTrajectoryTimeIndex(
    const stop_n_go::base::Trajectory & trajectory,
    std::size_t time_index) const;

  void overlayGroupState(
    stop_n_go::base::RobotId robot_id,
    const stop_n_go::base::RobotState & state,
    moveit::core::RobotState & moveit_state) const;

  moveit::core::RobotState makeCombinedMoveItState(
    stop_n_go::base::RobotId first_robot,
    const stop_n_go::base::RobotState & first_state,
    stop_n_go::base::RobotId second_robot,
    const stop_n_go::base::RobotState & second_state) const;

  bool moveitStateHasPairConflict(
    stop_n_go::base::RobotId first_robot,
    stop_n_go::base::RobotId second_robot,
    const moveit::core::RobotState & moveit_state) const;

  const stop_n_go::base::RobotRegistry & robot_registry_;
  moveit::core::RobotModelConstPtr robot_model_;
  LayoutMap layouts_;
  std::unordered_map<stop_n_go::base::RobotId, std::unordered_set<std::string>> group_links_;
  std::shared_ptr<const planning_scene::PlanningScene> planning_scene_;
  moveit::core::RobotState reference_state_;
};

}  // namespace stop_n_go::core
