#include <stop_n_go/core/conflict_check.hpp>

#include <moveit/collision_detection/collision_common.h>

#include <stdexcept>
#include <utility>

namespace stop_n_go::core
{

namespace
{

bool bodyBelongsToGroup(
  const std::string & body,
  const std::unordered_set<std::string> & link_names)
{
  return link_names.find(body) != link_names.end();
}

}  // namespace

ConflictChecker::ConflictChecker(
  const stop_n_go::base::RobotRegistry & robot_registry,
  moveit::core::RobotModelConstPtr robot_model,
  LayoutMap layouts,
  std::shared_ptr<const planning_scene::PlanningScene> planning_scene,
  moveit::core::RobotState reference_state)
: robot_registry_(robot_registry),
  robot_model_(std::move(robot_model)),
  layouts_(std::move(layouts)),
  planning_scene_(std::move(planning_scene)),
  reference_state_(std::move(reference_state))
{
  validateReferenceState(reference_state_);
  validate();

  for (const auto & [robot_id, layout] : layouts_) {
    std::unordered_set<std::string> links;
    for (const std::string & link_name : layout->jointModelGroup()->getLinkModelNames()) {
      links.insert(link_name);
    }
    group_links_.emplace(robot_id, std::move(links));
  }
}

const moveit::core::RobotState & ConflictChecker::referenceState() const
{
  return reference_state_;
}

void ConflictChecker::setReferenceState(const moveit::core::RobotState & reference_state)
{
  validateReferenceState(reference_state);
  reference_state_ = reference_state;
}

bool ConflictChecker::inConflict(
  stop_n_go::base::RobotId first_robot,
  const stop_n_go::base::RobotState & first_state,
  stop_n_go::base::RobotId second_robot,
  const stop_n_go::base::RobotState & second_state) const
{
  if (first_robot == second_robot) {
    throw std::invalid_argument("ConflictChecker requires two distinct robot ids");
  }

  const moveit::core::RobotState moveit_state = makeCombinedMoveItState(
    first_robot, first_state,
    second_robot, second_state);
  return moveitStateHasPairConflict(first_robot, second_robot, moveit_state);
}

std::optional<ConflictResult> ConflictChecker::conflictCheck(
  stop_n_go::base::RobotId first_robot,
  const stop_n_go::base::Trajectory & first_trajectory,
  stop_n_go::base::RobotId second_robot,
  const stop_n_go::base::Trajectory & second_trajectory,
  std::size_t time_index) const
{
  validateTrajectoryTimeIndex(first_trajectory, time_index);
  validateTrajectoryTimeIndex(second_trajectory, time_index);

  if (!inConflict(
      first_robot, first_trajectory.waypoint(time_index),
      second_robot, second_trajectory.waypoint(time_index)))
  {
    return std::nullopt;
  }

  return ConflictResult(first_robot, second_robot, time_index);
}

std::optional<ConflictResult> ConflictChecker::conflictCheck(
  const SearchNode & node,
  std::size_t time_index) const
{
  if (node.empty()) {
    return std::nullopt;
  }

  for (stop_n_go::base::RobotId first_robot = 0; first_robot < node.robotCount(); ++first_robot) {
    for (stop_n_go::base::RobotId second_robot = first_robot + 1; second_robot < node.robotCount();
      ++second_robot)
    {
      const std::optional<ConflictResult> conflict = conflictCheck(
        first_robot,
        node.trajectory(first_robot),
        second_robot,
        node.trajectory(second_robot),
        time_index);
      if (conflict.has_value()) {
        return conflict;
      }
    }
  }

  return std::nullopt;
}

const stop_n_go::base::StateLayout & ConflictChecker::layout(stop_n_go::base::RobotId robot_id)
const
{
  const auto iterator = layouts_.find(robot_id);
  if (iterator == layouts_.end() || iterator->second == nullptr) {
    throw std::out_of_range("ConflictChecker does not contain a layout for the requested robot id");
  }

  return *iterator->second;
}

const std::unordered_set<std::string> & ConflictChecker::groupLinks(
  stop_n_go::base::RobotId robot_id) const
{
  const auto iterator = group_links_.find(robot_id);
  if (iterator == group_links_.end()) {
    throw std::out_of_range(
            "ConflictChecker does not contain cached link names for the requested robot id");
  }

  return iterator->second;
}

void ConflictChecker::validate() const
{
  if (robot_model_ == nullptr) {
    throw std::invalid_argument("ConflictChecker robot model must not be null");
  }
  if (planning_scene_ == nullptr) {
    throw std::invalid_argument("ConflictChecker planning scene must not be null");
  }
  if (planning_scene_->getRobotModel().get() != robot_model_.get()) {
    throw std::invalid_argument("ConflictChecker planning scene model does not match robot model");
  }
  if (layouts_.size() != robot_registry_.size()) {
    throw std::invalid_argument(
            "ConflictChecker must receive exactly one layout per registered robot");
  }

  for (const auto & [robot_id, layout_ptr] : layouts_) {
    if (!robot_registry_.hasRobot(robot_id)) {
      throw std::invalid_argument("ConflictChecker received a layout for an unknown robot id");
    }
    if (layout_ptr == nullptr) {
      throw std::invalid_argument("ConflictChecker layout pointers must not be null");
    }

    const stop_n_go::base::RobotDescriptor & robot = robot_registry_.descriptor(robot_id);
    if (layout_ptr->robotModel() != robot_model_.get()) {
      throw std::invalid_argument(
              "ConflictChecker layout robot model does not match checker robot model");
    }
    if (layout_ptr->groupName() != robot.group_name) {
      throw std::invalid_argument(
              "ConflictChecker layout group does not match RobotRegistry metadata");
    }
  }
}

void ConflictChecker::validateReferenceState(const moveit::core::RobotState & reference_state) const
{
  if (robot_model_ != nullptr && reference_state.getRobotModel().get() != robot_model_.get()) {
    throw std::invalid_argument(
            "ConflictChecker reference state model does not match checker robot model");
  }
}

void ConflictChecker::validateTrajectoryTimeIndex(
  const stop_n_go::base::Trajectory & trajectory,
  std::size_t time_index) const
{
  if (time_index >= trajectory.size()) {
    throw std::out_of_range("ConflictChecker time_index is outside the trajectory range");
  }
}

void ConflictChecker::overlayGroupState(
  stop_n_go::base::RobotId robot_id,
  const stop_n_go::base::RobotState & state,
  moveit::core::RobotState & moveit_state) const
{
  const stop_n_go::base::StateLayout & robot_layout = layout(robot_id);
  if (!robot_layout.isCompatible(state)) {
    throw std::invalid_argument(
            "ConflictChecker received a RobotState incompatible with the robot layout");
  }

  moveit_state.setJointGroupActivePositions(robot_layout.jointModelGroup(), state.values());
}

moveit::core::RobotState ConflictChecker::makeCombinedMoveItState(
  stop_n_go::base::RobotId first_robot,
  const stop_n_go::base::RobotState & first_state,
  stop_n_go::base::RobotId second_robot,
  const stop_n_go::base::RobotState & second_state) const
{
  moveit::core::RobotState moveit_state(reference_state_);
  overlayGroupState(first_robot, first_state, moveit_state);
  overlayGroupState(second_robot, second_state, moveit_state);
  moveit_state.update();
  return moveit_state;
}

bool ConflictChecker::moveitStateHasPairConflict(
  stop_n_go::base::RobotId first_robot,
  stop_n_go::base::RobotId second_robot,
  const moveit::core::RobotState & moveit_state) const
{
  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 1000;
  request.max_contacts_per_pair = 1;

  collision_detection::CollisionResult result;
  planning_scene_->checkSelfCollision(request, result, moveit_state);
  if (!result.collision) {
    return false;
  }

  const std::unordered_set<std::string> & first_links = groupLinks(first_robot);
  const std::unordered_set<std::string> & second_links = groupLinks(second_robot);
  for (const auto & [body_pair, contacts] : result.contacts) {
    (void)contacts;
    const bool first_body_in_first_group = bodyBelongsToGroup(body_pair.first, first_links);
    const bool second_body_in_first_group = bodyBelongsToGroup(body_pair.second, first_links);
    const bool first_body_in_second_group = bodyBelongsToGroup(body_pair.first, second_links);
    const bool second_body_in_second_group = bodyBelongsToGroup(body_pair.second, second_links);

    if ((first_body_in_first_group && second_body_in_second_group) ||
      (first_body_in_second_group && second_body_in_first_group))
    {
      return true;
    }
  }

  return false;
}

}  // namespace stop_n_go::core
