#include <stop_n_go/app/conflict_scenario_dataset.hpp>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace stop_n_go::app
{

namespace
{

const stop_n_go::base::StateLayout & layoutForRobot(
  const std::vector<stop_n_go::base::StateLayout::ConstPtr> & layouts,
  stop_n_go::base::RobotId robot_id)
{
  if (robot_id >= layouts.size() || layouts[robot_id] == nullptr) {
    throw std::runtime_error("Missing state layout for robot id " + std::to_string(robot_id));
  }
  return *layouts[robot_id];
}

const boost::property_tree::ptree & childRequired(
  const boost::property_tree::ptree & tree,
  const std::string & key,
  const std::string & context)
{
  const auto child_optional = tree.get_child_optional(key);
  if (!child_optional.has_value()) {
    throw std::runtime_error("Missing JSON object key '" + key + "' in " + context);
  }
  return *child_optional;
}

template<typename ValueType>
ValueType valueRequired(
  const boost::property_tree::ptree & tree,
  const std::string & key,
  const std::string & context)
{
  const auto value_optional = tree.get_optional<ValueType>(key);
  if (!value_optional.has_value()) {
    throw std::runtime_error("Missing JSON value key '" + key + "' in " + context);
  }
  return *value_optional;
}

std::vector<double> parseDoubleArray(
  const boost::property_tree::ptree & array_tree,
  const std::string & context)
{
  std::vector<double> values;
  values.reserve(array_tree.size());
  for (const auto & element : array_tree) {
    const auto value_optional = element.second.get_value_optional<double>();
    if (!value_optional.has_value()) {
      throw std::runtime_error("Expected numeric array entry in " + context);
    }
    values.push_back(*value_optional);
  }
  return values;
}

std::vector<std::vector<double>> parseDoubleMatrix(
  const boost::property_tree::ptree & matrix_tree,
  const std::string & context)
{
  std::vector<std::vector<double>> matrix;
  matrix.reserve(matrix_tree.size());
  for (const auto & row : matrix_tree) {
    matrix.push_back(parseDoubleArray(row.second, context + ".row"));
  }
  return matrix;
}

}  // namespace

ConflictScenarioDataset::ConflictScenarioDataset(
  const stop_n_go::base::RobotRegistry & registry,
  moveit::core::RobotModelConstPtr robot_model,
  const std::string & json_path)
: registry_(registry),
  robot_model_(std::move(robot_model))
{
  if (robot_model_ == nullptr) {
    throw std::invalid_argument("ConflictScenarioDataset requires a non-null robot model");
  }

  layouts_.reserve(registry_.size());
  for (stop_n_go::base::RobotId robot_id = 0; robot_id < registry_.size(); ++robot_id) {
    layouts_.push_back(std::make_shared<stop_n_go::base::StateLayout>(
      *robot_model_,
      registry_.groupName(robot_id)));
  }

  loadFromJson(json_path);
  validate();
}

const MultiArmScenarioSearchRequest & ConflictScenarioDataset::request() const
{
  return request_;
}

const std::vector<std::size_t> & ConflictScenarioDataset::conflictTrialIndices() const
{
  return conflict_trial_indices_;
}

std::vector<std::size_t> ConflictScenarioDataset::availableTrialIndices() const
{
  std::vector<std::size_t> indices;
  indices.reserve(trials_.size());
  for (const auto & trial_entry : trials_) {
    indices.push_back(trial_entry.first);
  }
  std::sort(indices.begin(), indices.end());
  return indices;
}

std::vector<stop_n_go::base::Trajectory> ConflictScenarioDataset::buildInitialTrajectories(
  std::size_t trial_index) const
{
  const TrialRecord & trial_record = trial(trial_index);
  if (!trial_record.all_planning_succeeded || !trial_record.has_conflict) {
    throw std::invalid_argument(
            "Requested trial is not an all-success conflict trial: " + std::to_string(trial_index));
  }

  std::vector<stop_n_go::base::Trajectory> trajectories;
  trajectories.reserve(registry_.size());
  for (stop_n_go::base::RobotId robot_id = 0; robot_id < registry_.size(); ++robot_id) {
    const auto robot_entry = trial_record.robot_records.find(robot_id);
    if (robot_entry == trial_record.robot_records.end()) {
      throw std::runtime_error(
              "Missing robot trajectory record for robot id " + std::to_string(robot_id) +
              " in trial " + std::to_string(trial_index));
    }

    const RobotTrajectoryRecord & robot_record = robot_entry->second;
    if (robot_record.positions.empty()) {
      throw std::runtime_error(
              "Missing initial trajectory positions for robot id " + std::to_string(robot_id) +
              " in trial " + std::to_string(trial_index));
    }
    if (robot_record.positions.size() != robot_record.duration_from_previous.size()) {
      throw std::runtime_error(
              "positions/duration_from_previous size mismatch for robot id " +
              std::to_string(robot_id) + " in trial " + std::to_string(trial_index));
    }

    const stop_n_go::base::StateLayout & layout = layoutForRobot(layouts_, robot_id);
    stop_n_go::base::Trajectory trajectory(layouts_[robot_id]);
    for (std::size_t waypoint_index = 0; waypoint_index < robot_record.positions.size();
      ++waypoint_index)
    {
      const std::vector<double> & waypoint = robot_record.positions[waypoint_index];
      if (waypoint.size() != layout.size()) {
        throw std::runtime_error(
                "Waypoint dimension mismatch for robot id " + std::to_string(robot_id) +
                " in trial " + std::to_string(trial_index));
      }

      stop_n_go::base::RobotState state(layout.size());
      for (std::size_t joint_index = 0; joint_index < layout.size(); ++joint_index) {
        state[joint_index] = waypoint[joint_index];
      }
      trajectory.append(state, robot_record.duration_from_previous[waypoint_index]);
    }

    trajectories.push_back(std::move(trajectory));
  }

  return trajectories;
}

void ConflictScenarioDataset::loadFromJson(const std::string & json_path)
{
  if (json_path.empty()) {
    throw std::invalid_argument("ConflictScenarioDataset requires a non-empty JSON path");
  }

  boost::property_tree::ptree root;
  boost::property_tree::read_json(json_path, root);

  const boost::property_tree::ptree & experiment = childRequired(root, "experiment", "root");
  request_.random_seed = valueRequired<std::uint32_t>(experiment, "random_seed", "experiment");
  request_.synchronization_time_step =
    valueRequired<double>(experiment, "synchronization_time_step", "experiment");
  request_.max_attempts = valueRequired<std::size_t>(experiment, "max_attempts", "experiment");

  const boost::property_tree::ptree & region =
    childRequired(experiment, "shared_goal_region", "experiment");
  request_.region.frame_id = valueRequired<std::string>(region, "frame_id", "shared_goal_region");
  request_.region.center_x = valueRequired<double>(region, "center_x", "shared_goal_region");
  request_.region.center_y = valueRequired<double>(region, "center_y", "shared_goal_region");
  request_.region.center_z = valueRequired<double>(region, "center_z", "shared_goal_region");
  request_.region.half_extent_x =
    valueRequired<double>(region, "half_extent_x", "shared_goal_region");
  request_.region.half_extent_y =
    valueRequired<double>(region, "half_extent_y", "shared_goal_region");
  request_.region.half_extent_z =
    valueRequired<double>(region, "half_extent_z", "shared_goal_region");

  const boost::property_tree::ptree & orientation =
    childRequired(experiment, "fixed_orientation", "experiment");
  request_.fixed_orientation.x = valueRequired<double>(orientation, "x", "fixed_orientation");
  request_.fixed_orientation.y = valueRequired<double>(orientation, "y", "fixed_orientation");
  request_.fixed_orientation.z = valueRequired<double>(orientation, "z", "fixed_orientation");
  request_.fixed_orientation.w = valueRequired<double>(orientation, "w", "fixed_orientation");

  if (const auto conflict_indices = root.get_child_optional("conflict_trial_indices")) {
    for (const auto & index_entry : *conflict_indices) {
      const auto value_optional = index_entry.second.get_value_optional<std::size_t>();
      if (!value_optional.has_value()) {
        throw std::runtime_error("Expected numeric conflict_trial_indices entries");
      }
      conflict_trial_indices_.push_back(*value_optional);
    }
  }

  const boost::property_tree::ptree & trials = childRequired(root, "trials", "root");
  for (const auto & trial_entry : trials) {
    const boost::property_tree::ptree & trial_tree = trial_entry.second;
    TrialRecord trial_record;
    trial_record.trial_index = valueRequired<std::size_t>(trial_tree, "trial_index", "trial");
    trial_record.all_planning_succeeded =
      valueRequired<bool>(trial_tree, "all_planning_succeeded", "trial");
    trial_record.has_conflict = trial_tree.get("has_conflict", false);

    const boost::property_tree::ptree & robots = childRequired(trial_tree, "robots", "trial");
    for (const auto & robot_entry : robots) {
      const boost::property_tree::ptree & robot_tree = robot_entry.second;
      RobotTrajectoryRecord robot_record;
      robot_record.robot_id = valueRequired<stop_n_go::base::RobotId>(robot_tree, "id", "robot");
      robot_record.ik_succeeded = valueRequired<bool>(robot_tree, "ik_succeeded", "robot");
      robot_record.planning_succeeded = valueRequired<bool>(
        robot_tree,
        "planning_succeeded",
        "robot");

      if (const auto initial_trajectory = robot_tree.get_child_optional("initial_trajectory")) {
        robot_record.positions = parseDoubleMatrix(
          childRequired(*initial_trajectory, "positions", "initial_trajectory"),
          "initial_trajectory.positions");
        robot_record.duration_from_previous = parseDoubleArray(
          childRequired(*initial_trajectory, "duration_from_previous", "initial_trajectory"),
          "initial_trajectory.duration_from_previous");
      }

      const auto inserted = trial_record.robot_records.emplace(robot_record.robot_id, robot_record);
      if (!inserted.second) {
        throw std::runtime_error(
                "Duplicate robot id in trial " + std::to_string(trial_record.trial_index));
      }
    }

    const auto inserted_trial = trials_.emplace(trial_record.trial_index, std::move(trial_record));
    if (!inserted_trial.second) {
      throw std::runtime_error(
              "Duplicate trial_index in dataset: " + std::to_string(inserted_trial.first->first));
    }
  }

  if (conflict_trial_indices_.empty()) {
    for (const auto & trial_entry : trials_) {
      if (trial_entry.second.has_conflict) {
        conflict_trial_indices_.push_back(trial_entry.first);
      }
    }
  }

  std::sort(conflict_trial_indices_.begin(), conflict_trial_indices_.end());
  conflict_trial_indices_.erase(
    std::unique(conflict_trial_indices_.begin(), conflict_trial_indices_.end()),
    conflict_trial_indices_.end());
}

void ConflictScenarioDataset::validate() const
{
  if (request_.synchronization_time_step <= 0.0) {
    throw std::invalid_argument("Dataset synchronization_time_step must be positive");
  }
  if (trials_.empty()) {
    throw std::invalid_argument("Dataset contains no trial records");
  }

  for (const std::size_t trial_index : conflict_trial_indices_) {
    const TrialRecord & trial_record = trial(trial_index);
    if (!trial_record.all_planning_succeeded || !trial_record.has_conflict) {
      throw std::runtime_error(
              "conflict_trial_indices contains non-conflict trial index " +
              std::to_string(trial_index));
    }
  }
}

const ConflictScenarioDataset::TrialRecord & ConflictScenarioDataset::trial(
  std::size_t trial_index) const
{
  const auto trial_iterator = trials_.find(trial_index);
  if (trial_iterator == trials_.end()) {
    throw std::out_of_range("Trial index not found in dataset: " + std::to_string(trial_index));
  }
  return trial_iterator->second;
}

}  // namespace stop_n_go::app
