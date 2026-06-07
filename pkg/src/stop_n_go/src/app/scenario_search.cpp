#include <stop_n_go/app/scenario_search.hpp>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace stop_n_go::app
{

namespace
{

stop_n_go::app::RobotPlanningAttempt makePlanningFailureAttempt(
  stop_n_go::base::RobotId robot_id,
  const geometry_msgs::msg::PoseStamped & goal,
  bool ik_succeeded,
  const std::string & message)
{
  stop_n_go::app::RobotPlanningAttempt attempt;
  attempt.robot_id = robot_id;
  attempt.goal = goal;
  attempt.ik_succeeded = ik_succeeded;
  attempt.planning_succeeded = false;
  attempt.message = message;
  return attempt;
}

stop_n_go::app::RobotPlanningAttempt makePlanningSuccessAttempt(
  stop_n_go::base::RobotId robot_id,
  const geometry_msgs::msg::PoseStamped & goal,
  stop_n_go::base::Trajectory trajectory)
{
  stop_n_go::app::RobotPlanningAttempt attempt;
  attempt.robot_id = robot_id;
  attempt.goal = goal;
  attempt.ik_succeeded = true;
  attempt.planning_succeeded = true;
  attempt.initial_trajectory = std::move(trajectory);
  attempt.message = "planned";
  return attempt;
}

std_msgs::msg::ColorRGBA colorForRobot(stop_n_go::base::RobotId robot_id, bool success)
{
  std_msgs::msg::ColorRGBA color;
  color.a = success ? 0.8F : 0.25F;
  if (robot_id == 0U) {
    color.r = 1.0F;
  } else if (robot_id == 1U) {
    color.g = 1.0F;
  } else if (robot_id == 2U) {
    color.b = 1.0F;
  } else {
    color.r = 1.0F;
    color.g = 1.0F;
  }
  return color;
}

std::string quoteJsonString(const std::string & value)
{
  std::ostringstream escaped;
  escaped << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        escaped << "\\\"";
        break;
      case '\\':
        escaped << "\\\\";
        break;
      case '\b':
        escaped << "\\b";
        break;
      case '\f':
        escaped << "\\f";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        if (character < 0x20U) {
          escaped << "\\u"
                  << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<int>(character)
                  << std::dec;
        } else {
          escaped << static_cast<char>(character);
        }
        break;
    }
  }
  escaped << '"';
  return escaped.str();
}

void writeStringArray(std::ostream & stream, const std::vector<std::string> & values)
{
  stream << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << quoteJsonString(values[index]);
  }
  stream << "]";
}

void writeDoubleArray(std::ostream & stream, const std::vector<double> & values)
{
  stream << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  stream << "]";
}

void writeSizeArray(std::ostream & stream, const std::vector<std::size_t> & values)
{
  stream << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  stream << "]";
}

void writePose(std::ostream & stream, const geometry_msgs::msg::PoseStamped & goal)
{
  stream << "{\n";
  stream << "            \"frame_id\": " << quoteJsonString(goal.header.frame_id) << ",\n";
  stream << "            \"position\": {\n";
  stream << "              \"x\": " << goal.pose.position.x << ",\n";
  stream << "              \"y\": " << goal.pose.position.y << ",\n";
  stream << "              \"z\": " << goal.pose.position.z << "\n";
  stream << "            },\n";
  stream << "            \"orientation\": {\n";
  stream << "              \"x\": " << goal.pose.orientation.x << ",\n";
  stream << "              \"y\": " << goal.pose.orientation.y << ",\n";
  stream << "              \"z\": " << goal.pose.orientation.z << ",\n";
  stream << "              \"w\": " << goal.pose.orientation.w << "\n";
  stream << "            }\n";
  stream << "          }";
}

const stop_n_go::base::StateLayout & layoutForRobot(
  const std::vector<stop_n_go::base::StateLayout::ConstPtr> & layouts,
  stop_n_go::base::RobotId robot_id)
{
  if (robot_id >= layouts.size() || layouts[robot_id] == nullptr) {
    throw std::runtime_error("Missing state layout for robot id " + std::to_string(robot_id));
  }
  return *layouts[robot_id];
}

void writeExperimentMetadataJson(
  std::ostream & stream,
  const stop_n_go::app::MultiArmScenarioSearchRequest & request,
  const stop_n_go::base::RobotRegistry & registry,
  const std::vector<stop_n_go::base::StateLayout::ConstPtr> & layouts)
{
  stream << "{\n";
  stream << "    \"random_seed\": " << request.random_seed << ",\n";
  stream << "    \"synchronization_time_step\": " << request.synchronization_time_step << ",\n";
  stream << "    \"max_attempts\": " << request.max_attempts << ",\n";
  stream << "    \"shared_goal_region\": {\n";
  stream << "      \"frame_id\": " << quoteJsonString(request.region.frame_id) << ",\n";
  stream << "      \"center_x\": " << request.region.center_x << ",\n";
  stream << "      \"center_y\": " << request.region.center_y << ",\n";
  stream << "      \"center_z\": " << request.region.center_z << ",\n";
  stream << "      \"half_extent_x\": " << request.region.half_extent_x << ",\n";
  stream << "      \"half_extent_y\": " << request.region.half_extent_y << ",\n";
  stream << "      \"half_extent_z\": " << request.region.half_extent_z << "\n";
  stream << "    },\n";
  stream << "    \"fixed_orientation\": {\n";
  stream << "      \"x\": " << request.fixed_orientation.x << ",\n";
  stream << "      \"y\": " << request.fixed_orientation.y << ",\n";
  stream << "      \"z\": " << request.fixed_orientation.z << ",\n";
  stream << "      \"w\": " << request.fixed_orientation.w << "\n";
  stream << "    },\n";
  stream << "    \"robots\": [\n";

  for (stop_n_go::base::RobotId robot_id = 0; robot_id < registry.size(); ++robot_id) {
    const stop_n_go::base::StateLayout & layout = layoutForRobot(layouts, robot_id);
    stream << "      {\n";
    stream << "        \"id\": " << robot_id << ",\n";
    stream << "        \"group_name\": " << quoteJsonString(registry.groupName(robot_id)) << ",\n";
    stream << "        \"end_effector_name\": " <<
      quoteJsonString(registry.endEffectorLink(robot_id)) << ",\n";
    stream << "        \"joint_names\": ";
    writeStringArray(stream, layout.variableNames());
    stream << "\n";
    stream << "      }";
    if (robot_id + 1U < registry.size()) {
      stream << ",";
    }
    stream << "\n";
  }

  stream << "    ]\n";
  stream << "  }";
}

void writeTrialRecordJson(
  std::ostream & stream,
  const stop_n_go::app::MultiArmScenarioTrialRecord & trial)
{
  stream << "    {\n";
  stream << "      \"trial_index\": " << trial.attempt_index << ",\n";
  stream << "      \"all_planning_succeeded\": "
         << (trial.all_planning_succeeded ? "true" : "false") << ",\n";

  if (trial.all_planning_succeeded) {
    stream << "      \"has_conflict\": " << (trial.first_conflict.has_value() ? "true" : "false")
           << ",\n";
    stream << "      \"first_conflict\": ";
    if (trial.first_conflict.has_value()) {
      stream << "{\n";
      stream << "        \"first_robot_id\": " << trial.first_conflict->firstRobot() << ",\n";
      stream << "        \"second_robot_id\": " << trial.first_conflict->secondRobot() << ",\n";
      stream << "        \"time_index\": " << trial.first_conflict->timeIndex() << "\n";
      stream << "      },\n";
    } else {
      stream << "null,\n";
    }
  } else {
    stream << "      \"has_conflict\": null,\n";
    stream << "      \"first_conflict\": null,\n";
  }

  stream << "      \"robots\": [\n";

  for (std::size_t attempt_index = 0; attempt_index < trial.robot_attempts.size(); ++attempt_index) {
    const stop_n_go::app::RobotPlanningAttempt & attempt = trial.robot_attempts[attempt_index];

    stream << "        {\n";
    stream << "          \"id\": " << attempt.robot_id << ",\n";
    stream << "          \"goal\": ";
    writePose(stream, attempt.goal);
    stream << ",\n";
    stream << "          \"ik_succeeded\": " << (attempt.ik_succeeded ? "true" : "false") << ",\n";
    stream << "          \"planning_succeeded\": " <<
      (attempt.planning_succeeded ? "true" : "false") << ",\n";
    stream << "          \"message\": " << quoteJsonString(attempt.message);

    if (trial.all_planning_succeeded) {
      if (!attempt.initial_trajectory.has_value()) {
        throw std::runtime_error(
                "Trial is marked all_planning_succeeded but a robot is missing initial_trajectory");
      }

      const stop_n_go::base::Trajectory & trajectory = *attempt.initial_trajectory;
      stream << ",\n";
      stream << "          \"initial_trajectory\": {\n";
      stream << "            \"positions\": [\n";

      for (std::size_t waypoint_index = 0; waypoint_index < trajectory.size(); ++waypoint_index) {
        stream << "              ";
        writeDoubleArray(stream, trajectory.waypoint(waypoint_index).values());
        if (waypoint_index + 1U < trajectory.size()) {
          stream << ",";
        }
        stream << "\n";
      }

      stream << "            ],\n";
      stream << "            \"duration_from_previous\": [";
      for (std::size_t waypoint_index = 0; waypoint_index < trajectory.size(); ++waypoint_index) {
        if (waypoint_index > 0U) {
          stream << ", ";
        }
        stream << trajectory.durationFromPrevious(waypoint_index);
      }
      stream << "]\n";
      stream << "          }";
    }

    stream << "\n";
    stream << "        }";
    if (attempt_index + 1U < trial.robot_attempts.size()) {
      stream << ",";
    }
    stream << "\n";
  }

  stream << "      ]\n";
  stream << "    }";
}

class SearchResultJsonStreamWriter
{
public:
  SearchResultJsonStreamWriter(
    const std::string & file_path,
    const stop_n_go::app::MultiArmScenarioSearchRequest & request,
    const stop_n_go::base::RobotRegistry & registry,
    const std::vector<stop_n_go::base::StateLayout::ConstPtr> & layouts)
  : file_path_(file_path),
    temp_file_path_(file_path + ".tmp"),
    output_(temp_file_path_, std::ios::out | std::ios::trunc)
  {
    if (file_path_.empty()) {
      throw std::invalid_argument("JSON stream writer requires a non-empty file path");
    }
    if (!output_.is_open()) {
      throw std::runtime_error("Failed to open scenario-search JSON temp file: " + temp_file_path_);
    }

    output_ << std::setprecision(17);
    output_ << "{\n";
    output_ << "  \"schema_version\": \"1.0\",\n";
    output_ << "  \"experiment\": ";
    writeExperimentMetadataJson(output_, request, registry, layouts);
    output_ << ",\n";
    output_ << "  \"trials\": [\n";
  }

  ~SearchResultJsonStreamWriter()
  {
    if (!finalized_) {
      output_.close();
      std::remove(temp_file_path_.c_str());
    }
  }

  void appendTrial(const stop_n_go::app::MultiArmScenarioTrialRecord & trial)
  {
    if (!first_trial_) {
      output_ << ",\n";
    }
    writeTrialRecordJson(output_, trial);
    first_trial_ = false;
    if (!output_.good()) {
      throw std::runtime_error("Failed while streaming trial record to JSON file: " + file_path_);
    }
  }

  void finalize(const std::vector<std::size_t> & conflict_trial_indices)
  {
    output_ << "\n  ],\n";
    output_ << "  \"conflict_trial_indices\": ";
    writeSizeArray(output_, conflict_trial_indices);
    output_ << "\n";
    output_ << "}\n";

    output_.close();
    if (!output_.good()) {
      throw std::runtime_error("Failed to finalize scenario-search JSON file: " + temp_file_path_);
    }

    if (std::rename(temp_file_path_.c_str(), file_path_.c_str()) != 0) {
      throw std::runtime_error(
              "Failed to move scenario-search JSON temp file to target path: " + file_path_);
    }

    finalized_ = true;
  }

private:
  std::string file_path_;
  std::string temp_file_path_;
  std::ofstream output_;
  bool first_trial_ = true;
  bool finalized_ = false;
};

}  // namespace

MultiArmScenarioBuilder::MultiArmScenarioBuilder(
  const stop_n_go::base::RobotRegistry & registry,
  stop_n_go::base::InitialPlanner & initial_planner,
  const stop_n_go::core::ConflictChecker & conflict_checker,
  std::vector<stop_n_go::base::StateLayout::ConstPtr> layouts,
  moveit::core::RobotState reference_state)
: registry_(registry),
  initial_planner_(initial_planner),
  conflict_checker_(conflict_checker),
  layouts_(std::move(layouts)),
  reference_state_(std::move(reference_state))
{
  validate();
}

MultiArmScenarioSearchResult MultiArmScenarioBuilder::findScenario(
  const MultiArmScenarioSearchRequest & request,
  const MoveGroupMap & move_groups) const
{
  const MultiArmScenarioRecordOptions default_record_options;
  return findScenario(request, move_groups, default_record_options);
}

MultiArmScenarioSearchResult MultiArmScenarioBuilder::findScenario(
  const MultiArmScenarioSearchRequest & request,
  const MoveGroupMap & move_groups,
  const MultiArmScenarioRecordOptions & record_options) const
{
  MultiArmScenarioSearchResult result;

  if (move_groups.size() != registry_.size()) {
    throw std::invalid_argument(
            "MultiArmScenarioBuilder requires one move group per registered robot");
  }
  if (request.synchronization_time_step <= 0.0) {
    throw std::invalid_argument("MultiArmScenarioBuilder synchronization_time_step must be positive");
  }
  if (
    record_options.json_output_path.empty() &&
    record_options.max_trial_records_in_memory < request.max_attempts)
  {
    throw std::invalid_argument(
            "max_trial_records_in_memory is smaller than max_attempts but no json_output_path was "
            "provided to spill trial records to disk");
  }
  if (
    !record_options.json_output_path.empty() &&
    !record_options.conflict_only_json_output_path.empty() &&
    record_options.json_output_path == record_options.conflict_only_json_output_path)
  {
    throw std::invalid_argument(
            "json_output_path and conflict_only_json_output_path must be different when both are "
            "set");
  }

  std::unique_ptr<SearchResultJsonStreamWriter> stream_writer;
  std::unique_ptr<SearchResultJsonStreamWriter> conflict_only_stream_writer;
  if (!record_options.json_output_path.empty()) {
    stream_writer = std::make_unique<SearchResultJsonStreamWriter>(
      record_options.json_output_path,
      request,
      registry_,
      layouts_);
  }
  if (!record_options.conflict_only_json_output_path.empty()) {
    conflict_only_stream_writer = std::make_unique<SearchResultJsonStreamWriter>(
      record_options.conflict_only_json_output_path,
      request,
      registry_,
      layouts_);
  }

  std::mt19937 generator(request.random_seed);
  for (std::size_t attempt = 0; attempt < request.max_attempts; ++attempt) {
    MultiArmScenarioTrialRecord trial;
    trial.attempt_index = attempt;

    const std::vector<geometry_msgs::msg::PoseStamped> goals = sampleGoals(request, generator);
    std::vector<stop_n_go::base::Trajectory> initial_trajectories;
    initial_trajectories.reserve(registry_.size());

    bool planning_failed = false;
    for (stop_n_go::base::RobotId robot_id = 0; robot_id < registry_.size(); ++robot_id) {
      if (planning_failed) {
        trial.robot_attempts.push_back(
          makePlanningFailureAttempt(
            robot_id,
            goals[robot_id],
            false,
            "skipped_after_previous_failure"));
        continue;
      }

      const auto move_group_iterator = move_groups.find(robot_id);
      if (move_group_iterator == move_groups.end() || move_group_iterator->second == nullptr) {
        throw std::invalid_argument("MultiArmScenarioBuilder is missing a move group for a robot id");
      }

      if (!ikFeasible(*move_group_iterator->second, robot_id, goals[robot_id]))
      {
        trial.robot_attempts.push_back(
          makePlanningFailureAttempt(
            robot_id, goals[robot_id], false,
            "ik_failed"));
        planning_failed = true;
        continue;
      }

      try {
        stop_n_go::base::Trajectory trajectory = initial_planner_.computeInitialTrajectory(
          robot_id,
          goals[robot_id]);
        trial.robot_attempts.push_back(
          makePlanningSuccessAttempt(robot_id, goals[robot_id], trajectory));
        initial_trajectories.push_back(std::move(trajectory));
      } catch (const std::exception & exception) {
        trial.robot_attempts.push_back(
          makePlanningFailureAttempt(
            robot_id, goals[robot_id], true,
            exception.what()));
        planning_failed = true;
        continue;
      }
    }

    trial.all_planning_succeeded = !planning_failed &&
      trial.robot_attempts.size() == registry_.size();

    if (trial.all_planning_succeeded) {
      const std::vector<stop_n_go::base::Trajectory> synchronized_initial_trajectories =
        synchronizeAndEqualizeTrajectories(
        initial_trajectories,
        request.synchronization_time_step);
      trial.first_conflict = findFirstConflict(synchronized_initial_trajectories);
      if (trial.first_conflict.has_value()) {
        result.conflict_trial_indices.push_back(attempt);
      }
    } else {
      for (RobotPlanningAttempt & robot_attempt : trial.robot_attempts) {
        robot_attempt.initial_trajectory.reset();
      }
    }

    if (stream_writer != nullptr) {
      stream_writer->appendTrial(trial);
    }
    if (conflict_only_stream_writer != nullptr && trial.first_conflict.has_value()) {
      conflict_only_stream_writer->appendTrial(trial);
    }

    if (record_options.max_trial_records_in_memory > 0U) {
      result.trial_records.push_back(std::move(trial));
      while (result.trial_records.size() > record_options.max_trial_records_in_memory) {
        result.trial_records.erase(result.trial_records.begin());
      }
    }
  }

  if (stream_writer != nullptr) {
    stream_writer->finalize(result.conflict_trial_indices);
  }
  if (conflict_only_stream_writer != nullptr) {
    conflict_only_stream_writer->finalize(result.conflict_trial_indices);
  }

  return result;
}

std::vector<stop_n_go::base::Trajectory> MultiArmScenarioBuilder::synchronizeAndEqualizeTrajectories(
  const std::vector<stop_n_go::base::Trajectory> & trajectories,
  double time_step) const
{
  std::vector<stop_n_go::base::Trajectory> synchronized;
  synchronized.reserve(trajectories.size());
  std::size_t max_size = 0U;
  for (const stop_n_go::base::Trajectory & trajectory : trajectories) {
    synchronized.push_back(trajectory.resample(time_step));
    max_size = std::max(max_size, synchronized.back().size());
  }

  for (stop_n_go::base::Trajectory & trajectory : synchronized) {
    while (trajectory.size() < max_size) {
      trajectory.appendHold(time_step);
    }
  }

  return synchronized;
}

moveit_msgs::msg::DisplayTrajectory MultiArmScenarioBuilder::buildDisplayTrajectory(
  const std::vector<stop_n_go::base::Trajectory> & trajectories) const
{
  if (trajectories.empty()) {
    throw std::invalid_argument("buildDisplayTrajectory requires at least one trajectory");
  }
  if (trajectories.size() != layouts_.size()) {
    throw std::invalid_argument("buildDisplayTrajectory requires matching trajectory/layout counts");
  }

  robot_trajectory::RobotTrajectory full_trajectory(reference_state_.getRobotModel(), nullptr);
  for (std::size_t waypoint_index = 0; waypoint_index < trajectories.front().size();
    ++waypoint_index)
  {
    moveit::core::RobotState waypoint_state(reference_state_);
    for (std::size_t robot_index = 0; robot_index < trajectories.size(); ++robot_index) {
      waypoint_state.setJointGroupActivePositions(
        layouts_[robot_index]->jointModelGroup(),
        trajectories[robot_index].waypoint(waypoint_index).values());
    }
    waypoint_state.update();
    full_trajectory.addSuffixWayPoint(
      waypoint_state,
      trajectories.front().durationFromPrevious(waypoint_index));
  }

  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  moveit::core::robotStateToRobotStateMsg(reference_state_, display_trajectory.trajectory_start);
  moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
  full_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);
  display_trajectory.trajectory.push_back(robot_trajectory_msg);
  return display_trajectory;
}

visualization_msgs::msg::MarkerArray MultiArmScenarioBuilder::buildSearchMarkers(
  const MultiArmScenarioSearchRequest & request,
  const MultiArmScenarioSearchResult & result,
  std::size_t max_trials_to_visualize) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  const RobotRegionAssignment assignment = defaultAssignment(request.region);

  visualization_msgs::msg::Marker region_marker;
  region_marker.header.frame_id = request.region.frame_id;
  region_marker.ns = "search_region";
  region_marker.id = 0;
  region_marker.type = visualization_msgs::msg::Marker::CUBE;
  region_marker.action = visualization_msgs::msg::Marker::ADD;
  region_marker.pose.orientation.w = 1.0;
  region_marker.pose.position.x = request.region.center_x;
  region_marker.pose.position.y = request.region.center_y;
  region_marker.pose.position.z = request.region.center_z;
  region_marker.scale.x = 2.0 * request.region.half_extent_x;
  region_marker.scale.y = 2.0 * request.region.half_extent_y;
  region_marker.scale.z = 2.0 * request.region.half_extent_z;
  region_marker.color.r = 1.0F;
  region_marker.color.g = 1.0F;
  region_marker.color.b = 1.0F;
  region_marker.color.a = 0.08F;
  marker_array.markers.push_back(region_marker);

  int marker_id = 1;
  for (stop_n_go::base::RobotId robot_id = 0; robot_id < registry_.size(); ++robot_id) {
    const auto partition_iterator = assignment.partitions.find(robot_id);
    if (partition_iterator == assignment.partitions.end()) {
      continue;
    }

    const RegionPartition & partition = partition_iterator->second;
    visualization_msgs::msg::Marker partition_marker;
    partition_marker.header.frame_id = request.region.frame_id;
    partition_marker.ns = "search_partition";
    partition_marker.id = marker_id++;
    partition_marker.type = visualization_msgs::msg::Marker::CUBE;
    partition_marker.action = visualization_msgs::msg::Marker::ADD;
    partition_marker.pose.orientation.w = 1.0;
    partition_marker.pose.position.x = 0.5 * (partition.min_x + partition.max_x);
    partition_marker.pose.position.y = 0.5 * (partition.min_y + partition.max_y);
    partition_marker.pose.position.z = 0.5 * (partition.min_z + partition.max_z);
    partition_marker.scale.x = partition.max_x - partition.min_x;
    partition_marker.scale.y = partition.max_y - partition.min_y;
    partition_marker.scale.z = partition.max_z - partition.min_z;
    partition_marker.color = colorForRobot(robot_id, true);
    partition_marker.color.a = 0.12F;
    marker_array.markers.push_back(partition_marker);
  }

  const std::size_t trial_count = std::min(max_trials_to_visualize, result.trial_records.size());
  for (std::size_t trial_index = 0; trial_index < trial_count; ++trial_index) {
    const MultiArmScenarioTrialRecord & trial = result.trial_records[trial_index];
    for (const RobotPlanningAttempt & attempt : trial.robot_attempts) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = attempt.goal.header.frame_id;
      marker.ns = "sampled_goal_pose";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = attempt.goal.pose;
      marker.scale.x = 0.12;
      marker.scale.y = 0.015;
      marker.scale.z = 0.015;
      marker.color = colorForRobot(attempt.robot_id, attempt.planning_succeeded);
      marker_array.markers.push_back(marker);
    }
  }

  return marker_array;
}

std::string MultiArmScenarioBuilder::buildSearchResultJson(
  const MultiArmScenarioSearchRequest & request,
  const MultiArmScenarioSearchResult & result) const
{
  std::ostringstream stream;
  stream << std::setprecision(17);

  stream << "{\n";
  stream << "  \"schema_version\": \"1.0\",\n";
  stream << "  \"experiment\": ";
  writeExperimentMetadataJson(stream, request, registry_, layouts_);
  stream << ",\n";
  stream << "  \"trials\": [\n";

  for (std::size_t trial_index = 0; trial_index < result.trial_records.size(); ++trial_index) {
    writeTrialRecordJson(stream, result.trial_records[trial_index]);
    if (trial_index + 1U < result.trial_records.size()) {
      stream << ",";
    }
    stream << "\n";
  }

  stream << "  ],\n";
  stream << "  \"conflict_trial_indices\": ";
  writeSizeArray(stream, result.conflict_trial_indices);
  stream << "\n";
  stream << "}\n";
  return stream.str();
}

void MultiArmScenarioBuilder::writeSearchResultJson(
  const std::string & file_path,
  const MultiArmScenarioSearchRequest & request,
  const MultiArmScenarioSearchResult & result) const
{
  if (file_path.empty()) {
    throw std::invalid_argument("writeSearchResultJson requires a non-empty file path");
  }

  std::ofstream output(file_path, std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("Failed to open scenario-search JSON output file: " + file_path);
  }

  output << buildSearchResultJson(request, result);
  if (!output.good()) {
    throw std::runtime_error("Failed to write scenario-search JSON output file: " + file_path);
  }
}

RobotRegionAssignment MultiArmScenarioBuilder::defaultAssignment(
  const SharedGoalRegion & region) const
{
  RobotRegionAssignment assignment;

  const double min_x = region.center_x - region.half_extent_x;
  const double max_x = region.center_x + region.half_extent_x;
  const double min_y = region.center_y - region.half_extent_y;
  const double max_y = region.center_y + region.half_extent_y;
  const double min_z = region.center_z - region.half_extent_z;
  const double max_z = region.center_z + region.half_extent_z;

  const double mid_x = region.center_x;
  const double mid_y = region.center_y;

  assignment.partitions.emplace(0, RegionPartition{min_x, mid_x, min_y, mid_y, min_z, max_z});
  assignment.partitions.emplace(1, RegionPartition{mid_x, max_x, min_y, mid_y, min_z, max_z});
  assignment.partitions.emplace(2, RegionPartition{min_x, mid_x, mid_y, max_y, min_z, max_z});
  assignment.partitions.emplace(3, RegionPartition{mid_x, max_x, mid_y, max_y, min_z, max_z});

  return assignment;
}

std::vector<geometry_msgs::msg::PoseStamped> MultiArmScenarioBuilder::sampleGoals(
  const MultiArmScenarioSearchRequest & request,
  std::mt19937 & generator) const
{
  const RobotRegionAssignment assignment = defaultAssignment(request.region);

  std::vector<geometry_msgs::msg::PoseStamped> goals;
  goals.reserve(registry_.size());
  for (stop_n_go::base::RobotId robot_id = 0; robot_id < registry_.size(); ++robot_id) {
    const auto partition_iterator = assignment.partitions.find(robot_id);
    if (partition_iterator == assignment.partitions.end()) {
      throw std::invalid_argument("MultiArmScenarioBuilder is missing a region partition for a robot id");
    }

    const RegionPartition & partition = partition_iterator->second;
    std::uniform_real_distribution<double> sample_x(partition.min_x, partition.max_x);
    std::uniform_real_distribution<double> sample_y(partition.min_y, partition.max_y);
    std::uniform_real_distribution<double> sample_z(partition.min_z, partition.max_z);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = request.region.frame_id;
    goal.pose.position.x = sample_x(generator);
    goal.pose.position.y = sample_y(generator);
    goal.pose.position.z = sample_z(generator);
    goal.pose.orientation = request.fixed_orientation;
    goals.push_back(goal);
  }

  return goals;
}

bool MultiArmScenarioBuilder::ikFeasible(
  const moveit::planning_interface::MoveGroupInterface & move_group,
  stop_n_go::base::RobotId robot_id,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  moveit::planning_interface::MoveGroupInterface & mutable_move_group =
    const_cast<moveit::planning_interface::MoveGroupInterface &>(move_group);
  return mutable_move_group.setJointValueTarget(goal, registry_.endEffectorLink(robot_id));
}

std::optional<stop_n_go::core::ConflictResult> MultiArmScenarioBuilder::findFirstConflict(
  const std::vector<stop_n_go::base::Trajectory> & synchronized_trajectories) const
{
  if (synchronized_trajectories.empty()) {
    return std::nullopt;
  }

  stop_n_go::core::SearchNode node(synchronized_trajectories);
  for (std::size_t time_index = 0; time_index < synchronized_trajectories.front().size();
    ++time_index)
  {
    const std::optional<stop_n_go::core::ConflictResult> conflict =
      conflict_checker_.conflictCheck(node, time_index);
    if (conflict.has_value()) {
      return conflict;
    }
  }

  return std::nullopt;
}

void MultiArmScenarioBuilder::validate() const
{
  if (layouts_.size() != registry_.size()) {
    throw std::invalid_argument("MultiArmScenarioBuilder requires one layout per registered robot");
  }
  for (stop_n_go::base::RobotId robot_id = 0; robot_id < registry_.size(); ++robot_id) {
    if (layouts_[robot_id] == nullptr) {
      throw std::invalid_argument("MultiArmScenarioBuilder layout pointers must not be null");
    }
  }
}

}  // namespace stop_n_go::app
