#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <stop_n_go/app/scenario_search.hpp>
#include <stop_n_go/base/initial_planner.hpp>
#include <stop_n_go/base/robot_registry.hpp>
#include <stop_n_go/core/conflict_check.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <chrono>

namespace
{

constexpr double kVisualizationTimeStep = 0.05;

std::string poseString(const geometry_msgs::msg::PoseStamped & pose)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "frame=" << pose.header.frame_id
         << " pos=(" << pose.pose.position.x << ", " << pose.pose.position.y << ", "
         << pose.pose.position.z << ")"
         << " quat=(" << pose.pose.orientation.x << ", " << pose.pose.orientation.y << ", "
         << pose.pose.orientation.z << ", " << pose.pose.orientation.w << ")";
  return stream.str();
}

std::size_t parseSizeParameter(const rclcpp::Parameter & parameter, const std::string & name)
{
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    const int64_t value = parameter.as_int();
    if (value < 0) {
      throw std::invalid_argument(name + " must be non-negative");
    }
    return static_cast<std::size_t>(value);
  }

  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    std::size_t parsed_characters = 0U;
    const std::size_t value = std::stoull(parameter.as_string(), &parsed_characters);
    if (parsed_characters != parameter.as_string().size()) {
      throw std::invalid_argument(name + " must be a numeric string");
    }
    return value;
  }

  throw std::invalid_argument(name + " must be an integer or numeric string");
}

void printTrialSummary(const stop_n_go::app::MultiArmScenarioSearchResult & result)
{
  std::cout << "[sng.goal_search] recorded_trials=" << result.trial_records.size() << '\n';
  for (const stop_n_go::app::MultiArmScenarioTrialRecord & trial : result.trial_records) {
    std::cout << "[sng.goal_search] trial=" << trial.attempt_index
              << " all_planning_succeeded=" << std::boolalpha << trial.all_planning_succeeded
              << " has_conflict=" << trial.first_conflict.has_value() << '\n';
    for (const stop_n_go::app::RobotPlanningAttempt & attempt : trial.robot_attempts) {
      std::cout << "[sng.goal_search]   robot=" << attempt.robot_id
                << " ik=" << attempt.ik_succeeded
                << " planned=" << attempt.planning_succeeded
                << " goal=" << poseString(attempt.goal)
                << " message=" << attempt.message << '\n';
    }
  }
}

std::vector<stop_n_go::base::Trajectory> retimeForVisualization(
  const std::vector<stop_n_go::base::Trajectory> & trajectories,
  double visualization_time_step)
{
  std::vector<stop_n_go::base::Trajectory> retimed;
  retimed.reserve(trajectories.size());
  for (const stop_n_go::base::Trajectory & trajectory : trajectories) {
    stop_n_go::base::Trajectory display_trajectory(trajectory.layoutPtr());
    display_trajectory.reserve(trajectory.size());
    for (std::size_t waypoint_index = 0; waypoint_index < trajectory.size(); ++waypoint_index) {
      const double duration_from_previous = waypoint_index == 0U ? 0.0 : visualization_time_step;
      display_trajectory.append(trajectory.waypoint(waypoint_index), duration_from_previous);
    }
    retimed.push_back(std::move(display_trajectory));
  }
  return retimed;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "find_multi_arm_overlap_goals_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "/stop_n_go/display_planned_path", 10);
  auto marker_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/stop_n_go/search_markers", 10);

  std::thread spin_thread([node]() {rclcpp::spin(node);});

  try {
    stop_n_go::base::InitialPlanner::MoveGroupMap move_groups;
    move_groups.emplace(
      0,
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm1"));
    move_groups.emplace(
      1,
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm2"));
    move_groups.emplace(
      2,
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm3"));
    move_groups.emplace(
      3,
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm4"));

    const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
      {2, "arm3", "arm3_tool0"},
      {3, "arm4", "arm4_tool0"},
    });

    const moveit::core::RobotModelConstPtr robot_model = move_groups.at(0)->getRobotModel();
    moveit::core::RobotStatePtr current_state = move_groups.at(0)->getCurrentState(5.0);
    if (current_state == nullptr) {
      throw std::runtime_error("Failed to read current state before goal search");
    }
    moveit::core::RobotState reference_state = *current_state;
    reference_state.update();

    std::vector<stop_n_go::base::StateLayout::ConstPtr> layouts;
    layouts.reserve(4U);
    stop_n_go::core::ConflictChecker::LayoutMap layout_map;
    for (stop_n_go::base::RobotId robot_id = 0; robot_id < 4U; ++robot_id) {
      const auto layout = std::make_shared<stop_n_go::base::StateLayout>(
        *robot_model, registry.groupName(robot_id));
      layouts.push_back(layout);
      layout_map.emplace(robot_id, layout);
    }

    const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    const auto checker = std::make_shared<stop_n_go::core::ConflictChecker>(
      registry, robot_model, std::move(layout_map), planning_scene, reference_state);
    stop_n_go::base::InitialPlanner initial_planner(registry, robot_model, move_groups);
    stop_n_go::app::MultiArmScenarioBuilder scenario_builder(
      registry,
      initial_planner,
      *checker,
      layouts,
      reference_state);

    // Search request:
    // - sample candidate points directly in a shared world-frame box
    // - assign each robot to one nearby partition of that box
    // - build PoseStamped goals directly in the world frame
    // - use a fixed world-frame orientation for all sampled goals
    stop_n_go::app::MultiArmScenarioSearchRequest request;
    request.region.frame_id = "world";
    request.fixed_orientation.w = 1.0;
    request.synchronization_time_step = 0.2;
    request.max_attempts = 200;
    request.random_seed = 7U;
    if (node->has_parameter("max_attempts")) {
      request.max_attempts =
        parseSizeParameter(node->get_parameter("max_attempts"), "max_attempts");
    }

    std::cout << "[sng.goal_search] shared region frame=" << request.region.frame_id
              << " x=[" << request.region.center_x - request.region.half_extent_x << ", "
              << request.region.center_x + request.region.half_extent_x << "]"
              << " y=[" << request.region.center_y - request.region.half_extent_y << ", "
              << request.region.center_y + request.region.half_extent_y << "]"
              << " z=[" << request.region.center_z - request.region.half_extent_z << ", "
              << request.region.center_z + request.region.half_extent_z << "]" << '\n';
    std::cout <<
      "[sng.goal_search] default partitioning: x=0 and y=0 split the box into four nearby regions"
              << '\n';
    std::cout <<
      "[sng.goal_search] arm1: x<=0,y<=0 | arm2: x>=0,y<=0 | arm3: x<=0,y>=0 | arm4: x>=0,y>=0"
              << '\n';
    std::cout << "[sng.goal_search] fixed world orientation=(0, 0, 0, 1)" << '\n';
    std::cout << "[sng.goal_search] max_attempts=" << request.max_attempts << '\n';

    const std::string json_output_path = "/tmp/find_multi_arm_overlap_goals_demo_trials.json";
    const std::string conflict_only_json_output_path =
      "/tmp/find_multi_arm_overlap_goals_demo_conflict_only_trials.json";

    stop_n_go::app::MultiArmScenarioRecordOptions record_options;
    record_options.json_output_path = json_output_path;
    record_options.conflict_only_json_output_path = conflict_only_json_output_path;
    record_options.max_trial_records_in_memory = request.max_attempts;

    const stop_n_go::app::MultiArmScenarioSearchResult search_result =
      scenario_builder.findScenario(request, move_groups, record_options);
    std::cout << "[sng.goal_search] wrote json record to " << json_output_path << '\n';
    std::cout << "[sng.goal_search] wrote conflict-only json record to " <<
      conflict_only_json_output_path << '\n';

    printTrialSummary(search_result);

    const visualization_msgs::msg::MarkerArray marker_array =
      scenario_builder.buildSearchMarkers(request, search_result, 10U);
    marker_publisher->publish(marker_array);
    auto marker_timer = node->create_wall_timer(
      std::chrono::seconds(1),
      [marker_publisher, marker_array]() {
        marker_publisher->publish(marker_array);
      });
    std::cout << "[sng.goal_search] published search region and first 10 sampled goal markers" <<
      '\n';
    (void)marker_timer;

    rclcpp::TimerBase::SharedPtr display_timer;

    if (search_result.conflict_trial_indices.empty()) {
      std::cout << "[sng.goal_search] no all-success conflicting trial found" << '\n';
    } else {
      std::cout << "[sng.goal_search] conflicting_trial_count=" <<
        search_result.conflict_trial_indices.size() << '\n';
      std::cout << "[sng.goal_search] conflict_trial_indices=[";
      for (std::size_t i = 0; i < search_result.conflict_trial_indices.size(); ++i) {
        if (i > 0U) {
          std::cout << ", ";
        }
        std::cout << search_result.conflict_trial_indices[i];
      }
      std::cout << "]" << '\n';

      const std::size_t first_conflict_trial_index = search_result.conflict_trial_indices.front();
      const auto trial_iterator = std::find_if(
        search_result.trial_records.begin(),
        search_result.trial_records.end(),
        [first_conflict_trial_index](const stop_n_go::app::MultiArmScenarioTrialRecord & trial) {
          return trial.attempt_index == first_conflict_trial_index;
        });

      if (trial_iterator != search_result.trial_records.end()) {
        const stop_n_go::app::MultiArmScenarioTrialRecord & first_conflict_trial = *trial_iterator;
        for (const stop_n_go::app::RobotPlanningAttempt & attempt :
          first_conflict_trial.robot_attempts)
        {
          std::cout << "[sng.goal_search] sampled_world_goal robot=" << attempt.robot_id << " "
                    << poseString(attempt.goal) << '\n';
          if (attempt.initial_trajectory.has_value()) {
            std::cout << "[sng.goal_search] initial robot=" << attempt.robot_id
                      << " waypoint_count=" << attempt.initial_trajectory->size()
                      << " total_duration=" << attempt.initial_trajectory->totalDuration() << '\n';
          }
        }

        if (first_conflict_trial.first_conflict.has_value()) {
          std::cout << "[sng.goal_search] first_conflict pair=("
                    << first_conflict_trial.first_conflict->firstRobot() << ", "
                    << first_conflict_trial.first_conflict->secondRobot() << ")"
                    << " time_index=" << first_conflict_trial.first_conflict->timeIndex() << '\n';
        }

        std::vector<stop_n_go::base::Trajectory> initial_trajectories;
        initial_trajectories.reserve(registry.size());
        for (stop_n_go::base::RobotId robot_id = 0; robot_id < registry.size(); ++robot_id) {
          const auto robot_attempt_iterator = std::find_if(
            first_conflict_trial.robot_attempts.begin(),
            first_conflict_trial.robot_attempts.end(),
            [robot_id](const stop_n_go::app::RobotPlanningAttempt & attempt) {
              return attempt.robot_id == robot_id;
            });
          if (robot_attempt_iterator == first_conflict_trial.robot_attempts.end()) {
            throw std::runtime_error(
                    "Cannot publish combined trajectory: missing robot attempt for robot id " +
                    std::to_string(robot_id));
          }
          if (!robot_attempt_iterator->initial_trajectory.has_value()) {
            throw std::runtime_error(
                    "Cannot publish combined trajectory: missing initial trajectory for robot id " +
                    std::to_string(robot_id));
          }
          initial_trajectories.push_back(*robot_attempt_iterator->initial_trajectory);
        }

        const std::vector<stop_n_go::base::Trajectory> synchronized_trajectories =
          scenario_builder.synchronizeAndEqualizeTrajectories(
          initial_trajectories,
          request.synchronization_time_step);
        const std::vector<stop_n_go::base::Trajectory> display_trajectories =
          retimeForVisualization(synchronized_trajectories, kVisualizationTimeStep);
        const moveit_msgs::msg::DisplayTrajectory display_trajectory =
          scenario_builder.buildDisplayTrajectory(display_trajectories);

        display_timer = node->create_wall_timer(
          std::chrono::seconds(1),
          [display_publisher, display_trajectory]() {
            display_publisher->publish(display_trajectory);
          });

        for (int attempt = 0; attempt < 50 && display_publisher->get_subscription_count() == 0;
          ++attempt)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "[sng.goal_search] RViz subscribers=" <<
          display_publisher->get_subscription_count() << '\n';
        std::cout <<
          "[sng.goal_search] continuously publishing resampled synchronized 4-arm trajectory" <<
          '\n';
        std::cout << "[sng.goal_search] visualization-only time step=" <<
          kVisualizationTimeStep << " sec" << '\n';
        display_publisher->publish(display_trajectory);

        std::cout <<
          "[sng.goal_search] display timing is retimed for RViz; JSON data remains unchanged"
                  << '\n';
        std::cout << "[sng.goal_search] keep RViz open to inspect the conflict visually" << '\n';
      } else {
        std::cout << "[sng.goal_search] first conflict trial is not retained in memory" << '\n';
      }
    }

    (void)display_timer;

    std::cout << "[sng.goal_search] final_stats trials=" << search_result.trial_records.size()
              << " conflicts_collected=" << search_result.conflict_trial_indices.size() << '\n';

    std::cout << "[sng.goal_search] spinning so markers remain visible; press Ctrl-C to exit" <<
      '\n';
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "[sng.goal_search] error: " << exception.what() << '\n';
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
}
