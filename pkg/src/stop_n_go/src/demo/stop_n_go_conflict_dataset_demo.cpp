#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <stop_n_go/app/conflict_scenario_dataset.hpp>
#include <stop_n_go/base/conversion.hpp>
#include <stop_n_go/base/robot_registry.hpp>
#include <stop_n_go/core/conflict_check.hpp>
#include <stop_n_go/core/conflict_resolver.hpp>
#include <stop_n_go/core/stop_n_go_search.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

struct TrialPlaybackRecord
{
  std::size_t conflict_sample_index = 0U;
  std::size_t trial_index = 0U;
  bool solved = false;
  moveit_msgs::msg::DisplayTrajectory initial_display_trajectory;
  moveit_msgs::msg::DisplayTrajectory solved_display_trajectory;
};

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
    return static_cast<std::size_t>(std::stoull(parameter.as_string()));
  }
  throw std::invalid_argument(name + " must be an integer or numeric string");
}

stop_n_go::core::LogLevel parseLogLevel(const std::string & value)
{
  if (value == "quiet") {
    return stop_n_go::core::LogLevel::Quiet;
  }
  if (value == "summary") {
    return stop_n_go::core::LogLevel::Summary;
  }
  if (value == "debug") {
    return stop_n_go::core::LogLevel::Debug;
  }
  throw std::invalid_argument("stop_n_go_log_level must be quiet, summary, or debug");
}

trajectory_msgs::msg::JointTrajectory mergeJointTrajectories(
  const std::vector<trajectory_msgs::msg::JointTrajectory> & trajectories)
{
  if (trajectories.empty()) {
    throw std::invalid_argument("mergeJointTrajectories requires at least one trajectory");
  }

  const std::size_t point_count = trajectories.front().points.size();
  for (std::size_t trajectory_index = 0; trajectory_index < trajectories.size();
    ++trajectory_index)
  {
    const trajectory_msgs::msg::JointTrajectory & trajectory = trajectories[trajectory_index];
    if (trajectory.points.size() != point_count) {
      throw std::runtime_error(
              "mergeJointTrajectories requires equal waypoint counts: trajectory " +
              std::to_string(trajectory_index) + " has " +
              std::to_string(trajectory.points.size()) + " points, expected " +
              std::to_string(point_count));
    }
  }

  trajectory_msgs::msg::JointTrajectory merged;
  merged.header = trajectories.front().header;
  for (const trajectory_msgs::msg::JointTrajectory & trajectory : trajectories) {
    merged.joint_names.insert(
      merged.joint_names.end(),
      trajectory.joint_names.begin(),
      trajectory.joint_names.end());
  }

  merged.points.resize(point_count);
  for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
    merged.points[point_index].time_from_start =
      trajectories.front().points[point_index].time_from_start;
    for (const trajectory_msgs::msg::JointTrajectory & trajectory : trajectories) {
      const std::vector<double> & positions = trajectory.points[point_index].positions;
      merged.points[point_index].positions.insert(
        merged.points[point_index].positions.end(),
        positions.begin(),
        positions.end());
    }
  }

  return merged;
}

std::vector<stop_n_go::base::Trajectory> synchronizeAndEqualizeTrajectories(
  const std::vector<stop_n_go::base::Trajectory> & trajectories,
  double time_step)
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

moveit_msgs::msg::DisplayTrajectory buildDisplayTrajectory(
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit::core::RobotState & reference_state,
  const std::vector<stop_n_go::base::StateLayout::ConstPtr> & layouts,
  const std::vector<stop_n_go::base::Trajectory> & trajectories)
{
  if (trajectories.size() != layouts.size()) {
    throw std::invalid_argument("buildDisplayTrajectory requires matching trajectory/layout counts");
  }

  std::vector<trajectory_msgs::msg::JointTrajectory> per_robot_messages;
  per_robot_messages.reserve(trajectories.size());
  for (std::size_t robot_id = 0; robot_id < trajectories.size(); ++robot_id) {
    const stop_n_go::base::ConversionContext conversion_context(robot_model, layouts[robot_id],
      reference_state);
    per_robot_messages.push_back(conversion_context.toJointTrajectoryMsg(trajectories[robot_id]));
  }

  moveit_msgs::msg::DisplayTrajectory display_message;
  display_message.model_id = robot_model->getName();
  moveit::core::robotStateToRobotStateMsg(reference_state, display_message.trajectory_start);
  moveit_msgs::msg::RobotTrajectory display_robot_trajectory;
  display_robot_trajectory.joint_trajectory = mergeJointTrajectories(per_robot_messages);
  display_message.trajectory.push_back(display_robot_trajectory);
  return display_message;
}

moveit_msgs::msg::DisplayTrajectory buildEmptyDisplayTrajectory(
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit::core::RobotState & reference_state)
{
  moveit_msgs::msg::DisplayTrajectory display_message;
  display_message.model_id = robot_model->getName();
  moveit::core::robotStateToRobotStateMsg(reference_state, display_message.trajectory_start);
  return display_message;
}

void printTrajectorySetSummary(
  const std::string & label,
  const std::vector<stop_n_go::base::Trajectory> & trajectories)
{
  for (std::size_t index = 0; index < trajectories.size(); ++index) {
    std::cout << "[sng.dataset_demo] " << label << " robot=" << index
              << " waypoints=" << trajectories[index].size()
              << " total_duration=" << trajectories[index].totalDuration() << '\n';
  }
}

void printPlaybackRecords(const std::vector<TrialPlaybackRecord> & records)
{
  std::cout << "[sng.dataset_demo] playback records=" << records.size() << '\n';
  for (std::size_t index = 0; index < records.size(); ++index) {
    const TrialPlaybackRecord & record = records[index];
    std::cout << "[sng.dataset_demo] playback_index=" << index
              << " conflict_sample_index=" << record.conflict_sample_index
              << " trial_index=" << record.trial_index
              << " solved=" << std::boolalpha << record.solved << '\n';
  }
}

void publishPlaybackRecord(
  const std::vector<TrialPlaybackRecord> & records,
  std::size_t playback_index,
  const moveit_msgs::msg::DisplayTrajectory & empty_solved_display_trajectory,
  const rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr & initial_display_publisher,
  const rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr & solved_display_publisher,
  bool print_status)
{
  if (playback_index >= records.size()) {
    if (print_status) {
      std::cout << "[sng.dataset_demo] invalid playback_index=" << playback_index
                << " records=" << records.size() << '\n';
    }
    return;
  }

  const TrialPlaybackRecord & record = records[playback_index];
  initial_display_publisher->publish(record.initial_display_trajectory);
  if (record.solved) {
    solved_display_publisher->publish(record.solved_display_trajectory);
  } else {
    solved_display_publisher->publish(empty_solved_display_trajectory);
    if (print_status) {
      std::cout << "[sng.dataset_demo] trial_index=" << record.trial_index
                << " has no solved trajectory; cleared solved display" << '\n';
    }
  }
  if (print_status) {
    std::cout << "[sng.dataset_demo] published playback_index=" << playback_index
              << " conflict_sample_index=" << record.conflict_sample_index
              << " trial_index=" << record.trial_index
              << " solved=" << std::boolalpha << record.solved << '\n';
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "stop_n_go_conflict_dataset_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto initial_display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "/stop_n_go/display_initial_path", 10);
  auto solved_display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "/stop_n_go/display_solved_path", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  try {
    std::string dataset_path = "/tmp/find_multi_arm_overlap_goals_demo_conflict_only_trials.json";
    node->get_parameter_or("dataset_path", dataset_path, dataset_path);
    int64_t max_expanded_nodes_parameter = 10000;
    node->get_parameter_or(
      "max_expanded_nodes",
      max_expanded_nodes_parameter,
      max_expanded_nodes_parameter);
    if (max_expanded_nodes_parameter <= 0) {
      throw std::invalid_argument("max_expanded_nodes must be positive");
    }
    const std::size_t max_expanded_nodes =
      static_cast<std::size_t>(max_expanded_nodes_parameter);
    std::string log_level_text = "summary";
    node->get_parameter_or("stop_n_go_log_level", log_level_text, log_level_text);
    const stop_n_go::core::LogLevel log_level = parseLogLevel(log_level_text);
    bool store_trajectories = true;
    node->get_parameter_or("store_trajectories", store_trajectories, store_trajectories);
    std::string playback_index_topic = "/stop_n_go/playback_index";
    node->get_parameter_or("playback_index_topic", playback_index_topic, playback_index_topic);

    std::cout << "[sng.dataset_demo] dataset_path=" << dataset_path << '\n';
    std::cout << "[sng.dataset_demo] max_expanded_nodes=" << max_expanded_nodes << '\n';
    std::cout << "[sng.dataset_demo] stop_n_go_log_level=" << log_level_text << '\n';
    std::cout << "[sng.dataset_demo] store_trajectories=" << std::boolalpha <<
      store_trajectories << '\n';
    std::cout << "[sng.dataset_demo] playback_index_topic=" << playback_index_topic << '\n';

    auto arm1_group =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm1");
    auto arm2_group =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm2");
    auto arm3_group =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm3");
    auto arm4_group =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm4");

    moveit::core::RobotStatePtr current_state = arm1_group->getCurrentState(5.0);
    if (current_state == nullptr) {
      throw std::runtime_error("Failed to read current MoveIt state");
    }

    const moveit::core::RobotModelConstPtr robot_model = arm1_group->getRobotModel();
    moveit::core::RobotState reference_state = *current_state;
    reference_state.update();

    const stop_n_go::base::RobotRegistry registry({
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
      {2, "arm3", "arm3_tool0"},
      {3, "arm4", "arm4_tool0"},
    });

    std::vector<stop_n_go::base::StateLayout::ConstPtr> layouts;
    layouts.reserve(registry.size());
    stop_n_go::core::ConflictChecker::LayoutMap layout_map;
    for (stop_n_go::base::RobotId robot_id = 0; robot_id < registry.size(); ++robot_id) {
      const auto layout = std::make_shared<stop_n_go::base::StateLayout>(
        *robot_model,
        registry.groupName(robot_id));
      layouts.push_back(layout);
      layout_map.emplace(robot_id, layout);
    }

    const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    const auto checker = std::make_shared<stop_n_go::core::ConflictChecker>(
      registry,
      robot_model,
      std::move(layout_map),
      planning_scene,
      reference_state);
    const auto resolver = std::make_shared<stop_n_go::core::ConflictResolver>(checker);
    resolver->setLogLevel(log_level);

    const stop_n_go::app::ConflictScenarioDataset dataset(registry, robot_model, dataset_path);
    const std::vector<std::size_t> & conflict_indices = dataset.conflictTrialIndices();
    if (conflict_indices.empty()) {
      throw std::runtime_error("Dataset contains no conflict trial indices");
    }

    // Selection uses conflict-sample indices, not original dataset trial indices.
    // conflict_indices[trial_index] gives the first original trial to solve.
    std::size_t conflict_sample_start = 0U;
    if (node->has_parameter("trial_index")) {
      conflict_sample_start = parseSizeParameter(node->get_parameter("trial_index"), "trial_index");
    }
    if (conflict_sample_start >= conflict_indices.size()) {
      throw std::out_of_range(
              "trial_index is outside conflict samples: " + std::to_string(conflict_sample_start));
    }

    // num_conflict_sample == 0 means solve all conflict samples from the start index to the end.
    std::size_t num_conflict_sample = 1U;
    if (node->has_parameter("num_conflict_sample")) {
      num_conflict_sample = parseSizeParameter(
        node->get_parameter("num_conflict_sample"),
        "num_conflict_sample");
    }

    const std::size_t conflict_sample_end = num_conflict_sample == 0U ?
      conflict_indices.size() :
      std::min(conflict_indices.size(), conflict_sample_start + num_conflict_sample);
    std::vector<std::size_t> trial_indices_to_solve;
    trial_indices_to_solve.reserve(conflict_sample_end - conflict_sample_start);
    for (std::size_t sample_index = conflict_sample_start; sample_index < conflict_sample_end;
      ++sample_index)
    {
      trial_indices_to_solve.push_back(conflict_indices[sample_index]);
    }

    std::cout << "[sng.dataset_demo] conflict_sample_start=" << conflict_sample_start << '\n';
    std::cout << "[sng.dataset_demo] num_conflict_sample=" << num_conflict_sample << '\n';
    std::cout << "[sng.dataset_demo] conflict_sample_end_exclusive=" << conflict_sample_end << '\n';
    const bool print_trajectory_summaries = trial_indices_to_solve.size() == 1U;

    stop_n_go::core::StopNGoSearchOptions search_options;
    search_options.synchronization_time_step = dataset.request().synchronization_time_step;
    search_options.max_expanded_nodes = max_expanded_nodes;
    search_options.log_level = log_level;
    const stop_n_go::core::StopNGoSearch search(
      checker,
      resolver,
      search_options);

    std::vector<TrialPlaybackRecord> playback_records;
    if (store_trajectories) {
      playback_records.reserve(trial_indices_to_solve.size());
    }
    std::size_t solved_count = 0U;
    std::size_t failed_count = 0U;
    std::size_t successful_expanded_node_total = 0U;
    double successful_search_time_total_ms = 0.0;
    double failed_search_time_total_ms = 0.0;
    std::size_t failed_search_time_count = 0U;
    for (std::size_t index = 0; index < trial_indices_to_solve.size(); ++index) {
      const std::size_t trial_index = trial_indices_to_solve[index];
      const std::size_t conflict_sample_index = conflict_sample_start + index;
      std::cout << "[sng.dataset_demo] solving trial " << index + 1U << "/"
                << trial_indices_to_solve.size()
                << " conflict_sample_index=" << conflict_sample_index
                << " trial_index=" << trial_index << '\n';
      if (!std::binary_search(conflict_indices.begin(), conflict_indices.end(), trial_index)) {
        throw std::invalid_argument(
                "Requested trial_index is not in conflict_trial_indices: " +
                std::to_string(trial_index));
      }

      std::optional<TrialPlaybackRecord> playback_record;
      try {
        const std::vector<stop_n_go::base::Trajectory> initial_trajectories =
          dataset.buildInitialTrajectories(trial_index);
        if (print_trajectory_summaries) {
          printTrajectorySetSummary("initial(raw)", initial_trajectories);
        }

        const std::vector<stop_n_go::base::Trajectory> synchronized_initial_trajectories =
          synchronizeAndEqualizeTrajectories(
          initial_trajectories,
          dataset.request().synchronization_time_step);
        if (print_trajectory_summaries) {
          printTrajectorySetSummary("initial(synchronized)", synchronized_initial_trajectories);
        }

        if (store_trajectories) {
          moveit_msgs::msg::DisplayTrajectory initial_display_trajectory = buildDisplayTrajectory(
            robot_model,
            reference_state,
            layouts,
            synchronized_initial_trajectories);
          playback_record.emplace();
          playback_record->conflict_sample_index = conflict_sample_index;
          playback_record->trial_index = trial_index;
          playback_record->initial_display_trajectory = std::move(initial_display_trajectory);
        }

        std::optional<stop_n_go::core::SearchNode> solved_node;
        double search_time_ms = 0.0;
        const auto search_start_time = std::chrono::steady_clock::now();
        try {
          solved_node.emplace(
            search.solve(
              initial_trajectories,
              stop_n_go::core::ConflictResolver::Mode::Basic));
          const auto search_end_time = std::chrono::steady_clock::now();
          search_time_ms = std::chrono::duration<double, std::milli>(
            search_end_time - search_start_time).count();
          successful_search_time_total_ms += search_time_ms;
        } catch (...) {
          const auto search_end_time = std::chrono::steady_clock::now();
          failed_search_time_total_ms += std::chrono::duration<double, std::milli>(
            search_end_time - search_start_time).count();
          ++failed_search_time_count;
          throw;
        }
        const stop_n_go::core::SearchNode & solved = *solved_node;

        if (store_trajectories && playback_record.has_value()) {
          playback_record->solved_display_trajectory = buildDisplayTrajectory(
            robot_model,
            reference_state,
            layouts,
            solved.trajectories());
          playback_record->solved = true;
          playback_records.push_back(std::move(*playback_record));
        }

        ++solved_count;
        const std::size_t expanded_nodes = search.lastExpandedNodes();
        successful_expanded_node_total += expanded_nodes;
        std::cout << "[sng.dataset_demo] solved trial_index=" << trial_index
                  << " makespan=" << solved.h() + solved.g()
                  << " expanded_nodes=" << expanded_nodes
                  << " search_time_ms=" << search_time_ms << '\n';
      } catch (const stop_n_go::core::StopNGoSearchFailure & exception) {
        ++failed_count;
        if (store_trajectories && playback_record.has_value()) {
          playback_records.push_back(std::move(*playback_record));
        }
        std::cerr << "[sng.dataset_demo] search_failed trial_index=" << trial_index
                  << " reason=" << exception.what() << '\n';
      } catch (const std::exception & exception) {
        ++failed_count;
        if (store_trajectories && playback_record.has_value()) {
          playback_records.push_back(std::move(*playback_record));
        }
        std::cerr << "[sng.dataset_demo] data_or_runtime_error trial_index=" << trial_index
                  << " error=" << exception.what() << '\n';
      }
    }

    std::cout << "[sng.dataset_demo] solve_result total=" << trial_indices_to_solve.size()
              << " solved=" << solved_count
              << " failed=" << failed_count << '\n';
    if (solved_count > 0U) {
      const double average_expanded_nodes =
        static_cast<double>(successful_expanded_node_total) / static_cast<double>(solved_count);
      std::cout << "[sng.dataset_demo] average expanded nodes when successfully solve="
                << average_expanded_nodes << '\n';
    } else {
      std::cout <<
        "[sng.dataset_demo] successful_solve_average_expanded_nodes=nan successful_solves=0"
                << '\n';
    }
    if (solved_count > 0U) {
      std::cout << "[sng.dataset_demo] average Stop-N-Go search time when solve succeeds="
                << successful_search_time_total_ms / static_cast<double>(solved_count)
                << " ms" << '\n';
    } else {
      std::cout << "[sng.dataset_demo] average Stop-N-Go search time when solve succeeds=nan ms"
                << '\n';
    }
    if (failed_search_time_count > 0U) {
      std::cout << "[sng.dataset_demo] average Stop-N-Go search time when solve fails="
                << failed_search_time_total_ms / static_cast<double>(failed_search_time_count)
                << " ms" << '\n';
    } else {
      std::cout << "[sng.dataset_demo] average Stop-N-Go search time when solve fails=nan ms"
                << '\n';
    }

    if (!store_trajectories) {
      std::cout << "[sng.dataset_demo] trajectory storage disabled; skipping playback" << '\n';
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 0;
    }

    for (int attempt = 0;
      attempt < 50 &&
      (initial_display_publisher->get_subscription_count() == 0 ||
      solved_display_publisher->get_subscription_count() == 0);
      ++attempt)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[sng.dataset_demo] initial RViz subscribers=" <<
      initial_display_publisher->get_subscription_count() << '\n';
    std::cout << "[sng.dataset_demo] solved RViz subscribers=" <<
      solved_display_publisher->get_subscription_count() << '\n';
    const moveit_msgs::msg::DisplayTrajectory empty_solved_display_trajectory =
      buildEmptyDisplayTrajectory(robot_model, reference_state);

    if (playback_records.empty()) {
      std::cout << "[sng.dataset_demo] no stored trajectories for playback" << '\n';
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 0;
    }

    printPlaybackRecords(playback_records);
    std::cout << "[sng.dataset_demo] publish a playback index with:" << '\n';
    std::cout << "ros2 topic pub --once " << playback_index_topic
              << " std_msgs/msg/UInt64 '{data: 0}'" << '\n';
    std::cout << "[sng.dataset_demo] selected playback trajectory will be republished at 1 Hz" <<
      '\n';
    std::cout << "[sng.dataset_demo] press Ctrl-C to exit" << '\n';

    std::optional<std::size_t> active_playback_index;
    auto playback_index_subscription = node->create_subscription<std_msgs::msg::UInt64>(
      playback_index_topic,
      10,
      [&, empty_solved_display_trajectory](const std_msgs::msg::UInt64::SharedPtr message) {
        const std::size_t playback_index = static_cast<std::size_t>(message->data);
        if (playback_index >= playback_records.size()) {
          publishPlaybackRecord(
            playback_records,
            playback_index,
            empty_solved_display_trajectory,
            initial_display_publisher,
            solved_display_publisher,
            true);
          return;
        }
        active_playback_index = playback_index;
        publishPlaybackRecord(
          playback_records,
          playback_index,
          empty_solved_display_trajectory,
          initial_display_publisher,
          solved_display_publisher,
          true);
      });

    auto playback_publish_timer = node->create_wall_timer(
      std::chrono::seconds(1),
      [&, empty_solved_display_trajectory]() {
        if (!active_playback_index.has_value()) {
          return;
        }
        publishPlaybackRecord(
          playback_records,
          *active_playback_index,
          empty_solved_display_trajectory,
          initial_display_publisher,
          solved_display_publisher,
          false);
      });
    (void)playback_index_subscription;
    (void)playback_publish_timer;

    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } catch (const std::exception & exception) {
    std::cerr << "[sng.dataset_demo] error: " << exception.what() << '\n';
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
