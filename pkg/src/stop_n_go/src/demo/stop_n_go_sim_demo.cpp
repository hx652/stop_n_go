#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <stop_n_go/base/conversion.hpp>
#include <stop_n_go/base/robot_registry.hpp>
#include <stop_n_go/core/conflict_check.hpp>
#include <stop_n_go/core/conflict_resolver.hpp>
#include <stop_n_go/core/stop_n_go_search.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{

std::string joinDoubles(const std::vector<double> & values)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  return stream.str();
}

void printTrajectory(const std::string & label, const stop_n_go::base::Trajectory & trajectory)
{
  std::cout << "[sng.demo] " << label
            << " waypoint_count=" << trajectory.size()
            << " total_duration=" << trajectory.totalDuration() << '\n';
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    std::cout << "[sng.demo] " << label
              << " waypoint=" << index
              << " values=[" << joinDoubles(trajectory.waypoint(index).values()) << "]"
              << " dt_prev=" << trajectory.durationFromPrevious(index)
              << " t_from_start=" << trajectory.durationFromStart(index) << '\n';
  }
}

stop_n_go::base::RobotState extractGroupState(
  const moveit::core::RobotState & reference_state,
  const stop_n_go::base::StateLayout & layout)
{
  stop_n_go::base::RobotState state(layout.size());
  for (std::size_t index = 0; index < layout.size(); ++index) {
    state[index] = reference_state.getVariablePosition(layout.variableNameAt(index));
  }
  return state;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "stop_n_go_sim_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto arm1_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/arm1_controller/joint_trajectory", 10);
  auto arm2_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/arm2_controller/joint_trajectory", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  try {
    moveit::planning_interface::MoveGroupInterface arm1_group(node, "arm1");
    moveit::planning_interface::MoveGroupInterface arm2_group(node, "arm2");
    moveit::core::RobotStatePtr current_state = arm1_group.getCurrentState(5.0);
    if (current_state == nullptr) {
      throw std::runtime_error("Failed to read current MoveIt state for stop_n_go demo");
    }

    const moveit::core::RobotModelConstPtr robot_model = arm1_group.getRobotModel();
    moveit::core::RobotState reference_state = *current_state;
    reference_state.update();

    const auto arm1_layout = std::make_shared<stop_n_go::base::StateLayout>(*robot_model, "arm1");
    const auto arm2_layout = std::make_shared<stop_n_go::base::StateLayout>(*robot_model, "arm2");

    const stop_n_go::base::RobotState arm1_safe = extractGroupState(reference_state, *arm1_layout);
    const stop_n_go::base::RobotState arm2_safe = extractGroupState(reference_state, *arm2_layout);
    const stop_n_go::base::RobotState arm1_colliding({-3.566, 2.555, 1.382, 4.108, 1.470, -3.763});
    const stop_n_go::base::RobotState arm2_colliding({3.669, -3.365, 0.449, 5.114, 1.949, -1.633});

    stop_n_go::base::Trajectory arm1_trajectory(arm1_layout);
    stop_n_go::base::Trajectory arm2_trajectory(arm2_layout);
    arm1_trajectory.append(arm1_safe, 0.0);
    arm1_trajectory.append(arm1_colliding, 0.2);
    arm1_trajectory.append(arm1_safe, 0.2);
    arm2_trajectory.append(arm2_safe, 0.0);
    arm2_trajectory.append(arm2_colliding, 0.2);
    arm2_trajectory.append(arm2_safe, 0.2);

    std::cout << "[sng.demo] initial trajectories" << '\n';
    printTrajectory("arm1.initial", arm1_trajectory);
    printTrajectory("arm2.initial", arm2_trajectory);

    const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
    });
    stop_n_go::core::ConflictChecker::LayoutMap layouts;
    layouts.emplace(0, arm1_layout);
    layouts.emplace(1, arm2_layout);
    const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    const auto checker = std::make_shared<stop_n_go::core::ConflictChecker>(
      registry, robot_model, std::move(layouts), planning_scene, reference_state);
    const auto resolver = std::make_shared<stop_n_go::core::ConflictResolver>(checker);
    const stop_n_go::core::StopNGoSearch search(checker, resolver, 0.2);

    const stop_n_go::core::SearchNode solved = search.solve(
      {arm1_trajectory, arm2_trajectory},
      stop_n_go::core::ConflictResolver::Mode::Basic);

    std::cout << "[sng.demo] solved trajectories" << '\n';
    printTrajectory("arm1.solved", solved.trajectory(0));
    printTrajectory("arm2.solved", solved.trajectory(1));

    const stop_n_go::base::ConversionContext arm1_conversion(robot_model, arm1_layout,
      reference_state);
    const stop_n_go::base::ConversionContext arm2_conversion(robot_model, arm2_layout,
      reference_state);
    const trajectory_msgs::msg::JointTrajectory arm1_msg =
      arm1_conversion.toJointTrajectoryMsg(solved.trajectory(0));
    const trajectory_msgs::msg::JointTrajectory arm2_msg =
      arm2_conversion.toJointTrajectoryMsg(solved.trajectory(1));

    for (int attempt = 0;
      attempt < 50 &&
      (arm1_publisher->get_subscription_count() == 0 ||
      arm2_publisher->get_subscription_count() == 0);
      ++attempt)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[sng.demo] arm1 controller subscribers=" <<
      arm1_publisher->get_subscription_count() << '\n';
    std::cout << "[sng.demo] arm2 controller subscribers=" <<
      arm2_publisher->get_subscription_count() << '\n';

    arm1_publisher->publish(arm1_msg);
    arm2_publisher->publish(arm2_msg);
    std::cout << "[sng.demo] published resolved trajectories to arm1 and arm2 controllers" << '\n';

    const double wait_duration = std::max(
      solved.trajectory(0).totalDuration(), solved.trajectory(1).totalDuration()) + 2.0;
    std::cout << "[sng.demo] waiting " << wait_duration << " seconds for execution" << '\n';
    std::this_thread::sleep_for(std::chrono::duration<double>(wait_duration));
  } catch (const std::exception & exception) {
    std::cerr << "[sng.demo] error: " << exception.what() << '\n';
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
