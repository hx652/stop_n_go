#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stop_n_go/base/conversion.hpp>
#include <stop_n_go/base/initial_planner.hpp>
#include <stop_n_go/base/robot_registry.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

std::string poseString(const geometry_msgs::msg::Pose & pose)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "pos=(" << pose.position.x << ", " << pose.position.y << ", "
         << pose.position.z << ")"
         << " quat=(" << pose.orientation.x << ", " << pose.orientation.y << ", "
         << pose.orientation.z << ", " << pose.orientation.w << ")";
  return stream.str();
}

moveit_msgs::msg::DisplayTrajectory buildDisplayTrajectory(
  const stop_n_go::base::Trajectory & trajectory,
  const stop_n_go::base::StateLayout & layout,
  const moveit::core::RobotState & reference_state)
{
  robot_trajectory::RobotTrajectory full_trajectory(reference_state.getRobotModel(), nullptr);
  for (std::size_t waypoint_index = 0; waypoint_index < trajectory.size(); ++waypoint_index) {
    moveit::core::RobotState waypoint_state(reference_state);
    waypoint_state.setJointGroupActivePositions(
      layout.jointModelGroup(),
      trajectory.waypoint(waypoint_index).values());
    waypoint_state.update();
    full_trajectory.addSuffixWayPoint(
      waypoint_state,
      trajectory.durationFromPrevious(waypoint_index));
  }

  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  moveit::core::robotStateToRobotStateMsg(reference_state, display_trajectory.trajectory_start);
  moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
  full_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);
  display_trajectory.trajectory.push_back(robot_trajectory_msg);
  return display_trajectory;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "arm1_initial_planner_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "/stop_n_go/display_planned_path", 10);

  std::thread spin_thread([node]() {rclcpp::spin(node);});

  try {
    const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
    });

    stop_n_go::base::InitialPlanner::MoveGroupMap move_groups;
    move_groups.emplace(
      0,
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm1"));

    const moveit::core::RobotModelConstPtr robot_model = move_groups.at(0)->getRobotModel();
    moveit::core::RobotStatePtr current_state = move_groups.at(0)->getCurrentState(5.0);
    if (current_state == nullptr) {
      throw std::runtime_error("Failed to read current MoveIt state for arm1 initial planner demo");
    }

    moveit::core::RobotState reference_state = *current_state;
    reference_state.update();

    stop_n_go::base::InitialPlanner initial_planner(registry, robot_model, move_groups);
    const auto layout = std::make_shared<stop_n_go::base::StateLayout>(*robot_model, "arm1");

    const geometry_msgs::msg::PoseStamped current_ee_pose =
      move_groups.at(0)->getCurrentPose(registry.endEffectorLink(0));

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "arm1_base_link";
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;
    goal.pose.position.x = 0.10;
    goal.pose.position.y = 0.25;
    goal.pose.position.z = 0.50;

    std::cout << "[arm1.initial.demo] current_ee_pose=" << poseString(current_ee_pose.pose) << '\n';
    std::cout << "[arm1.initial.demo] world_goal=" << poseString(goal.pose) << '\n';

    // Run a direct MoveIt IK feasibility check first, but do not gate planning on the result.
    const bool ik_feasible = move_groups.at(0)->setJointValueTarget(goal, registry.endEffectorLink(0));
    std::cout << "[arm1.initial.demo] ik_feasible=" << std::boolalpha << ik_feasible << '\n';

    const stop_n_go::base::Trajectory trajectory = initial_planner.computeInitialTrajectory(0, goal);
    std::cout << "[arm1.initial.demo] trajectory waypoint_count=" << trajectory.size()
              << " total_duration=" << trajectory.totalDuration() << '\n';

    const moveit_msgs::msg::DisplayTrajectory display_trajectory =
      buildDisplayTrajectory(trajectory, *layout, reference_state);
    display_publisher->publish(display_trajectory);

    auto display_timer = node->create_wall_timer(
      std::chrono::seconds(1),
      [display_publisher, display_trajectory]() {
        display_publisher->publish(display_trajectory);
      });
    (void)display_timer;

    std::cout << "[arm1.initial.demo] publishing display trajectory; press Ctrl-C to exit" << '\n';
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "[arm1.initial.demo] error: " << exception.what() << '\n';
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
}
