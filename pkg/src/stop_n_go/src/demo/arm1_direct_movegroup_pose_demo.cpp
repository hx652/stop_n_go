#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace
{

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

moveit_msgs::msg::DisplayTrajectory buildDisplayTrajectory(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const moveit::core::RobotState & reference_state)
{
  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  moveit::core::robotStateToRobotStateMsg(reference_state, display_trajectory.trajectory_start);
  display_trajectory.trajectory.push_back(plan.trajectory_);
  return display_trajectory;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "arm1_direct_movegroup_pose_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "/stop_n_go/display_planned_path", 10);

  std::thread spin_thread([node]() {rclcpp::spin(node);});

  try {
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm1");
    move_group.setPlanningTime(5.0);
    move_group.setStartStateToCurrentState();

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(5.0);
    if (current_state == nullptr) {
      throw std::runtime_error("Failed to get current robot state before direct MoveGroup test");
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "arm1_base_link";
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;
    goal.pose.position.x = 0.10;
    goal.pose.position.y = 0.25;
    goal.pose.position.z = 0.50;

    // Frame info
    std::cout << "[arm1.direct.demo] getPlanningFrame: " << move_group.getPlanningFrame() << "\n";
    std::cout << "[arm1.direct.demo] getPoseReferenceFrame: " << move_group.getPoseReferenceFrame() << "\n";
    std::cout << "[arm1.direct.demo] getEndEffectorLink: " << move_group.getEndEffectorLink() << "\n";
    std::cout << "[arm1.direct.demo] getEndEffector: " << move_group.getEndEffector() << "\n";

    std::cout << "[arm1.direct.demo] goal=" << poseString(goal) << '\n';

    const bool ik_feasible = move_group.setJointValueTarget(goal, "arm1_tool0");
    std::cout << "[arm1.direct.demo] setJointValueTarget(goal, arm1_tool0)="
              << std::boolalpha << ik_feasible << '\n';

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(goal, "arm1_tool0");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const moveit::core::MoveItErrorCode result = move_group.plan(plan);
    move_group.clearPoseTargets();

    std::cout << "[arm1.direct.demo] planning_success=" << static_cast<bool>(result) << '\n';
    if (!static_cast<bool>(result)) {
      throw std::runtime_error("MoveGroupInterface failed to plan to the direct pose target");
    }

    moveit::core::RobotState reference_state = *current_state;
    if (!moveit::core::robotStateMsgToRobotState(plan.start_state_, reference_state)) {
      reference_state = *current_state;
      reference_state.update();
    }

    const moveit_msgs::msg::DisplayTrajectory display_trajectory =
      buildDisplayTrajectory(plan, reference_state);
    display_publisher->publish(display_trajectory);

    auto display_timer = node->create_wall_timer(
      std::chrono::seconds(1),
      [display_publisher, display_trajectory]() {
        display_publisher->publish(display_trajectory);
      });
    (void)display_timer;

    std::cout << "[arm1.direct.demo] publishing display trajectory; press Ctrl-C to exit" << '\n';
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "[arm1.direct.demo] error: " << exception.what() << '\n';
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
}
