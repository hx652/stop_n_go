#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <stop_n_go/base/conversion.hpp>

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
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

std::string joinStrings(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  return stream.str();
}

void printStopTrajectory(
  const std::string & label,
  const stop_n_go::base::Trajectory & trajectory)
{
  std::cout << "[demo] " << label << " waypoint_count=" << trajectory.size()
            << " total_duration=" << trajectory.totalDuration() << '\n';
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    std::cout << "[demo] " << label << " waypoint " << index
              << " values=[" << joinDoubles(trajectory.waypoint(index).values()) << "]"
              << " dt_prev=" << trajectory.durationFromPrevious(index)
              << " t_from_start=" << trajectory.durationFromStart(index) << '\n';
  }
}

void printJointTrajectoryMsg(const trajectory_msgs::msg::JointTrajectory & trajectory_msg)
{
  std::cout << "[demo] joint_trajectory joint_names=[" << joinStrings(trajectory_msg.joint_names) <<
    "]"
            << '\n';
  for (std::size_t index = 0; index < trajectory_msg.points.size(); ++index) {
    const trajectory_msgs::msg::JointTrajectoryPoint & point = trajectory_msg.points[index];
    const double time_from_start = static_cast<double>(point.time_from_start.sec) +
      static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
    std::cout << "[demo] joint_trajectory point " << index
              << " positions=[" << joinDoubles(point.positions) << "]"
              << " time_from_start=" << time_from_start << '\n';
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "plan_resample_execute_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Keep the demo fixed for now so it can run without extra ROS parameter wiring.
  const std::string group_name = "arm1";
  const std::string controller_topic = "/arm1_controller/joint_trajectory";
  const double resample_dt = 0.3;
  const double planning_time = 5.0;
  const double wait_after_publish = 2.0;
  const std::vector<double> target_offsets = {0.25, -0.50, 0.70, -0.40, 0.20, 0.00};

  auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    controller_topic,
    10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  try {
    moveit::planning_interface::MoveGroupInterface move_group(node, group_name);
    move_group.setPlanningTime(planning_time);
    move_group.setStartStateToCurrentState();

    std::cout << "[demo] planning group=" << move_group.getName() << '\n';
    std::cout << "[demo] active_joints=[" << joinStrings(move_group.getActiveJoints()) << "]" <<
      '\n';

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(5.0);
    if (current_state == nullptr) {
      throw std::runtime_error("Failed to get current robot state from move_group");
    }

    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    if (current_joint_values.size() != target_offsets.size()) {
      throw std::invalid_argument("target_offsets size must match the group's active variable count");
    }

    std::vector<double> target_joint_values = current_joint_values;
    for (std::size_t index = 0; index < target_joint_values.size(); ++index) {
      target_joint_values[index] += target_offsets[index];
    }

    std::cout << "[demo] current_joint_values=[" << joinDoubles(current_joint_values) << "]" <<
      '\n';
    std::cout << "[demo] target_joint_values=[" << joinDoubles(target_joint_values) << "]" << '\n';

    if (!move_group.setJointValueTarget(target_joint_values)) {
      throw std::runtime_error("MoveGroupInterface rejected the target joint values");
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const moveit::core::MoveItErrorCode planning_result = move_group.plan(plan);
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      throw std::runtime_error("MoveGroupInterface failed to plan a trajectory");
    }

    std::cout << "[demo] planning succeeded, planning_time=" << plan.planning_time_ << '\n';
    std::cout << "[demo] moveit joint trajectory points="
              << plan.trajectory_.joint_trajectory.points.size() << '\n';

    const moveit::core::RobotModelConstPtr robot_model = move_group.getRobotModel();
    moveit::core::RobotState reference_state(robot_model);
    if (!moveit::core::robotStateMsgToRobotState(plan.start_state_, reference_state)) {
      reference_state = *current_state;
      reference_state.update();
      std::cout << "[demo] plan.start_state_ conversion failed, falling back to current state" <<
        '\n';
    }

    const stop_n_go::base::StateLayout::ConstPtr layout =
      std::make_shared<stop_n_go::base::StateLayout>(*robot_model, group_name);
    const stop_n_go::base::ConversionContext conversion_context(robot_model, layout,
      reference_state);

    robot_trajectory::RobotTrajectory moveit_trajectory(robot_model, group_name);
    moveit_trajectory.setRobotTrajectoryMsg(reference_state, plan.trajectory_);

    stop_n_go::base::Trajectory stop_trajectory =
      conversion_context.fromMoveItTrajectory(moveit_trajectory);
    printStopTrajectory("stop_n_go(original)", stop_trajectory);

    stop_n_go::base::Trajectory resampled_trajectory = stop_trajectory.resample(resample_dt);
    printStopTrajectory("stop_n_go(resampled)", resampled_trajectory);

    trajectory_msgs::msg::JointTrajectory trajectory_msg =
      conversion_context.toJointTrajectoryMsg(resampled_trajectory);
    printJointTrajectoryMsg(trajectory_msg);

    std::cout << "[demo] waiting for controller subscriber on topic " << controller_topic << '\n';
    for (int attempt = 0; attempt < 50 && publisher->get_subscription_count() == 0; ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[demo] controller subscriber count=" << publisher->get_subscription_count() <<
      '\n';

    publisher->publish(trajectory_msg);
    std::cout << "[demo] published resampled trajectory message" << '\n';

    const double wait_duration = resampled_trajectory.totalDuration() + wait_after_publish;
    std::cout << "[demo] waiting " << wait_duration << " seconds so motion stays observable" <<
      '\n';
    std::this_thread::sleep_for(std::chrono::duration<double>(wait_duration));
  } catch (const std::exception & exception) {
    std::cerr << "[demo] error: " << exception.what() << '\n';
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
