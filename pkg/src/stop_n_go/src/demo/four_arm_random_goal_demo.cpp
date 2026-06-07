#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <stop_n_go/base/conversion.hpp>
#include <stop_n_go/base/initial_planner.hpp>
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

// ── 常量 ────────────────────────────────────────────────────────────────────
constexpr double kSyncTimeStep = 0.2;      // stop_n_go 同步步长 (s)
constexpr int kMaxPlanAttempts = 5;        // 随机目标规划最大重试次数
constexpr double kStateWaitTimeout = 5.0;  // 等待 MoveIt 当前状态超时 (s)

// ── 调试打印 ─────────────────────────────────────────────────────────────────
std::string joinDoubles(const std::vector<double> & v)
{
  std::ostringstream s;
  s << std::fixed << std::setprecision(3);
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i > 0) {
      s << ", ";
    }
    s << v[i];
  }
  return s.str();
}

void printTrajectory(const std::string & label, const stop_n_go::base::Trajectory & traj)
{
  std::cout << "[4arm.demo] " << label
            << " waypoints=" << traj.size()
            << " total_duration=" << traj.totalDuration() << '\n';
  for (std::size_t i = 0; i < traj.size(); ++i) {
    std::cout << "[4arm.demo]   [" << i << "] "
              << "values=[" << joinDoubles(traj.waypoint(i).values()) << "] "
              << "dt=" << traj.durationFromPrevious(i) << '\n';
  }
}

// ── 带重试的随机目标规划封装 ──────────────────────────────────────────────────
stop_n_go::base::Trajectory planToRandomGoalWithRetry(
  stop_n_go::base::InitialPlanner & planner,
  stop_n_go::base::RobotId robot_id,
  const std::string & label)
{
  for (int attempt = 1; attempt <= kMaxPlanAttempts; ++attempt) {
    try {
      std::cout << "[4arm.demo] " << label
                << " planning attempt " << attempt << "/" << kMaxPlanAttempts << '\n';
      return planner.computeInitialTrajectoryToRandomJointGoal(robot_id);
    } catch (const std::runtime_error & e) {
      std::cerr << "[4arm.demo] " << label << " attempt " << attempt
                << " failed: " << e.what() << '\n';
      if (attempt == kMaxPlanAttempts) {
        throw;
      }
    }
  }
  throw std::runtime_error("unreachable");
}

// ── 合并多条 JointTrajectory 为一条复合轨迹 ──────────────────────────────────
// 要求所有输入轨迹点数相同（stop_n_go::solve 后由 equalizeTrajectoryLengths 保证）。
// 合并结果：joint_names 拼接，每个点的 positions 拼接，time_from_start 取第一条的值。
trajectory_msgs::msg::JointTrajectory mergeJointTrajectories(
  const std::vector<trajectory_msgs::msg::JointTrajectory> & trajectories)
{
  if (trajectories.empty()) {
    throw std::invalid_argument("mergeJointTrajectories: empty input");
  }

  const std::size_t n_points = trajectories.front().points.size();
  for (const auto & traj : trajectories) {
    if (traj.points.size() != n_points) {
      throw std::runtime_error(
        "mergeJointTrajectories: trajectories have different point counts");
    }
  }

  trajectory_msgs::msg::JointTrajectory merged;
  merged.header = trajectories.front().header;

  // 拼接 joint_names
  for (const auto & traj : trajectories) {
    merged.joint_names.insert(
      merged.joint_names.end(),
      traj.joint_names.begin(),
      traj.joint_names.end());
  }

  // 逐点拼接 positions，time_from_start 取第一条轨迹的值
  merged.points.resize(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    merged.points[i].time_from_start = trajectories.front().points[i].time_from_start;
    for (const auto & traj : trajectories) {
      const auto & src = traj.points[i].positions;
      merged.points[i].positions.insert(
        merged.points[i].positions.end(), src.begin(), src.end());
    }
  }

  return merged;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "four_arm_random_goal_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // ── RViz 可视化发布器（/display_planned_path 是 MotionPlanning 插件的标准 topic）
  // QoS depth=10，volatile durability，与 RViz MotionPlanningDisplay 默认配置匹配。
  // 采用 1 Hz 循环发布，保证 RViz 任意时刻打开均能收到最新轨迹。
  auto display_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "/stop_n_go/display_planned_path", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  try {
    // ── MoveGroupInterface（每臂独立）────────────────────────────────────────
    auto mg1 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm1");
    auto mg2 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm2");
    auto mg3 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm3");
    auto mg4 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm4");

    // ── 共享机器人模型 & 参考状态 ────────────────────────────────────────────
    moveit::core::RobotStatePtr current_state = mg1->getCurrentState(kStateWaitTimeout);
    if (current_state == nullptr) {
      throw std::runtime_error("Failed to read current MoveIt state");
    }
    const moveit::core::RobotModelConstPtr robot_model = mg1->getRobotModel();
    moveit::core::RobotState reference_state = *current_state;
    reference_state.update();

    // ── StateLayout（每臂一个）──────────────────────────────────────────────
    auto layout1 = std::make_shared<stop_n_go::base::StateLayout>(*robot_model, "arm1");
    auto layout2 = std::make_shared<stop_n_go::base::StateLayout>(*robot_model, "arm2");
    auto layout3 = std::make_shared<stop_n_go::base::StateLayout>(*robot_model, "arm3");
    auto layout4 = std::make_shared<stop_n_go::base::StateLayout>(*robot_model, "arm4");

    // ── RobotRegistry ────────────────────────────────────────────────────────
    const stop_n_go::base::RobotRegistry registry({
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
      {2, "arm3", "arm3_tool0"},
      {3, "arm4", "arm4_tool0"},
    });

    // ── InitialPlanner ───────────────────────────────────────────────────────
    stop_n_go::base::InitialPlanner::MoveGroupMap move_groups;
    move_groups.emplace(0, mg1);
    move_groups.emplace(1, mg2);
    move_groups.emplace(2, mg3);
    move_groups.emplace(3, mg4);
    stop_n_go::base::InitialPlanner planner(registry, robot_model, std::move(move_groups));

    // ── 为各臂生成随机目标初始轨迹 ──────────────────────────────────────────
    std::cout << "[4arm.demo] generating initial trajectories to random joint goals\n";
    stop_n_go::base::Trajectory traj1 = planToRandomGoalWithRetry(planner, 0, "arm1");
    stop_n_go::base::Trajectory traj2 = planToRandomGoalWithRetry(planner, 1, "arm2");
    stop_n_go::base::Trajectory traj3 = planToRandomGoalWithRetry(planner, 2, "arm3");
    stop_n_go::base::Trajectory traj4 = planToRandomGoalWithRetry(planner, 3, "arm4");

    std::cout << "[4arm.demo] --- initial trajectories ---\n";
    printTrajectory("arm1.initial", traj1);
    printTrajectory("arm2.initial", traj2);
    printTrajectory("arm3.initial", traj3);
    printTrajectory("arm4.initial", traj4);

    // ── ConflictChecker & ConflictResolver ──────────────────────────────────
    stop_n_go::core::ConflictChecker::LayoutMap layouts;
    layouts.emplace(0, layout1);
    layouts.emplace(1, layout2);
    layouts.emplace(2, layout3);
    layouts.emplace(3, layout4);

    const auto planning_scene =
      std::make_shared<planning_scene::PlanningScene>(robot_model);
    const auto checker = std::make_shared<stop_n_go::core::ConflictChecker>(
      registry, robot_model, std::move(layouts), planning_scene, reference_state);
    const auto resolver = std::make_shared<stop_n_go::core::ConflictResolver>(checker);
    const stop_n_go::core::StopNGoSearch search(checker, resolver, kSyncTimeStep);

    // ── stop_n_go::solve ────────────────────────────────────────────────────
    std::cout << "[4arm.demo] running stop_n_go search\n";
    const stop_n_go::core::SearchNode solved = search.solve(
      {traj1, traj2, traj3, traj4},
      stop_n_go::core::ConflictResolver::Mode::Basic);

    std::cout << "[4arm.demo] --- solved trajectories ---\n";
    printTrajectory("arm1.solved", solved.trajectory(0));
    printTrajectory("arm2.solved", solved.trajectory(1));
    printTrajectory("arm3.solved", solved.trajectory(2));
    printTrajectory("arm4.solved", solved.trajectory(3));

    // ── 各臂分别转换，合并为单一复合 JointTrajectory ─────────────────────────
    const stop_n_go::base::ConversionContext conv1(robot_model, layout1, reference_state);
    const stop_n_go::base::ConversionContext conv2(robot_model, layout2, reference_state);
    const stop_n_go::base::ConversionContext conv3(robot_model, layout3, reference_state);
    const stop_n_go::base::ConversionContext conv4(robot_model, layout4, reference_state);

    const trajectory_msgs::msg::JointTrajectory combined = mergeJointTrajectories({
      conv1.toJointTrajectoryMsg(solved.trajectory(0)),
      conv2.toJointTrajectoryMsg(solved.trajectory(1)),
      conv3.toJointTrajectoryMsg(solved.trajectory(2)),
      conv4.toJointTrajectoryMsg(solved.trajectory(3)),
    });

    std::cout << "[4arm.demo] combined trajectory: "
              << combined.joint_names.size() << " joints, "
              << combined.points.size() << " points\n";

    // ── 构建 DisplayTrajectory 消息 ──────────────────────────────────────────
    // trajectory_start：使 RViz 从当前状态开始渲染动画
    // model_id：RViz 用于匹配机器人模型
    moveit_msgs::msg::DisplayTrajectory display_msg;
    display_msg.model_id = robot_model->getName();
    moveit::core::robotStateToRobotStateMsg(reference_state, display_msg.trajectory_start);

    moveit_msgs::msg::RobotTrajectory robot_traj_msg;
    robot_traj_msg.joint_trajectory = combined;
    display_msg.trajectory.push_back(robot_traj_msg);

    // ── 以 1 Hz 循环发布，保证 RViz 任意时刻打开均能收到 ────────────────────
    std::cout << "[4arm.demo] publishing on /stop_n_go/display_planned_path at 1 Hz"
              << " — Ctrl-C to exit\n";
    while (rclcpp::ok()) {
      display_pub->publish(display_msg);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

  } catch (const std::exception & e) {
    std::cerr << "[4arm.demo] error: " << e.what() << '\n';
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
