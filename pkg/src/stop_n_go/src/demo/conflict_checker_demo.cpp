#include <moveit/planning_scene/planning_scene.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#include <stop_n_go/base/robot_registry.hpp>
#include <stop_n_go/core/conflict_check.hpp>

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace
{

moveit::core::RobotModelPtr loadTrapezoidSceneModel()
{
  std::string urdf_string;
  if (!rdf_loader::RDFLoader::loadPkgFileToString(
      urdf_string, "trapezoid_ur_moveit_config", "config/trapezoid_ur7e_scene.urdf.xacro",
      {"sim_gazebo:=false"}))
  {
    throw std::runtime_error("Failed to load trapezoid scene URDF xacro");
  }

  std::string srdf_string;
  if (!rdf_loader::RDFLoader::loadPkgFileToString(
      srdf_string, "trapezoid_ur_moveit_config", "config/trapezoid_ur7e_scene.srdf", {}))
  {
    throw std::runtime_error("Failed to load trapezoid scene SRDF");
  }

  rdf_loader::RDFLoader loader(urdf_string, srdf_string);
  return std::make_shared<moveit::core::RobotModel>(loader.getURDF(), loader.getSRDF());
}

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

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const moveit::core::RobotModelPtr robot_model = loadTrapezoidSceneModel();
    const moveit::core::JointModelGroup * arm1_group = robot_model->getJointModelGroup("arm1");
    const moveit::core::JointModelGroup * arm2_group = robot_model->getJointModelGroup("arm2");
    if (arm1_group == nullptr || arm2_group == nullptr) {
      throw std::runtime_error("Failed to load arm1/arm2 planning groups");
    }

    const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
    });

    const auto arm1_layout = std::make_shared<stop_n_go::base::StateLayout>(*arm1_group);
    const auto arm2_layout = std::make_shared<stop_n_go::base::StateLayout>(*arm2_group);
    stop_n_go::core::ConflictChecker::LayoutMap layouts;
    layouts.emplace(0, arm1_layout);
    layouts.emplace(1, arm2_layout);

    moveit::core::RobotState reference_state(robot_model);
    reference_state.setToDefaultValues();
    reference_state.update();

    const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    const stop_n_go::core::ConflictChecker checker(
      registry,
      robot_model,
      std::move(layouts),
      planning_scene,
      reference_state);

    const stop_n_go::base::RobotState arm1_state({-3.566, 2.555, 1.382, 4.108, 1.470, -3.763});
    const stop_n_go::base::RobotState arm2_state({3.669, -3.365, 0.449, 5.114, 1.949, -1.633});

    std::cout << "[conflict_demo] checking a known colliding arm1/arm2 state pair" << '\n';
    std::cout << "[conflict_demo] arm1_state=[" << joinDoubles(arm1_state.values()) << "]" << '\n';
    std::cout << "[conflict_demo] arm2_state=[" << joinDoubles(arm2_state.values()) << "]" << '\n';
    std::cout << "[conflict_demo] inConflict="
              << std::boolalpha
              << checker.inConflict(0, arm1_state, 1, arm2_state)
              << '\n';
  } catch (const std::exception & exception) {
    std::cerr << "[conflict_demo] error: " << exception.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
