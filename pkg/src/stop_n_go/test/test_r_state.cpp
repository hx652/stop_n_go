#include <gtest/gtest.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <stop_n_go/base/conversion.hpp>
#include <stop_n_go/base/robot_registry.hpp>
#include <stop_n_go/core/conflict_check.hpp>
#include <stop_n_go/core/conflict_result.hpp>
#include <stop_n_go/core/conflict_resolver.hpp>
#include <stop_n_go/core/search_node.hpp>
#include <stop_n_go/core/stop_n_go_search.hpp>
#include <stop_n_go/base/r_state.hpp>
#include <stop_n_go/base/trajectory.hpp>

#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

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

std::string joinDoubles(const std::vector<double> & values)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(2);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  return stream.str();
}

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

class RobotStateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = loadTrapezoidSceneModel();
    ASSERT_NE(robot_model_, nullptr);

    arm1_group_ = robot_model_->getJointModelGroup("arm1");
    ASSERT_NE(arm1_group_, nullptr);
  }

  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup * arm1_group_ = nullptr;
};

TEST(RobotRegistryTest, StoresDescriptorsAndResolvesGroupMetadata)
{
  const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
    });

  EXPECT_EQ(registry.size(), 2U);
  EXPECT_FALSE(registry.empty());
  EXPECT_TRUE(registry.hasRobot(0));
  EXPECT_TRUE(registry.hasRobot(1));
  EXPECT_FALSE(registry.hasRobot(2));
  EXPECT_EQ(registry.groupName(0), "arm1");
  EXPECT_EQ(registry.endEffectorLink(1), "arm2_tool0");
}

TEST(RobotRegistryTest, RejectsDuplicateRobotIds)
{
  EXPECT_THROW(
    stop_n_go::base::RobotRegistry(
    {
      {0, "arm1", "arm1_tool0"},
      {0, "arm2", "arm2_tool0"},
    }),
    std::invalid_argument);
}

TEST_F(RobotStateTest, SearchNodeStoresTrajectorySetAndSearchMetadata)
{
  const stop_n_go::base::StateLayout::ConstPtr layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  stop_n_go::base::Trajectory first(layout);
  stop_n_go::base::Trajectory second(layout);
  first.append(stop_n_go::base::RobotState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), 0.0);
  second.append(stop_n_go::base::RobotState({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}), 0.0);

  stop_n_go::core::SearchNode node({first, second});
  node.setG(1.5);
  node.setH(2.5);
  node.setTS(3);

  EXPECT_EQ(node.robotCount(), 2U);
  EXPECT_FALSE(node.empty());
  EXPECT_DOUBLE_EQ(node.g(), 1.5);
  EXPECT_DOUBLE_EQ(node.h(), 2.5);
  EXPECT_DOUBLE_EQ(node.f(), 4.0);
  EXPECT_EQ(node.tS(), 3U);
  EXPECT_EQ(
    node.trajectory(0).waypoint(0).values(),
    std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_EQ(
    node.trajectory(1).waypoint(0).values(),
    std::vector<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
}

TEST(ConflictResultTest, StoresRobotPairAndConflictTimeIndex)
{
  const stop_n_go::core::ConflictResult conflict(1, 3, 7);

  EXPECT_EQ(conflict.firstRobot(), 1U);
  EXPECT_EQ(conflict.secondRobot(), 3U);
  EXPECT_EQ(conflict.timeIndex(), 7U);
}

TEST_F(RobotStateTest, ConflictCheckerReturnsNoConflictForDefaultSeparatedStates)
{
  const stop_n_go::base::StateLayout::ConstPtr arm1_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  const moveit::core::JointModelGroup * arm2_group = robot_model_->getJointModelGroup("arm2");
  ASSERT_NE(arm2_group, nullptr);
  const stop_n_go::base::StateLayout::ConstPtr arm2_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm2_group);

  moveit::core::RobotState reference_state(robot_model_);
  reference_state.setToDefaultValues();
  reference_state.update();

  stop_n_go::base::RobotState arm1_state(arm1_layout->size());
  stop_n_go::base::RobotState arm2_state(arm2_layout->size());
  for (std::size_t index = 0; index < arm1_layout->size(); ++index) {
    arm1_state[index] = reference_state.getVariablePosition(arm1_layout->variableNameAt(index));
  }
  for (std::size_t index = 0; index < arm2_layout->size(); ++index) {
    arm2_state[index] = reference_state.getVariablePosition(arm2_layout->variableNameAt(index));
  }

  const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
    });
  stop_n_go::core::ConflictChecker::LayoutMap layouts;
  layouts.emplace(0, arm1_layout);
  layouts.emplace(1, arm2_layout);
  const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  const stop_n_go::core::ConflictChecker checker(
    registry,
    robot_model_,
    std::move(layouts),
    planning_scene,
    reference_state);

  EXPECT_FALSE(checker.inConflict(0, arm1_state, 1, arm2_state));
}

TEST_F(RobotStateTest, ConflictCheckerDetectsKnownCollidingStates)
{
  const stop_n_go::base::StateLayout::ConstPtr arm1_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  const moveit::core::JointModelGroup * arm2_group = robot_model_->getJointModelGroup("arm2");
  ASSERT_NE(arm2_group, nullptr);
  const stop_n_go::base::StateLayout::ConstPtr arm2_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm2_group);

  moveit::core::RobotState reference_state(robot_model_);
  reference_state.setToDefaultValues();
  reference_state.update();

  const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
    });
  stop_n_go::core::ConflictChecker::LayoutMap layouts;
  layouts.emplace(0, arm1_layout);
  layouts.emplace(1, arm2_layout);
  const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  const stop_n_go::core::ConflictChecker checker(
    registry,
    robot_model_,
    std::move(layouts),
    planning_scene,
    reference_state);

  const stop_n_go::base::RobotState arm1_state({-3.566, 2.555, 1.382, 4.108, 1.470, -3.763});
  const stop_n_go::base::RobotState arm2_state({3.669, -3.365, 0.449, 5.114, 1.949, -1.633});

  std::cout << "[conflict_checker] arm1_colliding_state=[" << joinDoubles(arm1_state.values()) <<
    "]" << '\n';
  std::cout << "[conflict_checker] arm2_colliding_state=[" << joinDoubles(arm2_state.values()) <<
    "]" << '\n';

  EXPECT_TRUE(checker.inConflict(0, arm1_state, 1, arm2_state));
}

TEST_F(RobotStateTest, ConflictCheckerSearchNodeCheckReturnsNoConflictAtGivenTimeStep)
{
  const stop_n_go::base::StateLayout::ConstPtr arm1_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  const moveit::core::JointModelGroup * arm2_group = robot_model_->getJointModelGroup("arm2");
  ASSERT_NE(arm2_group, nullptr);
  const stop_n_go::base::StateLayout::ConstPtr arm2_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm2_group);

  moveit::core::RobotState reference_state(robot_model_);
  reference_state.setToDefaultValues();
  reference_state.update();

  stop_n_go::base::RobotState arm1_state(arm1_layout->size());
  stop_n_go::base::RobotState arm2_state(arm2_layout->size());
  for (std::size_t index = 0; index < arm1_layout->size(); ++index) {
    arm1_state[index] = reference_state.getVariablePosition(arm1_layout->variableNameAt(index));
  }
  for (std::size_t index = 0; index < arm2_layout->size(); ++index) {
    arm2_state[index] = reference_state.getVariablePosition(arm2_layout->variableNameAt(index));
  }

  stop_n_go::base::Trajectory first(arm1_layout);
  stop_n_go::base::Trajectory second(arm2_layout);
  first.append(arm1_state, 0.0);
  second.append(arm2_state, 0.0);

  const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
    });
  stop_n_go::core::ConflictChecker::LayoutMap layouts;
  layouts.emplace(0, arm1_layout);
  layouts.emplace(1, arm2_layout);
  const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  const stop_n_go::core::ConflictChecker checker(
    registry,
    robot_model_,
    std::move(layouts),
    planning_scene,
    reference_state);

  const stop_n_go::core::SearchNode node({first, second});
  const std::optional<stop_n_go::core::ConflictResult> conflict = checker.conflictCheck(node, 0);

  EXPECT_FALSE(conflict.has_value());
}

TEST_F(RobotStateTest, ConflictResolverSearchPauseStepSkipsRepeatedConflictState)
{
  const stop_n_go::base::StateLayout::ConstPtr arm1_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  const moveit::core::JointModelGroup * arm2_group = robot_model_->getJointModelGroup("arm2");
  ASSERT_NE(arm2_group, nullptr);
  const stop_n_go::base::StateLayout::ConstPtr arm2_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm2_group);

  moveit::core::RobotState reference_state(robot_model_);
  reference_state.setToDefaultValues();
  reference_state.update();

  const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
    });
  stop_n_go::core::ConflictChecker::LayoutMap layouts;
  layouts.emplace(0, arm1_layout);
  layouts.emplace(1, arm2_layout);
  const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  const auto checker = std::make_shared<stop_n_go::core::ConflictChecker>(
    registry,
    robot_model_,
    std::move(layouts),
    planning_scene,
    reference_state);
  const stop_n_go::core::ConflictResolver resolver(checker);

  stop_n_go::base::Trajectory trajectory(arm1_layout);
  trajectory.append(stop_n_go::base::RobotState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), 0.0);
  trajectory.append(stop_n_go::base::RobotState({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}), 0.2);
  trajectory.append(stop_n_go::base::RobotState({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}), 0.2);
  trajectory.append(stop_n_go::base::RobotState({2.0, 2.0, 2.0, 2.0, 2.0, 2.0}), 0.2);

  EXPECT_EQ(resolver.searchPauseStep(trajectory, 2, 1), 0U);
}

TEST_F(RobotStateTest, ConflictResolverInsertPauseDuplicatesHoldStateAndShiftsTail)
{
  const stop_n_go::base::StateLayout::ConstPtr arm1_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  const moveit::core::JointModelGroup * arm2_group = robot_model_->getJointModelGroup("arm2");
  ASSERT_NE(arm2_group, nullptr);
  const stop_n_go::base::StateLayout::ConstPtr arm2_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm2_group);

  moveit::core::RobotState reference_state(robot_model_);
  reference_state.setToDefaultValues();
  reference_state.update();

  const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
    });
  stop_n_go::core::ConflictChecker::LayoutMap layouts;
  layouts.emplace(0, arm1_layout);
  layouts.emplace(1, arm2_layout);
  const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  const auto checker = std::make_shared<stop_n_go::core::ConflictChecker>(
    registry,
    robot_model_,
    std::move(layouts),
    planning_scene,
    reference_state);
  const stop_n_go::core::ConflictResolver resolver(checker);

  stop_n_go::base::Trajectory trajectory(arm1_layout);
  trajectory.append(stop_n_go::base::RobotState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), 0.0);
  trajectory.append(stop_n_go::base::RobotState({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}), 0.2);
  trajectory.append(stop_n_go::base::RobotState({2.0, 2.0, 2.0, 2.0, 2.0, 2.0}), 0.2);
  trajectory.append(stop_n_go::base::RobotState({3.0, 3.0, 3.0, 3.0, 3.0, 3.0}), 0.2);

  resolver.insertPause(trajectory, 1, 3);

  ASSERT_EQ(trajectory.size(), 6U);
  EXPECT_EQ(
    trajectory.waypoint(0).values(),
    std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_EQ(
    trajectory.waypoint(1).values(),
    std::vector<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
  EXPECT_EQ(
    trajectory.waypoint(2).values(),
    std::vector<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
  EXPECT_EQ(
    trajectory.waypoint(3).values(),
    std::vector<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
  EXPECT_EQ(
    trajectory.waypoint(4).values(),
    std::vector<double>({2.0, 2.0, 2.0, 2.0, 2.0, 2.0}));
  EXPECT_EQ(
    trajectory.waypoint(5).values(),
    std::vector<double>({3.0, 3.0, 3.0, 3.0, 3.0, 3.0}));

  for (std::size_t index = 1; index < trajectory.size(); ++index) {
    EXPECT_DOUBLE_EQ(trajectory.durationFromPrevious(index), 0.2);
  }
}

TEST_F(RobotStateTest, ConflictResolverBasicResolveReturnsShiftedConflictFreePrefix)
{
  const stop_n_go::base::StateLayout::ConstPtr arm1_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  const moveit::core::JointModelGroup * arm2_group = robot_model_->getJointModelGroup("arm2");
  ASSERT_NE(arm2_group, nullptr);
  const stop_n_go::base::StateLayout::ConstPtr arm2_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm2_group);

  moveit::core::RobotState reference_state(robot_model_);
  reference_state.setToDefaultValues();
  reference_state.update();

  stop_n_go::base::RobotState arm1_safe(arm1_layout->size());
  stop_n_go::base::RobotState arm2_safe(arm2_layout->size());
  for (std::size_t index = 0; index < arm1_layout->size(); ++index) {
    arm1_safe[index] = reference_state.getVariablePosition(arm1_layout->variableNameAt(index));
  }
  for (std::size_t index = 0; index < arm2_layout->size(); ++index) {
    arm2_safe[index] = reference_state.getVariablePosition(arm2_layout->variableNameAt(index));
  }

  const stop_n_go::base::RobotState arm1_colliding({-3.566, 2.555, 1.382, 4.108, 1.470, -3.763});
  const stop_n_go::base::RobotState arm2_colliding({3.669, -3.365, 0.449, 5.114, 1.949, -1.633});

  stop_n_go::base::Trajectory first(arm1_layout);
  stop_n_go::base::Trajectory second(arm2_layout);
  first.append(arm1_safe, 0.0);
  first.append(arm1_colliding, 0.2);
  first.append(arm1_safe, 0.2);
  second.append(arm2_safe, 0.0);
  second.append(arm2_colliding, 0.2);
  second.append(arm2_safe, 0.2);

  const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
    });
  stop_n_go::core::ConflictChecker::LayoutMap layouts;
  layouts.emplace(0, arm1_layout);
  layouts.emplace(1, arm2_layout);
  const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  const auto checker = std::make_shared<stop_n_go::core::ConflictChecker>(
    registry,
    robot_model_,
    std::move(layouts),
    planning_scene,
    reference_state);
  const stop_n_go::core::ConflictResolver resolver(checker);

  const stop_n_go::core::SearchNode node({first, second});
  const stop_n_go::core::ConflictResult conflict(0, 1, 1);

  const stop_n_go::core::SearchNode resolved = resolver.resolve(
    node,
    conflict,
    0,
    stop_n_go::core::ConflictResolver::Mode::Basic);

  ASSERT_EQ(resolved.trajectory(0).size(), 4U);
  EXPECT_EQ(resolved.tS(), 1U);
  EXPECT_EQ(resolved.trajectory(0).waypoint(0).values(), arm1_safe.values());
  EXPECT_EQ(resolved.trajectory(0).waypoint(1).values(), arm1_safe.values());
  EXPECT_EQ(resolved.trajectory(0).waypoint(2).values(), arm1_colliding.values());
  EXPECT_EQ(resolved.trajectory(0).waypoint(3).values(), arm1_safe.values());
  EXPECT_FALSE(checker->conflictCheck(resolved, 0).has_value());
  EXPECT_FALSE(checker->conflictCheck(resolved, 1).has_value());
}

TEST_F(RobotStateTest, StopNGoSearchSolveReturnsConflictFreeNode)
{
  const stop_n_go::base::StateLayout::ConstPtr arm1_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  const moveit::core::JointModelGroup * arm2_group = robot_model_->getJointModelGroup("arm2");
  ASSERT_NE(arm2_group, nullptr);
  const stop_n_go::base::StateLayout::ConstPtr arm2_layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm2_group);

  moveit::core::RobotState reference_state(robot_model_);
  reference_state.setToDefaultValues();
  reference_state.update();

  stop_n_go::base::RobotState arm1_safe(arm1_layout->size());
  stop_n_go::base::RobotState arm2_safe(arm2_layout->size());
  for (std::size_t index = 0; index < arm1_layout->size(); ++index) {
    arm1_safe[index] = reference_state.getVariablePosition(arm1_layout->variableNameAt(index));
  }
  for (std::size_t index = 0; index < arm2_layout->size(); ++index) {
    arm2_safe[index] = reference_state.getVariablePosition(arm2_layout->variableNameAt(index));
  }

  const stop_n_go::base::RobotState arm1_colliding({-3.566, 2.555, 1.382, 4.108, 1.470, -3.763});
  const stop_n_go::base::RobotState arm2_colliding({3.669, -3.365, 0.449, 5.114, 1.949, -1.633});

  stop_n_go::base::Trajectory first(arm1_layout);
  stop_n_go::base::Trajectory second(arm2_layout);
  first.append(arm1_safe, 0.0);
  first.append(arm1_colliding, 0.2);
  first.append(arm1_safe, 0.2);
  second.append(arm2_safe, 0.0);
  second.append(arm2_colliding, 0.2);
  second.append(arm2_safe, 0.2);

  const stop_n_go::base::RobotRegistry registry(
    {
      {0, "arm1", "arm1_tool0"},
      {1, "arm2", "arm2_tool0"},
    });
  stop_n_go::core::ConflictChecker::LayoutMap layouts;
  layouts.emplace(0, arm1_layout);
  layouts.emplace(1, arm2_layout);
  const auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  const auto checker = std::make_shared<stop_n_go::core::ConflictChecker>(
    registry,
    robot_model_,
    std::move(layouts),
    planning_scene,
    reference_state);
  const auto resolver = std::make_shared<stop_n_go::core::ConflictResolver>(checker);
  const stop_n_go::core::StopNGoSearch search(checker, resolver, 0.2);

  EXPECT_FALSE(checker->inConflict(0, arm1_safe, 1, arm2_colliding));
  EXPECT_FALSE(checker->inConflict(0, arm1_colliding, 1, arm2_safe));

  const stop_n_go::core::SearchNode solved = search.solve(
    {first, second},
    stop_n_go::core::ConflictResolver::Mode::Basic);

  EXPECT_FALSE(checker->conflictCheck(solved, 0).has_value());
  EXPECT_FALSE(checker->conflictCheck(solved, 1).has_value());
  EXPECT_FALSE(checker->conflictCheck(solved, 2).has_value());
}

TEST_F(RobotStateTest, StateLayoutLoadsArm1PlanningGroupFromSimulationModel)
{
  const stop_n_go::base::StateLayout layout(*arm1_group_);

  const std::vector<std::string> expected_variable_names = {
    "arm1_shoulder_pan_joint",
    "arm1_shoulder_lift_joint",
    "arm1_elbow_joint",
    "arm1_wrist_1_joint",
    "arm1_wrist_2_joint",
    "arm1_wrist_3_joint",
  };

  std::cout << "[layout] group=" << layout.groupName() << '\n';
  std::cout << "[layout] variables=[" << joinStrings(layout.variableNames()) << "]" << '\n';

  EXPECT_EQ(layout.robotModel(), robot_model_.get());
  EXPECT_EQ(layout.jointModelGroup(), arm1_group_);
  EXPECT_EQ(layout.groupName(), "arm1");
  EXPECT_EQ(layout.size(), 6U);
  EXPECT_EQ(layout.variableNames(), expected_variable_names);
}

TEST_F(RobotStateTest, RobotStateRoundTripsArm1PlanningGroupValuesThroughMoveItState)
{
  const stop_n_go::base::StateLayout layout(*arm1_group_);
  const stop_n_go::base::RobotState state({0.10, -0.20, 0.30, -0.40, 0.50, -0.60});

  ASSERT_TRUE(layout.isCompatible(state));
  EXPECT_FALSE(layout.isCompatible(stop_n_go::base::RobotState(5)));

  moveit::core::RobotState moveit_state(robot_model_);
  moveit_state.setToDefaultValues();
  moveit_state.setVariablePositions(layout.variableNames(), state.values());

  std::vector<double> round_trip_values;
  moveit_state.copyJointGroupPositions(arm1_group_, round_trip_values);

  std::cout << "[state] input=[" << joinDoubles(state.values()) << "]" << '\n';
  std::cout << "[state] moveit_round_trip=[" << joinDoubles(round_trip_values) << "]" << '\n';

  EXPECT_EQ(round_trip_values, state.values());
}

TEST_F(RobotStateTest, InterpolationUsesStateLayoutToProduceIntermediateState)
{
  const stop_n_go::base::StateLayout layout(*arm1_group_);
  const stop_n_go::base::RobotState start({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  const stop_n_go::base::RobotState goal({1.0, 2.0, 3.0, 4.0, 5.0, 6.0});

  std::cout << "[interpolate] variables=[" << joinStrings(layout.variableNames()) << "]" << '\n';
  const stop_n_go::base::RobotState mid = stop_n_go::base::interpolate(start, goal, 0.5, layout);
  const stop_n_go::base::RobotState quarter =
    stop_n_go::base::interpolate(start, goal, 0.25, layout);
  const stop_n_go::base::RobotState three_quarter = stop_n_go::base::interpolate(
    start, goal, 0.75,
    layout);

  std::cout << "[interpolate] start=[" << joinDoubles(start.values()) << "]" << '\n';
  std::cout << "[interpolate] quarter (ratio=0.25)=[" << joinDoubles(quarter.values()) << "]" <<
    '\n';
  std::cout << "[interpolate] goal=[" << joinDoubles(goal.values()) << "]" << '\n';
  std::cout << "[interpolate] mid=[" << joinDoubles(mid.values()) << "]" << '\n';
  std::cout << "[interpolate] three_quarter (ratio=0.75)=[" <<
    joinDoubles(three_quarter.values()) << "]"
            << '\n';

  EXPECT_TRUE(mid.isApprox(stop_n_go::base::RobotState({0.5, 1.0, 1.5, 2.0, 2.5, 3.0}), 1e-9));
}

TEST_F(RobotStateTest, ResampleKeepsEndpointStateAndDelaysEndpointToGrid)
{
  const stop_n_go::base::StateLayout::ConstPtr layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);

  stop_n_go::base::Trajectory trajectory(layout);
  trajectory.append(stop_n_go::base::RobotState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), 0.0);
  trajectory.append(stop_n_go::base::RobotState({1.0, 2.0, 3.0, 4.0, 5.0, 6.0}), 1.1);

  const stop_n_go::base::Trajectory resampled = trajectory.resample(0.2);

  std::cout << "[resample] original_total_duration=" << trajectory.totalDuration() << '\n';
  std::cout << "[resample] resampled_total_duration=" << resampled.totalDuration() << '\n';
  std::cout << "[resample] resampled_waypoint_count=" << resampled.size() << '\n';
  std::cout << "[resample] final_waypoint=[" << joinDoubles(
    resampled.waypoint(
      resampled.size() - 1).values()) << "]"
            << '\n';

  ASSERT_EQ(resampled.size(), 7U);
  EXPECT_DOUBLE_EQ(resampled.totalDuration(), 1.2);
  EXPECT_EQ(resampled.waypoint(0).values(), trajectory.waypoint(0).values());
  EXPECT_EQ(resampled.waypoint(resampled.size() - 1).values(), trajectory.waypoint(1).values());

  for (std::size_t index = 1; index < resampled.size(); ++index) {
    EXPECT_DOUBLE_EQ(resampled.durationFromPrevious(index), 0.2);
  }
}

TEST_F(RobotStateTest, ConversionContextOverlaysGroupStateOnReferenceMoveItState)
{
  const stop_n_go::base::StateLayout::ConstPtr layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  moveit::core::RobotState reference_state(robot_model_);
  reference_state.setToDefaultValues();
  reference_state.setVariablePosition("arm2_shoulder_pan_joint", 1.23);
  reference_state.update();

  const stop_n_go::base::ConversionContext context(robot_model_, layout, reference_state);
  const stop_n_go::base::RobotState state({0.5, 0.4, 0.3, 0.2, 0.1, 0.0});

  const moveit::core::RobotState moveit_state = context.toMoveItState(state);
  const stop_n_go::base::RobotState projected_state = context.fromMoveItState(moveit_state);

  std::cout << "[conversion] reference arm2_shoulder_pan_joint="
            << reference_state.getVariablePosition("arm2_shoulder_pan_joint") << '\n';
  std::cout << "[conversion] projected arm1 values=[" << joinDoubles(projected_state.values()) <<
    "]"
            << '\n';

  EXPECT_DOUBLE_EQ(moveit_state.getVariablePosition("arm2_shoulder_pan_joint"), 1.23);
  EXPECT_TRUE(projected_state.isApprox(state, 1e-9));
}

TEST_F(RobotStateTest, ConversionContextExportsJointTrajectoryMsgForTargetGroup)
{
  const stop_n_go::base::StateLayout::ConstPtr layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  moveit::core::RobotState reference_state(robot_model_);
  reference_state.setToDefaultValues();
  const stop_n_go::base::ConversionContext context(robot_model_, layout, reference_state);

  stop_n_go::base::Trajectory trajectory(layout);
  trajectory.append(stop_n_go::base::RobotState({0.0, 0.1, 0.2, 0.3, 0.4, 0.5}), 0.0);
  trajectory.append(stop_n_go::base::RobotState({1.0, 1.1, 1.2, 1.3, 1.4, 1.5}), 0.2);
  trajectory.append(stop_n_go::base::RobotState({2.0, 2.1, 2.2, 2.3, 2.4, 2.5}), 0.3);

  const trajectory_msgs::msg::JointTrajectory trajectory_msg = context.toJointTrajectoryMsg(
    trajectory);

  std::cout << "[joint_trajectory] joint_names=[" << joinStrings(trajectory_msg.joint_names) <<
    "]" << '\n';
  for (std::size_t index = 0; index < trajectory_msg.points.size(); ++index) {
    const auto & point = trajectory_msg.points[index];
    const double time_from_start = static_cast<double>(point.time_from_start.sec) +
      static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
    std::cout << "[joint_trajectory] point " << index << " positions=[" << joinDoubles(
      point.positions)
              << "] time_from_start=" << time_from_start << '\n';
  }

  ASSERT_EQ(trajectory_msg.joint_names, layout->variableNames());
  ASSERT_EQ(trajectory_msg.points.size(), 3U);
  EXPECT_EQ(trajectory_msg.points[0].positions, trajectory.waypoint(0).values());
  EXPECT_EQ(trajectory_msg.points[1].positions, trajectory.waypoint(1).values());
  EXPECT_EQ(trajectory_msg.points[2].positions, trajectory.waypoint(2).values());
  EXPECT_EQ(trajectory_msg.points[0].time_from_start.sec, 0);
  EXPECT_EQ(trajectory_msg.points[0].time_from_start.nanosec, 0U);
  EXPECT_EQ(trajectory_msg.points[1].time_from_start.sec, 0);
  EXPECT_EQ(trajectory_msg.points[1].time_from_start.nanosec, 200000000U);
  EXPECT_EQ(trajectory_msg.points[2].time_from_start.sec, 0);
  EXPECT_EQ(trajectory_msg.points[2].time_from_start.nanosec, 500000000U);
}

TEST_F(RobotStateTest, ConversionContextProjectsMoveItTrajectoryToStopNGoTrajectory)
{
  const stop_n_go::base::StateLayout::ConstPtr layout =
    std::make_shared<stop_n_go::base::StateLayout>(*arm1_group_);
  moveit::core::RobotState reference_state(robot_model_);
  reference_state.setToDefaultValues();
  const stop_n_go::base::ConversionContext context(robot_model_, layout, reference_state);

  robot_trajectory::RobotTrajectory moveit_trajectory(robot_model_, arm1_group_);

  moveit::core::RobotState first_state(reference_state);
  first_state.setJointGroupActivePositions(
    arm1_group_,
    std::vector<double>({0.0, 0.1, 0.2, 0.3, 0.4, 0.5}));
  first_state.update();

  moveit::core::RobotState second_state(reference_state);
  second_state.setJointGroupActivePositions(
    arm1_group_,
    std::vector<double>({1.0, 1.1, 1.2, 1.3, 1.4, 1.5}));
  second_state.update();

  moveit_trajectory.addSuffixWayPoint(first_state, 0.0);
  moveit_trajectory.addSuffixWayPoint(second_state, 0.2);

  const stop_n_go::base::Trajectory trajectory = context.fromMoveItTrajectory(moveit_trajectory);

  std::cout << "[from_moveit_trajectory] waypoint0=[" <<
    joinDoubles(trajectory.waypoint(0).values()) << "]"
            << '\n';
  std::cout << "[from_moveit_trajectory] waypoint1=[" <<
    joinDoubles(trajectory.waypoint(1).values()) << "]"
            << '\n';

  ASSERT_EQ(trajectory.size(), 2U);
  EXPECT_EQ(trajectory.durationFromPrevious(0), 0.0);
  EXPECT_EQ(trajectory.durationFromPrevious(1), 0.2);
  EXPECT_TRUE(
    trajectory.waypoint(0).isApprox(
      stop_n_go::base::RobotState(
        {0.0, 0.1, 0.2, 0.3, 0.4,
          0.5}), 1e-9));
  EXPECT_TRUE(
    trajectory.waypoint(1).isApprox(
      stop_n_go::base::RobotState(
        {1.0, 1.1, 1.2, 1.3, 1.4,
          1.5}), 1e-9));
}

}  // namespace
