cmake_minimum_required(VERSION 3.8)
project(stop_n_go)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_base
  src/base/conversion.cpp
  src/base/initial_planner.cpp
  src/base/r_state.cpp
  src/base/robot_registry.cpp
  src/base/trajectory.cpp
)
target_compile_features(${PROJECT_NAME}_base PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_base PUBLIC geometry_msgs moveit_core moveit_msgs trajectory_msgs)

add_executable(plan_resample_execute_demo
  src/demo/plan_resample_execute_demo.cpp
)
target_compile_features(plan_resample_execute_demo PUBLIC cxx_std_17)
target_link_libraries(plan_resample_execute_demo ${PROJECT_NAME}_base)
target_link_libraries(plan_resample_execute_demo OpenSSL::SSL OpenSSL::Crypto)
ament_target_dependencies(plan_resample_execute_demo
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  trajectory_msgs
)

add_executable(conflict_checker_demo
  src/demo/conflict_checker_demo.cpp
)
target_compile_features(conflict_checker_demo PUBLIC cxx_std_17)
target_link_libraries(conflict_checker_demo ${PROJECT_NAME}_core)
ament_target_dependencies(conflict_checker_demo
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
)

add_executable(stop_n_go_sim_demo
  src/demo/stop_n_go_sim_demo.cpp
)
target_compile_features(stop_n_go_sim_demo PUBLIC cxx_std_17)
target_link_libraries(stop_n_go_sim_demo ${PROJECT_NAME}_base ${PROJECT_NAME}_core)
target_link_libraries(stop_n_go_sim_demo OpenSSL::SSL OpenSSL::Crypto)
ament_target_dependencies(stop_n_go_sim_demo
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  trajectory_msgs
)

add_executable(stop_n_go_conflict_dataset_demo
  src/demo/stop_n_go_conflict_dataset_demo.cpp
)
target_compile_features(stop_n_go_conflict_dataset_demo PUBLIC cxx_std_17)
target_link_libraries(stop_n_go_conflict_dataset_demo ${PROJECT_NAME}_base ${PROJECT_NAME}_core ${PROJECT_NAME}_app)
target_link_libraries(stop_n_go_conflict_dataset_demo OpenSSL::SSL OpenSSL::Crypto)
ament_target_dependencies(stop_n_go_conflict_dataset_demo
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  trajectory_msgs
)

add_executable(find_multi_arm_overlap_goals_demo
  src/demo/find_multi_arm_overlap_goals_demo.cpp
)
target_compile_features(find_multi_arm_overlap_goals_demo PUBLIC cxx_std_17)
target_link_libraries(find_multi_arm_overlap_goals_demo ${PROJECT_NAME}_base ${PROJECT_NAME}_core ${PROJECT_NAME}_app)
target_link_libraries(find_multi_arm_overlap_goals_demo OpenSSL::SSL OpenSSL::Crypto)
ament_target_dependencies(find_multi_arm_overlap_goals_demo
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  visualization_msgs
)

add_executable(four_arm_random_goal_demo
  src/demo/four_arm_random_goal_demo.cpp
)
target_compile_features(four_arm_random_goal_demo PUBLIC cxx_std_17)
target_link_libraries(four_arm_random_goal_demo ${PROJECT_NAME}_base ${PROJECT_NAME}_core)
target_link_libraries(four_arm_random_goal_demo OpenSSL::SSL OpenSSL::Crypto)
ament_target_dependencies(four_arm_random_goal_demo
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  trajectory_msgs
)

add_executable(arm1_initial_planner_demo
  src/demo/arm1_initial_planner_demo.cpp
)
target_compile_features(arm1_initial_planner_demo PUBLIC cxx_std_17)
target_link_libraries(arm1_initial_planner_demo ${PROJECT_NAME}_base)
target_link_libraries(arm1_initial_planner_demo OpenSSL::SSL OpenSSL::Crypto)
ament_target_dependencies(arm1_initial_planner_demo
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
)

add_executable(arm1_direct_movegroup_pose_demo
  src/demo/arm1_direct_movegroup_pose_demo.cpp
)
target_compile_features(arm1_direct_movegroup_pose_demo PUBLIC cxx_std_17)
target_link_libraries(arm1_direct_movegroup_pose_demo OpenSSL::SSL OpenSSL::Crypto)
ament_target_dependencies(arm1_direct_movegroup_pose_demo
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
)

add_library(${PROJECT_NAME}_core
  src/core/conflict_check.cpp
  src/core/conflict_result.cpp
  src/core/conflict_resolver.cpp
  src/core/search_node.cpp
  src/core/stop_n_go_search.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${PROJECT_NAME}_base)

add_library(${PROJECT_NAME}_app
  src/app/conflict_scenario_dataset.cpp
  src/app/scenario_search.cpp
)
target_compile_features(${PROJECT_NAME}_app PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_app PUBLIC ${PROJECT_NAME}_base ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_app PUBLIC moveit_core moveit_msgs moveit_ros_planning_interface visualization_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS
    ${PROJECT_NAME}_base
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_app
    plan_resample_execute_demo
    conflict_checker_demo
    stop_n_go_sim_demo
    stop_n_go_conflict_dataset_demo
    find_multi_arm_overlap_goals_demo
    four_arm_random_goal_demo
    arm1_initial_planner_demo
    arm1_direct_movegroup_pose_demo
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs moveit_core moveit_msgs moveit_ros_planning_interface rclcpp trajectory_msgs)
ament_export_libraries(${PROJECT_NAME}_base ${PROJECT_NAME}_core ${PROJECT_NAME}_app)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(moveit_ros_planning REQUIRED)

  ament_add_gtest(test_r_state test/test_r_state.cpp)
  target_link_libraries(test_r_state ${PROJECT_NAME}_base ${PROJECT_NAME}_core)
  ament_target_dependencies(test_r_state moveit_ros_planning)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
