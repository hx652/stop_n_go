from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="trapezoid_ur7e_scene",
            package_name="trapezoid_ur_moveit_config",
        )
        .robot_description(file_path="config/trapezoid_ur7e_scene.urdf.xacro")
        .robot_description_semantic(file_path="config/trapezoid_ur7e_scene.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    demo_node = Node(
        package="stop_n_go",
        executable="plan_resample_execute_demo",
        name="plan_resample_execute_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([demo_node])
