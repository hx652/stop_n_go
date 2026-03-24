from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_package = "trapezoid_ur_description"
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", "trapezoid_ur7e_scene.urdf.xacro"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "trapezoid_ur7e_scene.rviz"]
    )

    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(use_joint_state_publisher_gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_file],
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
