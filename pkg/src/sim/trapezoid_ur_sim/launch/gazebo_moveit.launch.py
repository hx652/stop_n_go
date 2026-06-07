from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder


def generate_launch_description():
    gz_args = LaunchConfiguration("gz_args")

    moveit_config_share = get_package_share_directory("trapezoid_ur_moveit_config")
    ros2_controllers = moveit_config_share + "/config/ros2_controllers.yaml"

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="trapezoid_ur7e_scene",
            package_name="trapezoid_ur_moveit_config",
        )
        .robot_description(
            file_path="config/trapezoid_ur7e_scene.urdf.xacro",
            mappings={
                "simulation_controllers": ros2_controllers,
            },
        )
        .robot_description_semantic(file_path="config/trapezoid_ur7e_scene.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    use_sim_time = {"use_sim_time": True}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[use_sim_time, moveit_config.robot_description],
        output="both",
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "trapezoid_ur7e_scene",
            "-allow_renaming",
        ],
        output="both",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            moveit_config.to_dict(),
            {"planning_plugin": "ompl_interface/OMPLPlanner"},
            use_sim_time,
        ],
        output="both",
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="both",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("trapezoid_ur_sim"), "config", "stop_n_go_moveit.rviz"]
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            use_sim_time,
        ],
        output="screen",
    )

    controller_names = [
        "joint_state_broadcaster",
        "arm1_controller",
        "arm2_controller",
        "arm3_controller",
        "arm4_controller",
    ]
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["-c", "controller_manager", name],
            output="both",
        )
        for name in controller_names
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("gz_args", default_value="-r -v 4 empty.sdf"),
            robot_state_publisher,
            start_gazebo,
            spawn_entity,
            move_group,
            rviz,
            gz_sim_bridge,
        ]
        + controller_spawners
    )
