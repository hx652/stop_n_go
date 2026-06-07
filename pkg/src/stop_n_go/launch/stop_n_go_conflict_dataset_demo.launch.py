from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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

    dataset_path_arg = DeclareLaunchArgument(
        "dataset_path",
        default_value="/tmp/find_multi_arm_overlap_goals_demo_conflict_only_trials.json",
    )
    trial_index_arg = DeclareLaunchArgument(
        "trial_index",
        default_value="0",
        description=(
            "Start index into conflict_trial_indices, "
            "not the original dataset trial_index."
        ),
    )
    num_conflict_sample_arg = DeclareLaunchArgument(
        "num_conflict_sample",
        default_value="1",
        description="Number of conflict samples to solve from trial_index; 0 means all remaining.",
    )
    max_expanded_nodes_arg = DeclareLaunchArgument(
        "max_expanded_nodes",
        default_value="2000",
    )
    stop_n_go_log_level_arg = DeclareLaunchArgument(
        "stop_n_go_log_level",
        default_value="summary",
        description="Stop-N-Go log level: quiet, summary, or debug.",
    )
    store_trajectories_arg = DeclareLaunchArgument(
        "store_trajectories",
        default_value="true",
        description="Store solved trial trajectories for terminal-controlled playback.",
    )
    playback_index_topic_arg = DeclareLaunchArgument(
        "playback_index_topic",
        default_value="/stop_n_go/playback_index",
        description="Topic used to select the stored trajectory playback index.",
    )

    gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("trapezoid_ur_sim"), "launch", "gazebo_moveit.launch.py"]
            )
        )
    )

    demo_node = Node(
        package="stop_n_go",
        executable="stop_n_go_conflict_dataset_demo",
        name="stop_n_go_conflict_dataset_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": True,
                "dataset_path": LaunchConfiguration("dataset_path"),
                "trial_index": LaunchConfiguration("trial_index"),
                "num_conflict_sample": LaunchConfiguration("num_conflict_sample"),
                "max_expanded_nodes": LaunchConfiguration("max_expanded_nodes"),
                "stop_n_go_log_level": LaunchConfiguration("stop_n_go_log_level"),
                "store_trajectories": LaunchConfiguration("store_trajectories"),
                "playback_index_topic": LaunchConfiguration("playback_index_topic"),
            },
        ],
    )

    return LaunchDescription(
        [
            dataset_path_arg,
            trial_index_arg,
            num_conflict_sample_arg,
            max_expanded_nodes_arg,
            stop_n_go_log_level_arg,
            store_trajectories_arg,
            playback_index_topic_arg,
            gazebo_moveit,
            TimerAction(period=8.0, actions=[demo_node]),
        ]
    )
