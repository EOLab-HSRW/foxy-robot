from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_args() -> list[LaunchDescriptionEntity]:

    declared_args = []

    declared_args.append(DeclareLaunchArgument(
        "robot_name",
        default_value="foxy",
        description="Robot name. Robot name, this name is used as namespace."
    ))

    declared_args.append(DeclareLaunchArgument(
        "pos_x",
        default_value="0.0",
        description="Start position in x axis (only for simulation environments)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "pos_y",
        default_value="0.0",
        description="Start position in y axis (only for simulation environments)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "pos_z",
        default_value="0.1",
        description="Start position in z axis (only for simulation environments)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "system",
        default_value="gz",
        description="Define whether to start the system in simulation or the hardware robot.",
        choices=['gz', 'robot']
    ))

    declared_args.append(DeclareLaunchArgument(
        "world",
        default_value="small_loop",
        choices=[
            "empty",
            "small_loop",
            "small_map",
            "large_map",
            "straight_lane"
        ],
        description="World in simulation (only `gz` sim supported for now)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "rviz_start",
        default_value="true",
        description="Launch Rviz"
    ))

    declared_args.append(DeclareLaunchArgument(
        "robot_localization",
        default_value="true",
        description="Start robot localization for sensor fusion"
    ))

    declared_args.append(DeclareLaunchArgument(
        "slam_toolbox",
        default_value="true",
        description="Start SLAM Toolbox"
    ))

    declared_args.append(DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([FindPackageShare("foxy_bringup"), "config", "default.rviz"]),
        description="Path to your rviz configuration file."
    ))

    return declared_args


def launch_setup(context) -> list[LaunchDescriptionEntity]:

    SetLaunchConfiguration(
        name="use_sim_time",
        value=("true" if LaunchConfiguration("system").perform(context) != 'robot' else "false")
    )

    spawn_robot = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("foxy_bringup"),
                "launch",
                "spawn.launch.py",
            ]
        ),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "pos_x": LaunchConfiguration("pos_x"),
            "pos_y": LaunchConfiguration("pos_y"),
            "pos_z": LaunchConfiguration("pos_z"),
            "system": LaunchConfiguration("system"),
            "world": PathJoinSubstitution([FindPackageShare("foxy_description"), "worlds", f"{LaunchConfiguration('world').perform(context)}.sdf"]),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_localization": LaunchConfiguration("robot_localization"),
            "slam_toolbox": LaunchConfiguration("slam_toolbox"),
            "rviz_start": LaunchConfiguration("rviz_start")
        }.items(),
    )


    return [
        spawn_robot,
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()
    ld.add_action(GroupAction(launch_args()))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
