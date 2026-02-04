from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context) -> list[LaunchDescriptionEntity]:

    info_msg = LogInfo(
        msg="Sorry. The real robot is not yet implemented, but it will be soon.",
        condition=LaunchConfigurationEquals("system", "real")
    )

    system = LaunchConfiguration("system").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world").perform(context)
    world_path = PathJoinSubstitution([FindPackageShare("foxy_description"), "worlds", f"{world_name}.world"]).perform(context)

    use_sim_time = "true" if system != 'real' else "false"

    spawn_robot = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("foxy_bringup"),
                "launch",
                "spawn.launch.py",
            ]
        ),
        launch_arguments={
            "robot_name": robot_name,
            "pos_x": LaunchConfiguration("pos_x"),
            "pos_y": LaunchConfiguration("pos_y"),
            "pos_z": LaunchConfiguration("pos_z"),
            "system": system,
            "world": world_path,
            "use_sim_time": use_sim_time,
            "robot_localization": LaunchConfiguration("robot_localization"),
            "slam_toolbox": LaunchConfiguration("slam_toolbox"),
            "rviz_start": LaunchConfiguration("rviz_start")
        }.items(),
        condition=LaunchConfigurationEquals("system", "sim")
    )


    return [
        info_msg,
        spawn_robot,
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()
    declared_args = []

    ld.add_action(DeclareLaunchArgument(
        "robot_name",
        default_value="foxy",
        description="Robot name. This name is used as namespace."
    ))

    ld.add_action(DeclareLaunchArgument(
        "pos_x",
        default_value="0.0",
        description="Start position in x axis (only for simulation environments)."
    ))

    ld.add_action(DeclareLaunchArgument(
        "pos_y",
        default_value="0.0",
        description="Start position in y axis (only for simulation environments)."
    ))

    ld.add_action(DeclareLaunchArgument(
        "pos_z",
        default_value="0.2",
        description="Start position in z axis (only for simulation environments)."
    ))

    ld.add_action(DeclareLaunchArgument(
        "system",
        default_value="sim",
        description="Define whether to start the system in simulation or the hardware robot. `sim` is Gazebo simulation.",
        choices=['sim', 'real']
    ))

    ld.add_action(DeclareLaunchArgument(
        "world",
        default_value="small_loop",
        choices=[
            "empty",
            "small_loop",
            "small_map",
            "large_map",
            "straight_lane"
        ],
        description="Simulated World (only if system is `sim`)."
    ))

    ld.add_action(DeclareLaunchArgument(
        "rviz_start",
        default_value="true",
        description="Launch Rviz"
    ))

    ld.add_action(DeclareLaunchArgument(
        "robot_localization",
        default_value="false",
        description="Start robot localization for sensor fusion"
    ))

    ld.add_action(DeclareLaunchArgument(
        "slam_toolbox",
        default_value="false",
        description="Start SLAM Toolbox"
    ))

    ld.add_action(DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([FindPackageShare("foxy_bringup"), "config", "default.rviz"]),
        description="Path to your rviz configuration file."
    ))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
