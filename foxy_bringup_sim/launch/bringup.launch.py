from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context):

    robot_name = LaunchConfiguration("robot_name").perform(context)
    world = LaunchConfiguration("world").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)
    headless = LaunchConfiguration("headless").perform(context)

    launch_world = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("foxy_bringup_sim"),
            "launch",
            "world.launch.py"
        ]),
        launch_arguments={
            "robot_name": robot_name,
            "world_name": world,
            "verbose": verbose,
            "headless": headless,
        }.items()
    )
    spawn_robot = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("foxy_bringup_sim"),
            "launch",
            "spawn.launch.py"
        ]),
        launch_arguments={
            "robot_name": robot_name,
            "world_name": world,
            "verbose": verbose,
            "pos_x": "0.0",
            "pos_y": "0.0",
            "pos_z": "0.2",
            "view_follow": LaunchConfiguration("view_follow"),
            "enable/camera/front": LaunchConfiguration("enable/camera/front"),
            "enable/tof/front": LaunchConfiguration("enable/tof/front"),
            "enable/imu/front": LaunchConfiguration("enable/imu/front"),
            "enable/leds": LaunchConfiguration("enable/leds"),
        }.items()
    )

    return [
        launch_world,
        spawn_robot
    ]


def generate_launch_description() -> LaunchDescription:

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name"
        ),
        DeclareLaunchArgument(
            "verbose"
        ),
        DeclareLaunchArgument(
            "pos_x"
        ),
        DeclareLaunchArgument(
            "pos_y"
        ),
        DeclareLaunchArgument(
            "pos_z"
        ),
        DeclareLaunchArgument(
            "world",
            choices=[
                "empty",
                "small_loop",
                "small_map",
                "large_map",
                "straight_lane"
            ]
        ),
        DeclareLaunchArgument(
            "headless"
        ),
        DeclareLaunchArgument(
            "view_follow"
        ),
        DeclareLaunchArgument(
            "enable/camera/front"
        ),
        DeclareLaunchArgument(
            "enable/tof/front"
        ),
        DeclareLaunchArgument(
            "enable/imu/front"
        ),
        DeclareLaunchArgument(
            "enable/leds"
        ),
        OpaqueFunction(function=launch_setup)
    ])
