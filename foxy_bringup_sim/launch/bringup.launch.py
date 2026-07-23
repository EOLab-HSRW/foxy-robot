from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context):

    pos_x = LaunchConfiguration("pos_x").perform(context)
    pos_y = LaunchConfiguration("pos_y").perform(context)
    pos_z = LaunchConfiguration("pos_z").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    world_path = LaunchConfiguration("world_path").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    gz_client_gui_path = LaunchConfiguration("gz_client_gui_path").perform(context)

    launch_world = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("foxy_bringup_sim"),
            "launch",
            "world.launch.py"
        ]),
        launch_arguments={
            "robot_name": robot_name,
            "world_name": world_name,
            "world_path": world_path,
            "verbose": verbose,
            "headless": headless,
            "gz_client_gui_path": gz_client_gui_path,
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
            "world_name": world_name,
            "verbose": verbose,
            "pos_x": pos_x,
            "pos_y": pos_y,
            "pos_z": pos_z,
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
            "world_name",
        ),
        DeclareLaunchArgument(
            "world_path",
            default_value=""
        ),
        DeclareLaunchArgument(
            "headless"
        ),
        DeclareLaunchArgument(
            "view_follow"
        ),
        DeclareLaunchArgument(
            "gz_client_gui_path"
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
