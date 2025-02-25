from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_args(context) -> list[LaunchDescriptionEntity]:

    declared_args = []

    declared_args.append(DeclareLaunchArgument(
        "robot_name",
        default_value="foxy",
        description="Robot name."
    ))

    declared_args.append(DeclareLaunchArgument(
        "pos_x",
        default_value="0.0",
        description="Robot spawn x position (only for simulation environments)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "pos_y",
        default_value="0.0",
        description="Robot spawn y position (only for simulation environments)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "pos_z",
        default_value="0.1",
        description="Robot spawn z position (only for simulation environments)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "system",
        default_value="gz",
        description="Choose system to start, e.g. robot or gz for Gazebo",
        choices=['gz', 'robot']
    ))

    declared_args.append(DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="World in simulation (only `gz` sim supported for now)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "rviz_start",
        default_value="true",
        description="Launch Rviz"
    ))

    declared_args.append(DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([FindPackageShare("foxy_bringup"), "config", "default.rviz"]),
        description="Configuration file for launching rviz."
    ))

    return declared_args


def launch_setup(context) -> list[LaunchDescriptionEntity]:

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
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration("rviz_config")],
        output='screen',
        condition=IfCondition(LaunchConfiguration("rviz_start"))
    )

    return [
        spawn_robot,
        rviz2
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_args))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
