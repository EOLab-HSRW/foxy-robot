import os
import xacro
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace

def launch_args(context) -> list[LaunchDescriptionEntity]:

    declared_args = []

    declared_args.append(DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Clock source."
    ))

    declared_args.append(DeclareLaunchArgument(
        "robot_name",
        default_value="foxy",
        description="Robot name."
    ))

    declared_args.append(DeclareLaunchArgument(
        "system",
        default_value="gz",
        description="Choose system to start, e.g. robot or gz for Gazebo",
        choices=['gz', 'robot']
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

    use_sim_time = {"use_sim_time": True if LaunchConfiguration("system").perform(context) != 'robot' else False }

    robot_desc_content = xacro.process_file(
        PathJoinSubstitution([FindPackageShare("foxy_description"), "urdf", "foxy.urdf.xacro"]).perform(context),
        mappings={
            "robot_name": LaunchConfiguration("robot_name").perform(context),
            "system": LaunchConfiguration("system").perform(context),
            "distro": os.getenv("ROS_DISTRO")
        }
    ).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc_content},
            use_sim_time
        ]
    )

    controllers = GroupAction(
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
        ]
    )

    gz = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py"
                ]),
                launch_arguments={
                    "gz_args": ["-r ", "empty.sdf"], # `-r` start running simulation immediately
                    'on_exit_shutdown': 'True'
                }.items()
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", LaunchConfiguration("robot_name"),
                    "-topic", "robot_description",
                    "-x", "0",
                    "-y", "0",
                    "-z", "0.3"
                ]
            ),
        ],
        condition=LaunchConfigurationEquals("system", "gz")
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
        PushRosNamespace(LaunchConfiguration("robot_name")),
        robot_state_publisher_node,
        controllers,
        gz,
        rviz2
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_args))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
