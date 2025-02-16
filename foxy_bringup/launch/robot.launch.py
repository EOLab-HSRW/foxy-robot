from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro
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
        description="Choose system to start, e.g. physical",
        choices=['gz', 'none', 'real']
    ))

    return declared_args


def launch_setup(context) -> list[LaunchDescriptionEntity]:

    robot_desc_content = xacro.process_file(
        PathJoinSubstitution([FindPackageShare("foxy_description"), "urdf", "foxy.urdf.xacro"]).perform(context),
        mappings={
            "robot_name": LaunchConfiguration("robot_name").perform(context),
            "system": LaunchConfiguration("system").perform(context)
        }
    ).toxml()

    print(robot_desc_content)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc_content}
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

    return [
        PushRosNamespace(LaunchConfiguration("robot_name")),
        robot_state_publisher_node,
        gz
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_args))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
