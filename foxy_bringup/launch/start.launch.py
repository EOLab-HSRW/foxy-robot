from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python.packages import PackageNotFoundError


def launch_setup(context) -> list[object]:
    mode: str = LaunchConfiguration("mode").perform(context)
    use_sim_time: bool = True if mode == "sim" else False

    pkg: str = "foxy_bringup_sim" if mode == "sim" else "foxy_bringup_hw"

    try:
        share: str = FindPackageShare(pkg).perform(context)
    except PackageNotFoundError:
        return [
            LogInfo(msg=f"'{pkg}' is not installed.")
        ]

    bringup: str = PathJoinSubstitution([FindPackageShare(pkg), "launch", "bringup.launch.py"])

    robot_desc_path: str = PathJoinSubstitution([
        FindPackageShare("foxy_description"),
        "urdf",
        "foxy.urdf.xacro"
    ]).perform(context)

    robot_desc_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        robot_desc_path,
        " ",
        "system:=", mode,
    ]).perform(context)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc_content},
            {"use_sim_time": use_sim_time }
        ]
    )

    controllers_path = PathJoinSubstitution([
        FindPackageShare("foxy_bringup"),
        "config",
        "controllers.yaml"
    ])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers_path,
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "diff_drive_base_controller"],
        output="screen",
    )

    on_start_cm = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[spawn_joint_state_broadcaster],
        )
    )


    include_bringup = IncludeLaunchDescription(
        bringup
    )

    return [
        robot_state_publisher,
        include_bringup,
        # spawn_joint_state_broadcaster
        ros2_control_node,
        on_start_cm,
    ]


def generate_launch_description() -> LaunchDescription:

    launch_args = []

    launch_args.append(DeclareLaunchArgument(
        "mode",
        default_value="sim",
        choices=["hw", "sim"],
        description="Select hardware or simulation bringup",
    ))

    ld = LaunchDescription(launch_args)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
