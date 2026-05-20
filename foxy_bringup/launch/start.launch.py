from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace

from ament_index_python.packages import PackageNotFoundError


def launch_setup(context) -> list[object]:
    mode: str = LaunchConfiguration("mode").perform(context)

    # for ROS-reasons the launch system uses string for all
    # launch arguments.
    is_sim: str = "true" if mode == "sim" else "false"

    # but, ROS nodes they typed the node parameters, so:
    use_sim_time: bool = True if is_sim == "true" else False


    pkg: str = "foxy_bringup_sim" if mode == "sim" else "foxy_bringup_hw"

    try:
        share: str = FindPackageShare(pkg).perform(context)
    except PackageNotFoundError:
        return [
            LogInfo(msg=f"'{pkg}' is not installed.")
        ]

    bringup: str = PathJoinSubstitution([FindPackageShare(pkg), "launch", "bringup.launch.py"])

    include_bringup_sim = IncludeLaunchDescription(
        bringup,
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "pos_x": LaunchConfiguration("pos_x"),
            "pos_y": LaunchConfiguration("pos_y"),
            "pos_z": LaunchConfiguration("pos_z"),
            "world": LaunchConfiguration("world"),
        }.items(),
        condition=IfCondition(is_sim)
    )
    include_bringup_hw = IncludeLaunchDescription(
        bringup,
        condition=UnlessCondition(is_sim)
    )

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
        " ",
        "robot_name:=", LaunchConfiguration("robot_name").perform(context),
        " ",
        "distro:=", EnvironmentVariable("ROS_DISTRO")
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
            ("~/robot_description", f"/{LaunchConfiguration('robot_name').perform(context)}/robot_description"),
        ],
        output="screen",
    )

    cm_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "diff_drive_base_controller"],
        output="screen",
    )

    on_start_cm = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[cm_spawner_node],
        )
    )


    return [
        PushRosNamespace(LaunchConfiguration("robot_name")),
        include_bringup_sim,
        include_bringup_hw,
        robot_state_publisher,
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

    launch_args.append(DeclareLaunchArgument(
        "robot_name",
        default_value="foxy",
        description="Robot name. This name is used as namespace."
    ))

    launch_args.append(DeclareLaunchArgument(
        "pos_x",
        default_value="0.0",
        description="Start position in x axis (only use in mode:=sim)."
    ))

    launch_args.append(DeclareLaunchArgument(
        "pos_y",
        default_value="0.0",
        description="Start position in y axis (only use in mode:=sim)."
    ))

    launch_args.append(DeclareLaunchArgument(
        "pos_z",
        default_value="0.2",
        description="Start position in z axis (only use in mode:=sim)."
    ))

    launch_args.append(DeclareLaunchArgument(
        "world",
        default_value="small_loop",
        choices=[
            "empty",
            "small_loop",
            "small_map",
            "large_map",
            "straight_lane"
        ],
        description="Start simulated world (only use in mode:=sim)."
    ))

    ld = LaunchDescription(launch_args)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
