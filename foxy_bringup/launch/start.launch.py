from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
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
        # " ",
        # "system:=", mode,
    ])

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


    include_bringup = IncludeLaunchDescription(
        bringup
    )

    return [include_bringup]


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
