import os
import re
import socket

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    LaunchConfiguration
)

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import PackageNotFoundError

def sanitize_ros_namespace(value: str) -> str:
    """Convert a string into a valid single-token ROS 2 namespace."""
    namespace = value.strip().lower()

    # Requested conversion.
    namespace = namespace.replace("-", "_")

    # Replace dots, spaces, and any other unsupported characters.
    namespace = re.sub(r"[^a-z0-9_]", "_", namespace)

    # Avoid repeated underscores and empty edge characters.
    namespace = re.sub(r"_+", "_", namespace).strip("_")

    # Ensure the namespace is non-empty.
    if not namespace:
        namespace = "host"

    # ROS name tokens must not begin with a number.
    if namespace[0].isdigit():
        namespace = f"host_{namespace}"

    return namespace

def launch_setup(context) -> list[object]:
    system = LaunchConfiguration("system").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)

    pkg = ""

    common_arguments = {
        "robot_name": robot_name,
        "verbose": verbose,
        "enable/button/top": LaunchConfiguration("enable/button/top"),
        "enable/camera/front": LaunchConfiguration("enable/camera/front"),
        "enable/tof/front": LaunchConfiguration("enable/tof/front"),
        "enable/imu/front": LaunchConfiguration("enable/imu/front"),
        "enable/leds": LaunchConfiguration("enable/leds"),
    }
    if (system == "gz"):
        pkg = "foxy_bringup_sim"
        system_arguments = {
            "pos_x": LaunchConfiguration("sim/pos_x"),
            "pos_y": LaunchConfiguration("sim/pos_y"),
            "pos_z": LaunchConfiguration("sim/pos_z"),
            "world_name": LaunchConfiguration("sim/world_name"),
            "world_path": LaunchConfiguration("sim/world_path"),
            "headless": LaunchConfiguration("sim/headless"),
            "view_follow": LaunchConfiguration("sim/view_follow"),
        }
    elif (system == "hw"):
        pkg = "foxy_bringup_hw"
        system_arguments = {
        }

    try:
        share: str = FindPackageShare(pkg).perform(context)
    except PackageNotFoundError:
        return [
            LogInfo(msg=f"'{pkg}' not found.")
        ]

    bringup_path = PathJoinSubstitution([share, "launch", "bringup.launch.py"])

    bringup_arguments = common_arguments | system_arguments

    bringup = IncludeLaunchDescription(
        bringup_path,
        launch_arguments=bringup_arguments.items(),
    )

    single = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("foxy_bringup"),
            "launch",
            "single.launch.py"
        ]),
        launch_arguments={
            "system": system,
            "robot_name": robot_name,
            "standalone": "true",
            "enable/camera/front": LaunchConfiguration("enable/camera/front"),
            "enable/tof/front": LaunchConfiguration("enable/tof/front"),
            "enable/imu/front": LaunchConfiguration("enable/imu/front"),
            "enable/leds": LaunchConfiguration("enable/leds"),
            "verbose": verbose,
        }.items()
    )

    return [
        single,
        bringup,
    ]

def generate_launch_description() -> LaunchDescription:

    default_name = os.environ.get("FOXY_NAME") or sanitize_ros_namespace(socket.gethostname())

    return LaunchDescription([
        DeclareLaunchArgument(
            name="system",
            default_value=EnvironmentVariable(
                "FOXY_SYSTEM",
                default_value="gz"
            ),
            choices=["hw", "gz"],
            description="Select the simulation or hardware system."
        ),
        DeclareLaunchArgument(
            name="robot_name",
            default_value=default_name,
            description="Name of the robot. This is use as namespace"
        ),
        DeclareLaunchArgument(
            name="enable/button/top",
            default_value="true",
            choices=["true", "false"],
            description="Enable button."
        ),
        DeclareLaunchArgument(
            name="enable/camera/front",
            default_value="true",
            choices=["true", "false"],
            description="Enable camera."
        ),
        DeclareLaunchArgument(
            name="enable/tof/front",
            default_value="true",
            choices=["true", "false"],
            description="Enable time-of-flight."
        ),
        DeclareLaunchArgument(
            name="enable/imu/front",
            default_value="true",
            choices=["true", "false"],
            description="Enable imu."
        ),
        DeclareLaunchArgument(
            name="enable/leds",
            default_value="true",
            choices=["true", "false"],
            description="Enable LEDs."
        ),
        DeclareLaunchArgument(
            name="verbose",
            default_value="false",
            choices=["true", "false"],
            description="Enable verbose launch output.",
        ),
        DeclareLaunchArgument(
            name="sim/pos_x",
            default_value="0.0",
            description="Robot spawn X position.",
        ),
        DeclareLaunchArgument(
            name="sim/pos_y",
            default_value="0.0",
            description="Robot spawn Y position.",
        ),
        DeclareLaunchArgument(
            name="sim/pos_z",
            default_value="0.2",
            description="Robot spawn Z position.",
        ),
        DeclareLaunchArgument(
            name="sim/world_name",
            default_value="small_loop",
            description=(
                "Runtime name of the Gazebo world to target. This value must match "
                    "the 'name' attribute of the <world> element in the selected SDF "
                    "file, for example: <world name='small_loop'>. "
                    "Gazebo services used to spawn and control entities are scoped by "
                    "this name, such as '/world/small_loop/create'. The world name is "
                    "independent of the SDF filename and therefore cannot reliably be "
                    "derived from sim/world_path."
            ),
            # this are available for demo and testing
            # choices=[
            #     "empty",
            #     "small_loop",
            #     "small_map",
            #     "large_map",
            #     "straight_lane"
            # ],
        ),
        DeclareLaunchArgument(
            name="sim/world_path",
            default_value="",
            description=(
                "Absolute path to the SDF world file to load. This selects the file "
                    "but does not identify the runtime world inside it. sim/world_name "
                    "must still match the <world name='...'> attribute in that file."
            ),
        ),
        DeclareLaunchArgument(
            name="sim/headless",
            default_value="false",
            choices=["true", "false"],
            description="Run the simulator without a GUI"
        ),
        DeclareLaunchArgument(
            name="sim/view_follow",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Make the simulator GUI camera follow the spawned drone."
                "Ignore if sim/headless:=true."
            )
        ),
        OpaqueFunction(function=launch_setup)
    ])
