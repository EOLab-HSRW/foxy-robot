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

def launch_setup(context) -> list[object]:
    system = LaunchConfiguration("system").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)

    pkg = ""

    common_arguments = {
        "robot_name": robot_name,
        "verbose": verbose,
        "enable/camera/front": LaunchConfiguration("enable/camera/front"),
        "enable/tof/front": LaunchConfiguration("enable/tof/front"),
        "enable/imu/front": LaunchConfiguration("enable/imu/front"),
    }
    if (system == "gz"):
        pkg = "foxy_bringup_sim"
        system_arguments = {
            "pos_x": LaunchConfiguration("sim/pos_x"),
            "pos_y": LaunchConfiguration("sim/pos_y"),
            "pos_z": LaunchConfiguration("sim/pos_z"),
            "world": LaunchConfiguration("sim/world"),
            "headless": LaunchConfiguration("sim/headless"),
            "sim/camera_follow": LaunchConfiguration("sim/camera_follow"), # TODO
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
            "verbose": verbose,
        }.items()
    )

    return [
        single,
        bringup,
    ]

def generate_launch_description() -> LaunchDescription:

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
            default_value=EnvironmentVariable(
                "FOXY_NAME",
                default_value="foxy"
            ),
            description="Name of the robot. This is use as namespace"
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
            default_value="0.8",
            description="Robot spawn Z position.",
        ),
        DeclareLaunchArgument(
            name="sim/world",
            default_value="small_loop",
            choices=[
                "empty",
                "small_loop",
                "small_map",
                "large_map",
                "straight_lane"
            ],
            description=(
                "Simulation world name without the '.sdf' extension."
            ),
        ),
        DeclareLaunchArgument(
            name="sim/headless",
            default_value="false",
            choices=["true", "false"],
            description="Run the simulator without a GUI"
        ),
        DeclareLaunchArgument(
            name="sim/camera_follow",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Make the simulator GUI camera follow the spawned drone."
                "Ignore if sim/headless:=true."
            )
        ),
        OpaqueFunction(function=launch_setup)
    ])
