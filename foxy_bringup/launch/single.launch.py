from launch_ros.actions import Node
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def controller_spawner(
    controller_name: str,
    robot_name: str,
    use_sim_time: bool,
    timeout: str = "60.0",
) -> Node:
    return Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        name=f"spawn_{controller_name}",
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        arguments=[
            controller_name,
            "--controller-manager",
            f"/{robot_name}/controller_manager",
            "--controller-manager-timeout",
            timeout,
        ],
        output="screen",
    )

def launch_setup(context):

    system = LaunchConfiguration("system").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    standalone = LaunchConfiguration("standalone").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)
    enable_camera_front = LaunchConfiguration("enable/camera/front").perform(context)
    enable_imu_front = LaunchConfiguration("enable/imu/front").perform(context)
    enable_tof_front = LaunchConfiguration("enable/tof/front").perform(context)

    mode = ""

    if (system == "hw"):
        LogInfo(msg="Running system in hardware.")
        use_sim_time = ParameterValue(
            False,
            value_type=bool
        )
        mode = "hardware"
    elif (system == "gz"):
        LogInfo(msg="Running system in gazebo.")
        use_sim_time = ParameterValue(
            True,
            value_type=bool
        )
        mode = "simulation"

    log_level = "debug" if (verbose == "true") else "info"

    if standalone:
        banner = f"""
        ______                                  __          __ 
       / ____/___  _  ____  __      _________  / /_  ____  / /_  System: {system}
      / /_  / __ \| |/_/ / / /_____/ ___/ __ \/ __ \/ __ \/ __/  Name:   {robot_name}
     / __/ / /_/ />  </ /_/ /_____/ /  / /_/ / /_/ / /_/ / /_    Mode:   {mode}
    /_/    \____/_/|_|\__, /     /_/   \____/_.___/\____/\__/  
                     /____/

    Power by: EOLab-HSRW (Germany Kamp-Lintfort)

    """
        print(banner)


    robot_description_path = PathJoinSubstitution([
        FindPackageShare("foxy_description"),
        "urdf",
        "foxy.urdf.xacro"
    ]).perform(context)

    robot_description = ParameterValue(
        xacro.process_file(
            str(robot_description_path),
            mappings={
                "robot_name": robot_name,   # help to scope the topics in gz mode
                "system": system,           # use to namespace topics in gz mode.
                                            # enable and disable gz plugins
                "enable/camera/front": enable_camera_front,
                "enable/imu/front": enable_imu_front,
                "enable/tof/front": enable_tof_front,
            }
        ).toxml(),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        exec_name=f"{robot_name}.robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description,
            "frame_prefix": f"{robot_name}/",
        }],
        ros_arguments=[
            "--log-level",
            log_level,
        ],
    )

    print(robot_description)

    joint_state_broadcaster = controller_spawner(
        controller_name="joint_state_broadcaster",
        robot_name=robot_name,
        use_sim_time=use_sim_time
    )

    diff_drive_controller = controller_spawner(
        controller_name="diff_drive_base_controller",
        robot_name=robot_name,
        use_sim_time=use_sim_time
    )


    return [
        robot_state_publisher,
        joint_state_broadcaster,
        diff_drive_controller,
    ]



def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            name="system",
            choices=["hw", "gz"],
            description="System to run"
        ),
        DeclareLaunchArgument(
            name="robot_name",
            description="Name of the robot. This is use as namespace"
        ),
        DeclareLaunchArgument(
            name="standalone",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Enable standalone single-robot presentation, including "
                "the startup banner."
            ),
        ),
        DeclareLaunchArgument(
            name="enable/camera/front",
            default_value="true", # HARLEY: remove this to force selection of sensor
            choices=["true", "false"],
            description=(
                "Enable front camera. Hardware and simulation."
            ),
        ),
        DeclareLaunchArgument(
            name="enable/tof/front",
            default_value="true", # HARLEY: remove this to force selection of sensor
            choices=["true", "false"],
            description=(
                "Enable front Time-of-Flight. Hardware and simulation."
            ),
        ),
        DeclareLaunchArgument(
            name="enable/imu/front",
            default_value="true", # HARLEY: remove this to force selection of sensor
            choices=["true", "false"],
            description=(
                "Enable IMU. Hardware and simulation."
            ),
        ),
        DeclareLaunchArgument(
            name="verbose",
        ),
        OpaqueFunction(function=launch_setup)
    ])
