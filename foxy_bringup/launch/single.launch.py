from pathlib import Path
from tempfile import gettempdir


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
    enable_leds = LaunchConfiguration("enable/leds").perform(context)

    mode = ""

    actions = []

    if (system == "hw"):
        actions.append(LogInfo(msg="Running system in hardware."))
        use_sim_time = ParameterValue(
            False,
            value_type=bool
        )
        mode = "hardware"
    elif (system == "gz"):
        actions.append(LogInfo(msg="Running system in gazebo."))
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
                "enable/leds": enable_leds,
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
        # ros_arguments=[
        #     "--log-level",
        #     log_level,
        # ],
    )
    actions.append(robot_state_publisher)

    if system == "hw":
        controllers_config = PathJoinSubstitution([
            FindPackageShare("foxy_bringup"),
            "config",
            "controllers.yaml",
        ])

        control_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=robot_name,
            exec_name=f"{robot_name}.ros2_control_node",
            output="screen",
            parameters=[
                controllers_config,
                {
                    "use_sim_time": use_sim_time,
                },
            ],
            remappings=[
                ("~/robot_description", f"robot_description",),
            ],
        )
        # Note: in simulation (gz) the control manager
        # is launch along side gazebo server, as part of the plugin:
        # `gz_ros2_control/GazeboSimSystem` in the URDF file of the robot.
        actions.append(control_manager)

    joint_state_broadcaster = controller_spawner(
        controller_name="joint_state_broadcaster",
        robot_name=robot_name,
        use_sim_time=use_sim_time
    )
    actions.append(joint_state_broadcaster)

    diff_drive_controller = controller_spawner(
        controller_name="diff_drive_base_controller",
        robot_name=robot_name,
        use_sim_time=use_sim_time
    )
    actions.append(diff_drive_controller)

    if enable_imu_front == "true":
        imu_frame_id = f"{robot_name}/imu_front_link"
        # imu_sensor_broadcaster parameters must be resolved before the
        # lifecycle controller is configured.
        imu_params_file = (
            Path(gettempdir())
                / f"foxy_{robot_name}_imu_sensor_broadcaster.yaml"
        )

        imu_params_file.write_text(
f"""
/**/imu_sensor_broadcaster:
    ros__parameters:
        sensor_name: imu_front
        frame_id: {imu_frame_id}
        static_covariance_orientation: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
""")

        imu_sensor_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            namespace=robot_name,
            name="spawn_imu_sensor_broadcaster",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
            }],
            arguments=[
                "imu_sensor_broadcaster",
                "--controller-manager",
                f"/{robot_name}/controller_manager",
                "--controller-manager-timeout",
                "60.0",
                "--param-file",
                str(imu_params_file),
            ],
        )
        actions.append(imu_sensor_broadcaster)

        # Raw input: /<robot_name>/imu_sensor_broadcaster/imu
        # Filtered output: /<robot_name>/imu_sensor_broadcaster/imu/filtered
        imu_filter_madgwick = Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            namespace=f"{robot_name}/imu_sensor_broadcaster",
            name="madgwick_filter",
            exec_name=f"{robot_name}.imu_filter_madgwick",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "use_mag": False,
                "publish_tf": False,
                "world_frame": "enu",
                "remove_gravity_vector": False,
            }],
            remappings=[
                ("imu/data_raw", "imu"),
                ("imu/data", "imu/filtered"),
            ],
        )
        actions.append(imu_filter_madgwick)

        # -------------------------------------------------------------------------
        # Local state estimation
        #
        # Inputs:
        #   /<robot_name>/diff_drive_base_controller/odom
        #   /<robot_name>/imu_sensor_broadcaster/imu/filtered
        #
        # Output:
        #   /<robot_name>/odometry/filtered
        #
        # TF:
        #   <robot_name>/odom -> <robot_name>/base_link
        # -------------------------------------------------------------------------

        localization_config = PathJoinSubstitution([
            FindPackageShare("foxy_bringup"),
            "config",
            "localization.yaml",
        ])

        localization_parameters = {
            "use_sim_time": use_sim_time,
             "reset_on_time_jump": True,

            # Multi-robot TF frames.
            "odom_frame": f"{robot_name}/odom",
            "base_link_frame": f"{robot_name}/base_link",
            "world_frame": f"{robot_name}/odom",

            # Wheel odometry is always available.
            "odom0": (
                f"/{robot_name}/"
                "diff_drive_base_controller/odom"
            ),
        }


        # Allow the EKF to continue operating as wheel-only odometry when the
        # front IMU is disabled.
        if enable_imu_front == "true":
            localization_parameters["imu0"] = (
                f"/{robot_name}/"
                "imu_sensor_broadcaster/imu/filtered"
            )

        ekf_node = Node(
            package="robot_localization",
            executable="ekf_node",
            namespace=robot_name,
            name="ekf_node",
            exec_name=f"{robot_name}.ekf_node",
            output="screen",
            parameters=[
                localization_config,
                localization_parameters,
            ],
        )
        actions.append(ekf_node)

    if (system == "hw"):
        # Note: Only load the battery_state_broadcaster
        # for hardware.
        # Gazebo offer a Linear Battery Plugint to emulate
        # batteries. See https://gazebosim.org/api/sim/8/battery.html
        # but we dont have the ros2 control hardware interface for it.
        battery_state_broadcaster = controller_spawner(
            controller_name="battery_state_broadcaster",
            robot_name=robot_name,
            use_sim_time=use_sim_time
        )
        actions.append(battery_state_broadcaster)

    return actions



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
            name="verbose",
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
            choices=["true", "false"],
            description=(
                "Enable front camera. Hardware and simulation."
            ),
        ),
        DeclareLaunchArgument(
            name="enable/tof/front",
            choices=["true", "false"],
            description=(
                "Enable front Time-of-Flight. Hardware and simulation."
            ),
        ),
        DeclareLaunchArgument(
            name="enable/imu/front",
            choices=["true", "false"],
            description=(
                "Enable IMU. Hardware and simulation."
            ),
        ),
        DeclareLaunchArgument(
            name="enable/leds",
            choices=["true", "false"],
            description=(
                "Enable LEDs. Hardware and simulation."
            ),
        ),
        OpaqueFunction(function=launch_setup)
    ])
