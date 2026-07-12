from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context):

    robot_name = LaunchConfiguration("robot_name").perform(context)
    world = LaunchConfiguration("world").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)
    headless = LaunchConfiguration("headless").perform(context)

    launch_world = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("foxy_bringup_sim"),
            "launch",
            "world.launch.py"
        ]),
        launch_arguments={
            "robot_name": robot_name,
            "world_name": world,
            "verbose": verbose,
            "headless": headless,
        }.items()
    )
    spawn_robot = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("foxy_bringup_sim"),
            "launch",
            "spawn.launch.py"
        ]),
        launch_arguments={
            "robot_name": robot_name,
            "world_name": world,
            "verbose": verbose,
            "pos_x": "0.0",
            "pos_y": "0.0",
            "pos_z": "0.2",
            "enable/camera/front": LaunchConfiguration("enable/camera/front"),
            "enable/tof/front": LaunchConfiguration("enable/tof/front"),
            "enable/imu/front": LaunchConfiguration("enable/imu/front"),
        }.items()
    )

    return [
        launch_world,
        spawn_robot
    ]

    # note: this is the launch for sim
    # so, always use_sim_time = True
    use_sim_time: bool = True

    world_name = LaunchConfiguration("world").perform(context)
    world_path = PathJoinSubstitution([FindPackageShare("foxy_description"), "worlds", f"{world_name}.world"]).perform(context)
    gz_gui_config_path = PathJoinSubstitution([FindPackageShare("foxy_description"), "gz-config", "client-gui.config"]).perform(context)

    include_gz = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ros_gz_sim"),
            "launch",
            "gz_sim.launch.py"
        ]),
        launch_arguments={
            # "gz_args": ["-r ", LaunchConfiguration("world"), " --gui-config ", gz_gui_config_path], # `-r` start running simulation immediately
            "gz_args": [world_path, " --gui-config ", gz_gui_config_path, " -r"], # `-r` start running simulation immediately
            'on_exit_shutdown': 'True',
            }.items()
    )


    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_name,
            "-topic", f"/{robot_name}/robot_description",
            "-x", LaunchConfiguration("pos_x"),
            "-y", LaunchConfiguration("pos_y"),
            "-z", LaunchConfiguration("pos_z")
            ],
        parameters=[
            {"use_sim_time": use_sim_time}
            ],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/{robot_name}/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{robot_name}/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # f'/{robot_name}/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # f'/{robot_name}/lidar_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        parameters=[
            {"use_sim_time": use_sim_time}
        ],
        # remappings=[
        #     (f'/{LaunchConfiguration("robot_name").perform(context)}/lidar_sensor/lidar', '/laser_scan')
        # ],
        # NOTE (harley): the expansion of gz topic only works for 1 level higher
        # parameters=[
        #     {"expand_gz_topic_names": True}
        # ],
        output='screen'
    )

    return [
        include_gz,
        spawn_robot,
        ros_gz_bridge
    ]

def generate_launch_description() -> LaunchDescription:

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name"
        ),
        DeclareLaunchArgument(
            "verbose"
        ),
        DeclareLaunchArgument(
            "pos_x"
        ),
        DeclareLaunchArgument(
            "pos_y"
        ),
        DeclareLaunchArgument(
            "pos_z"
        ),
        DeclareLaunchArgument(
            "world",
            choices=[
                "empty",
                "small_loop",
                "small_map",
                "large_map",
                "straight_lane"
            ]
        ),
        DeclareLaunchArgument(
            "headless"
        ),
        DeclareLaunchArgument(
            "enable/camera/front"
        ),
        DeclareLaunchArgument(
            "enable/tof/front"
        ),
        DeclareLaunchArgument(
            "enable/imu/front"
        ),
        OpaqueFunction(function=launch_setup)
    ])
