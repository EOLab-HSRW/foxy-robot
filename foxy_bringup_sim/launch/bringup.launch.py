from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context):

    robot_name: str = LaunchConfiguration("robot_name").perform(context)

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

    Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # f'/{robot_name}/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # f'/{robot_name}/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
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
        spawn_robot
    ]

def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "robot_name"
    ))
    ld.add_action(DeclareLaunchArgument(
        "pos_x"
    ))
    ld.add_action(DeclareLaunchArgument(
        "pos_y"
    ))
    ld.add_action(DeclareLaunchArgument(
        "pos_z"
    ))
    ld.add_action(DeclareLaunchArgument(
        "world",
        choices=[
            "empty",
            "small_loop",
            "small_map",
            "large_map",
            "straight_lane"
        ],
    ))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
