import os
import xacro
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, SetParameter

def launch_args(context) -> list[LaunchDescriptionEntity]:

    declared_args = []

    declared_args.append(DeclareLaunchArgument(
        "robot_name",
        description="Robot name."
    ))

    declared_args.append(DeclareLaunchArgument(
        "pos_x",
        description="Robot spawn x position (only for simulation environments)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "pos_y",
        description="Robot spawn y position (only for simulation environments)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "pos_z",
        description="Robot spawn z position (only for simulation environments)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "system",
        description="Choose system to start, e.g. robot or gz for Gazebo",
        choices=['gz', 'robot']
    ))

    declared_args.append(DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="World in simulation (only `gz` sim supported for now)."
    ))

    declared_args.append(DeclareLaunchArgument(
        "robot_localization",
        default_value="true",
        description="Start robot localization package for sensor fusion"
    ))

    declared_args.append(DeclareLaunchArgument(
        "slam_toolbox",
        default_value="true",
        description="Start slam_toolbox"
    ))

    declared_args.append(DeclareLaunchArgument(
        "use_sim_time",
        default_value = "true",
        description="Use simulation time /clock topic published by gazebo"
    ))

    declared_args.append(DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Launch rviz2"
    ))

    return declared_args


def launch_setup(context) -> list[LaunchDescriptionEntity]:

    # use_sim_time = True if LaunchConfiguration("system").perform(context) != 'robot' else False 

    robot_desc_content = xacro.process_file(
        PathJoinSubstitution([FindPackageShare("foxy_description"), "urdf", "foxy.urdf.xacro"]).perform(context),
        mappings={
            "robot_name": LaunchConfiguration("robot_name").perform(context),
            "system": LaunchConfiguration("system").perform(context),
            "distro": os.getenv("ROS_DISTRO")
        }
    ).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc_content},
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
            # {"frame_prefix": f'{LaunchConfiguration("robot_name").perform(context)}/'},
        ]
    )

    controllers = GroupAction(
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "diff_drive_base_controller"],
                output='screen',
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
                ]
            ),
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
                    "gz_args": ["-r ", LaunchConfiguration("world")], # `-r` start running simulation immediately
                    'on_exit_shutdown': 'True',
                }.items()
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", LaunchConfiguration("robot_name"),
                    "-topic", f"/{LaunchConfiguration('robot_name').perform(context)}/robot_description",
                    "-x", LaunchConfiguration("pos_x"),
                    "-y", LaunchConfiguration("pos_y"),
                    "-z", LaunchConfiguration("pos_z")
                ],
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
                ],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    f'/{LaunchConfiguration("robot_name").perform(context)}/front_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                    f'/{LaunchConfiguration("robot_name").perform(context)}/front_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                    f'/{LaunchConfiguration("robot_name").perform(context)}/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    f'/{LaunchConfiguration("robot_name").perform(context)}/lidar_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                ],
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
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
        ],
        condition=LaunchConfigurationEquals("system", "gz")
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("foxy_bringup"),
                "config",
                "localization.yaml"
            ]),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        condition=IfCondition(LaunchConfiguration("robot_localization"))
    )

    slam_toolbox = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ]
        ),
        launch_arguments={
            "slam_params_file": PathJoinSubstitution([
                FindPackageShare("foxy_bringup"),
                "config",
                "slam_toolbox.yaml"
            ]),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("slam_toolbox"))
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration("rviz_config")],
        output='screen',
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    return [
        PushRosNamespace(LaunchConfiguration("robot_name")),
        robot_state_publisher_node,
        controllers,
        gz,
        # robot_localization_node,
        # slam_toolbox,
        rviz2
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_args))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
