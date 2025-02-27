import os
import xacro
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace

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

    return declared_args


def launch_setup(context) -> list[LaunchDescriptionEntity]:

    use_sim_time = {"use_sim_time": True if LaunchConfiguration("system").perform(context) != 'robot' else False }

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
            # {"frame_prefix": f'{LaunchConfiguration("robot_name").perform(context)}/'},
            use_sim_time
        ]
    )

    controllers = GroupAction(
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "diff_drive_base_controller"],
                output='screen'
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

    return [
        PushRosNamespace(LaunchConfiguration("robot_name")),
        robot_state_publisher_node,
        controllers,
        gz,
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_args))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
