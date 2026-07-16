from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def arg_is_true(context, name: str) -> bool:
    return (
        LaunchConfiguration(name)
        .perform(context) == "true"
    )


def sensor_bridge(
    *,
    robot_name: str,
    sensor_name: str,
    arguments: list[str],
    **node_kwargs,
) -> Node:
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        exec_name=f"{robot_name}.parameter_bridge",
        namespace=robot_name,
        name=f"{sensor_name}",
        arguments=arguments,
        output="screen",
        **node_kwargs,
    )


def launch_setup(context):

    actions = []

    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    pos_x = LaunchConfiguration("pos_x").perform(context)
    pos_y = LaunchConfiguration("pos_y").perform(context)
    pos_z = LaunchConfiguration("pos_z").perform(context)

    view_follow = arg_is_true(context, "view_follow")

    enable_camera_front = arg_is_true(context, "enable/camera/front")
    enable_tof_front = arg_is_true(context, "enable/tof/front")
    enable_imu_front = arg_is_true(context, "enable/imu/front")
    enable_leds = arg_is_true(context, "enable/leds")

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            f"/{robot_name}/robot_description",
            "-x",
            pos_x,
            "-y",
            pos_y,
            "-z",
            pos_z,
            "-world",
            world_name,
        ],
        output="both",
    )

    if view_follow:
        follow_robot = ExecuteProcess(
            cmd=[
                "gz",
                "topic",
                "-t",
                "/gui/track",
                "-m",
                "gz.msgs.CameraTrack",
                "-p",
                (
                    "track_mode: FOLLOW "
                    f'follow_target: {{name: "{robot_name}"}} '
                    "follow_offset: {x: -1.0, y: 0.0, z: 0.7}"
                ),
            ],
            output="screen",
        )

        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[
                        TimerAction(
                            period=1.0,
                            actions=[follow_robot],
                        ),
                    ],
                )
            )
        )

    actions.append(spawn_robot)

    if enable_camera_front:
        camera_name = "front"
        camera_topic_prefix = f"/{robot_name}/camera/{camera_name}"
        actions.append(
            sensor_bridge(
                robot_name=robot_name,
                sensor_name="camera_front",
                arguments=[
                    (
                        f"/{camera_topic_prefix}/image"
                        "@sensor_msgs/msg/Image"
                        "[gz.msgs.Image"
                    ),
                    (
                        f"/{camera_topic_prefix}/camera_info"
                        "@sensor_msgs/msg/CameraInfo"
                        "[gz.msgs.CameraInfo"
                    ),
                ],
                parameters=[
                    {
                        "override_frame_id": (
                            f"{robot_name}/camera_front_optical_frame"
                        ),
                    },
                ],
            )
        )
        actions.append(
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                name="camera_front_image_bridge",
                arguments=[
                    f"{camera_topic_prefix}/image",
                ],
                parameters=[
                    {
                        "qos": "default",
                        "lazy": False,

                    }
                ],
                output="screen",
            )
        )

    if enable_tof_front:
        actions.append(
            sensor_bridge(
                robot_name=robot_name,
                sensor_name="tof_front",
                arguments=[
                    (
                        f"/{robot_name}/tof/front/range"
                        "@sensor_msgs/msg/LaserScan"
                        "[gz.msgs.LaserScan"
                    ),
                ],
            )
        )

    if enable_imu_front:
        actions.append(
            sensor_bridge(
                robot_name=robot_name,
                sensor_name="imu_front",
                arguments=[
                    (
                        f"/{robot_name}/imu/front/imu"
                        "@sensor_msgs/msg/Imu"
                        "[gz.msgs.IMU"
                    ),
                ],
            )
        )

    # Harley note: There is not need to play smart here
    # so all the mapping is done directly
    #
    # Important: These mappings relay in consistency of name
    # all the names define here need to match the <channel name="<here>">
    # define under `foxy_description/sensors/leds.xacro`.
    if enable_leds:
        actions.append(
            sensor_bridge(
                robot_name=robot_name,
                sensor_name="leds_bridge",
                arguments=[
                    # Gazebo -> ROS: current LED colors
                    ( f"/model/{robot_name}/led/front_left/get"  "@std_msgs/msg/ColorRGBA" "[gz.msgs.Color"),
                    ( f"/model/{robot_name}/led/front_right/get" "@std_msgs/msg/ColorRGBA" "[gz.msgs.Color"),
                    ( f"/model/{robot_name}/led/rear_left/get"   "@std_msgs/msg/ColorRGBA" "[gz.msgs.Color"),
                    ( f"/model/{robot_name}/led/rear_right/get"  "@std_msgs/msg/ColorRGBA" "[gz.msgs.Color"),
                    ( f"/model/{robot_name}/led/top/get"         "@std_msgs/msg/ColorRGBA" "[gz.msgs.Color"),

                    # ROS -> Gazebo: LED commands
                    ( f"/model/{robot_name}/led/front_left/set"  "@std_msgs/msg/ColorRGBA" "]gz.msgs.Color"),
                    ( f"/model/{robot_name}/led/front_right/set" "@std_msgs/msg/ColorRGBA" "]gz.msgs.Color"),
                    ( f"/model/{robot_name}/led/rear_left/set"   "@std_msgs/msg/ColorRGBA" "]gz.msgs.Color"),
                    ( f"/model/{robot_name}/led/rear_right/set"  "@std_msgs/msg/ColorRGBA" "]gz.msgs.Color"),
                    ( f"/model/{robot_name}/led/top/set"         "@std_msgs/msg/ColorRGBA" "]gz.msgs.Color"),

                    # ROS-facing topic names
                    "--ros-args",
                    "-r",
                    ( f"/model/{robot_name}/led/front_left/get" f":=/{robot_name}/led/front_left/get"),
                    "-r",
                    ( f"/model/{robot_name}/led/front_right/get" f":=/{robot_name}/led/front_right/get"),
                    "-r",
                    ( f"/model/{robot_name}/led/rear_left/get" f":=/{robot_name}/led/rear_left/get"),
                    "-r",
                    ( f"/model/{robot_name}/led/rear_right/get" f":=/{robot_name}/led/rear_right/get"),
                    "-r",
                    ( f"/model/{robot_name}/led/top/get" f":=/{robot_name}/led/top/get"),

                    "-r",
                    ( f"/model/{robot_name}/led/front_left/set" f":=/{robot_name}/led/front_left/set"),
                    "-r",
                    ( f"/model/{robot_name}/led/front_right/set" f":=/{robot_name}/led/front_right/set"),
                    "-r",
                    ( f"/model/{robot_name}/led/rear_left/set" f":=/{robot_name}/led/rear_left/set"),
                    "-r",
                    ( f"/model/{robot_name}/led/rear_right/set" f":=/{robot_name}/led/rear_right/set"),
                    "-r",
                    ( f"/model/{robot_name}/led/top/set" f":=/{robot_name}/led/top/set"),
                ],
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            name="robot_name"
        ),
        DeclareLaunchArgument(
            name="world_name"
        ),
        DeclareLaunchArgument(
            name="pos_x"
        ),
        DeclareLaunchArgument(
            name="pos_y"
        ),
        DeclareLaunchArgument(
            name="pos_z"
        ),
        DeclareLaunchArgument(
            name="view_follow",
        ),
        DeclareLaunchArgument(
            name="enable/camera/front",
        ),
        DeclareLaunchArgument(
            name="enable/tof/front",
        ),
        DeclareLaunchArgument(
            name="enable/imu/front",
        ),
        DeclareLaunchArgument(
            name="enable/leds",
        ),
        OpaqueFunction(function=launch_setup)
    ])
