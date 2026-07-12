from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    enable_camera_front = arg_is_true(context, "enable/camera/front")
    enable_tof_front = arg_is_true(context, "enable/tof/front")
    enable_imu_front = arg_is_true(context, "enable/imu/front")

    actions.append(
        Node(
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
    )



    if enable_camera_front:
        actions.append(
            sensor_bridge(
                robot_name=robot_name,
                sensor_name="camera_front",
                arguments=[
                    (
                        f"/{robot_name}/camera_front/image"
                        "@sensor_msgs/msg/Image"
                        "[gz.msgs.Image"
                    ),
                    (
                        f"/{robot_name}/camera_front/camera_info"
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

    if enable_tof_front:
        actions.append(
            sensor_bridge(
                robot_name=robot_name,
                sensor_name="tof_front",
                arguments=[
                    (
                        f"/{robot_name}/tof_front/range"
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
                        f"/{robot_name}/imu_front/imu"
                        "@sensor_msgs/msg/Imu"
                        "[gz.msgs.IMU"
                    ),
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
            name="enable/camera/front",
        ),
        DeclareLaunchArgument(
            name="enable/tof/front",
        ),
        DeclareLaunchArgument(
            name="enable/imu/front",
        ),
        OpaqueFunction(function=launch_setup)
    ])
