from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def launch_setup(context):

    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    pos_x = LaunchConfiguration("pos_x").perform(context)
    pos_y = LaunchConfiguration("pos_y").perform(context)
    pos_z = LaunchConfiguration("pos_z").perform(context)

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

    return [
        spawn_robot
    ]


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
