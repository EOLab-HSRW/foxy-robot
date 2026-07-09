from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(LogInfo(msg="Hello from real hardware"))

    ld.add_action(
        Node(
            namespace="foxy",
            package="foxy_hardware_interface",
            executable="button",
        )
    )

    return ld
