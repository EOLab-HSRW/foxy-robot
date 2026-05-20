from launch import LaunchDescription
from launch.actions import LogInfo

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(LogInfo(msg="Hello from real hardware"))

    return ld
