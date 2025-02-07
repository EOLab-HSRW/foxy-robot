import xacro
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_setup(context):

    urdf_path = PathJoinSubstitution([FindPackageShare("foxy_description"), "urdf", "foxy.urdf.xacro"])

    robot_description = {"robot_description": xacro.process_file(urdf_path.perform(context)).toxml()}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name='joint_state_publisher',
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[robot_description],  # Asegura que RViz reciba la descripción del robot
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ]

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

