import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context):

    world_name = LaunchConfiguration("world_name").perform(context)
    world_path = LaunchConfiguration("world_path").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context) == "true"

    if (world_path != ""):
        LogInfo(msg="world_path passed. Overide the world_name launch argument")
    else:
        world_path = PathJoinSubstitution([
            FindPackageShare("foxy_gz"),
            "worlds",
            f"{world_name}.world"
        ]).perform(context)

    server_config = PathJoinSubstitution([
        FindPackageShare("foxy_gz"),
        "gz-config",
        "server.config"
    ]).perform(context)

    set_server_config = SetEnvironmentVariable(
        name="GZ_SIM_SERVER_CONFIG_PATH",
        value=server_config
    )

    server_arguments = ["-r", "-s", str(world_path)]
    if verbose:
        server_arguments.extend(["-v", "4"])

    gz_args_server = " ".join(
        shlex.quote(argument)
        for argument in server_arguments
    )

    start_gz_server = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("foxy_bringup_sim"),
            "launch",
            "gz_sim.launch.py"
        ]),
        launch_arguments={
            "gz_args": gz_args_server,
            "on_exit_shutdown": "true",
        }.items(),
    )

    start_gz_gui = ExecuteProcess(
        name="gz_gui",
        cmd=[
            "gz",
            "sim",
            "-g",
            "-v",
            "4" if verbose else "1",
            "--gui-config",
            PathJoinSubstitution([
                FindPackageShare("foxy_gz"),
                "gz-config",
                "client-gui.config"
            ]).perform(context)
        ],
        condition=UnlessCondition(LaunchConfiguration("headless"))
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )


    return [
        set_server_config,
        start_gz_server,
        start_gz_gui,
        clock_bridge,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            name="robot_name",
        ),
        DeclareLaunchArgument(
            name="world_name",
            description=(
                "If you pass the world_path you still need to pass this "
                "because if use to bridge the clock"
                "Keep in mind that the world name, if no carefull can be"
                "different that the world file name."
            )
        ),
        DeclareLaunchArgument(
            name="world_path",
            default_value="",
            description="Absolute path with full world name. Overide the world_name selection."
        ),
        DeclareLaunchArgument(
            name="verbose",
            choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            name="headless",
        ),
        OpaqueFunction(function=launch_setup)
    ])
