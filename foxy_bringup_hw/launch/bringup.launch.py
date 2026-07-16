from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_bool(launch_arg: str, context):
    return LaunchConfiguration(launch_arg).perform(context) == "true"

def launch_setup(context):
    actions = []

    robot_name = LaunchConfiguration("robot_name").perform(context)
    enable_button_top = get_bool("enable/button/top", context)
    enable_camera_front = get_bool("enable/camera/front", context)

    if enable_button_top:
        actions.append(
            Node(
                namespace=robot_name,
                package="foxy_hardware_interface",
                executable="button",
            )
        )

    if enable_camera_front:
        # Capture using the native 4:3, 30 FPS Argus sensor mode and downscale
        # using the Jetson video converter.
        #
        # Pipeline:
        #   sensor: 1640x1232 NV12 @ 30 FPS
        #       -> nvvidconv
        #   output: 640x480 I420 @ 30 FPS
        #       -> NVIDIA JPEG encoder
        #   ROS: sensor_msgs/msg/CompressedImage
        #
        # Do not append an appsink. GSCam creates and connects its own appsink.
        GSCAM_PIPELINE = (
            "nvarguscamerasrc sensor-id=0 sensor-mode=3 ! "
                "video/x-raw(memory:NVMM),"
                "width=(int)1640,"
                "height=(int)1232,"
                "format=(string)NV12,"
                "framerate=(fraction)30/1 ! "
                "nvvidconv ! "
                "video/x-raw,"
                "width=(int)640,"
                "height=(int)480,"
                "format=(string)I420,"
                "framerate=(fraction)30/1 ! "
                "nvjpegenc"
        )

        actions.append(
            Node(
                package="gscam",
                executable="gscam_node",
                name="camera_front",
                namespace=robot_name,
                output="screen",
                parameters=[
                    {
                        # Passing the pipeline as a ROS parameter avoids relying on
                        # the process environment variable GSCAM_CONFIG.
                        "gscam_config": GSCAM_PIPELINE,

                        # Must match the name in the calibration YAML.
                        "camera_name": "camera_front",

                        # package:// is resolved from the installed package share.
                        "camera_info_url": (
                            "package://foxy_bringup_hw/"
                                "config/ov5647_640x480.yaml"
                        ),

                        # Matches the existing robot description.
                        "frame_id": "camera_front_optical_frame",

                        # Publish JPEG directly. This avoids cv_bridge, OpenCV and
                        # compressed_image_transport on the Jetson.
                        "image_encoding": "jpeg",

                        # Prevent GStreamer from synchronizing delivery to its sink.
                        "sync_sink": False,

                        # Reliable is compatible with the default RViz subscriber.
                        # The queue depth remains one in GSCam.
                        "use_sensor_data_qos": False,
                    }
                ],
                remappings=[
                    # GSCam's direct-JPEG default:
                    #   camera/image_raw/compressed
                    #
                    # Remap it to the hardware API matching the robot's camera name.
                    (
                        "camera/image_raw/compressed",
                        "camera_front/image/compressed",
                    ),
                    (
                        "camera/camera_info",
                        "camera_front/camera_info",
                    ),
                ],
            )

        )

    return actions


def generate_launch_description():


    return LaunchDescription([
        DeclareLaunchArgument(
            name="robot_name",
        ),
        DeclareLaunchArgument(
            name="verbose",
        ),
        DeclareLaunchArgument(
            name="enable/button/top",
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
