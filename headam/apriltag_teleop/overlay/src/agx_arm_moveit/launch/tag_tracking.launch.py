from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(__file__).parent.parent / "config" / "apriltag.yaml"
    source = LaunchConfiguration("source")
    is_source = lambda name: IfCondition(PythonExpression(["'", source, f"' == '{name}'"]))

    # Without a calibration file camera_info is empty and no pose is estimated.
    camera_info_url = LaunchConfiguration("camera_info_url")

    v4l2_camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="camera",
        output="screen",
        condition=is_source("v4l2"),
        parameters=[
            {
                "video_device": LaunchConfiguration("video_device"),
                "image_size": [640, 480],
                "camera_frame_id": "camera",
                "camera_info_url": camera_info_url,
            }
        ],
    )

    # WSL cannot stream a USB webcam; frames come from scripts/webcam_stream_windows.py.
    net_camera = Node(
        package="agx_arm_moveit",
        executable="agx_arm_net_camera",
        namespace="camera",
        output="screen",
        condition=is_source("net"),
        parameters=[
            {
                "port": 5600,
                "camera_frame_id": "camera",
                "camera_info_url": camera_info_url,
            }
        ],
    )

    # A webcam has little lens distortion, so detection runs on the raw image;
    # put image_proc in between if the tag pose drifts toward the image edges.
    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        output="screen",
        parameters=[str(config)],
        remappings=[
            ("image_rect", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source",
                default_value="v4l2",
                choices=["v4l2", "net"],
                description="v4l2: USB webcam on this machine; net: stream from webcam_stream_windows.py.",
            ),
            DeclareLaunchArgument(
                "video_device", default_value="/dev/video0", description="Webcam device (source:=v4l2)."
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value="file://" + str(Path.home() / ".ros/camera_info/webcam.yaml"),
                description="Calibration file from camera_calibration.",
            ),
            v4l2_camera,
            net_camera,
            apriltag,
        ]
    )
