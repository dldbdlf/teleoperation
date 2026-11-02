import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import yaml
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from _moveit_config_builder import build_moveit_config, declare_common_args


def _build_servo(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    follow = LaunchConfiguration("follow").perform(context) == "true"
    feedback_topic = LaunchConfiguration("feedback_topic").perform(context)
    control_topic = LaunchConfiguration("control_topic").perform(context)

    moveit_config = build_moveit_config(context)
    package_path = moveit_config.package_path

    servo_params = yaml.safe_load(
        (package_path / "config/servo.yaml").read_text(encoding="utf-8")
    )
    servo_params["joint_topic"] = feedback_topic if follow else control_topic

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            {"moveit_servo": servo_params},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return [GroupAction(actions=[PushRosNamespace(namespace), servo_node])]


def generate_launch_description():
    demo_launch = Path(__file__).parent / "demo.launch.py"
    # demo.launch.py declares the shared arguments (arm_type, effector_type, follow, ...)
    # and starts move_group, ros2_control and RViz; servo_node is added on top.
    return LaunchDescription(
        declare_common_args()
        + [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(demo_launch))),
            OpaqueFunction(function=_build_servo),
        ]
    )
