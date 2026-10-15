from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="Linux joystick device index used by joy_node",
            ),
            DeclareLaunchArgument(
                "use_gamepad",
                default_value="true",
                description="Launch the joystick driver and Xbox gamepad remote node",
            ),
            Node(
                package="quadruped_simulation",
                executable="simulation_node",
                name="mujoco_simulation",
                output="screen",
            ),
            Node(
                package="quadruped_simulation",
                executable="controller_node",
                name="quadruped_controller",
                output="screen",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[{"device_id": LaunchConfiguration("device_id")}],
                condition=IfCondition(LaunchConfiguration("use_gamepad")),
                output="screen",
            ),
            Node(
                package="quadruped_simulation",
                executable="gamepad_node",
                name="gamepad_remote",
                condition=IfCondition(LaunchConfiguration("use_gamepad")),
                output="screen",
            ),
        ]
    )
