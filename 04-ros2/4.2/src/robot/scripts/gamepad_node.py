#!/usr/bin/env python3
"""Translate Xbox-compatible joystick buttons into robot posture commands."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class GamepadNode(Node):
    def __init__(self) -> None:
        super().__init__("gamepad_remote")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("control_mode_topic", "/control_mode")
        self.declare_parameter("control_status_topic", "/control_status")
        self.declare_parameter("stand_button", 0)
        self.declare_parameter("crouch_button", 1)

        joy_topic = self.get_parameter("joy_topic").value
        control_mode_topic = self.get_parameter("control_mode_topic").value
        control_status_topic = self.get_parameter("control_status_topic").value
        self._stand_button = self.get_parameter("stand_button").value
        self._crouch_button = self.get_parameter("crouch_button").value
        if self._stand_button < 0 or self._crouch_button < 0:
            raise ValueError("Gamepad button indices must be non-negative")
        if self._stand_button == self._crouch_button:
            raise ValueError("Stand and crouch buttons must use different indices")

        self._previous_buttons: list[int] = []
        self._invalid_layout_reported = False
        self._button_conflict_reported = False
        self._mode_publisher = self.create_publisher(String, control_mode_topic, 10)
        self._joy_subscription = self.create_subscription(
            Joy, joy_topic, self._joy_callback, 10
        )
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_subscription = self.create_subscription(
            String, control_status_topic, self._status_callback, status_qos
        )
        self.get_logger().info(
            f"Xbox controls ready: button {self._stand_button} stands, "
            f"button {self._crouch_button} crouches"
        )

    def _joy_callback(self, message: Joy) -> None:
        required_button_count = max(self._stand_button, self._crouch_button) + 1
        if len(message.buttons) < required_button_count:
            if not self._invalid_layout_reported:
                self.get_logger().warning(
                    f"Joy message has {len(message.buttons)} buttons; "
                    f"need at least {required_button_count}"
                )
                self._invalid_layout_reported = True
            self._previous_buttons = list(message.buttons)
            return
        self._invalid_layout_reported = False

        stand_pressed = bool(message.buttons[self._stand_button])
        crouch_pressed = bool(message.buttons[self._crouch_button])
        if stand_pressed and crouch_pressed:
            if not self._button_conflict_reported:
                self.get_logger().warning("Stand and crouch buttons pressed together; ignoring")
                self._button_conflict_reported = True
            self._previous_buttons = list(message.buttons)
            return
        self._button_conflict_reported = False

        previous_stand = (
            self._stand_button < len(self._previous_buttons)
            and bool(self._previous_buttons[self._stand_button])
        )
        previous_crouch = (
            self._crouch_button < len(self._previous_buttons)
            and bool(self._previous_buttons[self._crouch_button])
        )
        if stand_pressed and not previous_stand:
            self._publish_mode("standing")
        elif crouch_pressed and not previous_crouch:
            self._publish_mode("damping")
        self._previous_buttons = list(message.buttons)

    def _publish_mode(self, mode: str) -> None:
        message = String()
        message.data = mode
        self._mode_publisher.publish(message)
        self.get_logger().info(f"Requested {mode} mode")

    def _status_callback(self, message: String) -> None:
        self.get_logger().info(f"Controller mode: {message.data}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GamepadNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
