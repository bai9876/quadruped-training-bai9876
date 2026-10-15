import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pynput import keyboard

class KeyboardToJoy(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.buttons = [0] * 10  # 假设手柄有10个按键
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.get_logger().info("键盘已绑定: 按 W 站立(按钮0), 按 S 蹲下(按钮1)")

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.buttons[0] = 1
                self.publish_joy()
            elif key.char == 's':
                self.buttons[1] = 1
                self.publish_joy()
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char == 'w':
                self.buttons[0] = 0
                self.publish_joy()
            elif key.char == 's':
                self.buttons[1] = 0
                self.publish_joy()
        except AttributeError:
            pass

    def publish_joy(self):
        msg = Joy()
        msg.buttons = self.buttons
        msg.axes = [0.0] * 6
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
