#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import tty
import termios
import select


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.pub = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.05, self.loop)

        self.throttle = 0.0
        self.steering = 0.0

        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('=== Keyboard Control ===')
        self.get_logger().info('W/S: forward/reverse')
        self.get_logger().info('A/D: steer + head')
        self.get_logger().info('Space: stop | X: quit')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        key = ''
        if rlist:
            key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key.lower()

    def loop(self):
        key = self.get_key()

        if key == 'w':
            self.throttle = min(1.0, self.throttle + 0.15)
        elif key == 's':
            self.throttle = max(-1.0, self.throttle - 0.15)
        elif key == 'a':
            self.steering = min(1.0, self.steering + 0.15)
        elif key == 'd':
            self.steering = max(-1.0, self.steering - 0.15)
        elif key == ' ':
            self.throttle = 0.0
            self.steering = 0.0
        elif key == 'x':
            raise SystemExit
        else:
            # Decay when no input
            self.throttle *= 0.9
            self.steering *= 0.9

        # A/D controls both steering and head together
        msg = Joy()
        msg.axes = [self.steering, self.throttle, self.steering, 0.0, 0.0, 0.0]
        msg.buttons = [0] * 8
        self.pub.publish(msg)

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
