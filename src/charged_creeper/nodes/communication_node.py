#!/usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # Try to connect to serial (retries 3 times)
        self.serial_port = None
        for attempt in range(3):
            self.serial_port = self.try_connect()
            if self.serial_port:
                break
            time.sleep(1)

        # Steering trim: adjust this if your robot doesn't drive straight
        # Positive = turn right, Negative = turn left
        self.STEERING_TRIM = 0.0

        # Direction pause: wait time when switching forward <-> reverse
        self.DIRECTION_PAUSE = 1.0  # seconds
        self.last_direction = 0
        self.pause_until = 0

        # Simple acceleration
        self.current_throttle = 0.0
        self.ACCEL_STEP = 0.1  # how fast to ramp up/down

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_timer(0.05, self.send_command)  # 20Hz

        self.target_throttle = 0.0
        self.target_steering = 0.0
        self.target_head = 0.0

    def try_connect(self):
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
            self.get_logger().info(f'Connected to {port}')
            return ser
        except serial.SerialException:
            self.get_logger().warn(f'Could not open {port}, retrying...')
            return None

    def get_direction(self, value):
        if value > 0.1:
            return 1   # forward
        elif value < -0.1:
            return -1  # reverse
        return 0       # stopped

    def joy_callback(self, msg: Joy):
        self.target_throttle = msg.axes[1] if len(msg.axes) > 1 else 0.0
        self.target_steering = msg.axes[0] if len(msg.axes) > 0 else 0.0
        self.target_head = msg.axes[2] if len(msg.axes) > 2 else 0.0

    def send_command(self):
        if not self.serial_port:
            return

        now = time.time()
        target_dir = self.get_direction(self.target_throttle)
        current_dir = self.get_direction(self.current_throttle)

        # Direction change: pause before switching forward <-> reverse
        if current_dir != 0 and target_dir != 0 and current_dir != target_dir:
            self.pause_until = now + self.DIRECTION_PAUSE
            self.current_throttle = 0.0

        # During pause, keep throttle at zero
        if now < self.pause_until:
            self.current_throttle = 0.0
        else:
            # Simple acceleration: step toward target
            if self.current_throttle < self.target_throttle:
                self.current_throttle = min(self.current_throttle + self.ACCEL_STEP, self.target_throttle)
            elif self.current_throttle > self.target_throttle:
                self.current_throttle = max(self.current_throttle - self.ACCEL_STEP, self.target_throttle)

        # Apply steering trim
        steering = self.target_steering + self.STEERING_TRIM

        # Send command
        command = f"{self.current_throttle:.3f},{steering:.3f},{self.target_head:.3f}\n"
        try:
            self.serial_port.write(command.encode())
        except serial.SerialException:
            self.serial_port = None

    def destroy_node(self):
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
