#!/usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')
        
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        self.serial_port = None
        for attempt in range(3):
            self.serial_port = self.try_connect()
            if self.serial_port:
                break
            time.sleep(1)
        
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )
        
        self.throttle = 0.0
        self.steering = 0.0
        self.head = 0.0
        
        self.create_timer(0.05, self.send_command)
    
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
    
    def cmd_callback(self, msg: Twist):
        self.throttle = max(-1.0, min(1.0, msg.linear.x))
        self.steering = max(-1.0, min(1.0, msg.angular.z))
        self.head = self.steering
    
    def send_command(self):
        if not self.serial_port:
            return
        
        command = f"{self.throttle:.3f},{self.steering:.3f},{self.head:.3f}\n"
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
