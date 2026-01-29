#!/usr/bin/env python3
import curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.running = True
        self.get_logger().info('=== El AI Creeper ===')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    
    def ros_spin():
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.01)
    
    spin_thread = threading.Thread(target=ros_spin)
    spin_thread.start()
    
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.nodelay(True)
    
    throttle_cmd = 0
    steering_cmd = 0
    
    try:
        while node.running:
            key = stdscr.getch()
            
            if key == ord('w'):
                throttle_cmd = 1
            elif key == ord('s'):
                throttle_cmd = -1
            elif key == ord('a'):
                steering_cmd = 1
            elif key == ord('d'):
                steering_cmd = -1
            elif key == ord(' '):
                throttle_cmd = 0
                steering_cmd = 0
            elif key == ord('q'):
                node.running = False
                break
            
            msg = Twist()
            msg.linear.x = float(throttle_cmd)
            msg.angular.z = float(steering_cmd)
            node.pub.publish(msg)
            
            time.sleep(0.05)
    
    finally:
        curses.nocbreak()
        curses.echo()
        curses.endwin()
        node.running = False
        spin_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
