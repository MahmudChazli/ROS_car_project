#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.publisher_ = self.create_publisher(String, 'key', 10)
        
        # Save terminal settings to switch to "raw" mode
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("Controller Node Started.")
        self.get_logger().info("Press WASD To Move.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    

    def run_loop(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in ['w', 's', 'a', 'd']:
                    msg = String()
                    msg.data = key
                    self.publisher_.publish(msg)
                elif key == '\x03':  # Ctrl+C
                    break
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            # Always restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    controller.run_loop()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()