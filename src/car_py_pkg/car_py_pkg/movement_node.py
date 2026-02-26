#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Movement(Node):
  def __init__(self):
    super().__init__("movement_node")

    self.vel_ = 0.0
    self.theta_ = 0.0

    self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

    self.subscriber_ = self.create_subscription(String, "key", self.movement_callback, 10)

    self.timer_ = self.create_timer(0.05, self.publish_vel)
    self.get_logger().info("Movement Node started.")

  def movement_callback(self, msg):
    key_ = msg.data.lower()

    if key_ == 'w':
      self.vel_ += 0.5
    
    elif key_ == 's':
      self.vel_ -= 0.5

    elif key_ == 'd':
      self.theta_ -= 0.5

    elif key_ == 'a':
      self.theta_ += 0.5
    
    elif key_ == 'q':
      self.vel_ = 0.0
      self.theta_ = 0.0
      
    self.vel_   = max(-1.0, min(1.0, self.vel_))
    self.theta_ = max(-1.0, min(1.0, self.theta_))

    if key_ in ['w', 's']:
      self.get_logger().info(f"Current velocity: {self.vel_}")

    else:
      self.get_logger().info(f"Current theta: {self.theta_}")


  def publish_vel(self):
    msg = Twist()

    msg.linear.x = self.vel_
    msg.angular.z = self.theta_
    self.publisher_.publish(msg)
    

def main(args = None):
  rclpy.init(args = args)
  node = Movement()

  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == "__main__":
  main()