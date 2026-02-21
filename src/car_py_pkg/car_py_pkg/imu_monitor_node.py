#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
import math

class ImuMonetorNode(Node):
  def __init__(self):
    super().__init__("imu_monetor_node")
    self.subscriber = self.create_subscription(Imu,
                                               "/imu",
                                               self.imu_callback,
                                               10)
    self.publisher = self.create_publisher(Point,
                                           "/imu/monitor",
                                           10)
    self.timer = self.create_timer(0.3, self.timer_callback)

    self.get_logger().info("Imu Monitor Node has started.")

    self.latest_imu = None
  
  def quaternion_to_euler(self, x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


  def imu_callback(self, msg):
    self.latest_imu = msg


  def timer_callback(self):
    if self.latest_imu is not None:
      orient = self.latest_imu.orientation

      r, p, y = self.quaternion_to_euler(orient.x,
                                         orient.y,
                                         orient.z,
                                         orient.w)
      
      monitor_msg = Point()
      monitor_msg.x = round(math.degrees(r), 1)
      monitor_msg.y = round(math.degrees(p), 1)
      monitor_msg.z = round(math.degrees(y), 1)

      self.publisher.publish(monitor_msg)

def main(args = None):
  rclpy.init(args=args)
  node = ImuMonetorNode()

  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == "__main__":
  main()