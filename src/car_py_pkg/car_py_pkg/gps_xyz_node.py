#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point

import math

class GpsXyzNode(Node):
  def __init__(self):
    super().__init__("gps_xyz_node")
    self.subscriber = self.create_subscription(NavSatFix,
                                               "/gps/data",
                                               self.gps_callback,
                                               10)

    self.publisher = self.create_publisher(Point, "/gps/xyz", 10)

    self.timer = self.create_timer(0.1, self.timer_callback)

    self.latest_gps  = None
    self.origin_lat  = 0.0
    self.origin_long = 0.0

    self.get_logger().info("GPS XYZ Node has started.")


  def gps_callback(self, msg):
    self.latest_gps = msg


  def timer_callback(self):
    if self.latest_gps is not None:
      lat = self.latest_gps.latitude
      long = self.latest_gps.longitude
      alt = self.latest_gps.altitude

      x = (long - self.origin_long) * (111320 * math.cos(math.radians(self.origin_lat)))
      y = (lat - self.origin_lat) * 111320

      x_meters = round(x, 2)
      y_meters = round(y, 2)
      alt_meters = round(alt, 2)

      xyz_msg = Point()
      xyz_msg.x = x_meters
      xyz_msg.y = y_meters
      xyz_msg.z = alt_meters

      self.publisher.publish(xyz_msg)

      


def main(args = None):
  rclpy.init(args = args)
  node = GpsXyzNode()

  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == "__main__":
  main()