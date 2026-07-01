#!/usr/bin/env python3
"""
scan_relay_node.py
==================
Subscribes to /robot/scan and republishes as /scan_fixed
with header.frame_id rewritten to 'base_link'.

WHY THIS EXISTS:
  Gazebo sets LaserScan.header.frame_id = 'robot/lidar_link/gpu_lidar'.
  static_transform_publisher in ROS2 Jazzy cannot use sim time reliably —
  it always stamps /tf_static with wall-clock time (~1.78 billion s) while
  all Gazebo messages carry sim time (~90 s). TF2 lookup fails every time
  → "queue is full" spam → slam_toolbox never initialises → no /map.

  Fix: rewrite frame_id to 'base_link', which IS in the TF tree at correct
  sim timestamps (DiffDrive plugin publishes odom→base_link via /tf bridge).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from copy import deepcopy


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self.sub = self.create_subscription(
            LaserScan, '/robot/scan', self.callback, 10)
        self.get_logger().info(
            'scan_relay: /robot/scan → /scan_fixed (frame_id → base_link)')

    def callback(self, msg: LaserScan):
        out = deepcopy(msg)
        out.header.frame_id = 'base_link'
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
