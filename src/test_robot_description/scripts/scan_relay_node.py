#!/usr/bin/env python3
"""
scan_relay_node.py
==================
Subscribes to /robot/scan and republishes as /scan_fixed,
AND publishes the base_link → robot/lidar_link/gpu_lidar transform
on /tf (NOT /tf_static) stamped with each scan's sim timestamp.

WHY WE PUBLISH TF HERE INSTEAD OF static_transform_publisher:
  ROS2 Jazzy's static_transform_publisher always stamps /tf_static with
  wall-clock time (~1.78e9 s). Gazebo messages carry sim time (~90–200 s).
  TF2 lookups fail every time → "queue is full" / "timestamp earlier than
  all data in transform cache" → slam_toolbox never gets valid scans → no /map.

  Fix: publish the lidar→base_link transform on /tf (dynamic topic) stamped
  with the scan's own header.stamp (sim time). TF2 finds it immediately.

WHY WE KEEP THE ORIGINAL FRAME_ID (robot/lidar_link/gpu_lidar):
  The previous approach of rewriting frame_id to 'base_link' told slam_toolbox
  "rays originate at the robot centre with no offset". But the lidar is at
  x=+0.2, z=+0.1 relative to base_link, so every ray origin was wrong by that
  offset. Walls were shifted/blurred in the map and scan-matching diverged.
  Keeping the real frame_id + providing the correct TF gives slam_toolbox
  accurate ray geometry → clean map.

LIDAR POSE (from robot.sdf lidar_link pose relative to base_link):
  x=0.2, y=0.0, z=0.1, roll=0, pitch=0, yaw=0
  The lidar is mounted 20 cm forward and 10 cm above the chassis centre.
  No rotation — it scans in the XY plane aligned with the robot forward axis.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from copy import deepcopy


# Lidar pose relative to base_link — read directly from robot.sdf lidar_link pose
LIDAR_X   =  0.2   # metres forward
LIDAR_Y   =  0.0
LIDAR_Z   =  0.1   # metres up
# No rotation (yaw=pitch=roll=0) → quaternion is identity
LIDAR_QX  =  0.0
LIDAR_QY  =  0.0
LIDAR_QZ  =  0.0
LIDAR_QW  =  1.0

PARENT_FRAME = 'base_link'
CHILD_FRAME  = 'robot/lidar_link/gpu_lidar'


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')

        # Dynamic TF broadcaster — publishes on /tf with scan's sim timestamp,
        # avoiding the Jazzy wall-clock bug of static_transform_publisher.
        self._tf_broadcaster = TransformBroadcaster(self)

        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self.sub = self.create_subscription(
            LaserScan, '/robot/scan', self.callback, 10)

        self.get_logger().info(
            f'scan_relay: /robot/scan → /scan_fixed  |  '
            f'TF: {PARENT_FRAME} → {CHILD_FRAME} '
            f'(x={LIDAR_X} y={LIDAR_Y} z={LIDAR_Z}, stamped per scan)')

    def callback(self, msg: LaserScan):
        # ── 1. Republish scan unchanged (frame_id kept as original) ───────
        out = deepcopy(msg)
        # Keep original frame_id: 'robot/lidar_link/gpu_lidar'
        # slam_toolbox will use the TF we publish below to project rays.
        self.pub.publish(out)

        # ── 2. Publish base_link → lidar frame TF at scan's sim timestamp ─
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp   # sim time from the scan itself
        t.header.frame_id = PARENT_FRAME
        t.child_frame_id  = CHILD_FRAME
        t.transform.translation.x = LIDAR_X
        t.transform.translation.y = LIDAR_Y
        t.transform.translation.z = LIDAR_Z
        t.transform.rotation.x = LIDAR_QX
        t.transform.rotation.y = LIDAR_QY
        t.transform.rotation.z = LIDAR_QZ
        t.transform.rotation.w = LIDAR_QW
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
