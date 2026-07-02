#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

class LidarTFPublisher(Node):
    def __init__(self):
        super().__init__('lidar_tf_publisher')
        self.pub = self.create_publisher(TFMessage, '/tf', 10)
        self.timer = self.create_timer(0.02, self.publish_tf)
        self.get_logger().info('lidar_tf_publisher running with sim time')

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'robot/lidar_link/gpu_lidar'
        t.transform.translation.x = 0.2
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        t.transform.rotation.w = 1.0
        self.pub.publish(TFMessage(transforms=[t]))

def main(args=None):
    rclpy.init(args=args)
    node = LidarTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
