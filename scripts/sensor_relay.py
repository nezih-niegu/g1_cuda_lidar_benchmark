#!/usr/bin/env python3
"""Separate DDS sensor producer. The real simulated head LiDAR remains upstream."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

class SensorRelay(Node):
    def __init__(self):
        super().__init__('g1_lidar_dds_publisher')
        self.declare_parameter('source_topic', '/head/points')
        self.declare_parameter('output_topic', '/benchmark/raw_points')
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        src = self.get_parameter('source_topic').value
        dst = self.get_parameter('output_topic').value
        if src == dst:
            raise ValueError('source_topic and output_topic must differ')
        self.pub = self.create_publisher(PointCloud2, dst, sensor_qos)
        self.sub = self.create_subscription(PointCloud2, src, self.on_cloud, sensor_qos)
        self.frames = 0
        self.get_logger().info(f'Relaying simulated LiDAR over DDS: {src} -> {dst}')

    def on_cloud(self, msg):
        # Preserve original sensor timestamp and frame_id; never regenerate data here.
        self.pub.publish(msg)
        self.frames += 1
        if self.frames % 100 == 0:
            self.get_logger().info(f'Published {self.frames} frames')

def main():
    rclpy.init()
    node = SensorRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
