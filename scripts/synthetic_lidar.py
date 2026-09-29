#!/usr/bin/env python3
"""Deterministic PointCloud2 publisher for integration smoke tests; NOT MuJoCo data."""
import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class SyntheticLidar(Node):
    def __init__(self):
        super().__init__('synthetic_lidar_smoke_test')
        self.declare_parameter('points', 16384)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('topic', '/benchmark/raw_points')
        self.declare_parameter('frame_id', 'lidar_smoke_frame')
        count = int(self.get_parameter('points').value)
        hz = float(self.get_parameter('rate_hz').value)
        if count < 1 or hz <= 0:
            raise ValueError('points and rate_hz must be positive')
        # Fixed payload between callbacks to catch CPU-vs-GPU nondeterminism.
        data = bytearray(count * 12)
        for i in range(count):
            a = i * 2.399963229728653
            r = 1.0 + (i % 125) * 0.08
            z = -1.0 + (i % 67) * 0.035
            struct.pack_into('<fff', data, 12 * i, r * math.cos(a), r * math.sin(a), z)
        self.cloud = PointCloud2()
        self.cloud.height = 1
        self.cloud.width = count
        self.cloud.fields = [PointField(name=axis, offset=4*j,
            datatype=PointField.FLOAT32, count=1) for j, axis in enumerate(('x','y','z'))]
        self.cloud.is_bigendian = False
        self.cloud.point_step = 12
        self.cloud.row_step = count * 12
        self.cloud.data = bytes(data)
        self.cloud.is_dense = True
        self.cloud.header.frame_id = self.get_parameter('frame_id').value
        self.pub = self.create_publisher(PointCloud2,
            self.get_parameter('topic').value, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0/hz, self.publish_cloud)

    def publish_cloud(self):
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cloud)


def main():
    rclpy.init()
    node = SyntheticLidar()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
