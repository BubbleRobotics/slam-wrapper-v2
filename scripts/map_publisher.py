#!/usr/bin/env python3
"""
Standalone node to load and publish PCD map files.
Can be used without running the full SLAM system.

Usage:
  ros2 run ros2_orb_slam3 map_publisher.py --ros-args -p pcd_file:=/path/to/map.pcd

Parameters:
  - pcd_file: Path to the PCD file to load
  - frame_id: TF frame for the pointcloud (default: 'map')
  - topic: Topic to publish on (default: '/map_publisher/pointcloud')
  - publish_rate: Publishing rate in Hz, 0 = publish once (default: 1.0)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import os


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # Parameters
        self.declare_parameter('pcd_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/map_publisher/pointcloud')
        self.declare_parameter('publish_rate', 1.0)  # Hz, 0 = publish once

        self.pcd_file = self.get_parameter('pcd_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Publisher
        self.publisher = self.create_publisher(PointCloud2, self.topic, 10)

        # Load PCD file
        self.points = []
        if self.pcd_file:
            self.load_pcd(self.pcd_file)
        else:
            self.get_logger().error('No PCD file specified! Use --ros-args -p pcd_file:=/path/to/file.pcd')
            return

        if not self.points:
            self.get_logger().error('No points loaded from PCD file')
            return

        self.get_logger().info(f'Loaded {len(self.points)} points from {self.pcd_file}')
        self.get_logger().info(f'Publishing on topic: {self.topic} with frame: {self.frame_id}')

        # Publish once immediately
        self.publish_pointcloud()

        # Set up periodic publishing if rate > 0
        if self.publish_rate > 0:
            period = 1.0 / self.publish_rate
            self.timer = self.create_timer(period, self.publish_pointcloud)
            self.get_logger().info(f'Publishing at {self.publish_rate} Hz')
        else:
            self.get_logger().info('Published once (rate=0)')

    def load_pcd(self, filepath):
        """Load points from a PCD file (ASCII or binary)."""
        if not os.path.exists(filepath):
            self.get_logger().error(f'File not found: {filepath}')
            return

        try:
            with open(filepath, 'rb') as f:
                # Read header
                header_lines = []
                while True:
                    line = f.readline().decode('utf-8', errors='ignore').strip()
                    header_lines.append(line)
                    if line.startswith('DATA'):
                        break

                # Parse header
                num_points = 0
                is_binary = False
                for line in header_lines:
                    if line.startswith('POINTS'):
                        num_points = int(line.split()[1])
                    elif line.startswith('DATA'):
                        is_binary = 'binary' in line.lower()

                self.get_logger().info(f'PCD format: {"binary" if is_binary else "ascii"}, points: {num_points}')

                if is_binary:
                    # Read binary data (assuming XYZ float32)
                    for _ in range(num_points):
                        data = f.read(12)  # 3 floats * 4 bytes
                        if len(data) == 12:
                            x, y, z = struct.unpack('fff', data)
                            self.points.append((x, y, z))
                else:
                    # Read ASCII data
                    for _ in range(num_points):
                        line = f.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            parts = line.split()
                            if len(parts) >= 3:
                                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                                self.points.append((x, y, z))

        except Exception as e:
            self.get_logger().error(f'Error loading PCD: {e}')

    def publish_pointcloud(self):
        """Publish loaded points as PointCloud2."""
        if not self.points:
            return

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.height = 1
        msg.width = len(self.points)
        msg.is_dense = True

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width

        # Pack points into binary data
        data = bytearray()
        for x, y, z in self.points:
            data.extend(struct.pack('fff', x, y, z))

        msg.data = bytes(data)

        self.publisher.publish(msg)
        self.get_logger().debug(f'Published {len(self.points)} points')


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
