#!/usr/bin/env python3
"""
Standalone node to load and publish occupancy grid files.
Can publish as PointCloud2 (3D voxels) or OccupancyGrid (2D projected).

Usage:
  ros2 run ros2_orb_slam3 occupancy_grid_publisher.py --ros-args -p pcd_file:=/path/to/occupancy.pcd

Parameters:
  - pcd_file: Path to the occupancy grid PCD file to load
  - frame_id: TF frame for the grid (default: 'map')
  - topic_3d: Topic for 3D PointCloud2 (default: '/occupancy_grid_publisher/voxels')
  - topic_2d: Topic for 2D OccupancyGrid (default: '/occupancy_grid_publisher/grid')
  - publish_rate: Publishing rate in Hz, 0 = publish once (default: 1.0)
  - publish_2d: Also publish as 2D OccupancyGrid (default: True)
  - resolution: Grid resolution in meters for 2D projection (default: 0.1)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
import struct
import os
import math


class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')

        # Parameters
        self.declare_parameter('pcd_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic_3d', '/occupancy_grid_publisher/voxels')
        self.declare_parameter('topic_2d', '/occupancy_grid_publisher/grid')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('publish_2d', True)
        self.declare_parameter('resolution', 0.1)

        self.pcd_file = self.get_parameter('pcd_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic_3d = self.get_parameter('topic_3d').get_parameter_value().string_value
        self.topic_2d = self.get_parameter('topic_2d').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.publish_2d = self.get_parameter('publish_2d').get_parameter_value().bool_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value

        # Publishers
        self.publisher_3d = self.create_publisher(PointCloud2, self.topic_3d, 10)
        self.publisher_2d = self.create_publisher(OccupancyGrid, self.topic_2d, 10)

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

        self.get_logger().info(f'Loaded {len(self.points)} voxels from {self.pcd_file}')
        self.get_logger().info(f'Publishing 3D on: {self.topic_3d}')
        if self.publish_2d:
            self.get_logger().info(f'Publishing 2D on: {self.topic_2d}')

        # Pre-compute 2D grid if needed
        self.occupancy_grid_msg = None
        if self.publish_2d:
            self.occupancy_grid_msg = self.create_2d_grid()

        # Publish once immediately
        self.publish_all()

        # Set up periodic publishing if rate > 0
        if self.publish_rate > 0:
            period = 1.0 / self.publish_rate
            self.timer = self.create_timer(period, self.publish_all)
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

    def create_2d_grid(self):
        """Project 3D voxels to 2D occupancy grid."""
        if not self.points:
            return None

        # Find bounds
        min_x = min(p[0] for p in self.points)
        max_x = max(p[0] for p in self.points)
        min_y = min(p[1] for p in self.points)
        max_y = max(p[1] for p in self.points)

        # Compute grid dimensions
        width = int(math.ceil((max_x - min_x) / self.resolution)) + 1
        height = int(math.ceil((max_y - min_y) / self.resolution)) + 1

        self.get_logger().info(f'2D grid: {width}x{height} cells, resolution: {self.resolution}m')

        # Create grid data (initialized to -1 = unknown)
        grid_data = [-1] * (width * height)

        # Project points to 2D
        occupied_cells = set()
        for x, y, z in self.points:
            ix = int(math.floor((x - min_x) / self.resolution))
            iy = int(math.floor((y - min_y) / self.resolution))
            ix = max(0, min(ix, width - 1))
            iy = max(0, min(iy, height - 1))
            occupied_cells.add((ix, iy))

        # Mark occupied cells (100 = occupied)
        for ix, iy in occupied_cells:
            idx = iy * width + ix
            grid_data[idx] = 100

        # Create message
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = self.resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = min_x
        msg.info.origin.position.y = min_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid_data

        return msg

    def publish_all(self):
        """Publish both 3D and 2D representations."""
        self.publish_pointcloud()
        if self.publish_2d and self.occupancy_grid_msg:
            self.publish_occupancy_grid()

    def publish_pointcloud(self):
        """Publish loaded voxels as PointCloud2."""
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

        self.publisher_3d.publish(msg)
        self.get_logger().debug(f'Published {len(self.points)} voxels (3D)')

    def publish_occupancy_grid(self):
        """Publish 2D occupancy grid."""
        if not self.occupancy_grid_msg:
            return

        self.occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_2d.publish(self.occupancy_grid_msg)
        self.get_logger().debug('Published 2D occupancy grid')


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
