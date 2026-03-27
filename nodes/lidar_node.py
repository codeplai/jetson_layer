#!/usr/bin/env python3
"""MINEBOT-Q LiDAR Node — 4D LiDAR L2 publisher for PointCloud2, LaserScan, Doppler."""

import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import Header


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('scan_frequency_hz', 10.0)

        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.frame_id = self.get_parameter('frame_id').value
        freq = self.get_parameter('scan_frequency_hz').value

        self.lidar = self._init_lidar()

        self.pub_points = self.create_publisher(
            PointCloud2, '/lidar/points', 10)
        self.pub_scan2d = self.create_publisher(
            LaserScan, '/lidar/scan2d', 10)
        self.pub_doppler = self.create_publisher(
            PointCloud2, '/lidar/doppler', 10)

        self.timer = self.create_timer(1.0 / freq, self._scan_callback)
        self.get_logger().info(
            f'LidarNode started — range=[{self.min_range}, {self.max_range}]m, '
            f'freq={freq}Hz')

    def _init_lidar(self):
        try:
            from livox_lidar_sdk import LivoxLidar
            lidar = LivoxLidar()
            lidar.connect()
            self.get_logger().info('4D LiDAR L2 connected via SDK')
            return lidar
        except ImportError:
            self.get_logger().warn(
                'livox_lidar_sdk not found — using simulated LiDAR data')
            return None

    def _get_raw_data(self):
        if self.lidar is not None:
            try:
                return self.lidar.get_point_cloud()
            except Exception as e:
                self.get_logger().error(f'LiDAR read error: {e}')
                return None

        # Simulated data: random point cloud for development
        n_points = 500
        angles = np.random.uniform(-math.pi, math.pi, n_points)
        ranges = np.random.uniform(self.min_range, self.max_range, n_points)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.random.uniform(-0.5, 2.0, n_points)
        intensity = np.random.uniform(0, 255, n_points)
        doppler = np.random.uniform(-2.0, 2.0, n_points)
        return {
            'x': x.astype(np.float32),
            'y': y.astype(np.float32),
            'z': z.astype(np.float32),
            'intensity': intensity.astype(np.float32),
            'doppler': doppler.astype(np.float32),
        }

    def _build_pointcloud2(self, header, data, include_doppler=False):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 16

        if include_doppler:
            fields.append(
                PointField(name='doppler', offset=16,
                           datatype=PointField.FLOAT32, count=1))
            point_step = 20

        n_points = len(data['x'])
        buf = bytearray()
        for i in range(n_points):
            buf += struct.pack('f', data['x'][i])
            buf += struct.pack('f', data['y'][i])
            buf += struct.pack('f', data['z'][i])
            buf += struct.pack('f', data['intensity'][i])
            if include_doppler:
                buf += struct.pack('f', data['doppler'][i])

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n_points
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * n_points
        msg.data = bytes(buf)
        msg.is_dense = True
        return msg

    def _build_laserscan(self, header, data):
        n_beams = 360
        ranges_arr = [float('inf')] * n_beams

        for i in range(len(data['x'])):
            x_val = float(data['x'][i])
            y_val = float(data['y'][i])
            z_val = float(data['z'][i])

            # Filter to horizontal slice (±0.15m around sensor height)
            if abs(z_val) > 0.15:
                continue

            r = math.sqrt(x_val ** 2 + y_val ** 2)
            if r < self.min_range or r > self.max_range:
                continue

            angle = math.atan2(y_val, x_val)
            idx = int((angle + math.pi) / (2 * math.pi) * n_beams) % n_beams
            if r < ranges_arr[idx]:
                ranges_arr[idx] = r

        scan = LaserScan()
        scan.header = header
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / n_beams
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = [r if r != float('inf') else self.max_range + 1.0
                       for r in ranges_arr]
        return scan

    def _scan_callback(self):
        data = self._get_raw_data()
        if data is None:
            return

        # Filter by range
        x, y, z = data['x'], data['y'], data['z']
        distances = np.sqrt(x ** 2 + y ** 2 + z ** 2)
        mask = (distances >= self.min_range) & (distances <= self.max_range)
        filtered = {k: v[mask] for k, v in data.items()}

        now = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = now
        header.frame_id = self.frame_id

        # PointCloud2 (full 3D)
        pc2 = self._build_pointcloud2(header, filtered, include_doppler=False)
        self.pub_points.publish(pc2)

        # Doppler PointCloud2
        doppler_pc2 = self._build_pointcloud2(
            header, filtered, include_doppler=True)
        self.pub_doppler.publish(doppler_pc2)

        # LaserScan (2D horizontal slice)
        scan = self._build_laserscan(header, filtered)
        self.pub_scan2d.publish(scan)

    def destroy_node(self):
        if self.lidar is not None:
            try:
                self.lidar.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
