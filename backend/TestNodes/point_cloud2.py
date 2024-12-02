#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct

class StereoPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('stereo_pointcloud_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/stereo/points', 10)
        self.timer = self.create_timer(0.1, self.publish_point_cloud)  # 10Hz
        self.t = 0.0

    def create_synthetic_scene(self):
        points = []
        
        # Create ground plane
        x = np.linspace(-5, 5, 100)
        z = np.linspace(0, 10, 100)
        X, Z = np.meshgrid(x, z)
        Y = np.zeros_like(X) - 1.0  # Ground plane at y=-1
        
        # Add some gentle hills to the ground
        Y += 0.2 * np.sin(X * 0.5 + self.t) * np.cos(Z * 0.5)
        
        # Add ground points
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                points.append([X[i,j], Y[i,j], Z[i,j]])

        # Add a moving person (represented by a cylinder)
        person_x = 2.0 * np.sin(self.t)
        person_z = 5.0 + np.cos(self.t)
        height = 1.7  # Person height
        radius = 0.3  # Person radius
        
        angles = np.linspace(0, 2*np.pi, 20)
        heights = np.linspace(-1, height, 20)
        
        for h in heights:
            for angle in angles:
                x = person_x + radius * np.cos(angle)
                z = person_z + radius * np.sin(angle)
                y = h
                points.append([x, y, z])

        # Add some static objects (boxes)
        def add_box(center, size):
            x, y, z = center
            dx, dy, dz = size
            for i in np.linspace(-dx/2, dx/2, 10):
                for j in np.linspace(-dy/2, dy/2, 10):
                    for k in np.linspace(-dz/2, dz/2, 10):
                        points.append([x + i, y + j, z + k])

        # Add some boxes of different sizes
        add_box([-3, -0.5, 3], [1, 1, 1])  # Small box
        add_box([3, -0.25, 7], [2, 1.5, 2])  # Medium box
        
        # Convert to numpy array
        return np.array(points, dtype=np.float32)

    def create_point_cloud_msg(self, points):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Point cloud fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 32
        msg.is_dense = True
        
        cloud_data = []
        for point in points:
            # Add XYZ
            cloud_data.extend(point)
            
            # Add some color based on height (grayscale + blue tint)
            intensity = (point[1] + 1.0) / 3.0  # Normalize height
            rgb = struct.pack('BBBx', 
                int(intensity * 200),  # R
                int(intensity * 200),  # G
                int(intensity * 255))  # B (more blue)
            cloud_data.extend([0, 0, 0, 0])  # Padding
            cloud_data.extend(struct.unpack('f', rgb))  # Add packed RGB
            cloud_data.extend([0, 0, 0, 0])  # More padding

        msg.height = 1
        msg.width = len(points)
        msg.row_step = msg.point_step * len(points)
        msg.data = np.array(cloud_data, dtype=np.uint8).tobytes()
        
        return msg

    def publish_point_cloud(self):
        points = self.create_synthetic_scene()
        msg = self.create_point_cloud_msg(points)
        self.publisher.publish(msg)
        self.t += 0.1  # Increment animation time

def main(args=None):
    rclpy.init(args=args)
    node = StereoPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()