#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/pointcloud_topic', 10)
        self.timer = self.create_timer(0.1, self.publish_point_cloud)  # 10Hz
        self.t = 0.0  # For animation

    def create_sphere_points(self, center, radius, density):
        # Create sphere using spherical coordinates
        phi = np.arange(0, np.pi, density)
        theta = np.arange(0, 2*np.pi, density)
        phi, theta = np.meshgrid(phi, theta)

        # Convert spherical to Cartesian coordinates
        x = center[0] + radius * np.sin(phi) * np.cos(theta)
        y = center[1] + radius * np.sin(phi) * np.sin(theta)
        z = center[2] + radius * np.cos(phi)

        points = np.column_stack((x.flatten(), y.flatten(), z.flatten()))
        return points

    def create_test_points(self):
        points = []
        colors = []
        
        # Cube parameters
        size = 1.0
        density = 0.1  # Point density
        
        # Create cube vertices
        for x in np.arange(-size, size, density):
            for y in np.arange(-size, size, density):
                for z in np.arange(-size, size, density):
                    # Only create points near the edges of the cube
                    if (abs(abs(x) - size) < density or 
                        abs(abs(y) - size) < density or 
                        abs(abs(z) - size) < density):
                        
                        # Rotate points around Y axis
                        theta = self.t
                        x_rot = x * np.cos(theta) - z * np.sin(theta)
                        z_rot = x * np.sin(theta) + z * np.cos(theta)
                        
                        points.append([x_rot, y, z_rot])
                        
                        # Create time-varying colors for cube
                        r = 0.5 + 0.5 * np.sin(self.t + x)
                        g = 0.5 + 0.5 * np.sin(self.t * 1.3 + y)
                        b = 0.5 + 0.5 * np.sin(self.t * 1.7 + z)
                        colors.append([r, g, b])

        # Add bouncing sphere
        sphere_y = np.sin(self.t * 2) * 0.5  # Bounce amplitude of 0.5
        sphere_points = self.create_sphere_points([0, sphere_y, 0], 0.3, 0.1)
        
        # Add sphere points and colors
        for point in sphere_points:
            points.append(point)
            # Create pulsing colors for sphere
            hue = (np.sin(self.t * 3) + 1) / 2
            # Convert HSV to RGB (simplified version)
            if hue < 1/3:
                colors.append([1, 3*hue, 0])
            elif hue < 2/3:
                colors.append([2-3*hue, 1, 0])
            else:
                colors.append([0, 1, 3*hue-2])

        # Add a moving sine wave surface
        x = np.arange(-size, size, density)
        z = np.arange(-size, size, density)
        X, Z = np.meshgrid(x, z)
        Y = 0.5 * np.sin(2 * X + self.t) * np.cos(2 * Z + self.t)
        
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                points.append([X[i,j], Y[i,j] - 2, Z[i,j]])  # Offset by -2 in Y
                
                # Create time-varying colors for wave
                phase = self.t + Y[i,j]
                r = 0.5 + 0.5 * np.sin(phase)
                g = 0.5 + 0.5 * np.sin(phase + 2*np.pi/3)
                b = 0.5 + 0.5 * np.sin(phase + 4*np.pi/3)
                colors.append([r, g, b])

        return np.array(points, dtype=np.float32), np.array(colors, dtype=np.float32)

    def create_point_cloud_msg(self, points, colors):
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 32
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.row_step = msg.point_step * points.shape[0]
        
        # Create structured array
        cloud_data = np.zeros(len(points), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.float32),
            ('padding', np.float32, (4,))  # 16 bytes of padding
        ])
        
        # Fill in coordinates
        cloud_data['x'] = points[:, 0]
        cloud_data['y'] = points[:, 1]
        cloud_data['z'] = points[:, 2]
        
        # Pack RGB into float32
        rgb_packed = np.zeros(len(points), dtype=np.float32)
        for i, (r, g, b) in enumerate(colors):
            rgb_packed[i] = struct.unpack('f', struct.pack('BBBB',
                int(b * 255),
                int(g * 255),
                int(r * 255),
                0))[0]
        
        cloud_data['rgb'] = rgb_packed
        
        msg.data = cloud_data.tobytes()
        return msg

    def publish_point_cloud(self):
        points, colors = self.create_test_points()
        msg = self.create_point_cloud_msg(points, colors)
        self.publisher.publish(msg)
        self.t += 0.05  # Increment animation time

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()