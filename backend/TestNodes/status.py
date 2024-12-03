#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CameraInfo

class MultiPublisherNode(Node):
    def __init__(self):
        super().__init__('multi_publisher_node')
        self.diag_array_pub = self.create_publisher(DiagnosticArray, 'diagnostics_array', 10)
        self.diag_status_pub = self.create_publisher(DiagnosticStatus, 'diagnostics_status', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        status1 = DiagnosticStatus(
            level=bytes([0]),  # OK
            name="CPU Status",
            message="CPU running normally",
            hardware_id="cpu_0",
            values=[
                KeyValue(key="Temperature", value="45C"),
                KeyValue(key="Usage", value="32%"),
                KeyValue(key="Frequency", value="3.2GHz")
            ]
        )

        status2 = DiagnosticStatus(
            level=bytes([1]),  # WARN
            name="Memory Status",
            message="Memory usage high",
            hardware_id="ram_0",
            values=[
                KeyValue(key="Total", value="16GB"),
                KeyValue(key="Used", value="12GB"),
                KeyValue(key="Free", value="4GB")
            ]
        )

        status3 = DiagnosticStatus(
            level=bytes([2]),  # ERROR
            name="Disk Status",
            message="Disk space critical",
            hardware_id="disk_0",
            values=[
                KeyValue(key="Total", value="500GB"),
                KeyValue(key="Used", value="450GB"),
                KeyValue(key="Free", value="50GB")
            ]
        )

        status4 = DiagnosticStatus(
            level=bytes([3]),  # STALE
            name="Network Status",
            message="No data received",
            hardware_id="net_0",
            values=[
                KeyValue(key="Packets Sent", value="0"),
                KeyValue(key="Packets Received", value="0"),
                KeyValue(key="Connection Status", value="Disconnected")
            ]
        )

        diag_array_msg = DiagnosticArray()
        diag_array_msg.header.stamp = self.get_clock().now().to_msg()
        diag_array_msg.status = [status1, status2, status3, status4]  
        self.diag_array_pub.publish(diag_array_msg)
        self.diag_status_pub.publish(status1)

        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        camera_info_msg.header.frame_id = "camera_frame"
        self.camera_info_pub.publish(camera_info_msg)

def main():
    rclpy.init()
    node = MultiPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
