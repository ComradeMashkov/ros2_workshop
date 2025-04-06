#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from lidar_node.lidar_capture import Capture

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'lidar_scan', 10)
        self.timer_ = self.create_timer(0.05, self.timer_callback) # 20 Hz

        self.lidar = Capture('/dev/ttyAMA0', data_size=480)
        self.lidar.start()
        self.get_logger().info('LIDAR started!')

    def timer_callback(self):
        if not self.lidar.data_obtained:
            return
        
        angles, distances = self.lidar.get_scan_data()

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'map'
        scan_msg.angle_min = min(angles)
        scan_msg.angle_max = max(angles)
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / len(angles)
        scan_msg.time_increment = 0.0
        scan_msg.range_min = 0.08
        scan_msg.range_max = 1.0
        scan_msg.ranges = distances
        scan_msg.intensities = []

        self.publisher_.publish(scan_msg)

    def destroy_node(self):
        self.lidar.stop()
        return super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LIDAR node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()