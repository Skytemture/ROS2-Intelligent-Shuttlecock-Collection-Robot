#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import MapMetaData
from builtin_interfaces.msg import Time

class MapMetadataPublisher(Node):
    def __init__(self):
        super().__init__('map_metadata_publisher')

        self.publisher_ = self.create_publisher(MapMetaData, '/map_metadata', 10)

        timer_period = 1.0  # 每秒發送一次
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('resolution', 0.05),
                ('width', 384),
                ('height', 384),
                ('origin', [0.0, 0.0, 0.0])
            ]
        )

        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.origin = self.get_parameter('origin').value

    def timer_callback(self):
        msg = MapMetaData()
        now = self.get_clock().now().to_msg()
        msg.map_load_time = now
        msg.resolution = self.resolution
        msg.width = self.width
        msg.height = self.height
        msg.origin.position.x = self.origin[0]
        msg.origin.position.y = self.origin[1]
        msg.origin.position.z = 0.0
        msg.origin.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing /map_metadata')

def main(args=None):
    rclpy.init(args=args)
    node = MapMetadataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

