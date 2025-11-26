import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import struct
import cv2  # Ensure this import is at the top

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')

        # Depth image subscriber
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # Color image subscriber
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)

        # Point cloud publisher
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/point_clouds/points', 10)

        # CvBridge for converting ROS image to OpenCV format
        self.bridge = CvBridge()

        # Camera intrinsics (you may need to calibrate these)
        self.fx, self.fy = 525.0, 525.0
        self.cx, self.cy = 319.5, 239.5

        # Latest color frame
        self.latest_color_frame = None

    def color_callback(self, color_msg):
        # Convert color image message to OpenCV format
        color_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        
        # Save the latest color frame
        self.latest_color_frame = color_frame

    def pack_rgb(self, r, g, b):
        return struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]

    def depth_callback(self, depth_msg):
        if self.latest_color_frame is None:
            self.get_logger().warn("No color frame available yet.")
            return

        # Convert depth to CV2 image
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

        height, width = depth_image.shape

        if self.latest_color_frame is not None:
            color_frame = cv2.resize(self.latest_color_frame, (width, height))
        else:
            self.get_logger().warn("Color frame is not available yet.")
            return

        # Valid depth mask
        valid_mask = depth_image > 0
        v_idxs, u_idxs = np.nonzero(valid_mask)
        valid_depths = depth_image[v_idxs, u_idxs] / 1000.0  # mm to meters

        # Compute 3D coordinates
        x = (u_idxs - self.cx) * valid_depths / self.fx
        y = (v_idxs - self.cy) * valid_depths / self.fy
        z = valid_depths

        # Get RGB colors
        bgr_colors = color_frame[v_idxs, u_idxs]
        rgb_colors = np.zeros_like(bgr_colors)
        rgb_colors[:, 0] = bgr_colors[:, 2]
        rgb_colors[:, 1] = bgr_colors[:, 1]
        rgb_colors[:, 2] = bgr_colors[:, 0]

        packed_rgb = [self.pack_rgb(r, g, b) for r, g, b in rgb_colors]

        # Pack PointCloud2
        cloud_data = bytearray()
        for i in range(len(x)):
            cloud_data.extend(struct.pack('fffI', x[i], y[i], z[i], packed_rgb[i]))

        cloud_msg = PointCloud2()
        cloud_msg.header = depth_msg.header
        cloud_msg.height = 1
        cloud_msg.width = len(x)
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = False
        cloud_msg.data = cloud_data

        self.get_logger().info(f"Publishing {cloud_msg.width} points")
        self.point_cloud_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

