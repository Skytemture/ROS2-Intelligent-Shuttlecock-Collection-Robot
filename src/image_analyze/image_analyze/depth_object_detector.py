import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class AstraCameraNode(Node):
    def __init__(self):
        super().__init__('astra_camera_node')
        self.br = CvBridge()
        self.image_received = False
        self.depth_received = False
        self.point_cloud_received = False

        # 訂閱顏色影像
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # 顏色影像主題
            self.color_listener_callback,
            10)

        # 訂閱深度影像
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',  # 深度影像主題
            self.depth_listener_callback,
            10)

        # 訂閱點雲
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',  # 點雲主題
            self.point_cloud_listener_callback,
            10)

        # 啟動定時器，每秒檢查一次連接
        self.create_timer(1.0, self.check_connection)

    def color_listener_callback(self, msg):
        # 將 ROS 影像消息轉換為 OpenCV 格式
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_received = True
        cv2.imshow('Astra Color Image', cv_image)
        cv2.waitKey(1)

    def depth_listener_callback(self, msg):
        # 將深度影像轉換為 NumPy 陣列
        depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.depth_received = True
        cv2.imshow('Astra Depth Image', depth_image)
        cv2.waitKey(1)

    def point_cloud_listener_callback(self, msg):
        # 處理點雲數據
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pc_data))
        self.point_cloud_received = True
        # 在此處可以添加對點雲的進一步處理或可視化代碼
        self.get_logger().info(f'Point Cloud: {points.shape[0]} points received.')

    def check_connection(self):
        if not self.image_received:
            self.get_logger().warning('No color image received from Astra Pro Camera.')
        if not self.depth_received:
            self.get_logger().warning('No depth image received from Astra Pro Camera.')
        if not self.point_cloud_received:
            self.get_logger().warning('No point cloud data received from Astra Pro Camera.')
        else:
            # 重置標誌
            self.image_received = False
            self.depth_received = False
            self.point_cloud_received = False

def main(args=None):
    rclpy.init(args=args)
    node = AstraCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
