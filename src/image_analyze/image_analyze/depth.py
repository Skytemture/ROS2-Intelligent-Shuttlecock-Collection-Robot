import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class DepthImageSubscriber(Node):
    def __init__(self):
        super().__init__('depth_image_subscriber')
        self.bridge = CvBridge()

        # 订阅深度图像话题
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # 初始化窗口
        cv2.namedWindow('Depth Image', cv2.WINDOW_NORMAL)

        self.depth_image = None
        self.last_update_time = time.time()
        self.depth_value = None  # 存储最新的深度值
        self.min_loc = (0, 0)  # 存储最小深度值的位置

    def depth_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 图像
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            # 每秒更新一次深度值
            current_time = time.time()
            if current_time - self.last_update_time >= 1:  # 每秒更新
                if self.depth_image is not None:
                    # 查找深度图像中的最小值和位置
                    min_val, _, self.min_loc, _ = cv2.minMaxLoc(self.depth_image)

                    # 仅在最小深度值小于 10 公分时更新
                    if min_val > 1000:  # 深度值以毫米为单位，10 公分 = 100 毫米
                        self.depth_value = min_val
                    else:
                        self.depth_value = None  # 超过 10 公分时设置为 None

                    self.last_update_time = current_time  # 更新时间

            # 显示深度图像
            if self.depth_image is not None:
                depth_display = cv2.applyColorMap(self.depth_image.astype(np.uint8), cv2.COLORMAP_JET)  # 使用伪彩色显示深度

                # 仅在深度值有效时显示最小深度
                if self.depth_value is not None:
                    cv2.putText(depth_display, f'Min Depth: {self.depth_value:.1f} mm', (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    # 在最小深度值的位置绘制矩形框
                    cv2.rectangle(depth_display, 
                                  (self.min_loc[0] - 10, self.min_loc[1] - 10), 
                                  (self.min_loc[0] + 10, self.min_loc[1] + 10), 
                                  (0, 255, 0), 2)  # 绿色框

                # 显示深度图像
                cv2.imshow('Depth Image', depth_display)

                # 检查按键，如果按下 'q' 键，则退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        # 清理
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    depth_image_subscriber = DepthImageSubscriber()
    depth_image_subscriber.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
