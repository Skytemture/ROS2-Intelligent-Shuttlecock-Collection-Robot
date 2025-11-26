import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()

        # 订阅深度图像话题
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.depth_display = None  # 初始化深度图像变量

        # 初始化窗口
        cv2.namedWindow('Color, Depth, and Mixed Image', cv2.WINDOW_NORMAL)

        # 使用 VideoCapture 捕获 MJPG 格式的颜色图像
        self.color_capture = cv2.VideoCapture(2)  # 设备索引可能需要调整

    def depth_callback(self, msg):
        # 将 ROS 深度图像消息转换为 OpenCV 格式
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # 归一化深度图像，并应用伪彩色映射
        depth_display = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        self.depth_display = cv2.applyColorMap(depth_display.astype('uint8'), cv2.COLORMAP_JET)

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            # 捕获颜色图像
            ret, color_image = self.color_capture.read()
            if not ret:
                continue  # 读取失败则跳过该帧

            if self.depth_display is not None:
                # 调整深度图像大小以匹配彩色图像
                resized_depth = cv2.resize(self.depth_display, (color_image.shape[1], color_image.shape[0]))

                # 生成混合图像（默认 50% 透明度）
                mixed_image = cv2.addWeighted(color_image, 0.5, resized_depth, 0.5, 0)

                # 组合三张图像并显示
                combined_image = cv2.vconcat([color_image, resized_depth, mixed_image])
                cv2.imshow('Color, Depth, and Mixed Image', combined_image)
            else:
                # 仅显示彩色图像
                cv2.imshow('Color, Depth, and Mixed Image', color_image)

            # 按 'q' 退出程序
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Exiting...")
                break

        self.destroy()  # 释放资源

    def destroy(self):
        self.color_capture.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    
    try:
        image_subscriber.spin()
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        image_subscriber.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
