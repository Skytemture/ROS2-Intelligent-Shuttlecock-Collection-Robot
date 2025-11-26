import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()

        # 发布彩色图像
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)

        # 订阅深度图像话题
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.depth_display = None  # 初始化深度图像变量
        self.last_time = time.time()
        self.fps = 0.0

        # 初始化窗口
        #cv2.namedWindow('Color and Depth Image', cv2.WINDOW_NORMAL)

        # 使用 VideoCapture 捕获颜色图像
        self.color_capture = cv2.VideoCapture(0)
        self.color_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.color_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.color_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    def depth_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # 归一化深度图像并应用伪彩色映射
        depth_display = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        self.depth_display = cv2.applyColorMap(depth_display.astype('uint8'), cv2.COLORMAP_JET)

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            ret, color_image = self.color_capture.read()
            if not ret:
                self.get_logger().warn("Failed to read color image.")
                continue

            # 计算并显示 FPS
            current_time = time.time()
            self.fps = 1.0 / (current_time - self.last_time)
            self.last_time = current_time

            #cv2.putText(color_image, f'FPS: {self.fps:.2f}', (10, 25),
                        #cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 发布彩色图像
            ros_color_image = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            ros_color_image.header.stamp = self.get_clock().now().to_msg()
            self.color_pub.publish(ros_color_image)

            if self.depth_display is not None:
                # 调整深度图像大小以匹配彩色图像
                resized_depth = cv2.resize(self.depth_display, (color_image.shape[1], color_image.shape[0]))
                combined_image = cv2.hconcat([color_image, resized_depth])
                #cv2.imshow('Color and Depth Image', combined_image)
            #else:
                #cv2.imshow('Color and Depth Image', color_image)

            #cv2.setWindowTitle("Color and Depth Image", f"FPS: {self.fps:.2f}")

            '''
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Exiting...")
                break
	     '''
        self.destroy()

    def destroy(self):
        self.color_capture.release()
        #cv2.destroyAllWindows()

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

