import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_camera_node')

        # ROS publishers
        self.image_pub = self.create_publisher(Image, 'yolo/image', 10)
        self.position_pub = self.create_publisher(String, 'yolo/positions', 10)

        # YOLO model and CvBridge
        self.bridge = CvBridge()
        self.model = YOLO('/home/robot/runs/runs/detect/train3/weights/best.pt')  # 替換成你的模型路徑

        # ROS subscriber to image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('✅ YOLO Camera Node 已啟動，等待影像輸入...')

    def image_callback(self, msg):
        try:
            # ROS Image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ CvBridge 錯誤: {e}")
            return

        # YOLO 推論
        results = self.model(frame)
        positions = []

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                # 畫框 + 顯示類別
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{cls} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                positions.append(f"{cls}:{conf:.2f}@({cx},{cy})")

        # 發布影像
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(out_msg)

        # 發布位置文字
        if positions:
            pos_msg = String()
            pos_msg.data = "; ".join(positions)
            self.position_pub.publish(pos_msg)

        # 顯示影像（可選）
        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

