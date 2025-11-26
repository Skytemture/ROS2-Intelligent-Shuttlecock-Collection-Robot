import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
import cv2
import math

class YoloMapMarkerNode(Node):
    def __init__(self):
        super().__init__('yolo_map_marker_node')

        # Load YOLO model
        self.model = YOLO('/home/robot/runs/runs/detect/train3/weights/best.pt')
        self.bridge = CvBridge()

        # Subscribers and publishers
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/badminton_marker', 10)
        self.point_pub = self.create_publisher(PointStamped, '/badminton_point', 10)

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_id_counter = 0
        self.active_point = None  # store currently active point

        # Parameter for camera frame
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.get_logger().info(f'YOLO map marker node started, using camera frame: {self.camera_frame}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion failed: {e}")
            return

        results = self.model(frame)
        height, width, _ = frame.shape
        self.marker_id_counter = 0

        closest_point = None
        min_distance = float('inf')

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                label = self.model.names[cls] if hasattr(self.model, 'names') else str(cls)
                if label.lower() != "badminton":
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # 畫出 bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 影像像素轉相機座標
                z_cam = 0.3
                fx = 600.0
                fy = 600.0
                cx_cam = width / 2
                cy_cam = height / 2

                x_cam = float((cx - cx_cam) * z_cam / fx)
                y_cam = float((cy - cy_cam) * z_cam / fy)

                camera_point = PointStamped()
                camera_point.header.frame_id = self.camera_frame
                camera_point.header.stamp = self.get_clock().now().to_msg()
                camera_point.point.x = x_cam
                camera_point.point.y = y_cam
                camera_point.point.z = 0.0  # 固定地面

                try:
                    # 1. 轉到 base_link
                    transform_base = self.tf_buffer.lookup_transform(
                        'base_link',
                        self.camera_frame,
                        rclpy.time.Time()
                    )
                    point_in_base = tf2_geometry_msgs.do_transform_point(camera_point, transform_base)

                    # 2. 沿 base_link 前方方向增加偏移
                    forward_offset = 0.3  # 往前 0.3 m
                    # base_link x向前，y向左，z向上
                    angle = math.atan2(point_in_base.point.y, point_in_base.point.x)
                    distance = math.hypot(point_in_base.point.x, point_in_base.point.y)
                    distance += forward_offset  # 前方增加距離
                    point_in_base.point.x = distance * math.cos(angle)
                    point_in_base.point.y = distance * math.sin(angle)
                    point_in_base.point.z = 0.0  # 固定地面

                    # 3. 轉回 map
                    transform_map = self.tf_buffer.lookup_transform(
                        'map',
                        'base_link',
                        rclpy.time.Time()
                    )
                    final_point = tf2_geometry_msgs.do_transform_point(point_in_base, transform_map)

                    # Marker 可視化所有羽球
                    self.publish_marker(final_point.point)

                    # 只保留最近點
                    dist_robot = math.hypot(final_point.point.x - transform_map.transform.translation.x,
                                            final_point.point.y - transform_map.transform.translation.y)
                    if dist_robot < min_distance:
                        min_distance = dist_robot
                        closest_point = final_point

                except Exception as e:
                    self.get_logger().warn(f"TF transform failed: {e}")

        # 發佈最近點
        if closest_point and self.active_point is None:
            self.point_pub.publish(closest_point)
            self.active_point = closest_point
            self.get_logger().info(
                f"Published closest badminton point at ({closest_point.point.x:.2f}, {closest_point.point.y:.2f})"
            )

        # marker lifetime 過後清除 active_point
        if self.active_point:
            now = self.get_clock().now().to_msg()
            elapsed = (now.sec + now.nanosec * 1e-9) - (self.active_point.header.stamp.sec + self.active_point.header.stamp.nanosec * 1e-9)
            if elapsed > 3.0:
                self.active_point = None

        try:
            cv2.imshow("YOLO Marker (Base_Link Forward Offset)", frame)
            cv2.waitKey(10)
        except Exception as e:
            self.get_logger().warn(f"GUI error: {e}")

    def publish_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "badminton"
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 3

        self.marker_pub.publish(marker)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloMapMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

