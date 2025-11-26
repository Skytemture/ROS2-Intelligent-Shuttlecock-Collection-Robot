#!/usr/bin/env python3
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
import numpy as np
import time


class YoloMapMarkerNode(Node):
    def __init__(self):
        super().__init__('yolo_map_marker_node')

        self.model = YOLO('/home/robot/runs/runs/detect/train3/weights/best.pt')  # ✅ your model path
        self.bridge = CvBridge()

        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/shuttlecock_marker', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.depth_image = None
        self.get_logger().info('✅ yolo_map_marker_node started')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"❌ Depth image conversion error: {e}")

    def image_callback(self, msg):
        if self.depth_image is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame)
        height, width, _ = frame.shape

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                label = self.model.names[cls] if hasattr(self.model, 'names') else str(cls)
                if label.lower() != "shuttlecock":
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # Draw box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Try to get depth
                try:
                    depth = float(self.depth_image[cy, cx])
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Depth error at ({cx},{cy}): {e}")
                    depth = -1

                if np.isnan(depth) or depth < 0.1 or depth > 4.0:
                    self.get_logger().warn(f"📏 Invalid depth ({depth:.2f}m), using fixed estimate")
                    depth = 0.3  # fallback to estimated depth

                # Camera intrinsics
                fx = 600.0
                fy = 600.0
                cx_cam = width / 2
                cy_cam = height / 2

                x = (cx - cx_cam) * depth / fx
                y = (cy - cy_cam) * depth / fy
                z = depth

                camera_point = PointStamped()
                camera_point.header.frame_id = "camera_link"  # ✅ Replace if your frame is different
                camera_point.header.stamp = self.get_clock().now().to_msg()
                camera_point.point.x = x
                camera_point.point.y = y
                camera_point.point.z = z

                try:
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        camera_point.header.frame_id,
                        rclpy.time.Time()
                    )
                    map_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
                    self.publish_marker(map_point.point)

                    self.get_logger().info(
                        f"🏸 Detected at map: ({map_point.point.x:.2f}, {map_point.point.y:.2f}, {map_point.point.z:.2f})"
                    )

                except Exception as e:
                    self.get_logger().warn(f"❌ TF transform failed: {e}")

        # Show image (GUI)
        try:
            cv2.imshow("YOLO Marker", frame)
            cv2.waitKey(10)
        except Exception as e:
            self.get_logger().warn(f"GUI unavailable: {e}")

    def publish_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "shuttlecock"
        marker.id = int(time.time() * 1000) % 100000  # unique ID
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 60

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

