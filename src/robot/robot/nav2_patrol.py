#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy

import numpy as np
import cv2
from scipy.ndimage import binary_dilation
from sklearn.cluster import KMeans
import math
import random
import time
from tqdm import tqdm

# Nav2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PointStamped


class PatrolPathMarkerNode(Node):
    def __init__(self):
        super().__init__('patrol_corner_kmeans')

        # === 參數 ===
        self.declare_parameter('num_points', None)
        self.declare_parameter('area_per_point', 10.0)
        self.declare_parameter('min_points', 2)
        self.declare_parameter('max_points', 20)
        self.declare_parameter('corner_radius', 20.0)
        self.declare_parameter('corner_point_num', 2)
        self.declare_parameter('enforce_90deg_turn', False)
        self.declare_parameter('strict_90deg_limit', 10)
        self.declare_parameter('blink_times', 3)  # 閃爍循環次數
        # 讀取 arrival 半徑參數
        self.declare_parameter("arrived_radius", 0.5)
        self.arrived_radius = self.get_parameter("arrived_radius").value

        # === 取得參數 ===
        self.num_points = None
        self.area_per_point = self.get_parameter('area_per_point').value
        self.min_points = self.get_parameter('min_points').value
        self.max_points = self.get_parameter('max_points').value
        self.corner_radius = self.get_parameter('corner_radius').value
        self.corner_point_num = self.get_parameter('corner_point_num').value
        self.enforce_90deg_turn = self.get_parameter('enforce_90deg_turn').value
        self.strict_90deg_limit = self.get_parameter('strict_90deg_limit').value
        self.blink_times = self.get_parameter('blink_times').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        # === Publisher / Subscriber ===
        marker_qos = QoSProfile(depth=10)
        marker_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(Marker, 'patrol_path_marker', marker_qos)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        
        # === Badminton handling ===
        self.badminton_pose = None       # store captured target
        self.handling_badminton = False  # remember which patrol point we paused at
        self.badminton_sub = self.create_subscription(
            PointStamped,
            "/badminton_point",
            self.badminton_callback,
            10
        )

        self.current_patrol_index = 0
        self.current_goal_future = None
        
        # === 狀態 ===
        self.map_data = None
        self.safe_area = None
        self.patrol_points = []

        self.corner_points = []
        self.kmeans_points = []

        self.blink_index = 0
        self.blink_count = 0
        self.blink_timer = None

        # Nav2
        self.navigator = BasicNavigator()

        self.get_logger().info('Waiting for /map topic...')

    # ============ 參數回調 ============
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'blink_times' and param.type_ == Parameter.Type.INTEGER:
                self.blink_times = param.value
                self.get_logger().info(f'blink_times changed to {self.blink_times}')
        return SetParametersResult(successful=True)

    # ============ Map callback ============
    def map_callback(self, msg: OccupancyGrid):
        if self.map_data is not None:
            return
        self.map_data = msg
        self.get_logger().info('Map received, generating patrol points...')

        # === 產生角落點 + KMeans ===
        self.generate_patrol_points(msg)

        # === 發布 Marker ===
        self.publish_corner_points(self.corner_points)
        self.publish_kmeans_points(self.kmeans_points)
        self.publish_path(self.patrol_points)

        # === 開始閃爍動畫 ===
        self.start_sequential_blinking()

    # ============ 巡邏點產生 ============
    def generate_patrol_points(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        obstacle_mask = (data > 0)
        dilation_iters = max(1, int(0.3 / resolution))
        dilated_obstacle = binary_dilation(obstacle_mask, iterations=dilation_iters)
        self.safe_area = np.logical_not(dilated_obstacle)

        # 角落檢測
        corners = cv2.goodFeaturesToTrack((self.safe_area*255).astype(np.uint8),
                                          maxCorners=100,
                                          qualityLevel=0.01,
                                          minDistance=3,
                                          blockSize=3,
                                          useHarrisDetector=False)
        if corners is None:
            corners = np.empty((0,1,2))
        corners = corners.reshape(-1, 2)

        filtered_corners = self.filter_corners_by_radius_priority(corners, int(self.corner_radius))
        if len(filtered_corners) > self.corner_point_num:
            filtered_corners = filtered_corners[:self.corner_point_num]

        self.corner_points = []
        for c in filtered_corners:
            wx = origin.x + c[0] * resolution
            wy = origin.y + c[1] * resolution
            self.corner_points.append(Point(x=float(wx), y=float(wy), z=0.0))

        # KMeans
        num_pts = self.auto_compute_num_points(msg) if self.num_points is None else self.num_points
        self.kmeans_points = self.generate_kmeans_points(msg, num_pts)

        # 合併
        combined_points = self.kmeans_points + self.corner_points
        self.get_logger().info(f'合併巡邏點與角點，共 {len(combined_points)} 個點，進行路徑優化')

        self.patrol_points = self.optimize_path(combined_points,
                                               enforce_90deg=self.enforce_90deg_turn,
                                               strict_limit=self.strict_90deg_limit)

        self.get_logger().info(f'✅ 產生總共 {len(self.patrol_points)} 個巡邏點')

    # ============ 閃爍顯示 ============
    def start_sequential_blinking(self):
        if self.blink_timer is not None:
            self.blink_timer.cancel()
        self.blink_index = 0
        self.blink_count = 0
        self.blink_timer = self.create_timer(0.6, self.blink_next_point)

    def blink_next_point(self):
        if not self.patrol_points:
            return
        total = len(self.patrol_points)

        marker = Marker()
        marker.header.frame_id = self.map_data.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sequential_blink"
        marker.id = 30
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.points = self.patrol_points

        marker.colors = []
        for i in range(total):
            if i == self.blink_index:
                marker.colors.append(self._make_color(0.0, 1.0, 0.0, 1.0))
            else:
                marker.colors.append(self._make_color(0.5, 0.5, 0.5, 0.2))
        self.marker_pub.publish(marker)

        # 更新 index
        self.blink_index = (self.blink_index + 1) % total
        if self.blink_index == 0:
            self.blink_count += 1

        # 閃爍完成後停止並開始導航
        if self.blink_count >= self.blink_times:
            self.blink_timer.cancel()
            # --- Reset all points to default color ---
            clear_marker = Marker()
            clear_marker.header.frame_id = self.map_data.header.frame_id
            clear_marker.header.stamp = self.get_clock().now().to_msg()
            clear_marker.ns = "sequential_blink"
            clear_marker.id = 30
            clear_marker.type = Marker.SPHERE_LIST
            clear_marker.action = Marker.DELETE  # 或者 Marker.ADD 但不加 colors
            self.marker_pub.publish(clear_marker)
            self.get_logger().info("閃爍完成，開始導航巡邏")
            
            #self.start_navigation()
            self.navigator_timer = self.create_timer(0.5, self.navigation_loop)

    # ================= Badminton callback =================
    def badminton_callback(self, msg: PointStamped):
        self.get_logger().info(f"🏸 Captured badminton target at ({msg.point.x:.2f}, {msg.point.y:.2f})")

        # convert to PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0  # neutral orientation

        # save it
        self.badminton_pose = pose
        self.handling_badminton = True

        # if we’re in patrol, pause and go collect
        if self.navigator.isTaskComplete() is False:
            self.navigator.cancelTask()
            self.get_logger().warn("⏸️ Pausing patrol to collect badminton")

    # ============ Nav2 導航 ============
    def navigation_loop(self):      
        # 1️⃣ 沒有巡邏點
        if not self.patrol_points:
            self.get_logger().warn("沒有巡邏點可導航")
            self.navigator_timer.cancel()
            return

        # 2️⃣ 如果已經完成所有巡邏點
        if self.current_patrol_index >= len(self.patrol_points):
            self.get_logger().info("🎉 巡邏完成！")
            self.navigator_timer.cancel()
            return

        # 3️⃣ 如果 Badminton 打斷
        if self.handling_badminton or self.badminton_pose is not None:
            self.handling_badminton = True
            self.handle_badminton_task(self.current_patrol_index)
            # 回來後恢復巡邏
            self.navigator.goToPose(self.make_patrol_pose(self.patrol_points[self.current_patrol_index]))
            self.last_log_time = time.time()
            self.handling_badminton = False
            return

        # 4️⃣ 如果還沒開始導航，就啟動
        if not hasattr(self, "nav_started") or not self.nav_started:
            target_pose = self.make_patrol_pose(self.patrol_points[self.current_patrol_index])
            self.navigator.goToPose(target_pose)
            self.nav_started = True
            self.arrived = False
            self.last_log_time = time.time()
            self.get_logger().info(f"➡️ Going to point {self.current_patrol_index+1} ...")
            return

        # 5️⃣ 獲取 feedback
        feedback = self.navigator.getFeedback()
        if feedback is not None:
            now = time.time()
            if now - getattr(self, "last_log_time", 0) >= 1.0:
                robot_x = feedback.current_pose.pose.position.x
                robot_y = feedback.current_pose.pose.position.y
                target = self.patrol_points[self.current_patrol_index]
                dx = target.x - robot_x
                dy = target.y - robot_y
                dist = math.sqrt(dx*dx + dy*dy)

                self.get_logger().info(f"{dist:.1f} m left to point {self.current_patrol_index+1}...")
                self.publish_current_target_marker(self.current_patrol_index)
                self.last_log_time = now

                # ✅ 到達判斷
                if dist <= self.arrived_radius and not self.arrived:
                    self.get_logger().info(f"✅ Arrived at point {self.current_patrol_index+1}!")
                    self.arrived = True
                    self.navigator.cancelTask()
                    return

        # 6️⃣ 如果任務結束而且已經到達 → 換下一個點
        if self.navigator.isTaskComplete() and getattr(self, "arrived", False):
            result = self.navigator.getResult()
            if result in [TaskResult.SUCCEEDED, TaskResult.CANCELED]:
                self.current_patrol_index += 1
                self.nav_started = False
            elif result == TaskResult.FAILED:
                self.get_logger().error(f"❌ Navigation to point {self.current_patrol_index+1} failed")
                self.navigator_timer.cancel()

    def make_patrol_pose(self, point):
        pose = PoseStamped()
        pose.header.frame_id = self.map_data.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.w = 1.0
        return pose


                
    def start_navigation(self):
        if not self.patrol_points:
            self.get_logger().warn("沒有巡邏點可導航")
            return

        # 讀取 arrival 半徑參數 (可在 launch/命令行設定)
        self.declare_parameter("arrived_radius", 0.5)
        arrived_radius = self.get_parameter("arrived_radius").value

        self.get_logger().info("等待 Nav2 Action Server 可用...")
        self.navigator.waitUntilNav2Active()

        poses = []
        for p in self.patrol_points:
            pose = PoseStamped()
            pose.header.frame_id = self.map_data.header.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = p.x
            pose.pose.position.y = p.y
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        # === 一個一個導航 ===
        for i, pose in enumerate(poses):
            self.get_logger().info(f"➡️ Going to point {i+1} ...")
            self.navigator.goToPose(pose)

            last_log_time = time.time()
            arrived = False

            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.5)  # <-- allow callbacks to run
                feedback = self.navigator.getFeedback()
                
                # Badminton interrupt
                if self.handling_badminton or self.badminton_pose is not None:
                    self.handling_badminton = True
                    self.handle_badminton_task(i)
                    # After returning from badminton, resume current patrol pose
                    self.navigator.goToPose(pose)
                    last_log_time = time.time()  # reset logging
                    self.handling_badminton = False
                                              
                if feedback is not None:
                    now = time.time()
                    if now - last_log_time >= 1.0:  # 每 1 秒報告一次
                        # Robot position
                        robot_x = feedback.current_pose.pose.position.x
                        robot_y = feedback.current_pose.pose.position.y

                        # Target position
                        dx = pose.pose.position.x - robot_x
                        dy = pose.pose.position.y - robot_y
                        dist = math.sqrt(dx * dx + dy * dy)

                        self.get_logger().info(f"{dist:.1f} m left to point {i+1}...")
                        last_log_time = now
                        
                        self.publish_current_target_marker(i)     
                                        
                        # 到達判斷
                        if dist <= arrived_radius and not arrived:
                            self.get_logger().info(f"✅ Arrived at point {i+1}!")
                            arrived = True
                            self.navigator.cancelTask()
                            while not self.navigator.isTaskComplete():
                                 rclpy.spin_once(self, timeout_sec=0.1)
                            break  # 跳出 while loop，開始下一個點

            # === 判斷結果 ===
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                pass  # 已經印過 Arrived
            elif result == TaskResult.CANCELED and arrived:
                pass  # 手動取消視為成功
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f"⚠️ Navigation to point {i+1} was canceled")
                break
            elif result == TaskResult.FAILED:
                self.get_logger().error(f"❌ Navigation to point {i+1} failed")
                break
        self.get_logger().info("🎉 巡邏完成！")


        
    # ================= Handle badminton =================
    def handle_badminton_task(self, current_index):
        """Interrupt patrol, navigate to badminton, then return to current patrol point."""
        if not self.badminton_pose:
            self.get_logger().warn("🏸 No badminton pose available")
            self.handling_badminton = False
            return

        # Navigate to badminton point
        self.get_logger().warn("⏸️ Interrupt patrol: Going to badminton point")

        # Use self.badminton_pose directly
        self.navigator.goToPose(self.badminton_pose)
        last_log_time = time.time()
        arrived = False

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                now = time.time()
                if now - last_log_time >= 1.0:
                    # Distance to badminton
                    dx = self.badminton_pose.pose.position.x - feedback.current_pose.pose.position.x
                    dy = self.badminton_pose.pose.position.y - feedback.current_pose.pose.position.y
                    dist = math.sqrt(dx*dx + dy*dy)
                    self.get_logger().info(f"🏸 {dist:.1f} m left to badminton point...")
                    last_log_time = now

                    if dist <= 0.5 and not arrived:  # Arrival threshold
                        self.get_logger().info("✅ Arrived at badminton point!")
                        arrived = True
                        self.navigator.cancelTask()
                        while not self.navigator.isTaskComplete():
                            rclpy.spin_once(self, timeout_sec=0.1)
                        break

        # Check result
        result = self.navigator.getResult()
        if result in [TaskResult.SUCCEEDED, TaskResult.CANCELED]:
            self.get_logger().info("🏸 Finished badminton task, returning to patrol...")
        else:
            self.get_logger().error("❌ Badminton navigation failed!")

        # Reset badminton flag
        self.badminton_pose = None
        self.handling_badminton = False

        # Navigate back to current patrol point
        #patrol_pose = self.patrol_poses[current_index]
        self.get_logger().info(f"↩️ Returning to patrol point {current_index+1}")
        arrived = False
        return



    # ================= 工具函式 =================
    def publish_current_target_marker(self, target_index):
        if not self.patrol_points or self.map_data is None:
           return

        marker = Marker()
        marker.header.frame_id = self.map_data.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "patrol_path_active"
        marker.id = 50
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35

        marker.points = self.patrol_points
        marker.colors = []
        for i in range(len(self.patrol_points)):
            if i == target_index:
                marker.colors.append(self._make_color(0.0, 1.0, 0.0, 1.0))  # green = current target
            else:
                marker.colors.append(self._make_color(0.5, 0.5, 0.5, 0.2))  # gray = others

        self.marker_pub.publish(marker)


    def _make_color(self, r, g, b, a):
        from std_msgs.msg import ColorRGBA
        c = ColorRGBA()
        c.r = r
        c.g = g
        c.b = b
        c.a = a
        return c

    # 發布 marker
    def publish_corner_points(self, points):
        marker = Marker()
        marker.header.frame_id = self.map_data.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "corner_points"
        marker.id = 10
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = points
        self.marker_pub.publish(marker)

    def publish_kmeans_points(self, points):
        marker = Marker()
        marker.header.frame_id = self.map_data.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "kmeans_points"
        marker.id = 11
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = points
        self.marker_pub.publish(marker)

    def publish_path(self, points):
        if len(points) < 2:
            return
        marker = Marker()
        marker.header.frame_id = self.map_data.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "patrol_path"
        marker.id = 20
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 0.8
        marker.points = points + [points[0]]
        self.marker_pub.publish(marker)

    # ================= 角落篩選 / KMeans / GA =================

    def filter_corners_by_radius_priority(self, corners, radius):
        corners = corners.astype(int)
        used = np.zeros(len(corners), dtype=bool)
        kept = []

        for i in range(len(corners)):
            if used[i]:
                continue
            dists = np.linalg.norm(corners - corners[i], axis=1)
            neighbors_idx = np.where(dists <= radius)[0]
            if len(neighbors_idx) >= 3:
                kept.append(corners[i])
            used[neighbors_idx] = True

        return np.array(kept)

    def auto_compute_num_points(self, map_data: OccupancyGrid):
        resolution = map_data.info.resolution
        data = np.array(map_data.data, dtype=np.int8).reshape((map_data.info.height, map_data.info.width))
        obstacle_mask = (data > 0)
        dilation_iters = max(1, int(0.3 / resolution))
        dilated_obstacle = binary_dilation(obstacle_mask, iterations=dilation_iters)
        safe_area = np.logical_not(dilated_obstacle)
        safe_cells = np.count_nonzero(safe_area)
        free_area_m2 = safe_cells * (resolution ** 2)
        estimated = int(max(self.min_points, min(self.max_points, math.floor(free_area_m2 / self.area_per_point))))
        estimated = max(1, estimated)
        safe_pixels = np.argwhere(safe_area)
        if len(safe_pixels) < estimated:
            estimated = max(1, len(safe_pixels))
        self.get_logger().info(f"Auto num_points: {estimated} (safe_cells={safe_cells}, free_area={free_area_m2:.2f} m²)")
        return estimated

    def generate_kmeans_points(self, map_data, num_points):
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin = map_data.info.origin.position
        data = np.array(map_data.data, dtype=np.int8).reshape((height, width))
        obstacle_mask = (data > 0)
        dilation_iters = max(1, int(0.3 / resolution))
        dilated_obstacle = binary_dilation(obstacle_mask, iterations=dilation_iters)
        safe_area = np.logical_not(dilated_obstacle)
        safe_pixels = np.argwhere(safe_area)
        if len(safe_pixels) < num_points:
            self.get_logger().warn("Safe area too small for requested points, reducing num_points.")
            num_points = max(1, len(safe_pixels))
        sample_limit = 5000
        if len(safe_pixels) > sample_limit:
            idx = np.random.choice(len(safe_pixels), sample_limit, replace=False)
            kmeans_input = safe_pixels[idx]
        else:
            kmeans_input = safe_pixels
        n_clusters = min(num_points, len(kmeans_input))
        kmeans = KMeans(n_clusters=n_clusters, n_init=10, random_state=0)
        kmeans.fit(kmeans_input)
        centers = kmeans.cluster_centers_
        patrol_points = []
        for cy, cx in centers:
            cy_int, cx_int = int(round(cy)), int(round(cx))
            if not safe_area[cy_int, cx_int]:
                neighbors = safe_pixels[np.linalg.norm(safe_pixels - np.array([cy_int, cx_int]), axis=1) < 3]
                if len(neighbors) == 0:
                    dists = np.linalg.norm(safe_pixels - np.array([cy_int, cx_int]), axis=1)
                    ni = np.argmin(dists)
                    cy_int, cx_int = safe_pixels[ni]
                else:
                    cy_int, cx_int = neighbors[0]
            wx = origin.x + cx_int * resolution
            wy = origin.y + cy_int * resolution
            patrol_points.append(Point(x=float(wx), y=float(wy), z=0.0))
        return patrol_points

    def optimize_path(self, points, enforce_90deg=False, strict_limit=10):
        if len(points) <= 2:
            return points
        unvisited = points.copy()
        path = [unvisited.pop(0)]
        while unvisited:
            last = path[-1]
            dists = [math.hypot(p.x-last.x, p.y-last.y) for p in unvisited]
            min_idx = np.argmin(dists)
            path.append(unvisited.pop(min_idx))
        return path


def main(args=None):
    rclpy.init(args=args)
    node = PatrolPathMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested')
    finally:
        if node.blink_timer:
            node.blink_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

