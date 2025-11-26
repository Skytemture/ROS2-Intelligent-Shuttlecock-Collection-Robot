#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
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


class PatrolPathMarkerNode(Node):
    def __init__(self):
        super().__init__('patrol_corner_kmeans')

        # 宣告參數（有預設值）
        self.declare_parameter('num_points', None)  # KMeans聚類點數，None 自動決定
        self.declare_parameter('area_per_point', 10.0)
        self.declare_parameter('min_points', 4)
        self.declare_parameter('max_points', 40)
        self.declare_parameter('corner_radius', 20.0)       # 角點去重半徑(單位：格子)
        self.declare_parameter('corner_point_num', 10)      # 最大角點數量限制
        self.declare_parameter('enforce_90deg_turn', False)
        self.declare_parameter('strict_90deg_limit', 10)

        # 取得參數初始值
        self.num_points = self.get_parameter('num_points').get_parameter_value().integer_value if self.get_parameter('num_points').type_ == Parameter.Type.INTEGER else None
        self.area_per_point = self.get_parameter('area_per_point').get_parameter_value().double_value
        self.min_points = self.get_parameter('min_points').get_parameter_value().integer_value
        self.max_points = self.get_parameter('max_points').get_parameter_value().integer_value
        self.corner_radius = self.get_parameter('corner_radius').get_parameter_value().double_value
        self.corner_point_num = self.get_parameter('corner_point_num').get_parameter_value().integer_value
        self.enforce_90deg_turn = self.get_parameter('enforce_90deg_turn').get_parameter_value().bool_value
        self.strict_90deg_limit = self.get_parameter('strict_90deg_limit').get_parameter_value().integer_value

        self.add_on_set_parameters_callback(self.parameter_callback)

        # Publisher & Subscriber
        marker_qos = QoSProfile(depth=10)
        marker_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(Marker, 'patrol_path_marker', marker_qos)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        self.map_data = None
        self.safe_area = None
        self.patrol_points = []

        self.corner_points = []  # 角落點（Point list）
        self.kmeans_points = []  # KMeans 巡邏點（Point list）

        self.blink_index = 0
        self.blink_timer = None

        self.get_logger().info('Waiting for /map topic...')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'corner_radius' and param.type_ == Parameter.Type.DOUBLE:
                self.corner_radius = param.value
                self.get_logger().info(f'corner_radius changed to {self.corner_radius}')
            elif param.name == 'corner_point_num' and param.type_ == Parameter.Type.INTEGER:
                self.corner_point_num = param.value
                self.get_logger().info(f'corner_point_num changed to {self.corner_point_num}')
            elif param.name == 'num_points':
                if param.type_ == Parameter.Type.INTEGER:
                    self.num_points = param.value
                    self.get_logger().info(f'num_points changed to {self.num_points}')
                elif param.type_ == Parameter.Type.NOT_SET:
                    self.num_points = None
                    self.get_logger().info('num_points changed to None (auto)')
            elif param.name == 'area_per_point' and param.type_ == Parameter.Type.DOUBLE:
                self.area_per_point = param.value
                self.get_logger().info(f'area_per_point changed to {self.area_per_point}')
            elif param.name == 'min_points' and param.type_ == Parameter.Type.INTEGER:
                self.min_points = param.value
                self.get_logger().info(f'min_points changed to {self.min_points}')
            elif param.name == 'max_points' and param.type_ == Parameter.Type.INTEGER:
                self.max_points = param.value
                self.get_logger().info(f'max_points changed to {self.max_points}')
            elif param.name == 'enforce_90deg_turn' and param.type_ == Parameter.Type.BOOL:
                self.enforce_90deg_turn = param.value
                self.get_logger().info(f'enforce_90deg_turn changed to {self.enforce_90deg_turn}')
            elif param.name == 'strict_90deg_limit' and param.type_ == Parameter.Type.INTEGER:
                self.strict_90deg_limit = param.value
                self.get_logger().info(f'strict_90deg_limit changed to {self.strict_90deg_limit}')
        return SetParametersResult(successful=True)

    def map_callback(self, msg: OccupancyGrid):
        if self.map_data is not None:
            return  # 只處理第一次地圖

        self.map_data = msg
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # 產生障礙物mask並膨脹(安全區域外圍留0.3公尺)
        obstacle_mask = (data > 0)
        dilation_iters = max(1, int(0.3 / resolution))
        dilated_obstacle = binary_dilation(obstacle_mask, iterations=dilation_iters)
        self.safe_area = np.logical_not(dilated_obstacle)

        self.get_logger().info('Map received, generating patrol points...')

        # 先用 Shi-Tomasi 找角點 (OpenCV)
        corners = cv2.goodFeaturesToTrack((self.safe_area*255).astype(np.uint8),
                                          maxCorners=100,
                                          qualityLevel=0.01,
                                          minDistance=3,
                                          blockSize=3,
                                          useHarrisDetector=False)
        if corners is None:
            corners = np.empty((0,1,2))
        corners = corners.reshape(-1, 2)  # Nx2

        # 篩選角點，半徑內只保留1點，半徑內若少於3點就捨棄
        filtered_corners = self.filter_corners_by_radius_priority(corners, int(self.corner_radius))

        # 限制角點最大數量（優先保留代表點）
        if len(filtered_corners) > self.corner_point_num:
            filtered_corners = filtered_corners[:self.corner_point_num]

        # 將角點轉成世界座標
        self.corner_points = []
        for c in filtered_corners:
            wx = origin.x + c[0] * resolution
            wy = origin.y + c[1] * resolution
            self.corner_points.append(Point(x=float(wx), y=float(wy), z=0.0))

        # KMeans 產生主要巡邏點
        if self.num_points is None:
            num_pts = self.auto_compute_num_points(msg)
        else:
            num_pts = self.num_points

        self.kmeans_points = self.generate_kmeans_points(msg, num_pts)

        # 合併兩者（角點 + KMeans 點）
        combined_points = self.kmeans_points + self.corner_points

        self.get_logger().info(f'合併巡邏點與角點，共 {len(combined_points)} 個點，進行路徑優化')

        # 優化路徑
        optimized_points = self.optimize_path(combined_points,
                                             enforce_90deg=self.enforce_90deg_turn,
                                             strict_limit=self.strict_90deg_limit)
        self.patrol_points = optimized_points

        self.get_logger().info(f'✅ 產生總共 {len(self.patrol_points)} 個巡邏點 (KMeans + 角落)')

        # 發布分色 Marker
        self.publish_corner_points(self.corner_points)
        self.publish_kmeans_points(self.kmeans_points)
        self.publish_path(self.patrol_points)

        # 開啟依序閃爍
        self.start_sequential_blinking()

    def filter_corners_by_radius_priority(self, corners, radius):
        """
        半徑內只保留一點，且半徑內若少於3點就捨棄
        優先保留原始Shi-Tomasi所偵測較多點的代表點

        corners: Nx2 np.array，座標為 map 格子座標
        radius: int，半徑（格子）
        """
        corners = corners.astype(int)
        used = np.zeros(len(corners), dtype=bool)
        kept = []

        for i in range(len(corners)):
            if used[i]:
                continue
            # 找半徑內的點
            dists = np.linalg.norm(corners - corners[i], axis=1)
            neighbors_idx = np.where(dists <= radius)[0]
            if len(neighbors_idx) >= 3:
                # 保留該 cluster 中點數最多的點，即這個點
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

    def optimize_path(self, points, enforce_90deg=False, strict_limit=10, ga_generations=50, ga_population=30):
        if len(points) <= 1:
            return points

        def distance(p1, p2):
            return math.hypot(p1.x - p2.x, p1.y - p2.y)

        def path_length(order):
            total = 0
            for i in range(len(order)):
                total += distance(order[i], order[(i+1) % len(order)])
            return total

        population = []
        for _ in range(ga_population):
            individual = list(range(len(points)))
            random.shuffle(individual)
            population.append(individual)

        def fitness(ind):
            ordered = [points[i] for i in ind]
            penalty = 0

            # 路徑障礙檢查 (可自行改)
            for i in range(len(ordered)):
                p1 = ordered[i]
                p2 = ordered[(i+1) % len(ordered)]
                if not self.is_path_clear(p1, p2):
                    penalty += 1000

            if enforce_90deg:
                for i in range(len(ordered)):
                    p1 = ordered[i-1]
                    p2 = ordered[i]
                    p3 = ordered[(i+1) % len(ordered)]
                    angle = self.angle_between_three_points(p1, p2, p3)
                    if not (80 <= angle <= 100):
                        penalty += 10

            return path_length(ordered) + penalty

        def select(pop):
            weights = [1.0 / fitness(ind) for ind in pop]
            total = sum(weights)
            pick = random.uniform(0, total)
            current = 0
            for ind, w in zip(pop, weights):
                current += w
                if current > pick:
                    return ind
            return pop[-1]

        def crossover(p1, p2):
            size = len(p1)
            start, end = sorted(random.sample(range(size), 2))
            child = [None]*size
            child[start:end] = p1[start:end]
            pos = 0
            for i in range(size):
                if child[i] is None:
                    while p2[pos] in child:
                        pos += 1
                    child[i] = p2[pos]
                    pos += 1
            return child

        def mutate(ind, rate=0.1):
            if random.random() < rate:
                i, j = random.sample(range(len(ind)), 2)
                ind[i], ind[j] = ind[j], ind[i]

        last_log = time.time()
        for gen in tqdm(range(ga_generations), desc='Optimizing Path with GA'):
            new_pop = []
            for _ in range(ga_population):
                p1 = select(population)
                p2 = select(population)
                child = crossover(p1, p2)
                mutate(child)
                new_pop.append(child)
            population = new_pop
            now = time.time()
            if now - last_log > 1.0:
                best_fit = fitness(min(population, key=fitness))
                self.get_logger().info(f'GA generation {gen}, best fitness: {best_fit:.2f}')
                last_log = now

        best_individual = min(population, key=fitness)
        optimized = [points[i] for i in best_individual]
        return optimized

    def is_path_clear(self, p1, p2):
        def world_to_map(p):
            mx = int((p.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            my = int((p.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
            return mx, my

        x0, y0 = world_to_map(p1)
        x1, y1 = world_to_map(p2)

        points_on_line = self.bresenham(x0, y0, x1, y1)
        for x, y in points_on_line:
            if x < 0 or y < 0 or y >= self.safe_area.shape[0] or x >= self.safe_area.shape[1]:
                return False
            if not self.safe_area[y, x]:
                return False
        return True

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    def angle_between_three_points(self, p1, p2, p3):
        a = np.array([p1.x - p2.x, p1.y - p2.y])
        b = np.array([p3.x - p2.x, p3.y - p2.y])
        dot = np.dot(a, b)
        norm_a = np.linalg.norm(a)
        norm_b = np.linalg.norm(b)
        if norm_a * norm_b == 0:
            return 0
        cos_angle = dot / (norm_a * norm_b)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        return np.degrees(angle)

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
        # 閉環
        marker.points = points + [points[0]]
        self.marker_pub.publish(marker)

    def start_sequential_blinking(self):
        if self.blink_timer is not None:
            self.blink_timer.cancel()
        self.blink_index = 0
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
                # 亮綠色
                marker.colors.append(self._make_color(0.0, 1.0, 0.0, 1.0))
            else:
                # 淡灰色
                marker.colors.append(self._make_color(0.5, 0.5, 0.5, 0.2))

        self.marker_pub.publish(marker)

        self.blink_index = (self.blink_index + 1) % total

    def _make_color(self, r, g, b, a):
        from std_msgs.msg import ColorRGBA
        c = ColorRGBA()
        c.r = r
        c.g = g
        c.b = b
        c.a = a
        return c


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
