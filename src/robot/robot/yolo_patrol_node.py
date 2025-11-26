#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import math
import random
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

class YoloPatrolNode(Node):
    def __init__(self):
        super().__init__('yolo_patrol_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.serial_pub = self.create_publisher(String, '/robot_control2', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Robot params
        self.wheel_base = 0.3
        self.linear_speed = 0.2
        self.angular_speed = 0.6

        # Court mask
        self.min_x, self.max_x = 0.0, 13.0
        self.min_y, self.max_y = 0.0, 6.0
        self.buffer = 0.2

        # Path check
        self.path_width = 0.3  # robot width to check
        self.path_distance = 2.0  # check 2 meters ahead

        # State
        self.map_data = None
        self.scan_data = None
        self.current_goal = None
        self.mapping_finished = False

        # Timer
        self.create_timer(0.5, self.explore_loop)

        self.get_logger().info(
            "YOLO Patrol Node started (LiDAR front check, map frontier/freespace, court mask, goal marker)"
        )

    # Callbacks
    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def scan_callback(self, msg: LaserScan):
        self.scan_data = msg

    # Main loop
    def explore_loop(self):
        if self.map_data is None or self.scan_data is None:
            self.get_logger().warn("Waiting for map or scan...")
            return

        # Get robot pose
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            yaw = self.quaternion_to_yaw(t.transform.rotation)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Cannot find transform map → base_link")
            return

        # Select new goal if needed
        goal_source = ""
        if self.current_goal is None or self.distance(rx, ry, *self.current_goal) < 0.3:
            goal = self.find_clear_space_ahead(rx, ry, yaw)
            if goal is not None:
                goal_source = "laser_free_space"
            else:
                # Try left/right 90 deg
                angles = [math.pi/2, -math.pi/2]
                for delta in angles:
                    goal = self.find_clear_space_ahead(rx, ry, yaw + delta)
                    if goal is not None:
                        goal_source = "laser_free_space_fallback"
                        break

            if goal is None:
                # Fallback: frontier points
                frontier = self.find_frontier(self.map_data)
                if frontier is None:
                    self.get_logger().info("[yolo_patrol_node]: No valid goal, mapping finished!")
                    self.mapping_finished = True
                    return
                # pick random frontier that passes LiDAR check
                valid_frontier = []
                for fx, fy in frontier:
                    if self.is_front_clear(rx, ry, fx, fy):
                        valid_frontier.append((fx, fy))
                if valid_frontier:
                    goal = random.choice(valid_frontier)
                    goal_source = "frontier"
                else:
                    self.get_logger().info("[yolo_patrol_node]: No valid goal, mapping finished!")
                    self.mapping_finished = True
                    return

            self.current_goal = goal
            self.publish_goal_marker(*goal)
            self.get_logger().info(f"New goal selected ({goal_source}): {goal}")

        # Move towards goal
        fx, fy = self.current_goal
        dx, dy = fx - rx, fy - ry
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - yaw)

        twist = Twist()
        if abs(angle_error) > 0.3:
            twist.angular.z = self.angular_speed * (1 if angle_error > 0 else -1)
        else:
            twist.linear.x = self.linear_speed

        self.publish_twist(twist.linear.x, twist.angular.z, goal_source)

    # Helper functions
    def distance(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    def publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.target_marker_pub.publish(marker)

    def find_clear_space_ahead(self, rx, ry, yaw):
        if self.scan_data is None:
            return None
        scan = self.scan_data
        angle_increment = scan.angle_increment
        angles = np.arange(scan.angle_min, scan.angle_max, angle_increment)
        ranges = np.array(scan.ranges[:len(angles)])

        half_width_angle = math.atan2(self.path_width / 2, self.path_distance)
        indices = np.where(np.abs(angles) <= half_width_angle)[0]
        if len(indices) == 0:
            return None

        front_ranges = ranges[indices]
        if np.all(front_ranges > self.path_distance):
            gx = rx + self.path_distance * math.cos(yaw)
            gy = ry + self.path_distance * math.sin(yaw)
            if (self.min_x + self.buffer <= gx <= self.max_x - self.buffer and
                self.min_y + self.buffer <= gy <= self.max_y - self.buffer):
                return (gx, gy)
        return None

    def is_front_clear(self, rx, ry, gx, gy):
        # check if LiDAR shows front clear for distance to goal
        dx = gx - rx
        dy = gy - ry
        yaw = math.atan2(dy, dx)
        distance = self.distance(rx, ry, gx, gy)
        scan = self.scan_data
        if scan is None:
            return False
        angle_increment = scan.angle_increment
        angles = np.arange(scan.angle_min, scan.angle_max, angle_increment)
        ranges = np.array(scan.ranges[:len(angles)])

        half_width_angle = math.atan2(self.path_width / 2, distance)
        indices = np.where(np.abs(angles) <= half_width_angle)[0]
        if len(indices) == 0:
            return False

        front_ranges = ranges[indices]
        return np.all(front_ranges > distance)

    def find_frontier(self, grid):
        w, h = grid.info.width, grid.info.height
        data = np.array(grid.data).reshape((h, w))
        frontier_points = []
        for y in range(1, h-1):
            for x in range(1, w-1):
                if data[y,x] == -1:
                    neighbors = data[y-1:y+2, x-1:x+2].flatten()
                    if 0 in neighbors:
                        wx = grid.info.origin.position.x + x * grid.info.resolution
                        wy = grid.info.origin.position.y + y * grid.info.resolution
                        if (self.min_x + self.buffer <= wx <= self.max_x - self.buffer and
                            self.min_y + self.buffer <= wy <= self.max_y - self.buffer):
                            frontier_points.append((wx, wy))
        return frontier_points if frontier_points else None

    def publish_twist(self, linear, angular, goal_source=""):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

        serial_cmd = self.cmdvel_to_serial(twist)
        self.serial_pub.publish(String(data=serial_cmd))

        self.get_logger().info(f"cmd_vel → /robot_control2: {serial_cmd} (goal_source: {goal_source})")

    def cmdvel_to_serial(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        left = linear - angular * self.wheel_base / 2
        right = linear + angular * self.wheel_base / 2
        left_cmd = self.speed_to_code(left)
        right_cmd = self.speed_to_code(right)
        return f"{left_cmd}{right_cmd}"

    def speed_to_code(self, speed):
        abs_speed = min(abs(speed), 1.0)
        level = int(abs_speed * 9) + 1
        if speed > 0:
            return f"1{level}"
        elif speed < 0:
            return f"0{level}"
        else:
            return "00"

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0*math.pi
        while angle < -math.pi:
            angle += 2.0*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = YoloPatrolNode()
    try:
        while rclpy.ok() and not node.mapping_finished:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

