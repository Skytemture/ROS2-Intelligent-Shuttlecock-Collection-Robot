from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
import numpy as np
import math
import random

class SimpleLocalization(Node):
    def __init__(self):
        super().__init__('simple_localization')
        self.get_logger().info("SimpleLocalization node started")

        self.num_particles = 200
        self.particles = None
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.initialized = False
        self.prev_odom = None

        # 先只訂閱 /map
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.get_logger().info("Waiting for /map to initialize particles...")

        # 創建 global localization client
        self.gl_service_client = self.create_client(Empty, '/reinitialize_global_localization')

    def map_callback(self, msg: OccupancyGrid):
        if self.map_data is None:
            self.get_logger().info("Received /map, initializing map data")
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        if not self.initialized:
            self.initialize_particles()
            self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 50)
            self.get_logger().info("Particles initialized and odom/scan subscribed")

            # 自動 global localization
            self.call_global_localization()

    def call_global_localization(self):
        if not self.gl_service_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("/reinitialize_global_localization service not available")
            return
        req = Empty.Request()
        future = self.gl_service_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info("Global localization requested, particles re-scattered"))

    def initialize_particles(self):
        self.particles = []
        h, w = self.map_data.shape
        free_cells = np.argwhere(self.map_data == 0)
        for _ in range(self.num_particles):
            cell = free_cells[random.randint(0, len(free_cells)-1)]
            x = self.map_origin[0] + cell[1]*self.map_resolution + random.uniform(0, self.map_resolution)
            y = self.map_origin[1] + cell[0]*self.map_resolution + random.uniform(0, self.map_resolution)
            yaw = random.uniform(-math.pi, math.pi)
            weight = 1.0/self.num_particles
            self.particles.append([x, y, yaw, weight])
        self.initialized = True
        self.get_logger().info(f"Particles initialized: {self.num_particles} particles (random in map)")

    def odom_callback(self, msg: Odometry):
        if not self.initialized:
            return
        if self.prev_odom is None:
            self.prev_odom = msg
            return
        dx = msg.pose.pose.position.x - self.prev_odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.prev_odom.pose.pose.position.y
        dyaw = self.get_yaw(msg.pose.pose.orientation) - self.get_yaw(self.prev_odom.pose.pose.orientation)
        for i in range(self.num_particles):
            self.particles[i][0] += dx + random.gauss(0, 0.01)
            self.particles[i][1] += dy + random.gauss(0, 0.01)
            self.particles[i][2] += dyaw + random.gauss(0, 0.005)
        self.prev_odom = msg

    def scan_callback(self, msg: LaserScan):
        if not self.initialized or self.map_data is None:
            return
        for i in range(self.num_particles):
            self.particles[i][3] *= random.uniform(0.9, 1.1)
        weights = [p[3] for p in self.particles]
        s = sum(weights)
        for i in range(self.num_particles):
            self.particles[i][3] /= s
        self.resample_particles()
        self.publish_pose()

    def resample_particles(self):
        weights = [p[3] for p in self.particles]
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=weights)
        new_particles = [self.particles[i][:3] + [1.0/self.num_particles] for i in indices]
        self.particles = new_particles

    def publish_pose(self):
        x = sum(p[0]*p[3] for p in self.particles)
        y = sum(p[1]*p[3] for p in self.particles)
        yaw = sum(p[2]*p[3] for p in self.particles)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)
        msg.pose.covariance = [0.0]*36
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[35] = 0.05
        self.pose_pub.publish(msg)
        self.get_logger().info(f"Published /amcl_pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")

    @staticmethod
    def get_yaw(q: Quaternion):
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw/2)
        q.w = math.cos(yaw/2)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

