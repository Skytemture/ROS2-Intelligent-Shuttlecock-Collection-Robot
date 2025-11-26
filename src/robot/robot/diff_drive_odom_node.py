import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class DiffDriveOdometryNode(Node):
    def __init__(self):
        super().__init__('diff_drive_odom_node')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.2)  # distance between left and right wheels

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Joint states for wheels
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        self.x = 0.0  # robot pose x
        self.y = 0.0  # robot pose y
        self.theta = 0.0  # robot yaw angle

        self.last_time = self.get_clock().now()

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("control start!")

    def joint_state_callback(self, msg):
        # Find wheel positions from joint_states message
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
        except ValueError:
            return

        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        # Calculate delta wheel rotations
        delta_left = left_pos - self.left_wheel_pos
        delta_right = right_pos - self.right_wheel_pos

        self.left_wheel_pos = left_pos
        self.right_wheel_pos = right_pos

        # Time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return
        self.last_time = current_time

        # Compute wheel traveled distances
        d_left = delta_left * self.wheel_radius
        d_right = delta_right * self.wheel_radius

        # Compute robot linear and angular displacement
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # Update robot pose
        if d_theta != 0:
            r = d_center / d_theta
            self.x += r * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y -= r * (math.cos(self.theta + d_theta) - math.cos(self.theta))
        else:
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)

        self.theta += d_theta
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi  # normalize

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # You can add velocity info if needed here (optional)

        self.odom_pub.publish(odom)

        # Publish TF from odom to base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveOdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

