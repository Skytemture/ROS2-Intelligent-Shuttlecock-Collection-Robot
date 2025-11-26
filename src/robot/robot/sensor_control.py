#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
import serial
from smbus2 import SMBus
import time
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import re  #正則表達式模組

# Encoder and wheel parameters
ENCODER_TICKS_PER_REV = 2700
WHEEL_RADIUS = 0.06  # in meters
WHEEL_BASE = 0.25    # 兩輪距離，請依實際調整（公尺）

# I2C parameters for BNO055
I2C_BUS = 1
BNO055_ADDRESS = 0x28
REG_CHIP_ID = 0x00
REG_OPR_MODE = 0x3D
REG_UNIT_SEL = 0x3B
REG_SYS_TRIGGER = 0x3F
REG_EULER_H_LSB = 0x1A
MODE_CONFIG = 0x00
MODE_NDOF = 0x0C

class SensorControlNode(Node):
    def __init__(self):
        super().__init__('sensor_control')

        # Initialize serial port for encoders
        self.serial_port = serial.Serial('/dev/ttyTHS0', 115200, timeout=1)
        self.get_logger().info("Serial port initialized")

        # Initialize IMU (BNO055)
        try:
            self.bus = SMBus(I2C_BUS)
            self.initialize_imu()
        except Exception as e:
            self.get_logger().error(f"IMU initialization failed: {e}")
            rclpy.shutdown()
            return

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left_ticks = None
        self.last_right_ticks = None

        # Joint state
        self.left_pos = 0.0
        self.right_pos = 0.0

        # 輪子相對 base_link 的固定座標 (可根據實際機械調整)
        self.left_wheel_pos = (0.0, WHEEL_BASE / 2, 0.0)   # y 向左輪方向
        self.right_wheel_pos = (0.0, -WHEEL_BASE / 2, 0.0) # y 向右輪方向

        # Timer
        self.timer = self.create_timer(0.1, self.update_state)
        self.tf_timer = self.create_timer(0.1, self.publish_wheel_tf)

        self.get_logger().info("Sensor control node started")

    def initialize_imu(self):
        chip_id = self.bus.read_byte_data(BNO055_ADDRESS, REG_CHIP_ID)
        if chip_id != 0xA0:
            raise RuntimeError(f"BNO055 not detected. Chip ID: {hex(chip_id)}")

        self.bus.write_byte_data(BNO055_ADDRESS, REG_SYS_TRIGGER, 0x20)
        time.sleep(0.7)
        self.bus.write_byte_data(BNO055_ADDRESS, REG_OPR_MODE, MODE_CONFIG)
        time.sleep(0.02)
        self.bus.write_byte_data(BNO055_ADDRESS, REG_UNIT_SEL, 0x00)
        self.bus.write_byte_data(BNO055_ADDRESS, REG_OPR_MODE, MODE_NDOF)
        time.sleep(0.02)

    def read_imu(self):
        data = self.bus.read_i2c_block_data(BNO055_ADDRESS, REG_EULER_H_LSB, 6)
        heading = (data[1] << 8 | data[0]) / 16.0
        roll = (data[3] << 8 | data[2]) / 16.0
        pitch = (data[5] << 8 | data[4]) / 16.0
        return heading, roll, pitch

    def publish_joint_states(self, left_delta, right_delta):
        self.left_pos += left_delta
        self.right_pos += right_delta

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.left_pos, self.right_pos]
        self.joint_pub.publish(msg)

        self.get_logger().info(f'Published JointState: L={self.left_pos:.2f}, R={self.right_pos:.2f}')

    def publish_wheel_tf(self):
        now = self.get_clock().now().to_msg()

        t_left = TransformStamped()
        t_left.header.stamp = now
        t_left.header.frame_id = 'base_link'
        t_left.child_frame_id = 'left_wheel'
        t_left.transform.translation.x = self.left_wheel_pos[0]
        t_left.transform.translation.y = self.left_wheel_pos[1]
        t_left.transform.translation.z = self.left_wheel_pos[2]
        t_left.transform.rotation.x = 0.0
        t_left.transform.rotation.y = 0.0
        t_left.transform.rotation.z = 0.0
        t_left.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_left)

        t_right = TransformStamped()
        t_right.header.stamp = now
        t_right.header.frame_id = 'base_link'
        t_right.child_frame_id = 'right_wheel'
        t_right.transform.translation.x = self.right_wheel_pos[0]
        t_right.transform.translation.y = self.right_wheel_pos[1]
        t_right.transform.translation.z = self.right_wheel_pos[2]
        t_right.transform.rotation.x = 0.0
        t_right.transform.rotation.y = 0.0
        t_right.transform.rotation.z = 0.0
        t_right.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_right)

    def update_state(self):
        try:
            line = self.serial_port.readline().decode().strip()
            self.get_logger().warn(f"Read encoder: {line}")
            if not line:
                return
            match = re.match(r"Enc1\s*=\s*(-?\d+),\s*Enc2\s*=\s*(-?\d+)", line)
            if not match:
                self.get_logger().warn(f"Invalid encoder format: {line}")
                return
            left_ticks = int(match.group(2))
            right_ticks = int(match.group(1)) #模型的左右移動相反
        except Exception as e:
            self.get_logger().warn(f"Failed to read encoder: {e}")
            return

        # 初始化上一筆ticks
        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            return

        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        # 計算輪子旋轉弧度
        left_rad = (delta_left / ENCODER_TICKS_PER_REV) * 2 * math.pi
        right_rad = (delta_right / ENCODER_TICKS_PER_REV) * 2 * math.pi

        # 計算距離和角度變化
        left_dist = left_rad * WHEEL_RADIUS
        right_dist = right_rad * WHEEL_RADIUS
        delta_dist = (left_dist + right_dist) / 2.0
        
        # 加入角度修正係數（避免旋轉過快）
        ANGLE_CORRECTION_FACTOR = 0.9  #可以調整這個值 (0.6~0.9 之間微調)
        delta_theta = (right_dist - left_dist) / WHEEL_BASE
        delta_theta *= ANGLE_CORRECTION_FACTOR


        # 更新位姿
        self.x += delta_dist * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_dist * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # 正規化角度

        now = self.get_clock().now().to_msg()

        # 發佈 Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = delta_dist / 0.1
        odom.twist.twist.angular.z = delta_theta / 0.1
        
        # 建議的 pose covariance（只啟用 x, y, yaw）
        odom.pose.covariance = [
            0.01, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.01, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  99999.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  99999.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0,  99999.0, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.02
        ]

        odom.twist.covariance = [
            0.01, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.01, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  99999.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  99999.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0,  99999.0, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.02
        ]



        self.odom_pub.publish(odom)

        # 發佈 IMU
        heading, roll, pitch = self.read_imu()
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = math.sin(math.radians(heading) / 2)
        imu.orientation.w = math.cos(math.radians(heading) / 2)
        imu.angular_velocity.x = math.radians(roll)
        imu.angular_velocity.y = math.radians(pitch)
        imu.angular_velocity.z = math.radians(heading)
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        self.imu_pub.publish(imu)

        # 發佈 TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # 發佈關節狀態
        self.publish_joint_states(left_rad, right_rad)


def main(args=None):
    rclpy.init(args=args)
    node = SensorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

