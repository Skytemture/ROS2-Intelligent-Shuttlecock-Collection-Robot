#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # 使用 Float32MultiArray 來發佈位置數據

from smbus2 import SMBus
import time
import math

# 設定 I2C bus 和 BNO055 地址
bus_number = 1  # Jetson 上的 /dev/i2c-1
address = 0x28  # 預設地址

# BNO055 暫存器定義
REG_CHIP_ID = 0x00
REG_OPR_MODE = 0x3D
REG_UNIT_SEL = 0x3B
REG_SYS_TRIGGER = 0x3F
REG_EULER_H_LSB = 0x1A  # Heading, pitch, roll 各佔 2 bytes

# 模式定義
MODE_CONFIG = 0x00
MODE_NDOF = 0x0C  # Sensor fusion 模式

class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')

        try:
            self.bus = SMBus(bus_number)
            self.initialize_sensor()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize BNO055: {e}")
            rclpy.shutdown()
            return

        self.publisher_ = self.create_publisher(Float32MultiArray, 'position', 10)
        self.timer = self.create_timer(1.0, self.publish_position_data)  # 每秒發佈一次位置數據
        self.get_logger().info("BNO055 Position Publisher Started")

    def initialize_sensor(self):
        # 驗證裝置
        chip_id = self.bus.read_byte_data(address, REG_CHIP_ID)
        if chip_id != 0xA0:
            self.get_logger().error(f"BNO055 not detected! Chip ID: {hex(chip_id)}")
            exit(1)
        self.get_logger().info(f"BNO055 detected! Chip ID: {hex(chip_id)}")

        # 重置感測器
        self.write_register(REG_SYS_TRIGGER, 0x20)
        time.sleep(0.7)

        # 設定為 CONFIG 模式
        self.write_register(REG_OPR_MODE, MODE_CONFIG)
        time.sleep(0.02)

        # 設定 UNIT（選擇角度為度）
        self.write_register(REG_UNIT_SEL, 0x00)

        # 啟用 NDOF 模式（融合模式，輸出姿態）
        self.write_register(REG_OPR_MODE, MODE_NDOF)
        time.sleep(0.02)

    def write_register(self, reg, value):
        self.bus.write_byte_data(address, reg, value)
        time.sleep(0.01)

    def read_register(self, reg, length):
        return self.bus.read_i2c_block_data(address, reg, length)

    def publish_position_data(self):
        position_msg = Float32MultiArray()

        # 讀取姿態數據
        data = self.read_register(REG_EULER_H_LSB, 6)

        # 原始數據
        heading = (data[1] << 8 | data[0]) / 16.0
        roll = (data[3] << 8 | data[2]) / 16.0
        pitch = (data[5] << 8 | data[4]) / 16.0

        # 打印原始數據以便調試
        self.get_logger().info(f"Raw Heading: {heading}, Raw Roll: {roll}, Raw Pitch: {pitch}")

        # 將數據填充到訊息中
        position_msg.data = [heading, roll, pitch]

        # 發布位置數據
        self.publisher_.publish(position_msg)
        self.get_logger().info(f"Published Position data: X: {heading:.2f}, Y: {roll:.2f}, Z: {pitch:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

