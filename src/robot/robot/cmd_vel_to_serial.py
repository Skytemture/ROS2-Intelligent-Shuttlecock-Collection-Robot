import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelToSerialBridge(Node):
    def __init__(self):
        super().__init__('cmdvel_to_serial_bridge')

        self.publisher_ = self.create_publisher(String, '/robot_control2', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        wheel_base = 0.3

        left = linear - angular * wheel_base / 2
        right = linear + angular * wheel_base / 2

        left_cmd = self.speed_to_code(left)
        right_cmd = self.speed_to_code(right)

        serial_cmd = f"{left_cmd}{right_cmd}"
        self.publisher_.publish(String(data=serial_cmd))
        self.get_logger().info(f"Published to /robot_control2: {serial_cmd}")

    def speed_to_code(self, speed):
        # 輸入速度為 m/s，轉換為 0~9 間整數 PWM 等級
        abs_speed = min(abs(speed), 1.0)
        level = int(abs_speed * 9) +1

        if speed > 0:
            return f"1{level}"  # 正轉
        elif speed < 0:
            return f"0{level}"  # 反轉
        else:
            return "00"         # 停止

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

