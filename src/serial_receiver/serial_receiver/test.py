import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class RobotControlPublisher(Node):
    def __init__(self):
        super().__init__('robot_control_publisher')

        # Publisher
        self.cmd_pub = self.create_publisher(String, '/robot_control2', 10)

        # commands cycle
        self.commands = ['0202', '1212', '0212', '1202']
        self.current_index = 0
        self.latest_command = self.commands[self.current_index]
        self.command_lock = threading.Lock()

        # 啟動自動切換指令 thread
        threading.Thread(target=self.command_switch_loop, daemon=True).start()

    def publish_command(self, cmd: str):
        """發佈純指令 (不加 A)"""
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'[PUB] Sent: {msg.data}')

    def command_switch_loop(self):
        """循環切換四個指令"""
        while rclpy.ok():
            with self.command_lock:
                # 取下一個 command
                self.current_index = (self.current_index + 1) % len(self.commands)
                new_cmd = self.commands[self.current_index]
                self.latest_command = new_cmd

            # ✅ 切換時先發 0000，再發新的指令
            self.publish_command('0000')
            time.sleep(0.1)
            self.publish_command(new_cmd)

            self.get_logger().info(f'[AUTO] Changed command to: {new_cmd}')
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

