import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time


class RobotControlSerialNode(Node):
    def __init__(self):
        super().__init__('robot_control_serial_node')

        # Set up serial connection
        try:
            self.serial_port = serial.Serial('/dev/ttyTHS0', 115200, timeout=1)
            self.get_logger().info('Serial port /dev/ttyTHS0 opened successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial_port = None

        # Subscriber to /robot_control2
        self.subscription = self.create_subscription(
            String,
            '/robot_control2',
            self.listener_callback,
            10
        )

        # Publisher for incoming serial data (if needed)
        self.publisher_ = self.create_publisher(String, '/serial_input', 10)

    def listener_callback(self, msg: String):
        """Callback when receiving commands from /robot_control2"""
        cmd = f"A{msg.data}"  # ✅ Always add 'A' in front
        self.get_logger().info(f'Sending: {cmd}')

        if self.serial_port and self.serial_port.is_open:
            try:
                # 先送 Axxxx (command)
                data_to_send = (cmd + '\n').encode('utf-8')
                start_time = time.time()
                while time.time() - start_time < 0.49:  # 90ms
                    self.serial_port.write(data_to_send)
                    self.get_logger().info(f'Sent over serial: {cmd}')
                    time.sleep(0.01)

                # 再送 A0000
                stop_cmd = b'A0000\n'
                stop_end_time = time.time() + 0.01  # 10ms
                while time.time() < stop_end_time:
                    self.serial_port.write(stop_cmd)
                    self.get_logger().info("Sent over serial: A0000")
                    time.sleep(0.01)

            except serial.SerialException as e:
                self.get_logger().error(f'Error writing to serial port: {e}')

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                # 在關閉前連續送 1 秒的 A0000
                end_time = time.time() + 1.0
                stop_cmd = b'A0000\n'
                while time.time() < end_time:
                    self.serial_port.write(stop_cmd)
                    time.sleep(0.001)
                self.get_logger().info("Sent 'A0000' for 1 second before shutdown.")
            except serial.SerialException as e:
                self.get_logger().error(f'Error writing to serial port during shutdown: {e}')
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

