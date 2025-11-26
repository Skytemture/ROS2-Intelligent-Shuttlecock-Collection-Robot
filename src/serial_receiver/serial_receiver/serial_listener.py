import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialListener(Node):
    def __init__(self):
        super().__init__('serial_listener')
        self.publisher_ = self.create_publisher(String, 'stm32_data', 10)

        # Set up the serial port
        try:
            self.serial_port = serial.Serial('/dev/ttyTHS0', 115200, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            rclpy.shutdown()

        # Create a timer to read serial data
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line:
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f"Received from STM32: {msg.data}")

                # Send "0303" back to STM32

        except Exception as e:
            self.get_logger().error(f"Error reading from serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

