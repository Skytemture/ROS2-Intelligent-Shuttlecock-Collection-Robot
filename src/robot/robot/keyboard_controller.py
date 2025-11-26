import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk

class WheelCommandPublisher(Node):
    def __init__(self):
        super().__init__('wheel_command_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Joint positions
        self.left_pos = 0.0
        self.right_pos = 0.0

        # Movement step per update
        self.step = 0.5

        # Current velocity increments
        self.left_vel = 0.0
        self.right_vel = 0.0

        # Timer for periodic publishing
        self.timer = self.create_timer(0.1, self.publish_continuous_joint_states)

        # Publish initial state
        self.publish_joint_states(0.0, 0.0)

    def publish_joint_states(self, left_delta, right_delta):
        self.left_pos += left_delta
        self.right_pos += right_delta

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.left_pos, self.right_pos]
        self.pub.publish(msg)
        self.get_logger().info(f'Published L:{self.left_pos:.2f} R:{self.right_pos:.2f}')

    def publish_continuous_joint_states(self):
        # If velocity zero, do nothing
        if self.left_vel == 0.0 and self.right_vel == 0.0:
            return
        self.publish_joint_states(self.left_vel, self.right_vel)

    def set_velocity(self, left_vel, right_vel):
        self.left_vel = left_vel
        self.right_vel = right_vel

    def stop(self):
        self.set_velocity(0.0, 0.0)

# Tkinter UI setup
class UI:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Robot Controller")

        tk.Button(self.root, text="↑ Forward", width=20, height=2, command=self.forward).pack()
        tk.Button(self.root, text="← Left", width=10, height=2, command=self.left).pack(side=tk.LEFT)
        tk.Button(self.root, text="→ Right", width=10, height=2, command=self.right).pack(side=tk.RIGHT)
        tk.Button(self.root, text="↓ Backward", width=20, height=2, command=self.backward).pack(side=tk.BOTTOM)
        tk.Button(self.root, text="■ Stop", width=20, height=2, command=self.stop).pack(side=tk.BOTTOM)

    def forward(self):
        self.node.set_velocity(self.node.step, self.node.step)

    def backward(self):
        self.node.set_velocity(-self.node.step, -self.node.step)

    def left(self):
        self.node.set_velocity(-self.node.step, self.node.step)

    def right(self):
        self.node.set_velocity(self.node.step, -self.node.step)

    def stop(self):
        self.node.stop()

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = WheelCommandPublisher()
    ui = UI(node)

    def spin_once():
        rclpy.spin_once(node, timeout_sec=0.01)

    def update():
        spin_once()
        ui.root.after(10, update)  # repeat every 10 ms

    ui.root.after(10, update)
    ui.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

