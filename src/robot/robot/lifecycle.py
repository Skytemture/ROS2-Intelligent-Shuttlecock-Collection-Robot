import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class LifecycleManagerNode(Node):
    def __init__(self):
        super().__init__('custom_lifecycle_manager')
        self.node_names = [
            '/map_server',
            '/amcl',
            '/controller_server',
            '/planner_server',
            '/bt_navigator',
            '/recoveries_server',
            '/waypoint_follower',
            '/global_costmap/global_costmap',
            '/local_costmap/local_costmap'
        ]
        self.lifecycle_clients = {}
        self.current_index = 0
        self.stage = 'configure'  # 'configure' or 'activate'
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.current_index >= len(self.node_names):
            if self.stage == 'configure':
                self.get_logger().info('All nodes configured. Starting activation...')
                self.stage = 'activate'
                self.current_index = 0
            else:
                self.get_logger().info('All nodes activated. Shutting down lifecycle manager.')
                rclpy.shutdown()
                return

        node_name = self.node_names[self.current_index]
        if node_name not in self.lifecycle_clients:
            client = self.create_client(ChangeState, node_name + '/change_state')
            self.lifecycle_clients[node_name] = client

        client = self.lifecycle_clients[node_name]

        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {node_name}/change_state not available yet.')
            return

        if self.stage == 'configure':
            transition_id = Transition.TRANSITION_CONFIGURE
        else:
            transition_id = Transition.TRANSITION_ACTIVATE

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)
        future.add_done_callback(lambda f, n=node_name: self.response_callback(f, n))

        # move to next node immediately, don't wait for callback (simplification)
        self.current_index += 1

    def response_callback(self, future, node_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'{node_name} transition success.')
            else:
                self.get_logger().error(f'{node_name} transition failed.')
        except Exception as e:
            self.get_logger().error(f'Failed to call service on {node_name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LifecycleManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down and returning...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

