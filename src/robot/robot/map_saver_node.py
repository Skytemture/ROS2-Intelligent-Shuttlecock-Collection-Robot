#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from slam_toolbox.srv import SaveMap
import os
from std_msgs.msg import String as StdString



class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')

        # 儲存地圖的資料夾與檔名
        #self.map_dir = '/home/robot/ros2_ws/src/robot/maps'
        self.map_name = 'map'

        # 建立 SaveMap 服務 client
        self.client = self.create_client(SaveMap, '/slam_toolbox/save_map')

        # 設定定時器：每 15 秒儲存一次地圖
        self.timer = self.create_timer(3.0, self.check_and_save_map)

        self.map_saved = False
        self.get_logger().info('map_saver_node 啟動中，將監測地圖狀態並自動儲存...')

    def check_and_save_map(self):
        if self.map_saved:
            return  # 地圖已儲存過就不重複儲存

        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/slam_toolbox/save_map 服務尚未可用')
            return

        request = SaveMap.Request()
        request.name = StdString()
        request.name.data = 'robot_map'
        #request.relative = True

        self.get_logger().info(f'儲存地圖至: /home/robot/ros2_ws/robot_map.yaml') #{response}
        #self.map_saved = True

        future = self.client.call_async(request)

        def callback(future):
           try:
              response = future.result()
              print("SaveMap 服務回應:", response)  # 印出完整回應物件
           except Exception as e:
              self.get_logger().error(f'呼叫失敗: {e}')



        future.add_done_callback(callback)


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

