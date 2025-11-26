from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
import os
from launch_ros.substitutions import FindPackageShare
import launch_ros
from launch.actions import TimerAction
import launch
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # 地圖檢查路徑
    map_path = '/home/robot/ros2_ws/robot_map.yaml'
    map_exists = os.path.exists(map_path)

    # 取得套件內 launch 資料夾位置
    pkg_share = FindPackageShare('sam_bot_description').find('sam_bot_description')
    launch_dir = os.path.join(pkg_share, 'launch')

    # 沒有地圖 → 啟動 slam mapping 模式
    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_mapping.launch.py')),
        condition=UnlessCondition(str(map_exists))
    )

    # 有地圖 → 啟動 localization 模式
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_patrol.launch.py')),
        launch_arguments={'map': map_path}.items(),
        condition=IfCondition(str(map_exists))
    )
    serial_commander_Node = launch_ros.actions.Node(
       package='robot',
       executable='serial_out',
       name='serial_out',
       output='screen'
    )
    yolo_map_marker_Node = launch_ros.actions.Node(
       package='robot',
       executable='mark',
       name='yolo_map_marker',
       output='screen'
    )

    return LaunchDescription([
        LogInfo(msg="使用現有地圖，啟動 Localization 模式..." if map_exists else "未找到地圖，啟動 Mapping 模式..."),
        slam_mapping,
        slam_localization,
        serial_commander_Node,
        yolo_map_marker_Node,
    ])

