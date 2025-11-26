from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': '/home/robot/ros2_ws/robot_map.yaml'}],
        output='screen',
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }],
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node,
    ])

