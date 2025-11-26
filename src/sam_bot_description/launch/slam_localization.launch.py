from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'urdf/sam_bot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/localization.rviz')
    default_nav2_params = os.path.join(pkg_share, 'config/nav2_params.yaml')
    default_map_path = '/home/robot/ros2_ws/robot_map.yaml'
    default_slam_config_path = os.path.join(pkg_share, 'config/localization.yaml')

    nav2_params = LaunchConfiguration('params_file')
    slam_params = LaunchConfiguration('slam_params')

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # Launch all nav2 nodes separately
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings
    )

    recoveries_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
                'recoveries_server',
                'waypoint_follower',
                'local_costmap/local_cost_map',
                'global_costmap/global_costmap'
            ]
        }]
    )
    custom_lifecycle_manager_node = Node(
       package='robot',
       executable='lifecycle',
       name='lifecycle',
       output='screen'
    )
    delayed_lifecycle_manager = TimerAction(
       period=5.0,  # delay 5 seconds
       actions=[custom_lifecycle_manager_node]
    )
    patrol_node = Node(
        package='robot',
        executable='patrol',
        name='nav2_patrol_node',
        output='screen'
    )

    delayed_patrol = TimerAction(
        period=10.0,
        actions=[patrol_node]
    )


    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path),
        DeclareLaunchArgument('params_file', default_value=default_nav2_params),
        DeclareLaunchArgument('map', default_value=default_map_path),
        DeclareLaunchArgument('slam_params', default_value=default_slam_config_path),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),

        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),

        Node(
            package='robot',
            executable='map_metadata_publisher',
            name='map_metadata_publisher',
            output='screen',
            parameters=[{'map_yaml_path': default_map_path}]
        ),

        Node(
            package='robot',
            executable='run',
            name='sensor_control',
            output='screen'
        ),

        Node(
            package='robot',
            executable='nav2serial',
            name='cmd_vel_to_serial',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            output='screen'
        ),

        planner_node,
        controller_node,
        bt_navigator_node,
        recoveries_node,
        waypoint_follower_node,
        delayed_lifecycle_manager,
        delayed_patrol,
    ])

