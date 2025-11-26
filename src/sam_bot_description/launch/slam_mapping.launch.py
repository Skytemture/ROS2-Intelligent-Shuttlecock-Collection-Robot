import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'urdf/sam_bot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    default_slam_config_path = os.path.join(pkg_share, 'config/slam_toolbox.yaml') 
    

    # robot_state_publisher：用於發布 TF
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    # joint_state_publisher：讓 fixed joint（如 laser）TF 有效
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )


    # 自訂控制器
    wheel_command_publisher_node = launch_ros.actions.Node(
        package='robot',
        executable='run',
        name='sensor_control',
        output='screen',
    )

    control_node = launch_ros.actions.Node(
        package='robot',
        executable='control',
        name='diff_drive_odom_node',
        output='screen'
    )

    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )


    # RViz2 可視化
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # 延遲啟動 slam_toolbox，避免 TF 尚未準備就啟動
    slam_toolbox_node = TimerAction(
        period=3.0,  # 延遲 3 秒
        actions=[
            launch_ros.actions.Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',  # 或改為 async_slam_toolbox_node
                name='slam_toolbox',
                output='screen',
                parameters=[LaunchConfiguration('slam_params')]
            )
        ]
    )
    yolo_patrol_node = TimerAction(
        period=3.0,  #delay sec
        actions=[
            launch_ros.actions.Node(
                package='robot',
                executable='yolo_patrol_node',
                name='yolo_patrol_node',
                output='screen'
            )
        ]
    )
    mapping_node = TimerAction(
       period=1.0,  #delay sec
        actions=[
            launch_ros.actions.Node(
                package='robot',
                executable='map',
                name='map_saver_node',
                output='screen'
            )
        ]
    )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'),

        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),

        launch.actions.DeclareLaunchArgument(
            name='slam_params',
            default_value=default_slam_config_path,
            description='Path to the slam_toolbox configuration YAML'),

        #joint_state_publisher_node,
        robot_state_publisher_node,
        wheel_command_publisher_node,
        rviz_node,
        #control_node,
        yolo_patrol_node,
        ekf_node, 
        slam_toolbox_node,
        mapping_node,
    ])

