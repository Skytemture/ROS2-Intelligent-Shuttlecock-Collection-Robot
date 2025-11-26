from setuptools import setup

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv = robot.cv_out:main',
            'serial_out = robot.serial_out:main',
            'control = robot.diff_drive_odom_node:main',
            'kc = robot.keyboard_controller:main',
            'run = robot.sensor_control:main',
            'yolo_patrol_node = robot.yolo_patrol_node:main',
            'map = robot.map_saver_node:main',
            'mark = robot.yolo_map_marker:main',
            'nav2serial = robot.cmd_vel_to_serial:main',
            'nav2_patrol = robot.nav2_patrol:main',
            'initial_pose_publisher = robot.initial_pose_publisher:main',
            'nav2_pointer = robot.nav2_pointer:main',
            'map_metadata_publisher = robot.map_metadata_publisher:main',
            'lifecycle = robot.lifecycle:main',
            'patrol_point_generate = robot.patrol_point_generate:main',
        ],
    },
)
