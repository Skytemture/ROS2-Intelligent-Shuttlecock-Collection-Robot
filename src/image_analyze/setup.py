from setuptools import find_packages, setup

package_name = 'image_analyze'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skytemture',
    maintainer_email='skytemture@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_detector = image_analyze.depth_object_detector:main",
            "test = image_analyze.test:main",
            "ball= image_analyze.ball:main",
            "pc= image_analyze.point_clouds:main",
            "pub = image_analyze.color_and_depth:main",
            "yolo = image_analyze.cv:main",
        ],
    },
)
