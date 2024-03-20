import os
from setuptools import setup
from glob import glob

package_name = 'monkey_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'eds'), glob('eds/*.eds')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'sensor_msgs'
        ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = monkey_rover.motor_controller:main',
            'odom_test = monkey_rover.odom_test:main',
            'ps4_controller_converter = monkey_rover.ps4_controller_converter:main'
        ],
    },
)
