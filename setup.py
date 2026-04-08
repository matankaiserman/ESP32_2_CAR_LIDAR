import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'esp32_bridge_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # נתיבים מתוקנים עבור colcon
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), ['urdf/robot.urdf']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matan',
    maintainer_email='matan@todo.todo',
    description='Bridge for ESP32 Lidar and Odom',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # שים לב: הקובץ הוא bridge_node.py והפונקציה היא main
            'motor_bridge = esp32_bridge_pkg.bridge_node:main',
            'lidar_bridge = esp32_bridge_pkg.lidar_bridge:main',
        ],
    },
)