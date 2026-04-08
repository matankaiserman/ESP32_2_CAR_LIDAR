import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'esp32_bridge_pkg'
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'robot.urdf')

    return LaunchDescription([
        # 1. Robot State Publisher - קורא את ה-URDF ושולח TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # 2. ה-Bridge של הליידר (הקוד המעודכן שלך)
        Node(
            package=package_name,
            executable='lidar_bridge',
            output='screen'
        ),
        
        # 3. ה-Bridge של האודומטריה
        Node(
            package=package_name,
            executable='motor_bridge',
            output='screen'
        ),

        # 4. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'map_update_interval': 0.2,
                'max_laser_range': 8.0,
            }]
        ),

        # 5. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])