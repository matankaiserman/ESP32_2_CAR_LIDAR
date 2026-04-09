import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'esp32_bridge_pkg'
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'robot.urdf')

    return LaunchDescription([
        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # 2. ה-Bridge של הליידר
        Node(
            package=package_name,
            executable='lidar_bridge',
            output='screen'
        ),
        
        # 3. ה-Bridge של המנועים (האודומטריה וה-IMU)
        Node(
            package=package_name,
            executable='motor_bridge',
            output='screen'
        ),

        # 4. EKF Node - "המוח" שמשלב אודומטריה ו-IMU
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'frequency': 50.0,
                'sensor_timeout': 0.1,
                'two_d_mode': True, # אנחנו רובוט שנוסע על רצפה
                'publish_tf': True, # ה-EKF עכשיו אחראי על ה-TF!

                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',

                # הגדרת אודומטריה (מהגלגלים)
                'odom0': '/odom',
                'odom0_config': [False,  False,  False, # x, y, z
                                 False, False, False,  # roll, pitch, yaw
                                 True, False, False, # vx, vy, vz
                                 False, False, False, # vroll, vpitch, vyaw
                                 False, False, False],# ax, ay, az

                # הגדרת IMU
                'imu0': '/imu/data',
                'imu0_config': [False, False, False, # x, y, z
                                False, False, False,  # roll, pitch, yaw (מסתמכים על הגירו לסיבוב)
                                False, False, False, # vx, vy, vz
                                False, False, True,  # vroll, vpitch, vyaw (מהירות זוויתית)
                                False,  False,  False],# ax, ay, az (תאוצה קווית)
                
                'imu0_relative': True # מתייחס לשינוי יחסי ולא לערך מוחלט (מצפן)
            }]
        ),

        # 5. SLAM Toolbox
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

        # 6. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])