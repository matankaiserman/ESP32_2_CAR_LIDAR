# Autonomous Robot Bridge & EKF Integration (ROS 2)

This project implements a robust communication and estimation layer for a differential drive robot using **ESP32**, **ROS 2 (Humble)**, and a **LiDAR Delta-2G**.

## 🛠 System Architecture
## 🧩 Multi-MCU Gateway Architecture
To optimize processing power and minimize latency, the robot utilizes a distributed multi-ESP32 setup communicating through a central **Main Gateway ESP32**:

- **Sub-system A (LiDAR Node):** Dedicated ESP32 for high-frequency sampling and buffering of the Xiaomi Delta-2G LiDAR data.
- **Sub-system B (Vision Node):** ESP32-CAM module handling real-time video stream and visual feedback.
- **Main Gateway (Comm Hub):** Aggregates data from sub-systems, processes IMU and Encoder feedback, and maintains the UDP link with the ROS 2 host.

## 📊 Expanded Data Flow
1. **LiDAR Node** -> (Serial/I2C) -> **Main Gateway** -> (UDP Port 8888) -> `lidar_bridge` -> `/scan`
2. **Vision Node** -> (UDP/HTTP) -> `video_stream_node` -> `/camera/image_raw`
3. **Encoders/IMU** -> **Main Gateway** -> (UDP Port 8888) -> `motor_bridge` -> `/odom` & `/imu/data`

## 🚀 Key Features
- **Time Synchronization:** Implemented `odom_offset` logic to align ESP32 internal clock with ROS 2 system time, ensuring accurate laser-scan transforms.
- **Sensor Fusion:** Merged wheel encoders (linear velocity) with IMU (angular velocity) to prevent drift and handle mechanical slippage.
- **Stability:** Fixed common UDP socket issues (Port Reuse) and deadzone filtering for IMU noise.

## 📊 Data Flow
1. ESP32 (UDP Port 8888) -> `motor_bridge`
2. `motor_bridge` -> `/odom` & `/imu/data`
3. `robot_localization` (EKF) -> TF Transform (`odom` -> `base_link`)
4. `lidar_bridge` -> `/scan`
5. `slam_toolbox` -> `/map`
