import rclpy
from rclpy.node import Node
import socket
import math
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class ESP32BridgeUDP(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.get_logger().info("!!! BRIDGE START - EKF READY MODE !!!")

        # פרמטרים
        self.WHEEL_DIAMETER = 0.056
        self.TICKS_PER_REV = 8
        self.METERS_PER_TICK = (self.WHEEL_DIAMETER * math.pi) / self.TICKS_PER_REV
        
        # משתני מצב
        self.last_left_ticks, self.last_right_ticks = 0, 0
        self.first_run = True
        self.odom_offset_ns = None
        self.last_msg_time_obj = None

        # תקשורת
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # השורה הבאה מאפשרת למערכת להשתמש בפורט שוב מיד אחרי סגירה
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", 8888))
        self.sock.settimeout(1.0) 

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        self.running = True
        self.data_thread = threading.Thread(target=self.receive_and_publish_loop)
        self.data_thread.daemon = True 
        self.data_thread.start()

    def receive_and_publish_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                raw_msg = data.decode('utf-8', errors='ignore').strip()
                
                # בדיקה אם המידע תקין
                if not raw_msg or 'E' not in raw_msg:
                    continue

                parts = [p.strip() for p in raw_msg[raw_msg.find('E'):].split(',')]
                
                if len(parts) >= 10:
                    l_ticks = int(parts[1])
                    r_ticks = int(parts[2])
                    a_x, a_y, a_z = float(parts[3]), float(parts[4]), float(parts[5])
                    g_x, g_y, g_z = float(parts[6]), float(parts[7]), float(parts[8])
                    esp_millis = int(parts[9])
                    
                    # חישוב זמן מסונכרן
                    esp_time_ns = int(esp_millis * 1e6)
                    now_ns = self.get_clock().now().nanoseconds
                    if self.odom_offset_ns is None:
                        self.odom_offset_ns = now_ns - esp_time_ns
                    
                    creation_time_ns = esp_time_ns + self.odom_offset_ns
                    current_msg_time_obj = rclpy.time.Time(nanoseconds=creation_time_ns)

                    if self.first_run:
                        self.last_left_ticks = l_ticks
                        self.last_right_ticks = r_ticks
                        self.last_msg_time_obj = current_msg_time_obj
                        self.first_run = False
                        self.get_logger().info("First packet received, starting Odom...")
                        continue

                    # שידור
                    self.send_odometry(current_msg_time_obj, l_ticks, r_ticks)
                    self.send_imu(current_msg_time_obj, a_x, a_y, a_z, g_x, g_y, g_z)
                    
                    # עדכון זמן אחרון (זה מה שהיה חסר!)
                    self.last_msg_time_obj = current_msg_time_obj

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Critical error in loop: {e}")

    def send_odometry(self, time_obj, l_ticks, r_ticks):
        # חישוב DT
        dt = (time_obj.nanoseconds - self.last_msg_time_obj.nanoseconds) / 1e9
        
        if dt <= 0:
            return

        odom = Odometry()
        odom.header.stamp = time_obj.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # מרחק ומהירות
        d_left = (l_ticks - self.last_left_ticks) * self.METERS_PER_TICK
        d_right = (r_ticks - self.last_right_ticks) * self.METERS_PER_TICK
        v_linear = ((d_left + d_right) / 2.0) / dt
        
        odom.twist.twist.linear.x = float(v_linear)
        odom.twist.covariance[0] = 0.05 
        
        # פרסום
        self.odom_pub.publish(odom)
        
        # עדכון טיקים
        self.last_left_ticks = l_ticks
        self.last_right_ticks = r_ticks

    def send_imu(self, time_obj, ax, ay, az, gx, gy, gz):
        imu_msg = Imu()
        imu_msg.header.stamp = time_obj.to_msg()
        imu_msg.header.frame_id = 'base_link'
        
        # ניקוי רעשים
        gz = 0.0 if abs(gz) < 0.01 else gz
        imu_msg.angular_velocity.z = float(gz)
        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)
        
        imu_msg.angular_velocity_covariance[8] = 0.001
        imu_msg.orientation_covariance[8] = 1000.0 
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeUDP()
    try:
        # שימוש ב-spin_once בתוך לולאה לפעמים עוזר במקרים של Threading
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()