import rclpy
from rclpy.node import Node
import socket
import math
import tf2_ros
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class ESP32BridgeUDP(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.get_logger().info("!!! BRIDGE START - SYNCED TIMESTAMP MODE !!!")

        self.WHEEL_DIAMETER = 0.0557
        #self.WHEEL_DIAMETER = 0.06
        self.WHEEL_BASE = 0.155
        self.TICKS_PER_REV = 8
        self.METERS_PER_TICK = (self.WHEEL_DIAMETER * math.pi) / self.TICKS_PER_REV
        
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_left_ticks, self.last_right_ticks = 0, 0
        self.first_run = True
        
        self.odom_offset_ns = None  # לוגיקת סנכרון
        self.last_msg_time_obj = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 8888))
        self.sock.settimeout(1.0) 

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.running = True
        self.data_thread = threading.Thread(target=self.receive_and_publish_loop)
        self.data_thread.daemon = True 
        self.data_thread.start()

    def receive_and_publish_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                raw_msg = data.decode('utf-8', errors='ignore').strip()
                if not raw_msg or 'E' not in raw_msg: continue

                clean_msg = raw_msg[raw_msg.find('E'):].strip()
                parts = [p.strip() for p in clean_msg.split(',')]
                
                if len(parts) >= 8:
                    l_ticks = int(parts[1])
                    r_ticks = int(parts[2])
                    g_z = float(parts[6])
                    esp_millis = int(parts[7])
                    esp_time_ns = int(esp_millis * 1e6)

                    # --- לוגיקת סנכרון זמן ---
                    now_ns = self.get_clock().now().nanoseconds
                    if self.odom_offset_ns is None:
                        self.odom_offset_ns = now_ns - esp_time_ns
                    
                    # זמן יצירה משוחזר (זמן המחשב שבו ה-ESP דגם את החיישנים)
                    creation_time_ns = esp_time_ns + self.odom_offset_ns
                    current_msg_time_obj = rclpy.time.Time(nanoseconds=creation_time_ns)

                    if self.first_run:
                        self.last_left_ticks = l_ticks
                        self.last_right_ticks = r_ticks
                        self.last_msg_time_obj = current_msg_time_obj
                        self.first_run = False
                        continue

                    # חישוב DT מבוסס על זמן היצירה האמיתי
                    dt = (current_msg_time_obj - self.last_msg_time_obj).nanoseconds / 1e9
                    if dt <= 0: continue

                    # חישוב אודומטריה
                    d_left = (l_ticks - self.last_left_ticks) * self.METERS_PER_TICK
                    d_right = (r_ticks - self.last_right_ticks) * self.METERS_PER_TICK
                    self.last_left_ticks = l_ticks
                    self.last_right_ticks = r_ticks
                    
                    d_center = (d_left + d_right) / 2.0
                    if abs(g_z) < 0.015: g_z = 0.0 # Deadzone לגירו
                    
                    scale_correction = 0.9895  # IMU מזייף %
                    self.th += (g_z) * dt * scale_correction
                    self.x += d_center * math.cos(self.th)
                    self.y += d_center * math.sin(self.th)

                    self.last_msg_time_obj = current_msg_time_obj
                    self.send_odometry(current_msg_time_obj)

            except socket.timeout: continue
            except Exception as e:
                self.get_logger().error(f"Loop error: {e}")

    def send_odometry(self, time_obj):
        stamp = time_obj.to_msg()
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeUDP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.running = False
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()