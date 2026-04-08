import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import socket
import math
import tf2_ros
import time

class ESP32BridgeUDP(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # --- הגדרות פיזיקליות ---
        self.WHEEL_DIAMETER = 0.0665
        self.WHEEL_BASE = 0.16
        self.TICKS_PER_REV = 8
        self.METERS_PER_TICK = (self.WHEEL_DIAMETER * math.pi) / self.TICKS_PER_REV
        
        # --- משתני מצב ---
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_left_ticks, self.last_right_ticks = 0, 0
        self.first_run = True
        self.last_time = time.time()
        
        # --- הגדרות UDP ---
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 8888
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.sock.bind((self.UDP_IP, self.UDP_PORT))
            self.sock.settimeout(0.1)
            self.get_logger().info(f"Bridge listening on {self.UDP_IP}:{self.UDP_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to start socket: {e}")
            raise e

        # --- ROS2 Publishers ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_timer(0.02, self.update_logic)

    def update_logic(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            raw_msg = data.decode('utf-8', errors='ignore').strip()
            
            if raw_msg and 'E' in raw_msg:
                # ניקוי ופיצול ההודעה
                clean_msg = raw_msg[raw_msg.find('E'):].strip()
                parts = [p.strip() for p in clean_msg.split(',')]
                
                if len(parts) >= 7:
                    left_ticks = int(parts[1])
                    right_ticks = int(parts[2])
                    gyro_z = float(parts[6])
                    
                    current_time = time.time()
                    dt = current_time - self.last_time
                    self.last_time = current_time

                    if self.first_run:
                        self.last_left_ticks = left_ticks
                        self.last_right_ticks = right_ticks
                        self.first_run = False
                        self.get_logger().info("First packet parsed!")
                        return

                    self.calculate_odometry(left_ticks, right_ticks, gyro_z, dt)
                    
        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().error(f"Loop error: {e}")

    def calculate_odometry(self, left_ticks, right_ticks, gyro_z, dt):
        d_left = (left_ticks - self.last_left_ticks) * self.METERS_PER_TICK
        d_right = (right_ticks - self.last_right_ticks) * self.METERS_PER_TICK
        
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        
        d_center = (d_left + d_right) / 2.0
        
        # שימוש בג'יירו לעדכון הזווית (יותר מדויק מאנקודרים)
        self.th += gyro_z * dt
        
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)
        
        self.publish_odom()

    def publish_odom(self):
        now = self.get_clock().now().to_msg()
        q_z = math.sin(self.th / 2.0)
        q_w = math.cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeUDP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()