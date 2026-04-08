import rclpy
from rclpy.node import Node
import socket
import struct
import math
from sensor_msgs.msg import LaserScan

class LidarUdpNode(Node):
    def __init__(self):
        super().__init__('lidar_bridge')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        
        # הגדרות UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(("0.0.0.0", 9876))
            self.sock.settimeout(0.1) # Timeout קצר כדי לא לתקוע את ה-Node
            self.get_logger().info("Socket Bound to 9876 - Waiting for data...")
        except Exception as e:
            self.get_logger().error(f"Failed to bind socket: {e}")

        # מערך צבירה של 360 מעלות
        self.current_ranges = [float('inf')] * 360
        
        # טיימר יחיד שמנהל את הכל (20Hz)
        self.create_timer(0.05, self.update_and_publish)

    def update_and_publish(self):
        try:
            # נסיון לקרוא חבילה אחת בכל מחזור טיימר
            data, addr = self.sock.recvfrom(2048)
            
            if len(data) > 0:
                num_points = len(data) // 8
                # הדפסה קטנה כדי לדעת שה-Node חי
                print(f"Recv {num_points} pts from {addr}")
                
                for i in range(num_points):
                    chunk = data[i*8 : (i+1)*8]
                    if len(chunk) < 8: continue
                    
                    ang_deg, dist_mm = struct.unpack('<ff', chunk)
                    
                    # צבירה: מעדכנים רק את האינדקס הספציפי במערך הקיים
                    idx = int(round(ang_deg)) % 360
                    self.current_ranges[idx] = dist_mm / 1000.0

        except socket.timeout:
            # אין נתונים כרגע, זה תקין
            pass
        except Exception as e:
            print(f"Error: {e}")

        # פרסום המערך (כולל המידע שנצבר מחבילות קודמות)
        self.publish_scan()

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.range_min = 0.1
        msg.range_max = 12.0
        msg.ranges = self.current_ranges
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarUdpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()