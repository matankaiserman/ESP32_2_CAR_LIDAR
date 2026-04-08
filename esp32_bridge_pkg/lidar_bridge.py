import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import socket
import struct
import math
import array
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
#from tf2_ros import StaticTransformBroadcaster

class LidarUdpNode(Node):
    def __init__(self):
        super().__init__('lidar_bridge')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher_ = self.create_publisher(LaserScan, '/scan', qos_profile)
        #self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        #self.publish_static_transforms()
        
        self.full_scan = array.array('f', [float('inf')] * 360)
        self.last_packet_angle = 0.0
        self.lidar_offset_ns = None # לוגיקת סנכרון
        
        self.lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lidar_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.lidar_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        
        try:
            self.lidar_sock.bind(("0.0.0.0", 9876))
            self.lidar_sock.setblocking(False) 
            self.get_logger().info("Lidar Bridge: Sync Logic Enabled")
        except Exception as e:
            self.get_logger().error(f"Socket error: {e}")

        self.create_timer(0.02, self.receive_and_publish)

    # def publish_static_transforms(self, time_ns=None):
    #     t = TransformStamped()
    #     if time_ns:
    #         t.header.stamp = rclpy.time.Time(nanoseconds=time_ns).to_msg()
    #     else:
    #         t.header.stamp = self.get_clock().now().to_msg()
        
    #     t.header.frame_id = 'base_link'
    #     t.child_frame_id = 'laser_frame'
    #     t.transform.translation.z = 0.11 
    #     t.transform.rotation.w = 1.0
    #     self.static_tf_broadcaster.sendTransform(t)

    def receive_and_publish(self):
        while True:
            try:
                data, addr = self.lidar_sock.recvfrom(2048)
                if not data or len(data) < 4: break

                # 1. חילוץ זמן ה-ESP מהבתים האחרונים
                esp_millis = struct.unpack('<I', data[-4:])[0]
                esp_time_ns = int(esp_millis * 1e6)

                # 2. חישוב אופסט מול שעון המחשב (פעם אחת בלבד)
                now_ns = self.get_clock().now().nanoseconds
                if self.lidar_offset_ns is None:
                    self.lidar_offset_ns = now_ns - esp_time_ns
                
                # 3. שחזור זמן היצירה של הפקטה הנוכחית
                current_msg_time_ns = esp_time_ns + self.lidar_offset_ns

                # 4. עיבוד הנקודות
                n_points = (len(data) - 4) // 8
                for i in range(n_points):
                    ang_deg, dist_mm = struct.unpack('<ff', data[i*8 : (i+1)*8])
                    idx = (360-int(round(ang_deg))) % 360
                    dist_m = dist_mm / 1000.0
                    
                    if 0.1 < dist_m < 8.0:
                        self.full_scan[idx] = dist_m
                    else:
                        self.full_scan[idx] = float('inf')
                    
                    # זיהוי סיום סיבוב (קפיצה בזווית)
                    if ang_deg < self.last_packet_angle - 10:
                        self.publish_scan(current_msg_time_ns)
                    
                    self.last_packet_angle = ang_deg

            except (BlockingIOError, socket.error):
                break

    def publish_scan(self, time_ns):
        msg = LaserScan()
        # שימוש בזמן המשוחזר מה-ESP
        msg.header.stamp = rclpy.time.Time(nanoseconds=time_ns).to_msg()
        msg.header.frame_id = "laser_frame"
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = math.radians(1.0)
        msg.range_min = 0.1
        msg.range_max = 12.0
        msg.ranges = self.full_scan.tolist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarUdpNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()