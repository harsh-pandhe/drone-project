import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
import time
import math

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

class PixhawkBridge(Node):
    def __init__(self):
        super().__init__('pixhawk_bridge')
        
        # 1. Connect to Pixhawk (Assuming TELEM2 is wired to /dev/ttyAMA0)
        self.get_logger().info("Waiting for Pixhawk heartbeat on /dev/ttyAMA0...")
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
        self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat received! Bridge is ACTIVE.")

        # 2. Listen to FAST-LIO
        self.sub = self.create_subscription(Odometry, '/Odometry', self.odom_cb, 10)

    def odom_cb(self, msg):
        # Extract X, Y, Z
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract Orientation
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Send directly to Pixhawk EKF2
        self.master.mav.vision_position_estimate_send(
            int(time.time() * 1e6), # Time in microseconds
            x, y, z,
            roll, pitch, yaw
        )

def main():
    rclpy.init()
    node = PixhawkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
