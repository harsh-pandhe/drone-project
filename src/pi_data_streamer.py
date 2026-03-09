import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
import socket
import json
import threading
import time

# PC Configuration (Your Laptop's Wi-Fi IP)
PC_IP = "100.117.129.57" 
PC_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# The Master Data Dictionary
unified_data = {
    "timestamp": 0.0,
    "lidar_x": 0.0, "lidar_y": 0.0, "lidar_z": 0.0,
    "flow_x": 0.0, "flow_y": 0.0, "flow_quality": 0,
    "mag_x": 0, "mag_y": 0, "mag_z": 0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0
}

# --- ROS 2 Node for FAST-LIO ---
class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.sub = self.create_subscription(Odometry, '/Odometry', self.odom_cb, 10)

    def odom_cb(self, msg):
        unified_data["lidar_x"] = round(msg.pose.pose.position.x, 3)
        unified_data["lidar_y"] = round(msg.pose.pose.position.y, 3)
        unified_data["lidar_z"] = round(msg.pose.pose.position.z, 3)

def run_ros_thread():
    rclpy.init()
    node = LidarSubscriber()
    rclpy.spin(node)

# --- MAVLink & UDP Broadcasting ---
def main():
    print("Starting ROS 2 LiDAR Thread...")
    threading.Thread(target=run_ros_thread, daemon=True).start()
    
    print("Connecting to Pixhawk MAVLink Splitter...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
    
    last_send_time = time.time()
    print(f"Streaming Unified Data to {PC_IP}:{PC_PORT}...")
    
    while True:
        # Catch Pixhawk Data (Flow, Mag, Attitude)
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            if msg_type == 'OPTICAL_FLOW':
                unified_data["flow_x"] = round(msg.flow_comp_m_x, 4)
                unified_data["flow_y"] = round(msg.flow_comp_m_y, 4)
                unified_data["flow_quality"] = msg.quality
            elif msg_type == 'RAW_IMU':
                unified_data["mag_x"] = msg.xmag
                unified_data["mag_y"] = msg.ymag
                unified_data["mag_z"] = msg.zmag
            elif msg_type == 'ATTITUDE':
                unified_data["roll"] = round(msg.roll, 3)
                unified_data["pitch"] = round(msg.pitch, 3)
                unified_data["yaw"] = round(msg.yaw, 3)
        
        # Blast to PC at 10Hz
        current_time = time.time()
        if current_time - last_send_time > 0.1:
            unified_data["timestamp"] = round(current_time, 2)
            sock.sendto(json.dumps(unified_data).encode(), (PC_IP, PC_PORT))
            last_send_time = current_time

if __name__ == '__main__':
    main()