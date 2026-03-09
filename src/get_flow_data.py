from pymavlink import mavutil
import time

# Use the port and baud rate confirmed by your success
connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

print("Reading Fused Optical Flow... (Ensure MAVProxy is closed)")

try:
    while True:
        # Since it's fused, we look for OPTICAL_FLOW or filtered velocities
        msg = connection.recv_match(type=['OPTICAL_FLOW', 'EKF_STATUS_REPORT'], blocking=True, timeout=1.0)
        
        if msg and msg.get_type() == 'OPTICAL_FLOW':
            # flow_comp_m_x/y is the ground-compensated flow in m/s
            print(f"VX: {msg.flow_comp_m_x:.3f} | VY: {msg.flow_comp_m_y:.3f} | Height: {msg.ground_distance:.2f}m")
        
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nStopped.")