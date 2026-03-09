import csv
import time
from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
filename = "ascend_flight_log.csv"

print(f"Logging data to {filename}...")

with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Header: Timestamp, Velocity X, Velocity Y, Altitude
    writer.writerow(["timestamp", "vx", "vy", "alt"])

    try:
        while True:
            msg = connection.recv_match(type='OPTICAL_FLOW', blocking=True, timeout=1.0)
            if msg:
                t = time.time()
                writer.writerow([t, msg.flow_comp_m_x, msg.flow_comp_m_y, msg.ground_distance])
                print(f"Logged: VX:{msg.flow_comp_m_x:.3f} | Alt:{msg.ground_distance:.2f}")
    except KeyboardInterrupt:
        print("\nLog saved.")