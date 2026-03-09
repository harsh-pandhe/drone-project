import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
master.wait_heartbeat()

print("--- SEARCHING FOR SENSOR PACKETS ---")

try:
    while True:
        # Check for ANY message
        msg = master.recv_match(blocking=True, timeout=1.0)
        if not msg:
            print("No MAVLink data at all! Check Pi-to-Pixhawk cable.")
            continue
            
        msg_type = msg.get_type()
        
        # We are looking for these specific IDs
        if msg_type == 'OPTICAL_FLOW':
            print(f"FOUND FLOW: Quality {msg.quality}")
        elif msg_type == 'DISTANCE_SENSOR':
            print(f"FOUND LASER: {msg.current_distance} cm")
        elif msg_type == 'BAD_DATA':
            print("Received Corrupt Data - Check Baud Rates!")
            
except KeyboardInterrupt:
    print("\nSearch stopped.")