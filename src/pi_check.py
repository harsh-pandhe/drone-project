from pymavlink import mavutil
import time

# Update to your identified port
connection_string = '/dev/ttyAMA0' 
baud = 57600

print(f"Connecting to Pixhawk on {connection_string}...")
master = mavutil.mavlink_connection(connection_string, baud=baud)

# Wait for the first heartbeat to confirm the link is active
msg = master.wait_heartbeat(timeout=10)
if msg:
    print(f"Heartbeat received from System {master.target_system}!")
    print(f"Autopilot: {msg.autopilot}, Type: {msg.type}")
else:
    print("No heartbeat. Check TX/RX wiring and baud rate!")