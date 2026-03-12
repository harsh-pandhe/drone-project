from pymavlink import mavutil

print("Connecting to Pixhawk...")

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)

master.wait_heartbeat()
print("Heartbeat received!")

while True:
    msg = master.recv_match(blocking=True)
    if msg:
        print(msg)
