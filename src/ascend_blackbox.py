import time
import csv
from pymavlink import mavutil

# 1. Connect to the Pixhawk 6c
PORT = '/dev/ttyAMA0'
BAUD = 921600
print(f"Connecting to Pixhawk on {PORT} at {BAUD} baud...")
master = mavutil.mavlink_connection(PORT, baud=BAUD)

master.wait_heartbeat()
print(f"Connected! Target System: {master.target_system}")

# 2. Request ALL Data Streams (10 Hz)
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
)

# 3. Setup the Master CSV Log File
filename = f"full_flight_log_{int(time.time())}.csv"
print(f"Logging ALL telemetry to {filename}...")
print("Press Ctrl+C to stop logging and save the file.")

# These are all the columns we will track
headers = [
    "Timestamp", "Flight_Mode",
    "Roll_deg", "Pitch_deg", "Yaw_deg",
    "Opt_Flow_X", "Opt_Flow_Y", "Opt_Flow_Qual",
    "Lidar_Dist_m", "Baro_Alt_m", "Climb_Rate_ms",
    "Batt_Voltage", "Batt_Current_A", "Batt_Remain_Pct",
    "RC1_Roll", "RC2_Pitch", "RC3_Throttle", "RC4_Yaw",
    "Vibration_X", "Vibration_Y", "Vibration_Z",
    "CPU_Load_Pct"
]

# Initialize the "State" dictionary with zeros
state = {header: 0.0 for header in headers}
state["Flight_Mode"] = "UNKNOWN"

# Timer to trigger CSV writes exactly at 10Hz (every 0.1 seconds)
last_write_time = time.time()
WRITE_INTERVAL = 0.1 

with open(filename, mode='w', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=headers)
    writer.writeheader()

    try:
        # 4. The Infinite Intercept Loop
        while True:
            # Grab the next packet (timeout quickly to keep the loop moving)
            msg = master.recv_match(blocking=True, timeout=0.05)
            
            if msg:
                msg_type = msg.get_type()
                
                # Update the state dictionary with whatever packet just arrived
                if msg_type == 'ATTITUDE':
                    state['Roll_deg'] = round(msg.roll * 57.2958, 2)
                    state['Pitch_deg'] = round(msg.pitch * 57.2958, 2)
                    state['Yaw_deg'] = round(msg.yaw * 57.2958, 2)
                
                elif msg_type == 'OPTICAL_FLOW':
                    state['Opt_Flow_X'] = round(msg.flow_comp_m_x, 4)
                    state['Opt_Flow_Y'] = round(msg.flow_comp_m_y, 4)
                    state['Opt_Flow_Qual'] = msg.quality
                
                elif msg_type == 'DISTANCE_SENSOR':
                    # Lidar distance usually comes in centimeters, convert to meters
                    state['Lidar_Dist_m'] = round(msg.current_distance / 100.0, 3)
                
                elif msg_type == 'VFR_HUD':
                    state['Baro_Alt_m'] = round(msg.alt, 2)
                    state['Climb_Rate_ms'] = round(msg.climb, 2)
                
                elif msg_type == 'SYS_STATUS':
                    state['Batt_Voltage'] = round(msg.voltage_battery / 1000.0, 2)
                    state['Batt_Current_A'] = round(msg.current_battery / 100.0, 2)
                    state['Batt_Remain_Pct'] = msg.battery_remaining
                    state['CPU_Load_Pct'] = round(msg.load / 10.0, 1)
                
                elif msg_type == 'RC_CHANNELS':
                    # Raw PWM values from your controller (1000 to 2000)
                    state['RC1_Roll'] = msg.chan1_raw
                    state['RC2_Pitch'] = msg.chan2_raw
                    state['RC3_Throttle'] = msg.chan3_raw
                    state['RC4_Yaw'] = msg.chan4_raw
                
                elif msg_type == 'VIBRATION':
                    # Super important for checking if your props are balanced
                    state['Vibration_X'] = round(msg.vibration_x, 4)
                    state['Vibration_Y'] = round(msg.vibration_y, 4)
                    state['Vibration_Z'] = round(msg.vibration_z, 4)
                
                elif msg_type == 'HEARTBEAT':
                    # Translates the ArduPilot mode numbers into text (e.g., "LOITER")
                    state['Flight_Mode'] = mavutil.mode_string_v10(msg)

            # 5. The 10Hz Snapshot Trigger
            current_time = time.time()
            if current_time - last_write_time >= WRITE_INTERVAL:
                state['Timestamp'] = round(current_time, 3)
                writer.writerow(state)
                file.flush() # Save to SD card immediately
                last_write_time = current_time

    except KeyboardInterrupt:
        print(f"\nLogging stopped. Data safely saved to {filename}")