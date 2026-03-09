import socket
import json
import csv
import tkinter as tk
import os

# UDP Setup to listen to the Pi
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# Setup CSV Logging
CSV_FILE = "stage1_telemetry_log.csv"
file_exists = os.path.isfile(CSV_FILE)
f = open(CSV_FILE, "a", newline="")
writer = csv.writer(f)

if not file_exists:
    # Write the header row if it's a new file
    writer.writerow(["Timestamp", "LiDAR_X", "LiDAR_Y", "LiDAR_Z", "Flow_X", "Flow_Y", "Flow_Qual", "Mag_X", "Mag_Y", "Mag_Z", "Roll", "Pitch", "Yaw"])

# Build the Dashboard
app = tk.Tk()
app.title("Project ASCEND - Live Data Visualizer")
app.geometry("500x400")
app.configure(bg="#1e1e1e")

tk.Label(app, text="LIVE SENSOR FEED", fg="#00ff00", bg="#1e1e1e", font=("Consolas", 18, "bold")).pack(pady=10)
data_label = tk.Label(app, text="Waiting for Raspberry Pi...", fg="white", bg="#1e1e1e", font=("Consolas", 12), justify="left")
data_label.pack(pady=10)

def update_gui():
    try:
        data, addr = sock.recvfrom(2048)
        telemetry = json.loads(data.decode())
        
        # Log to CSV immediately
        writer.writerow([
            telemetry["timestamp"], telemetry["lidar_x"], telemetry["lidar_y"], telemetry["lidar_z"],
            telemetry["flow_x"], telemetry["flow_y"], telemetry["flow_quality"],
            telemetry["mag_x"], telemetry["mag_y"], telemetry["mag_z"],
            telemetry["roll"], telemetry["pitch"], telemetry["yaw"]
        ])
        
        # Update Dashboard Text
        display_text = (
            f"--- LiDAR (FAST-LIO) ---\n"
            f"X: {telemetry['lidar_x']} | Y: {telemetry['lidar_y']} | Z: {telemetry['lidar_z']}\n\n"
            f"--- OPTICAL FLOW ---\n"
            f"Flow X: {telemetry['flow_x']} | Flow Y: {telemetry['flow_y']} | Quality: {telemetry['flow_quality']}%\n\n"
            f"--- MAGNETOMETER ---\n"
            f"Mag X: {telemetry['mag_x']} | Mag Y: {telemetry['mag_y']} | Mag Z: {telemetry['mag_z']}\n\n"
            f"--- ATTITUDE ---\n"
            f"Roll: {telemetry['roll']} | Pitch: {telemetry['pitch']} | Yaw: {telemetry['yaw']}"
        )
        data_label.config(text=display_text)
    except BlockingIOError:
        pass # No new data this millisecond, just loop
    except Exception as e:
        print(f"Error: {e}")

    app.after(20, update_gui) # Check for new data every 20ms

app.after(20, update_gui)
app.mainloop()

# Cleanly close CSV when window is closed
f.close()