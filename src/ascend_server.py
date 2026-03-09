import time
import threading
from flask import Flask
from flask_socketio import SocketIO
from pymavlink import mavutil

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Connection to Pixhawk
print("Connecting to Pixhawk...")
# Using 57600 as established in your previous successful MAVProxy test
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
master.wait_heartbeat()
print("Connected!")

def telemetry_loop():
    """Captures a wide range of MAVLink data for the dashboard"""
    while True:
        # Request data stream if necessary (standard for many flight controllers)
        master.mav.request_data_stream_send(master.target_system, master.target_component,
                                          mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        
        msg = master.recv_match(type=['VFR_HUD', 'HEARTBEAT', 'ATTITUDE', 'SYS_STATUS', 'GPS_RAW_INT', 'STATUSTEXT'], blocking=True, timeout=0.1)
        
        if msg:
            m_type = msg.get_type()
            
            if m_type == 'VFR_HUD':
                socketio.emit('vfr_data', {
                    'alt': round(msg.alt, 2),
                    'climb': round(msg.climb, 2),
                    'hdg': msg.heading,
                    'throttle': msg.throttle
                })
            
            elif m_type == 'ATTITUDE':
                socketio.emit('attitude', {
                    'roll': round(msg.roll * (180/3.14159), 2),
                    'pitch': round(msg.pitch * (180/3.14159), 2)
                })

            elif m_type == 'SYS_STATUS':
                socketio.emit('sys_status', {
                    'voltage': msg.voltage_battery / 1000.0,
                    'current': msg.current_battery / 100.0,
                    'level': msg.battery_remaining
                })

            elif m_type == 'STATUSTEXT':
                # Critical for debugging pre-arm checks
                socketio.emit('console', {'text': msg.text, 'severity': msg.severity})

            elif m_type == 'HEARTBEAT':
                is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                mode = mavutil.mode_string_v10(msg)
                socketio.emit('state', {'armed': bool(is_armed), 'mode': mode})
        
        time.sleep(0.05)

@socketio.on('command')
def handle_command(data):
    cmd = data['type']
    print(f"Executing: {cmd}")
    
    if cmd == 'ARM':
        master.arducopter_arm() # Simplified pymavlink helper
    elif cmd == 'DISARM':
        master.arducopter_disarm()
    elif cmd == 'TAKEOFF':
        altitude = data.get('alt', 2)
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    elif cmd == 'SET_MODE':
        mode = data['mode']
        if mode in master.mode_mapping():
            master.set_mode(master.mode_mapping()[mode])

if __name__ == '__main__':
    threading.Thread(target=telemetry_loop, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5000)