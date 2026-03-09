import time
import threading
import curses
import csv
from datetime import datetime
from pymavlink import mavutil

# --- MISSION PARAMETERS ---
TARGET_ALTITUDE = 2.0     # Target hover height in meters
LIFTOFF_THR = 1680        # Increased for a stronger, decisive climb
HOVER_THR = 1500          # Neutral throttle for indoor modes
MIN_SAFE_VOLTAGE = 10.5   # Safety limit

# --- LOGGING INITIALIZATION ---
session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f"telemetry_{session_id}.csv"
txt_filename = f"events_{session_id}.txt"

telemetry_file = open(csv_filename, "w", newline="")
csv_writer = csv.writer(telemetry_file)
csv_writer.writerow([
    "Timestamp", "Flight_Mode", "Armed", "Alt_m", "Climb_ms", 
    "Roll_deg", "Pitch_deg", "Yaw_deg", "Batt_V", "Batt_A", "Batt_Pct",
    "Thr_PWM", "OptFlow_Qual", "Vib_X", "Vib_Y", "Vib_Z", "CPU_Load_Pct"
])

event_file = open(txt_filename, "w")
event_file.write(f"=== ASCEND MISSION CONTROL ULTIMATE v6.1 SESSION: {session_id} ===\n")

# --- GLOBAL STATE ---
state = {
    'alt': 0.0, 'climb': 0.0, 'hdg': 0, 'roll': 0.0, 'pitch': 0.0,
    'batt_v': 0.0, 'batt_pct': 0, 'batt_curr': 0.0,
    'mode': 'WAITING...', 'armed': False,
    'log': [],
    'rc_throttle': 1000, 'rc_pitch': 1500, 'rc_roll': 1500, 'rc_yaw': 1500,
    'ekf_flags': 0, 'flow_qual': 0, 'vibration': [0.0, 0.0, 0.0], 'cpu_load': 0
}

def update_log(msg):
    """Updates UI and Black Box event log simultaneously."""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    state['log'].append(msg)
    if len(state['log']) > 8: state['log'].pop(0)
    event_file.write(f"[{timestamp}] {msg}\n")
    event_file.flush()

def send_rc_override(master):
    """Sends current stick positions to Pixhawk."""
    if not master: return
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        state['rc_roll'], state['rc_pitch'], state['rc_throttle'], state['rc_yaw'],
        0, 0, 0, 0
    )

# --- ULTIMATE FLIGHT AUTOMATION ---

def auto_takeoff_sequence(master):
    """Smooth 2m automatic flight. Forced Origin + Arm + PID-style Climb."""
    update_log("AUTO: Starting Smooth Takeoff...")
    
    # 1. Force Origin (Required for indoor stabilization)
    update_log("AUTO: Anchoring EKF Origin...")
    master.mav.param_set_send(master.target_system, master.target_component, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    master.mav.set_gps_global_origin_send(master.target_system, int(19.033 * 1e7), int(73.029 * 1e7), 0)
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1.0)

    # 2. Enter ALT_HOLD
    master.set_mode(master.mode_mapping()['ALT_HOLD'])
    time.sleep(0.5)

    # 3. Capture ground and Arm
    ground_ref = state['alt']
    target_abs_alt = ground_ref + TARGET_ALTITUDE
    update_log(f"AUTO: Ground: {ground_ref}m. Target: {target_abs_alt}m")
    
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(2.0)
    
    if not state['armed']:
        update_log("AUTO: Arming FAILED. Press [o] manually.")
        return

    # 4. Controlled Climb
    update_log(f"AUTO: Ascending (PWM {LIFTOFF_THR})...")
    state['rc_throttle'] = LIFTOFF_THR
    send_rc_override(master)

    # 5. Climb Phase (Push exactly to target altitude)
    while state['alt'] < target_abs_alt:
        send_rc_override(master)
        time.sleep(0.05)
        if abs(state['roll']) > 30 or abs(state['pitch']) > 30:
            update_log("AUTO: Safety Tilt. Cutting power.")
            state['rc_throttle'] = 1000
            send_rc_override(master)
            return

    # 6. Final Hover
    update_log("AUTO: Target reached. Engaging Hover.")
    state['rc_throttle'] = HOVER_THR
    send_rc_override(master)
    time.sleep(0.5) # Let the drone's momentum settle
    
    update_log("AUTO: Activating Optical Flow (LOITER)...")
    master.set_mode(master.mode_mapping()['LOITER'])
    update_log("AUTO: 2m Flight Stable.")

def auto_land_sequence(master):
    """Proper Landing: Uses ArduPilot LAND mode with scripted disarm."""
    update_log("AUTO: Starting Precision Landing...")
    
    if 'LAND' in master.mode_mapping():
        master.set_mode(master.mode_mapping()['LAND'])
        update_log("AUTO: Engaging Mode LAND...")
    
    timeout = time.time() + 15
    while state['armed'] and time.time() < timeout:
        state['rc_throttle'] = 1300 
        send_rc_override(master)
        time.sleep(0.2)
        if state['alt'] < 0.15: # Proximity sensor hit ground level
            update_log("AUTO: Ground contact detected.")
            break

    update_log("AUTO: Final Disarming...")
    state['rc_throttle'] = 1000
    send_rc_override(master)
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    update_log("AUTO: Disarmed. Landing Complete.")

# --- DATA PROCESSING & UI ---

def mavlink_loop(master):
    master.wait_heartbeat()
    update_log("Connected to Pixhawk!")
    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    while True:
        msg = master.recv_match(blocking=True, timeout=0.1)
        if not msg: continue
        m_type = msg.get_type()
        
        if m_type == 'GLOBAL_POSITION_INT':
            state['alt'] = round(msg.relative_alt / 1000.0, 2)
            csv_writer.writerow([
                datetime.now().strftime("%H:%M:%S.%f"), state['mode'], state['armed'], 
                state['alt'], state['climb'], state['roll'], state['pitch'], state['hdg'],
                state['batt_v'], state['batt_curr'], state['batt_pct'], state['rc_throttle'],
                state['flow_qual'], state['vibration'][0], state['vibration'][1], state['vibration'][2], state['cpu_load']
            ])
            telemetry_file.flush()
        elif m_type == 'VFR_HUD':
            state['climb'] = round(msg.climb, 2)
        elif m_type == 'ATTITUDE':
            state['roll'], state['pitch'] = round(msg.roll * 57.3, 1), round(msg.pitch * 57.3, 1)
            state['hdg'] = round(msg.yaw * 57.3, 1)
        elif m_type == 'SYS_STATUS':
            state['batt_v'], state['batt_pct'] = round(msg.voltage_battery / 1000.0, 2), msg.battery_remaining
            state['batt_curr'], state['cpu_load'] = msg.current_battery / 100.0, msg.load / 10.0
        elif m_type == 'HEARTBEAT':
            state['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            state['mode'] = mavutil.mode_string_v10(msg)
        elif m_type == 'EKF_STATUS_REPORT':
            state['ekf_flags'] = msg.flags
        elif m_type == 'OPTICAL_FLOW':
            state['flow_qual'] = msg.quality
        elif m_type == 'VIBRATION':
            state['vibration'] = [round(msg.vibration_x, 3), round(msg.vibration_y, 3), round(msg.vibration_z, 3)]
        elif m_type == 'STATUSTEXT':
            update_log(f"SYS: {msg.text}")

def draw_dashboard(stdscr, master):
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.keypad(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_RED, -1)
    curses.init_pair(3, curses.COLOR_CYAN, -1)
    curses.init_pair(4, curses.COLOR_YELLOW, -1)

    while True:
        stdscr.erase()
        c_grn, c_red, c_cyn, c_ylw = curses.color_pair(1)|curses.A_BOLD, curses.color_pair(2)|curses.A_BOLD, curses.color_pair(3)|curses.A_BOLD, curses.color_pair(4)|curses.A_BOLD

        stdscr.addstr(0, 0, "█"*68, c_cyn)
        stdscr.addstr(1, 10, "🚀 ASCEND MISSION CONTROL - SMOOTH v6.1", c_grn)
        stdscr.addstr(2, 0, "█"*68, c_cyn)

        stdscr.addstr(4, 2, "[ FLIGHT STATE ]", c_ylw)
        stdscr.addstr(5, 2, f"MODE   : {state['mode']}", curses.color_pair(1))
        stdscr.addstr(6, 2, f"STATUS : {'ARMED (LIVE)' if state['armed'] else 'DISARMED'}", c_red if state['armed'] else c_grn)
        stdscr.addstr(7, 2, f"POWER  : {state['batt_v']}V / {state['batt_curr']}A", curses.color_pair(1))

        stdscr.addstr(4, 38, "[ SENSORS ]", c_ylw)
        stdscr.addstr(5, 38, f"ALTITUDE : {state['alt']} m", curses.color_pair(1))
        stdscr.addstr(6, 38, f"FLOW QUAL: {state['flow_qual']}", curses.color_pair(1))
        stdscr.addstr(7, 38, f"EKF POS  : {'VALID' if (state['ekf_flags'] & 8) else 'INVALID'}", c_grn if (state['ekf_flags'] & 8) else c_ylw)

        stdscr.addstr(10, 0, "-"*68, c_cyn)
        stdscr.addstr(11, 2, "[ MISSION ] [c] CALIB [o] SET ORIGIN [k] AUTO TAKEOFF", c_ylw)
        stdscr.addstr(12, 2, "            [h] ALTHOLD [l] LOITER     [j] AUTO LAND", c_ylw)
        stdscr.addstr(13, 2, "            [a] ARM     [d] DISARM     [q] QUIT", c_ylw)

        stdscr.addstr(15, 0, "-"*68, c_cyn)
        stdscr.addstr(16, 2, "[ MANUAL ] [W/S] Pitch [A/D] Roll [UP/DN] Thr [x] KILL", c_ylw)
        stdscr.addstr(17, 2, f"STICKS: THR:{state['rc_throttle']} PCH:{state['rc_pitch']} ROL:{state['rc_roll']}", c_cyn)

        for i, log_msg in enumerate(state['log']):
            if 19+i < curses.LINES - 1:
                stdscr.addstr(19+i, 2, f"> {log_msg}", curses.color_pair(1))

        stdscr.refresh()
        key = stdscr.getch()
        
        state['rc_pitch'], state['rc_roll'], state['rc_yaw'] = 1500, 1500, 1500
        
        if key != -1:
            if key == ord('q'): break
            elif key == ord('c'): master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 1, 0, 0, 0, 0)
            elif key == ord('o'):
                update_log("CMD: Forcing Origin Reset...")
                master.mav.param_set_send(master.target_system, master.target_component, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                master.mav.set_gps_global_origin_send(master.target_system, int(19.033 * 1e7), int(73.029 * 1e7), 0)
                master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0, 0, 0, 0, 0, 0)
            elif key == ord('h'): master.set_mode(master.mode_mapping()['ALT_HOLD'])
            elif key == ord('l'): master.set_mode(master.mode_mapping()['LOITER'])
            elif key == ord('a'): master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
            elif key == ord('d'): master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            elif key == ord('k'): threading.Thread(target=auto_takeoff_sequence, args=(master,), daemon=True).start()
            elif key == ord('j'): threading.Thread(target=auto_land_sequence, args=(master,), daemon=True).start()
            elif key == ord('w'): state['rc_pitch'] = 1420
            elif key == ord('s'): state['rc_pitch'] = 1580
            elif key == ord('a'): state['rc_roll'] = 1420
            elif key == ord('d'): state['rc_roll'] = 1580
            elif key == curses.KEY_UP: state['rc_throttle'] = min(state['rc_throttle'] + 25, 2000)
            elif key == curses.KEY_DOWN: state['rc_throttle'] = max(state['rc_throttle'] - 25, 1000)
            elif key == ord(' '): state['rc_throttle'] = HOVER_THR
            elif key == ord('x'): 
                state['rc_throttle'] = 1000
                update_log("CMD: EMERGENCY STOP")
                master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

        send_rc_override(master)
        time.sleep(0.05)

if __name__ == '__main__':
    try:
        master_conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
        thread = threading.Thread(target=mavlink_loop, args=(master_conn,), daemon=True)
        thread.start()
        curses.wrapper(draw_dashboard, master_conn)
    finally:
        telemetry_file.close()
        event_file.close()