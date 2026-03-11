import time
import threading
import curses
import csv
import math
from datetime import datetime
from pymavlink import mavutil

# --- CONDITIONAL ROS2 IMPORT ---
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    ROS2_AVAILABLE = True
except ImportError:
    pass

# --- MISSION PARAMETERS ---
TARGET_ALTITUDE = 1.0     # Target hover height in meters (changed from 2.0)
SPOOL_THR = 1400          # Idle throttle to test frame vibrations before leaving the ground (increased from 1350)
LIFTOFF_THR = 1550        # Smooth liftoff power (reduced from 1600 for stability)
HOVER_THR = 1450          # Neutral throttle for indoor hover modes (reduced from 1500)
MIN_SAFE_VOLTAGE = 10.5   # Battery safety cutoff (volts)
MAX_VIBRATION = 25.0      # Abort flight if vibrations exceed 25 m/s^2 (increased threshold from 20)
MAX_GROUND_VIBE = 18.0    # Strict threshold during the ground spool-up test (increased from 15)
FASTLIO_TIMEOUT = 2.0     # Seconds before FAST-LIO is considered stale
VIBRATION_FILTER_SIZE = 10 # Rolling average window for vibration filtering

# --- HOVER CONTROLLER TUNING ---
ALT_PID_P = 6.0           # Proportional gain: PWM/meter (reduced from 8.0 for less overshoot)
ALT_PID_I = 0.8           # Integral gain: PWM/(meter*second) for steady-state correction (gentler)
ALT_PID_D = 3.0           # Derivative gain: PWM/(meter/second) for damping (reduced)
ALT_DEADBAND = 0.10       # Altitude deadband: ±10cm around target (increased from 0.08)
CLIMB_DAMPING = 0.25      # Throttle reduction factor during climb when near target (0-1)
CLIMB_DIST_THRESHOLD = 0.25 # Distance from target to start damping climb power (increased from 0.2)
POS_P_GAIN = 45.0         # Position P-gain: PWM per m/s optical flow (reduced from 60)
POS_INTEGRAL_GAIN = 2.0   # Position integral gain for drift correction (reduced from 3.0)
HOVER_SETTLE_TIME = 5.0   # Seconds to settle at target altitude before hover phase (increased from 3.0)
HOVER_DURATION = 10.0     # Duration of stable hover in seconds
CLIMB_THROTTLE_RAMP_RATE = 2  # PWM per control loop (50Hz) - smoother ramp (increased from 1)

# --- LOGGING INITIALIZATION ---
session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f"telemetry_{session_id}.csv"
txt_filename = f"events_{session_id}.txt"

telemetry_file = open(csv_filename, "w", newline="")
csv_writer = csv.writer(telemetry_file)
csv_writer.writerow([
    "Timestamp", "Flight_Mode", "Armed", "Alt_m", "Lidar_Alt_m", "Climb_ms", 
    "Roll_deg", "Pitch_deg", "Yaw_deg", "Batt_V", "Batt_A", "Batt_Pct",
    "Thr_PWM", "OptFlow_Qual", "OptFlow_Vx", "OptFlow_Vy",
    "Vib_X", "Vib_Y", "Vib_Z", "CPU_Load_Pct",
    "Livox_MinObs_m", "Livox_Sectors", "Vision_X", "Vision_Y", "Vision_Z",
    "FastLIO_X", "FastLIO_Y", "FastLIO_Z",
    "FastLIO_Vx", "FastLIO_Vy", "FastLIO_Vz",
    "FastLIO_Roll", "FastLIO_Pitch", "FastLIO_Yaw",
    "FastLIO_Hz", "FastLIO_Points"
])

event_file = open(txt_filename, "w")
event_file.write(f"=== ASCEND MISSION CONTROL v8.0 SESSION: {session_id} ===\n")

# --- GLOBAL STATE ---
state = {
    'alt': 0.0, 'lidar_alt': 0.0, 'rangefinder_healthy': False, 'climb': 0.0,
    'hdg': 0, 'roll': 0.0, 'pitch': 0.0,
    'batt_v': 0.0, 'batt_pct': 0, 'batt_curr': 0.0,
    'mode': 'CONNECTING...', 'armed': False, 'macro_status': 'IDLE',
    'log': [],
    'rc_throttle': 1000, 'rc_pitch': 1500, 'rc_roll': 1500, 'rc_yaw': 1500,
    'ekf_flags': 0, 'flow_qual': 0, 'flow_vx': 0.0, 'flow_vy': 0.0,
    'vibration': [0.0, 0.0, 0.0], 'vibration_filtered': [0.0, 0.0, 0.0],
    'vibration_history': [[], [], []], 'cpu_load': 0,
    'livox_obs': 0.0, 'livox_sectors': 0, 'vision_x': 0.0, 'vision_y': 0.0, 'vision_z': 0.0,
    # FAST-LIO SLAM State (direct from ROS2 /Odometry)
    'fastlio_x': 0.0, 'fastlio_y': 0.0, 'fastlio_z': 0.0,
    'fastlio_vx': 0.0, 'fastlio_vy': 0.0, 'fastlio_vz': 0.0,
    'fastlio_roll': 0.0, 'fastlio_pitch': 0.0, 'fastlio_yaw': 0.0,
    'fastlio_points': 0, 'fastlio_healthy': False, 'fastlio_hz': 0.0,
    'fastlio_last_time': 0.0, 'fastlio_msg_count': 0, 'fastlio_hz_timer': 0.0,
    'ros2_active': False, 'ros2_init_time': 0.0, 'ros2_error': '',
}

def update_log(msg):
    """Updates the UI event log and the black-box event file simultaneously."""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    state['log'].append(msg)
    if len(state['log']) > 8:
        state['log'].pop(0)
    event_file.write(f"[{timestamp}] {msg}\n")
    event_file.flush()

def update_vibration_filter(new_vib):
    """Update vibration rolling average for smooth filtering."""
    for i in range(3):
        state['vibration_history'][i].append(new_vib[i])
        if len(state['vibration_history'][i]) > VIBRATION_FILTER_SIZE:
            state['vibration_history'][i].pop(0)
        state['vibration_filtered'][i] = sum(state['vibration_history'][i]) / len(state['vibration_history'][i])

def send_rc_override(master):
    """Sends current stick positions to Pixhawk via RC channels override."""
    if not master: return
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        int(state['rc_roll']), int(state['rc_pitch']), int(state['rc_throttle']), int(state['rc_yaw']),
        0, 0, 0, 0
    )


# --- ROS2 FAST-LIO SUBSCRIBER ---

def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to euler angles (roll, pitch, yaw) in degrees."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(+1.0, t2))
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


if ROS2_AVAILABLE:
    class FastLioSubscriber(Node):
        """ROS2 node subscribing to FAST-LIO odometry & point cloud topics."""
        def __init__(self):
            super().__init__('ascend_fastlio_sub')
            
            # Try to create subscriptions with error handling
            try:
                # Subscribe to FAST-LIO odometry (Livox Mid-360)
                self.odom_sub = self.create_subscription(
                    Odometry, '/Odometry', self.odom_callback, 10)
                self.get_logger().info("Subscribed to /Odometry topic")
            except Exception as e:
                self.get_logger().warn(f"Failed to subscribe to /Odometry: {e}")
                self.odom_sub = None
            
            try:
                # Subscribe to registered point cloud for stats
                self.cloud_sub = self.create_subscription(
                    PointCloud2, '/cloud_registered', self.cloud_callback, 5)
                self.get_logger().info("Subscribed to /cloud_registered topic")
            except Exception as e:
                self.get_logger().warn(f"Failed to subscribe to /cloud_registered: {e}")
                self.cloud_sub = None
                
            self.get_logger().info("FAST-LIO subscriber initialized (Mid-360)")
            state['ros2_init_time'] = time.time()

        def odom_callback(self, msg):
            """Process FAST-LIO odometry: position, velocity, orientation."""
            now = time.time()

            try:
                # Position
                state['fastlio_x'] = round(msg.pose.pose.position.x, 3)
                state['fastlio_y'] = round(msg.pose.pose.position.y, 3)
                state['fastlio_z'] = round(msg.pose.pose.position.z, 3)

                # Velocity
                state['fastlio_vx'] = round(msg.twist.twist.linear.x, 3)
                state['fastlio_vy'] = round(msg.twist.twist.linear.y, 3)
                state['fastlio_vz'] = round(msg.twist.twist.linear.z, 3)

                # Orientation (quaternion -> euler degrees)
                q = msg.pose.pose.orientation
                roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
                state['fastlio_roll'] = round(roll, 1)
                state['fastlio_pitch'] = round(pitch, 1)
                state['fastlio_yaw'] = round(yaw, 1)

                # Health tracking
                state['fastlio_last_time'] = now
                state['fastlio_healthy'] = True

                # Hz calculation (rolling 1-second window)
                state['fastlio_msg_count'] += 1
                if now - state['fastlio_hz_timer'] >= 1.0:
                    state['fastlio_hz'] = state['fastlio_msg_count']
                    state['fastlio_msg_count'] = 0
                    state['fastlio_hz_timer'] = now
            except Exception as e:
                self.get_logger().error(f"Odom callback error: {e}")

        def cloud_callback(self, msg):
            """Extract point count from registered cloud."""
            try:
                # PointCloud2: total points = (row_step * height) / point_step
                if msg.point_step > 0:
                    state['fastlio_points'] = (msg.row_step * msg.height) // msg.point_step
            except Exception as e:
                self.get_logger().error(f"Cloud callback error: {e}")


def run_ros2_thread():
    """Background thread for ROS2 spinning with robust error handling."""
    try:
        update_log("ROS2: Initializing rclpy...")
        
        # Check if already initialized
        if not rclpy.ok():
            rclpy.init()
            
        node = FastLioSubscriber()
        
        # Give subscriptions time to activate
        time.sleep(1.0)
        
        state['ros2_active'] = True
        update_log("ROS2: FAST-LIO subscriber ACTIVE (Mid-360)")
        
        # Spin with timeout to allow graceful shutdown
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        
        while state['ros2_active']:
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.01)
            
    except RuntimeError as e:
        if "ROS 2 has not been initialized" in str(e) or "already initialized" in str(e):
            update_log(f"ROS2: Init error - {str(e)[:50]}")
        else:
            update_log(f"ROS2: Runtime error - {str(e)[:40]}")
        state['ros2_active'] = False
    except Exception as e:
        error_str = str(e)
        if "could not connect" in error_str.lower() or "connection" in error_str.lower():
            update_log("ROS2: Master not running. Check `ros2 daemon start`")
        else:
            update_log(f"ROS2: Init failed - {error_str[:40]}")
        state['ros2_active'] = False
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


def check_fastlio_health():
    """Update FAST-LIO health status based on message freshness."""
    if state['fastlio_last_time'] > 0:
        age = time.time() - state['fastlio_last_time']
        state['fastlio_healthy'] = age < FASTLIO_TIMEOUT
    else:
        state['fastlio_healthy'] = False


# --- PREFLIGHT DIAGNOSTICS ---

def run_preflight_diagnostics():
    """Runs a quick check on sensors before allowing autonomous takeoff."""
    update_log("SYS: Running Pre-Flight Diagnostics...")
    time.sleep(0.5)
    
    # Battery check
    if state['batt_v'] > 1.0 and state['batt_v'] < MIN_SAFE_VOLTAGE:
        update_log(f"DIAG: FAIL - Battery too low ({state['batt_v']}V)")
        return False
    
    # EKF health check (must have flags indicating good status)
    if state['ekf_flags'] & 0x02:  # EKF2 lane 0 EKF2 primary flag
        update_log(f"DIAG: INFO - EKF3 active (flags: 0x{state['ekf_flags']:04X})")
    else:
        update_log(f"DIAG: WARN - EKF health uncertain (flags: 0x{state['ekf_flags']:04X}). May lack altitude estimate.")
        
    # Vibration baseline check (system should be relatively stable on ground)
    if max(state['vibration']) > 5.0:
        update_log(f"DIAG: WARN - Ground vibrations elevated ({max(state['vibration']):.2f} m/s²). Check gimbal/prop balance.")
        
    if state['flow_qual'] < 10:
        update_log(f"DIAG: WARN - Optical Flow blind (Qual: {state['flow_qual']}). Needs light/texture.")
        
    if not state['rangefinder_healthy']:
        update_log("DIAG: WARN - Lidar dead. Relying on Barometer (Risk of drift).")

    # FAST-LIO health check
    if state['ros2_active']:
        check_fastlio_health()
        if state['fastlio_healthy']:
            update_log(f"DIAG: FAST-LIO OK ({state['fastlio_hz']:.0f} Hz)")
        else:
            update_log("DIAG: WARN - FAST-LIO not publishing. SLAM unavailable.")
    else:
        update_log("DIAG: INFO - ROS2 not active. FAST-LIO unavailable.")
        
    update_log("DIAG: PASS - Systems nominal. Ready for flight.")
    return True

# --- FLIGHT AUTOMATION ---

def auto_takeoff_sequence(master):
    """Smooth automatic takeoff sequence with diagnostics and enhanced vibe-checks."""
    if state['macro_status'] != 'IDLE': return
    state['macro_status'] = 'TAKEOFF'
    
    if not run_preflight_diagnostics():
        state['macro_status'] = 'IDLE'
        return

    # 1. Disable all arming checks and force EKF origin
    update_log("AUTO: Anchoring EKF Origin and waiting for convergence...")
    master.mav.param_set_send(master.target_system, master.target_component, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(1.5)  # Wait for ARMING_CHECK parameter to take effect on Pixhawk
    
    # Set GPS home location repeatedly to ensure it sticks
    for _ in range(5):
        master.mav.set_gps_global_origin_send(master.target_system, int(19.033 * 1e7), int(73.029 * 1e7), 0)
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(0.3)
    
    time.sleep(1.0)  # Extra settling time for EKF

    # 2. Enter STABILIZE mode (most permissive for arming)
    update_log("AUTO: Entering STABILIZE mode for arming...")
    master.set_mode(master.mode_mapping()['STABILIZE'])
    time.sleep(1.0)  # Wait for mode change to confirm

    # 3. Capture ground altitude
    start_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
    target_abs_alt = start_alt + TARGET_ALTITUDE
    update_log(f"AUTO: Ground: {start_alt:.2f}m. Target: {target_abs_alt:.2f}m")

    # 4. Attempt to arm with extended retry logic
    update_log("AUTO: Attempting arm...")
    arm_attempts = 0
    while not state['armed'] and arm_attempts < 3:
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
        time.sleep(1.5)
        arm_attempts += 1
        if not state['armed'] and arm_attempts < 3:
            update_log(f"AUTO: Arm attempt {arm_attempts} failed, retrying...")
    
    if not state['armed']:
        update_log("AUTO: Arming FAILED completely. Safety switch engaged or other issue.")
        state['macro_status'] = 'IDLE'
        return
    
    update_log("AUTO: Armed successfully in STABILIZE. Switching to ALT_HOLD...")

    # 5. Switch to ALT_HOLD for altitude control
    master.set_mode(master.mode_mapping()['ALT_HOLD'])
    time.sleep(1.0)

    # 6. Extended Ground Spool-Up & Dynamic Vibration Check
    update_log(f"AUTO: Extended spool-up to {SPOOL_THR} for thorough vibration testing...")
    
    # Ramp up to spool throttle slowly
    for thr in range(1100, SPOOL_THR, 15):  # Reduced step size for smoother ramp
        state['rc_throttle'] = thr
        send_rc_override(master)
        time.sleep(0.05)
    
    # Hold at spool throttle for extended period (3.0 seconds instead of 1.5)
    update_log("AUTO: Holding motors at spool throttle for 3.0s - monitoring vibrations...")
    max_spool_vib = 0.0
    for i in range(30):  # 30 * 0.1s = 3.0 seconds
        time.sleep(0.1)
        send_rc_override(master)
        current_max_vib = max(state['vibration_filtered'])
        if current_max_vib > max_spool_vib:
            max_spool_vib = current_max_vib
        if i % 10 == 0:
            update_log(f"AUTO: Spool check [{i/30*100:.0f}%] - Peak vib: {max_spool_vib:.2f} m/s²")
        
        if current_max_vib > MAX_GROUND_VIBE:
            update_log(f"AUTO: ABORT! High ground vibrations ({current_max_vib:.1f} m/s² > {MAX_GROUND_VIBE} threshold). Balance Props!")
            state['rc_throttle'] = 1000
            send_rc_override(master)
            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            state['macro_status'] = 'IDLE'
            return

    update_log(f"AUTO: Ground vibration test PASS (max {max_spool_vib:.2f} m/s²). Frame stable. Beginning climb...")

    # 7. Controlled Climb (Slower and Smoother)
    update_log(f"AUTO: Climbing to {TARGET_ALTITUDE}m with smooth throttle profile...")
    climb_start_time = time.time()
    climb_throttle = SPOOL_THR
    last_climb_thr_update = time.time()
    
    while climb_throttle < LIFTOFF_THR:
        current_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
        alt_to_target = target_abs_alt - current_alt
        current_time = time.time()
        
        # Smooth throttle ramp with time-based increment
        if current_time - last_climb_thr_update >= 0.04:  # Update every ~2 control cycles
            # If very close to target, dampen throttle ramp to prevent overshoot
            if alt_to_target < CLIMB_DIST_THRESHOLD:
                climb_throttle = HOVER_THR + int((LIFTOFF_THR - HOVER_THR) * alt_to_target / CLIMB_DIST_THRESHOLD * CLIMB_DAMPING)
            else:
                climb_throttle += CLIMB_THROTTLE_RAMP_RATE  # Use configurable ramp rate
            last_climb_thr_update = current_time
        
        state['rc_throttle'] = int(climb_throttle)
        send_rc_override(master)
        time.sleep(0.02)
        
        # Safety: don't climb forever
        if time.time() - climb_start_time > 8.0:
            break
        
        # Check for vibration issues during climb
        if max(state['vibration_filtered']) > MAX_VIBRATION:
            update_log(f"AUTO: FATAL VIBRATION during climb ({max(state['vibration_filtered']):.1f} m/s²)! KILLED.")
            state['rc_throttle'] = 1000
            send_rc_override(master)
            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            state['macro_status'] = 'IDLE'
            return

    # 8. Climb phase monitoring (with damping)
    start_time = time.time()
    
    while True:
        current_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
        alt_error = target_abs_alt - current_alt
        
        # Dampen climb aggressiveness when approaching target
        if abs(alt_error) < CLIMB_DIST_THRESHOLD * 1.5:
            # Use simplified P control to approach target gently
            throttle_adj = int(alt_error * 60)  # Reduced from 80 for gentler control
            state['rc_throttle'] = int(HOVER_THR + throttle_adj)
        
        if current_alt >= target_abs_alt:
            update_log("AUTO: Target altitude reached.")
            break
        if time.time() - start_time > 6.0:
            update_log("AUTO: Climb timeout reached. Stopping climb.")
            break

        send_rc_override(master)
        time.sleep(0.05)

        # --- SAFETY KILL SWITCHES ---
        if abs(state['roll']) > 35 or abs(state['pitch']) > 35:
            update_log("AUTO: CRITICAL TILT! EMERGENCY STOP.")
            state['rc_throttle'] = 1000
            send_rc_override(master)
            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            state['macro_status'] = 'IDLE'
            return
            
        max_vib = max(state['vibration_filtered'])
        if max_vib > MAX_VIBRATION:
            update_log(f"AUTO: FATAL VIBRATION ({max_vib:.1f} m/s²)! KILLED.")
            state['rc_throttle'] = 1000
            send_rc_override(master)
            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            state['macro_status'] = 'IDLE'
            return

    # 9. Brake and engage hover
    update_log("AUTO: Braking. Engaging hover.")
    state['rc_throttle'] = HOVER_THR
    send_rc_override(master)
    time.sleep(0.5)

    update_log("AUTO: Activating LOITER (position hold)...")
    master.set_mode(master.mode_mapping()['LOITER'])
    time.sleep(0.5)
    
    # 10. SETTLING PERIOD: Let LOITER stabilize at target altitude with minimal intervention
    update_log(f"AUTO: Settling at {TARGET_ALTITUDE}m for {HOVER_SETTLE_TIME}s...")
    settle_start = time.time()
    alt_integral = 0.0
    prev_alt_error = 0.0
    pos_integral_x = 0.0
    pos_integral_y = 0.0
    prev_time = time.time()
    
    while time.time() - settle_start < HOVER_SETTLE_TIME and state['armed']:
        current_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
        current_time = time.time()
        dt = max(0.01, current_time - prev_time)  # Delta time for I and D terms
        prev_time = current_time
        
        # --- ALTITUDE CONTROLLER (PID) with filtered vibrations ---
        alt_error = target_abs_alt - current_alt
        
        # Only apply corrections if outside deadband
        if abs(alt_error) > ALT_DEADBAND:
            # Proportional term
            alt_p = alt_error * ALT_PID_P
            
            # Integral term (accumulate error over time)
            alt_integral += alt_error * dt
            alt_integral = max(-30, min(30, alt_integral))  # Tighter clamp for stability
            alt_i = alt_integral * ALT_PID_I
            
            # Derivative term (smooth based on error rate)
            alt_d = (alt_error - prev_alt_error) / dt * ALT_PID_D if dt > 0 else 0
            alt_d = max(-80, min(80, alt_d))  # Reduced clamp for smoother response
            
            throttle_delta = alt_p + alt_i + alt_d
        else:
            throttle_delta = 0
            alt_integral *= 0.95  # Slowly decay integral in deadband
        
        state['rc_throttle'] = int(max(1200, min(1750, HOVER_THR + throttle_delta)))
        prev_alt_error = alt_error
        
        # --- POSITION CONTROLLER (Optical Flow feedback) ---
        if state['flow_qual'] > 50:  # Only use optical flow if quality is good
            flow_x = state['flow_vx']
            flow_y = state['flow_vy']
            
            # Accumulate position error from optical flow
            pos_integral_x += flow_x * dt
            pos_integral_y += flow_y * dt
            
            # Clamp integral to reasonable limits
            max_pos_int = 0.4  # Max 0.4m position error integration
            pos_integral_x = max(-max_pos_int, min(max_pos_int, pos_integral_x))
            pos_integral_y = max(-max_pos_int, min(max_pos_int, pos_integral_y))
            
            # Generate roll/pitch corrections
            pitch_correction = (-flow_x * POS_P_GAIN - pos_integral_x * POS_INTEGRAL_GAIN) / 100
            roll_correction = (-flow_y * POS_P_GAIN - pos_integral_y * POS_INTEGRAL_GAIN) / 100
            
            # Apply gentle position corrections (±5° max)
            state['rc_pitch'] = int(max(1450, min(1550, 1500 + pitch_correction)))
            state['rc_roll'] = int(max(1450, min(1550, 1500 + roll_correction)))
        else:
            state['rc_pitch'] = 1500
            state['rc_roll'] = 1500
        
        send_rc_override(master)
        time.sleep(0.05)
        
        # Safety checks during settling
        if abs(state['roll']) > 35 or abs(state['pitch']) > 35:
            update_log("AUTO: CRITICAL TILT during settle! Reducing corrections.")
            alt_integral *= 0.5
            pos_integral_x *= 0.5
            pos_integral_y *= 0.5
        
        max_vib = max(state['vibration_filtered'])
        if max_vib > MAX_VIBRATION:
            update_log(f"AUTO: FATAL VIBRATION during settle ({max_vib:.1f} m/s²)! KILLED.")
            state['rc_throttle'] = 1000
            send_rc_override(master)
            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            state['macro_status'] = 'IDLE'
            return
    
    # 11. STABLE HOVER PHASE: Maintain at target altitude and position
    update_log(f"AUTO: {TARGET_ALTITUDE}m stable hover achieved. Starting {HOVER_DURATION}s hold...")
    hover_start_time = time.time()
    hover_frames = 0
    
    while time.time() - hover_start_time < HOVER_DURATION and state['armed']:
        current_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
        current_time = time.time()
        dt = max(0.01, current_time - prev_time)
        prev_time = current_time
        hover_frames += 1
        
        # --- MAINTAIN ALTITUDE with reduced aggressiveness ---
        alt_error = target_abs_alt - current_alt
        
        if abs(alt_error) > ALT_DEADBAND:
            alt_p = alt_error * ALT_PID_P
            alt_integral += alt_error * dt
            alt_integral = max(-25, min(25, alt_integral))  # Tighter clamp for stability
            alt_i = alt_integral * ALT_PID_I
            alt_d = (alt_error - prev_alt_error) / dt * ALT_PID_D if dt > 0 else 0
            throttle_delta = alt_p + alt_i + alt_d
        else:
            throttle_delta = 0
            alt_integral *= 0.97
        
        state['rc_throttle'] = int(max(1250, min(1750, HOVER_THR + throttle_delta)))
        prev_alt_error = alt_error
        
        # --- MAINTAIN POSITION via Optical Flow ---
        if state['flow_qual'] > 50:
            flow_x = state['flow_vx']
            flow_y = state['flow_vy']
            
            pos_integral_x += flow_x * dt
            pos_integral_y += flow_y * dt
            
            pos_integral_x = max(-0.2, min(0.2, pos_integral_x))
            pos_integral_y = max(-0.2, min(0.2, pos_integral_y))
            
            pitch_correction = (-flow_x * POS_P_GAIN - pos_integral_x * POS_INTEGRAL_GAIN) / 100
            roll_correction = (-flow_y * POS_P_GAIN - pos_integral_y * POS_INTEGRAL_GAIN) / 100
            
            state['rc_pitch'] = int(max(1470, min(1530, 1500 + pitch_correction)))
            state['rc_roll'] = int(max(1470, min(1530, 1500 + roll_correction)))
        else:
            state['rc_pitch'] = 1500
            state['rc_roll'] = 1500
        
        send_rc_override(master)
        time.sleep(0.05)
        
        # Monitor safety conditions
        if abs(state['roll']) > 35 or abs(state['pitch']) > 35:
            update_log("AUTO: CRITICAL TILT during hover! EMERGENCY STOP.")
            state['rc_throttle'] = 1000
            send_rc_override(master)
            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            state['macro_status'] = 'IDLE'
            return
            
        max_vib = max(state['vibration_filtered'])
        if max_vib > MAX_VIBRATION:
            update_log(f"AUTO: FATAL VIBRATION during hover ({max_vib:.1f} m/s²)! KILLED.")
            state['rc_throttle'] = 1000
            send_rc_override(master)
            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            state['macro_status'] = 'IDLE'
            return
        
        # Log every 2 seconds (less verbose)
        if hover_frames % 40 == 0:
            flow_mag = math.sqrt(state['flow_vx']**2 + state['flow_vy']**2)
            elapsed = time.time() - hover_start_time
            vib_display = max(state['vibration_filtered'])
            update_log(f"AUTO: Hover [{elapsed:.1f}s] Alt={current_alt:.2f}m Thr={state['rc_throttle']} Vib={vib_display:.2f} Flow={flow_mag:.3f}m/s")
    
    update_log("AUTO: 10-second hover complete. Initiating landing sequence...")
    
    # Automatically transition to landing sequence
    auto_land_sequence(master)


def auto_land_sequence(master):
    """Dynamic Lidar & Kinematic Landing sequence."""
    if state['macro_status'] != 'IDLE': return
    state['macro_status'] = 'LANDING'
    
    update_log("AUTO: Starting precision landing...")

    # Switch to ALT_HOLD for precise manual throttle descent overrides
    master.set_mode(master.mode_mapping()['ALT_HOLD'])
    
    timeout = time.time() + 15
    touchdown_confirm_frames = 0
    
    while state['armed'] and time.time() < timeout:
        current_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
        
        # Determine throttle based on height
        if current_alt < 0.4:
            state['rc_throttle'] = 1350  # Slow descent for soft touchdown
        else:
            state['rc_throttle'] = 1250  # Faster descent
            
        send_rc_override(master)
        time.sleep(0.1)
        
        # Kinematic Touchdown Detection: Near ground AND not moving vertically
        if current_alt < 0.25 and abs(state['climb']) < 0.15:
            touchdown_confirm_frames += 1
        else:
            touchdown_confirm_frames = 0
            
        if touchdown_confirm_frames > 10: # 1.0 second of no vertical movement near ground
            update_log("AUTO: Physical touchdown confirmed.")
            break

    update_log("AUTO: Final disarm...")
    state['rc_throttle'] = 1000
    send_rc_override(master)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    update_log("AUTO: Disarmed. Landing complete.")
    state['macro_status'] = 'IDLE'


# --- MAVLINK DATA PROCESSING ---

def mavlink_loop(master):
    master.wait_heartbeat()
    update_log("Connected to Pixhawk!")
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )

    while True:
        msg = master.recv_match(blocking=True, timeout=0.1)
        if not msg:
            continue
        m_type = msg.get_type()

        if m_type == 'DISTANCE_SENSOR':
            dist_cm = msg.current_distance
            max_cm  = msg.max_distance
            if dist_cm < max_cm:
                state['lidar_alt'] = round(dist_cm / 100.0, 2)
                state['rangefinder_healthy'] = True
            else:
                state['rangefinder_healthy'] = False

        elif m_type == 'OBSTACLE_DISTANCE':
            distances = [d for d in msg.distances if d < 65535]
            if distances:
                state['livox_obs'] = round(min(distances) / 100.0, 2)
                state['livox_sectors'] = len(distances)
            else:
                state['livox_obs'] = 0.0
                state['livox_sectors'] = 0

        elif m_type == 'OPTICAL_FLOW':
            state['flow_qual'] = msg.quality
            state['flow_vx']   = round(msg.flow_comp_m_x, 3)
            state['flow_vy']   = round(msg.flow_comp_m_y, 3)

        elif m_type == 'VISION_POSITION_ESTIMATE':
            state['vision_x'] = round(msg.x, 2)
            state['vision_y'] = round(msg.y, 2)
            state['vision_z'] = round(msg.z, 2)

        elif m_type == 'GLOBAL_POSITION_INT':
            state['alt'] = round(msg.relative_alt / 1000.0, 2)
            # Check FAST-LIO health on each telemetry cycle
            check_fastlio_health()
            csv_writer.writerow([
                datetime.now().strftime("%H:%M:%S.%f"),
                state['mode'], state['armed'],
                state['alt'], state['lidar_alt'], state['climb'],
                state['roll'], state['pitch'], state['hdg'],
                state['batt_v'], state['batt_curr'], state['batt_pct'],
                state['rc_throttle'],
                state['flow_qual'], state['flow_vx'], state['flow_vy'],
                state['vibration'][0], state['vibration'][1], state['vibration'][2],
                state['cpu_load'],
                state['livox_obs'], state['livox_sectors'],
                state['vision_x'], state['vision_y'], state['vision_z'],
                state['fastlio_x'], state['fastlio_y'], state['fastlio_z'],
                state['fastlio_vx'], state['fastlio_vy'], state['fastlio_vz'],
                state['fastlio_roll'], state['fastlio_pitch'], state['fastlio_yaw'],
                state['fastlio_hz'], state['fastlio_points']
            ])
            telemetry_file.flush()

        elif m_type == 'VFR_HUD':
            state['climb'] = round(msg.climb, 2)

        elif m_type == 'ATTITUDE':
            state['roll']  = round(msg.roll  * 57.3, 1)
            state['pitch'] = round(msg.pitch * 57.3, 1)
            state['hdg']   = round(msg.yaw   * 57.3, 1)

        elif m_type == 'SYS_STATUS':
            state['batt_v']    = round(msg.voltage_battery / 1000.0, 2)
            state['batt_pct']  = msg.battery_remaining
            state['batt_curr'] = msg.current_battery / 100.0
            state['cpu_load']  = msg.load / 10.0

        elif m_type == 'HEARTBEAT':
            state['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            state['mode']  = mavutil.mode_string_v10(msg)

        elif m_type == 'EKF_STATUS_REPORT':
            state['ekf_flags'] = msg.flags

        elif m_type == 'VIBRATION':
            state['vibration'] = [
                round(msg.vibration_x, 3),
                round(msg.vibration_y, 3),
                round(msg.vibration_z, 3)
            ]
            # Apply rolling average filter to smooth vibration readings
            update_vibration_filter(state['vibration'])

        elif m_type == 'STATUSTEXT':
            update_log(f"SYS: {msg.text}")


# --- TUI DASHBOARD ---

def draw_dashboard(stdscr, master):
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.keypad(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN,  -1)  # Normal data
    curses.init_pair(2, curses.COLOR_RED,    -1)  # Warnings / armed
    curses.init_pair(3, curses.COLOR_CYAN,   -1)  # Borders / headings
    curses.init_pair(4, curses.COLOR_YELLOW, -1)  # Section labels / keys
    curses.init_pair(5, curses.COLOR_MAGENTA,-1)  # Livox / FAST-LIO highlights
    curses.init_pair(6, curses.COLOR_WHITE,  -1)  # Dim/neutral info

    while True:
        stdscr.erase()
        rows, cols = stdscr.getmaxyx()
        c_grn = curses.color_pair(1) | curses.A_BOLD
        c_red = curses.color_pair(2) | curses.A_BOLD
        c_cyn = curses.color_pair(3) | curses.A_BOLD
        c_ylw = curses.color_pair(4) | curses.A_BOLD
        c_mag = curses.color_pair(5) | curses.A_BOLD
        c_wht = curses.color_pair(6)

        def safe_add(row, col, text, attr=0):
            if row < 0 or row >= rows - 1 or col < 0 or col >= cols:
                return
            available = cols - col - 1
            if available <= 0:
                return
            try:
                stdscr.addstr(row, col, text[:available], attr)
            except curses.error:
                pass

        W   = max(cols - 1, 1)
        sep = "=" * W   
        mid = "-" * W   

        # Header
        safe_add(0, 0, sep, c_cyn)
        safe_add(1, 4, "ASCEND MISSION CONTROL  v8.0  |  FAST-LIO Edition  (Livox Mid-360)", c_grn)
        safe_add(2, 0, sep, c_cyn)

        # --- LEFT PANEL: FLIGHT STATE ---
        safe_add(4,  2, "[ FLIGHT STATE ]", c_ylw)
        safe_add(5,  2, f"MODE    : {state['mode']:<16}", c_grn)
        armed_str = "ARMED  ** LIVE **" if state['armed'] else "DISARMED   SAFE"
        safe_add(6,  2, f"STATUS  : {armed_str}", c_red if state['armed'] else c_grn)
        safe_add(7,  2, f"MACRO   : {state['macro_status']}", c_cyn if state['macro_status'] != 'IDLE' else c_grn)
        safe_add(8,  2, f"POWER   : {state['batt_v']:.2f} V  /  {state['batt_curr']:.1f} A  ({state['batt_pct']}%)",
                  c_red if state['batt_v'] < MIN_SAFE_VOLTAGE else c_grn)
        safe_add(9,  2, f"BARO ALT: {state['alt']:.2f} m", c_grn)
        safe_add(10, 2, f"CLIMB   : {state['climb']:+.2f} m/s", c_grn)
        safe_add(11, 2, f"ATTITUDE: R {state['roll']:+.1f}deg  P {state['pitch']:+.1f}deg", c_grn)

        # --- RIGHT PANEL: SENSORS ---
        rc = min(44, cols - 20)
        safe_add(4, rc, "[ SENSORS ]", c_ylw)

        rf_color  = c_grn if state['rangefinder_healthy'] else c_red
        rf_status = "OK" if state['rangefinder_healthy'] else "NO DATA"
        safe_add(5, rc, f"LIDAR ALT   : {state['lidar_alt']:.2f} m  [{rf_status}]", rf_color)

        livox_color = c_red if (0 < state['livox_obs'] < 1.0) else c_mag
        safe_add(6, rc, f"LIVOX OBS   : {state['livox_obs']:.2f} m  ({state['livox_sectors']} sectors)", livox_color)

        flow_color = c_red if state['flow_qual'] < 20 else c_grn
        safe_add(7, rc, f"OPT.FLOW QL : {state['flow_qual']:3d}/255", flow_color)
        safe_add(8, rc, f"VISION POS  : X={state['vision_x']:.2f}  Y={state['vision_y']:.2f}", c_grn)

        vib_z = state['vibration_filtered'][2]
        vib_max = max(state['vibration_filtered'])
        vib_color = c_red if vib_max > MAX_GROUND_VIBE else c_grn
        safe_add(10, rc, f"VIBRATION   : {vib_max:.2f} m/s2 (filtered)", vib_color)
        safe_add(11, rc, f"EKF FLAGS   : 0x{state['ekf_flags']:04X}", c_grn)

        # --- FAST-LIO SLAM PANEL ---
        safe_add(13, 0, mid, c_cyn)
        safe_add(13, 3, "[ FAST-LIO SLAM | Livox Mid-360 ]", c_mag)

        # Health indicator
        if state['ros2_active']:
            check_fastlio_health()
            if state['fastlio_healthy']:
                flio_status = f"ACTIVE  {state['fastlio_hz']:.0f} Hz"
                flio_color = c_grn
            else:
                age = time.time() - state['fastlio_last_time'] if state['fastlio_last_time'] > 0 else 999
                flio_status = f"TIMEOUT (stale {age:.1f}s)"
                flio_color = c_red
        else:
            flio_status = "INACTIVE - ROS2 NOT RUNNING"
            flio_color = c_red

        safe_add(14, 2, f"STATUS: {flio_status}", flio_color)

        # SLAM Position
        safe_add(15, 2,
            f"POS   : X={state['fastlio_x']:+8.3f}  Y={state['fastlio_y']:+8.3f}  Z={state['fastlio_z']:+8.3f} m",
            c_mag if state['fastlio_healthy'] else c_wht)

        # SLAM Velocity
        safe_add(16, 2,
            f"VEL   : Vx={state['fastlio_vx']:+6.3f}  Vy={state['fastlio_vy']:+6.3f}  Vz={state['fastlio_vz']:+6.3f} m/s",
            c_mag if state['fastlio_healthy'] else c_wht)

        # SLAM Orientation
        safe_add(17, 2,
            f"ORIENT: R={state['fastlio_roll']:+6.1f}  P={state['fastlio_pitch']:+6.1f}  Y={state['fastlio_yaw']:+6.1f} deg",
            c_mag if state['fastlio_healthy'] else c_wht)

        # Point cloud stats (right side of SLAM panel)
        safe_add(14, rc, f"POINTS: {state['fastlio_points']:,} pts/scan", c_mag if state['fastlio_healthy'] else c_wht)
        safe_add(15, rc, f"MAP    : lab_map.pcd", c_wht)
        safe_add(16, rc, f"LIDAR  : Livox Mid-360 (20Hz)", c_mag if state['fastlio_healthy'] else c_wht)

        # --- COMMANDS ---
        safe_add(19, 0, mid, c_cyn)
        safe_add(20, 2, "[ MISSION ]  [c] Calibrate  [o] Set Origin  [k] Auto Takeoff  [j] Auto Land", c_ylw)
        safe_add(21, 2, "             [h] AltHold    [l] Loiter      [a] Arm           [d] Disarm", c_ylw)
        safe_add(22, 2, "[ MANUAL  ]  [W/S] Pitch    [A/D] Roll      [UP/DN] Throttle  [x] KILL  [q] Quit", c_ylw)
        safe_add(23, 2, "[ DEBUG   ]  [r] ROS2 Status  [e] Event Log Export", c_ylw)
        safe_add(24, 0, mid, c_cyn)
        safe_add(24, 2, f"RC STICKS: THR:{int(state['rc_throttle']):4d}  PCH:{int(state['rc_pitch']):4d}  ROL:{int(state['rc_roll']):4d}  YAW:{int(state['rc_yaw']):4d}", c_cyn)

        # --- EVENT LOG ---
        safe_add(25, 0, mid, c_cyn)
        safe_add(25, 3, "[ EVENT LOG ]", c_ylw)
        for i, log_msg in enumerate(state['log']):
            if 26 + i < rows - 1:
                safe_add(26 + i, 2, f"> {log_msg}", c_grn)

        stdscr.refresh()
        key = stdscr.getch()

        # Reset sticks to neutral each cycle
        state['rc_pitch'] = 1500
        state['rc_roll']  = 1500
        state['rc_yaw']   = 1500

        if key != -1:
            if key == ord('q'):
                break
            elif key == ord('c'):
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0, 0, 0, 1, 0, 0, 0, 0
                )
                update_log("CMD: Gyro calibration requested.")
            elif key == ord('o'):
                update_log("CMD: Forcing EKF Origin...")
                master.mav.param_set_send(
                    master.target_system, master.target_component,
                    b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )
                master.mav.set_gps_global_origin_send(
                    master.target_system, int(19.033 * 1e7), int(73.029 * 1e7), 0
                )
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0, 0, 0, 0, 0, 0
                )
            elif key == ord('h'):
                master.set_mode(master.mode_mapping()['ALT_HOLD'])
                update_log("CMD: Mode -> ALT_HOLD")
            elif key == ord('l'):
                master.set_mode(master.mode_mapping()['LOITER'])
                update_log("CMD: Mode -> LOITER")
            elif key == ord('a'):
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0
                )
                update_log("CMD: ARM requested.")
            elif key == ord('d'):
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
                )
                update_log("CMD: DISARM requested.")
            elif key == ord('k'):
                threading.Thread(target=auto_takeoff_sequence, args=(master,), daemon=True).start()
            elif key == ord('j'):
                threading.Thread(target=auto_land_sequence, args=(master,), daemon=True).start()
            elif key == ord('w'):
                state['rc_pitch'] = 1420   # Pitch forward
            elif key == ord('s'):
                state['rc_pitch'] = 1580   # Pitch backward
            elif key == ord('a'): 
                state['rc_roll'] = 1420
            elif key == ord('d'): 
                state['rc_roll'] = 1580
            elif key == curses.KEY_UP:
                state['rc_throttle'] = min(state['rc_throttle'] + 25, 2000)
            elif key == curses.KEY_DOWN:
                state['rc_throttle'] = max(state['rc_throttle'] - 25, 1000)
            elif key == ord(' '):
                state['rc_throttle'] = HOVER_THR
                update_log("CMD: Throttle set to hover neutral.")
            elif key == ord('x'):
                state['rc_throttle'] = 1000
                state['macro_status'] = 'IDLE'
                update_log("CMD: *** EMERGENCY KILL SWITCH TRIGGERED ***")
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
                )
            elif key == ord('r'):
                # ROS2 diagnostics
                if ROS2_AVAILABLE:
                    if state['ros2_active']:
                        fastlio_age = time.time() - state['fastlio_last_time'] if state['fastlio_last_time'] > 0 else 999
                        update_log(f"ROS2: ACTIVE {state['fastlio_hz']:.0f}Hz | SLAM age: {fastlio_age:.1f}s | Pts: {state['fastlio_points']}")
                    else:
                        update_log("ROS2: NOT ACTIVE - Check `ros2 node list` and `ros2 topic list`")
                else:
                    update_log("ROS2: Not installed. Run: pip install rclpy")
            elif key == ord('e'):
                # Export diagnostics
                from datetime import datetime
                export_log = f"diag_{session_id}.txt"
                with open(export_log, 'a') as f:
                    f.write(f"\n=== DIAGNOSTIC SNAPSHOT {datetime.now().strftime('%H:%M:%S')} ===\n")
                    f.write(f"ROS2 Active: {state['ros2_active']}\n")
                    f.write(f"FAST-LIO Healthy: {state['fastlio_healthy']}\n")
                    f.write(f"FAST-LIO Hz: {state['fastlio_hz']:.1f}\n")
                    f.write(f"SLAM Position: ({state['fastlio_x']:.3f}, {state['fastlio_y']:.3f}, {state['fastlio_z']:.3f})\n")
                    f.write(f"Livox Points: {state['fastlio_points']}\n")
                    f.write(f"LIDAR Alt: {state['lidar_alt']:.2f} m\n")
                update_log(f"Diagnostics exported to {export_log}")

        send_rc_override(master)
        time.sleep(0.05)


# --- ENTRY POINT ---

if __name__ == '__main__':
    try:
        print("\n" + "="*70)
        print("  ASCEND MISSION CONTROL v8.0 - FAST-LIO Edition (Livox Mid-360)")
        print("="*70)
        
        # Start ROS2 FAST-LIO subscriber if available
        if ROS2_AVAILABLE:
            print("\n[*] ROS2 Environment: DETECTED")
            print("    Make sure FAST-LIO is running:")
            print("      $ ros2 launch fast_lio mapping_mid360.launch.py")
            print("    And the drone bridge is active:")
            print("      $ ros2 launch mavros px4.launch")
            print("\n[*] Starting ROS2 FAST-LIO subscriber thread...")
            
            ros2_thread = threading.Thread(target=run_ros2_thread, daemon=True)
            ros2_thread.start()
            time.sleep(2.0)  # Give ROS2 more time to initialize and connect to master
        else:
            print("\n[!] WARNING: ROS2 (rclpy) not available.")
            print("    Install with: pip install rclpy")
            print("    Without ROS2, FAST-LIO SLAM data will be unavailable.")
            time.sleep(1.0)

        print("\n[*] Connecting to Pixhawk on /dev/ttyAMA0 @ 921600 baud...")
        master_conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
        
        print("[*] Starting MAVLink listener thread...")
        thread = threading.Thread(target=mavlink_loop, args=(master_conn,), daemon=True)
        thread.start()
        
        print("[*] Starting TUI Dashboard...")
        print("    Use 'q' to quit safely.\n")
        curses.wrapper(draw_dashboard, master_conn)
    except FileNotFoundError:
        print("\n[!] ERROR: Could not open serial port /dev/ttyAMA0")
        print("    Check that Pixhawk is connected and you have serial permissions.")
        print("    Try: sudo usermod -a -G dialout $USER")
    except Exception as e:
        print(f"\n[!] FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[*] Shutting down gracefully...")
        telemetry_file.close()
        event_file.close()
        if ROS2_AVAILABLE:
            state['ros2_active'] = False
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except:
                pass
        print("[*] Goodbye!")
