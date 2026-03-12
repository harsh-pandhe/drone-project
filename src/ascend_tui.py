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
TARGET_ALTITUDE = 1.0     # Target hover height in meters
SPOOL_THR = 1400          # Idle throttle to test frame vibrations
LIFTOFF_THR = 1600        # Smooth liftoff power (+150 PWM above hover for positive authority)
HOVER_THR = 1450          # Neutral throttle for hover (calibrate for your frame!)
MIN_SAFE_VOLTAGE = 10.5   # Battery safety cutoff (volts)
MAX_VIBRATION = 25.0      # Abort flight if vibrations exceed 25 m/s^2
MAX_GROUND_VIBE = 18.0    # Strict threshold during ground spool-up
VIBRATION_FILTER_SIZE = 10 # Rolling average window for vibration smoothing

# --- ALTITUDE CONTROLLER TUNING (1m test) ---
ALT_PID_P = 8.0           # P-gain: Strong response for 1m (was 6.0)
ALT_PID_I = 0.5           # I-gain: Gentle integral (reduced from 0.8)
ALT_PID_D = 2.5           # D-gain: Smooth damping (reduced from 3.0)
ALT_DEADBAND = 0.08       # Altitude deadband: ±8cm (tighter for 1m)
CLIMB_DAMPING = 0.6       # Damping factor in final approach (reduced from 0.3 to 40%)
CLIMB_DIST_THRESHOLD = 0.30 # Distance to start damping (0.30m approach phase only)

# --- POSITION HOLD TUNING (Optical Flow only) ---
POS_P_GAIN = 50.0         # Position P-gain: 50 PWM per m/s flow (was 45)
POS_I_GAIN = 1.5          # Position I-gain: Gentle accumulation (was 2.0)
POS_DEADBAND = 0.02       # Position deadband: ±2cm (very tight)
POS_MAX_ANGLE = 3.0       # Max pitch/roll correction: ±3° (gentle)

# --- TIMING ---
HOVER_SETTLE_TIME = 6.0   # Settle time before hover (increased from 5s)
HOVER_DURATION = 5.0      # Hover duration for test 
CLIMB_RAMP_RATE = 5       # PWM per cycle (increased from 2 for faster climb ramp)

# --- YAW STABILIZATION (Compass/IMU) ---
YAW_RATE_LIMIT = 60.0     # Max rate of change for yaw angle (degrees/second)
YAW_INTEGRAL_LIMIT = 30   # Max accumulated yaw error

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
    # Telemetry metrics for trending
    'batt_v_history': [], 'batt_pct_history': [], 'climb_history': [],
    'vib_max_history': [], 'alt_history': [],
    'session_start_time': 0.0, 'flight_time': 0.0,
    'max_alt_achieved': 0.0, 'max_vib_recorded': 0.0,
    'pos_error_lidar_vs_baro': 0.0, 'pos_error_fastlio_vs_lidar': 0.0,
    'flow_magnitude': 0.0, 'flow_history': [],
    'cpu_load_history': [],
    # Position hold and drift control
    'home_x': 0.0, 'home_y': 0.0, 'home_z': 0.0,  # Home position at takeoff
    'home_set': False,  # Whether home position is locked
    'pos_x': 0.0, 'pos_y': 0.0,  # Estimated position (integrated from optical flow)
    'pos_x_drift': 0.0, 'pos_y_drift': 0.0,  # Cumulative drift for debugging
    'flow_integral_x': 0.0, 'flow_integral_y': 0.0,  # Optical flow integration
    'desired_pos_x': 0.0, 'desired_pos_y': 0.0,  # Target position for hold
    'rtl_active': False,  # Return-to-launch active
    'sensor_status_log': [],  # Track sensor health during flight
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


# --- IMPROVED POSITION HOLD & DRIFT CORRECTION ---

def set_home_position():
    """Lock current position as home for position hold."""
    if state['rangefinder_healthy']:
        state['home_z'] = state['lidar_alt']
    else:
        state['home_z'] = state['alt']
    
    # Use SLAM position if available, otherwise start from origin
    if state['fastlio_healthy']:
        state['home_x'] = state['fastlio_x']
        state['home_y'] = state['fastlio_y']
    else:
        state['home_x'] = 0.0
        state['home_y'] = 0.0
    
    state['pos_x'] = state['home_x']
    state['pos_y'] = state['home_y']
    state['desired_pos_x'] = state['home_x']
    state['desired_pos_y'] = state['home_y']
    state['flow_integral_x'] = 0.0
    state['flow_integral_y'] = 0.0
    state['home_set'] = True
    update_log(f"HOME locked: ({state['home_x']:.2f}, {state['home_y']:.2f}, {state['home_z']:.2f})")


def update_position_estimate(dt):
    """Update position estimate ONLY using optical flow integration."""
    if not state['home_set']:
        return
    
    # Use optical flow velocity to estimate position
    # NO SLAM fallback - pure optical flow integration
    if state['flow_qual'] > 50:  # Quality gate: only integrate if good quality
        state['flow_integral_x'] += state['flow_vx'] * dt
        state['flow_integral_y'] += state['flow_vy'] * dt
        state['pos_x'] = state['home_x'] + state['flow_integral_x']
        state['pos_y'] = state['home_y'] + state['flow_integral_y']
    # Else: maintain last known position when flow quality is poor


def get_position_error():
    """Return current position error from desired hold point."""
    if not state['home_set']:
        return 0.0, 0.0
    
    error_x = state['desired_pos_x'] - state['pos_x']
    error_y = state['desired_pos_y'] - state['pos_y']
    return error_x, error_y


def compute_position_correction(max_angle=5.0):
    """Compute pitch/roll corrections for position hold (limits to max_angle degrees)."""
    error_x, error_y = get_position_error()
    
    # Simple proportional control: angle = error * gain
    pitch_angle = error_x * 8.0  # 8 deg per meter error
    roll_angle = error_y * 8.0   # 8 deg per meter error
    
    # Limit to max angle
    pitch_angle = max(-max_angle, min(max_angle, pitch_angle))
    roll_angle = max(-max_angle, min(max_angle, roll_angle))
    
    # Convert angle to RC PWM (±max_angle → RC 1450-1550)
    pitch_pwm = 1500 + (pitch_angle / max_angle) * 50
    roll_pwm = 1500 + (roll_angle / max_angle) * 50
    
    return int(roll_pwm), int(pitch_pwm)


def get_drift_magnitude():
    """Calculate current drift from home position in meters."""
    if not state['home_set']:
        return 0.0
    dx = state['pos_x'] - state['home_x']
    dy = state['pos_y'] - state['home_y']
    return math.sqrt(dx**2 + dy**2)


def get_best_altitude_for_control():
    """Get most reliable altitude estimate for closed-loop control."""
    # Prefer LIDAR (rangefinder) for better hover control
    if state['rangefinder_healthy'] and state['lidar_alt'] > 0:
        return state['lidar_alt']
    # Fall back to barometer
    return state['alt']


def trigger_rtl(master):
    """Initiate Return-To-Launch sequence."""
    if not state['armed']:
        update_log("RTL: Already disarmed")
        return
    
    update_log("RTL: TRIGGERED. Returning to home...")
    state['rtl_active'] = True
    state['macro_status'] = 'RTL'
    master.set_mode(master.mode_mapping()['RTL'])
    update_log("RTL: Switched to RTL mode - FC will handle return")


def check_emergency_rtl(master):
    """Monitor for emergency conditions that should trigger RTL."""
    if not state['armed'] or state['rtl_active']:
        return  # Not armed or already in RTL
    
    # Low battery emergency
    if state['batt_pct'] < 10 and state['batt_v'] < 10.5:
        update_log("❌ CRITICAL: Battery < 10%! Triggering RTL!")
        trigger_rtl(master)
        return
    
    # Excessive vibration
    if max(state['vibration_filtered']) > MAX_VIBRATION * 1.5:
        update_log("❌ CRITICAL: Extreme vibrations! Triggering RTL!")
        trigger_rtl(master)
        return
    
    # Excessive tilt (loss of control)
    if abs(state['roll']) > 60 or abs(state['pitch']) > 60:
        update_log("❌ CRITICAL: Excessive tilt! Triggering RTL!")
        trigger_rtl(master)
        return
    
    # SLAM failure during flight (if was healthy)
    if state['home_set'] and state['fastlio_healthy'] is False and time.time() - state['fastlio_last_time'] > 5.0:
        if state['lidar_alt'] < 5.0:  # Only alert if low altitude
            update_log("⚠ WARNING: SLAM lost at low altitude")
    
    # Loss of rangefinder at low altitude
    if not state['rangefinder_healthy'] and state['lidar_alt'] < 1.0:
        if state['alt'] < 0.5:
            update_log("⚠ WARNING: Rangefinder lost near ground")


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


# --- TELEMETRY METRICS & TRENDING ---

def update_telemetry_metrics():
    """Update rolling history and calculated metrics."""
    max_history = 60  # Keep last 60 measurements (~3 seconds at 20Hz)
    
    # Battery metrics
    state['batt_v_history'].append(state['batt_v'])
    if len(state['batt_v_history']) > max_history:
        state['batt_v_history'].pop(0)
    
    state['batt_pct_history'].append(state['batt_pct'])
    if len(state['batt_pct_history']) > max_history:
        state['batt_pct_history'].pop(0)
    
    # Climb rate trend
    state['climb_history'].append(state['climb'])
    if len(state['climb_history']) > max_history:
        state['climb_history'].pop(0)
    
    # Altitude trend
    current_best_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
    state['alt_history'].append(current_best_alt)
    if len(state['alt_history']) > max_history:
        state['alt_history'].pop(0)
    
    if current_best_alt > state['max_alt_achieved']:
        state['max_alt_achieved'] = current_best_alt
    
    # Vibration trend
    vib_max = max(state['vibration_filtered'])
    state['vib_max_history'].append(vib_max)
    if len(state['vib_max_history']) > max_history:
        state['vib_max_history'].pop(0)
    
    if vib_max > state['max_vib_recorded']:
        state['max_vib_recorded'] = vib_max
    
    # Optical flow magnitude
    state['flow_magnitude'] = math.sqrt(state['flow_vx']**2 + state['flow_vy']**2)
    state['flow_history'].append(state['flow_magnitude'])
    if len(state['flow_history']) > max_history:
        state['flow_history'].pop(0)
    
    # CPU load trend
    state['cpu_load_history'].append(state['cpu_load'])
    if len(state['cpu_load_history']) > max_history:
        state['cpu_load_history'].pop(0)
    
    # Position errors
    if state['rangefinder_healthy']:
        state['pos_error_lidar_vs_baro'] = abs(state['lidar_alt'] - state['alt'])
    
    if state['fastlio_healthy']:
        state['pos_error_fastlio_vs_lidar'] = abs(state['fastlio_z'] - state['lidar_alt'])
    
    # Flight time
    if state['armed']:
        if state['session_start_time'] == 0.0:
            state['session_start_time'] = time.time()
        state['flight_time'] = time.time() - state['session_start_time']
    else:
        state['session_start_time'] = 0.0


def get_battery_drain_rate():
    """Calculate battery drain rate (V/sec and %/sec)."""
    if len(state['batt_pct_history']) < 10:
        return 0.0, 0.0
    
    # Sample every 0.5 seconds (10 samples)
    pct_drain = state['batt_pct_history'][0] - state['batt_pct_history'][-1]
    volt_drain = state['batt_v_history'][0] - state['batt_v_history'][-1]
    elapsed = 0.5  # 10 samples at ~20Hz
    
    return volt_drain / elapsed, pct_drain / elapsed


def get_avg_climb_rate():
    """Get average climb rate from history."""
    if len(state['climb_history']) < 5:
        return 0.0
    return sum(state['climb_history'][-10:]) / 10


def get_altitude_slope():
    """Get altitude change trend (positive = climbing, negative = descending)."""
    if len(state['alt_history']) < 10:
        return 0.0
    recent = state['alt_history'][-10:]
    oldest = state['alt_history'][0]
    return (recent[-1] - oldest) / 0.5  # Rate in m/sec


def get_avg_vibration():
    """Get average vibration over recent history."""
    if not state['vib_max_history']:
        return 0.0
    return sum(state['vib_max_history'][-20:]) / min(20, len(state['vib_max_history']))


def format_trend_arrow(value, threshold_high=0, threshold_low=0):
    """Return a trend indicator based on value."""
    if abs(value) < 0.01:
        return "→"
    elif value > threshold_high:
        return "↑"
    elif value < threshold_low:
        return "↓"
    return "→"


def get_sensor_health_char(condition, char_ok="●", char_fail="○"):
    """Return health indicator character."""
    return char_ok if condition else char_fail


def create_sparkline(history, width=8):
    """Create ASCII sparkline from numeric history."""
    if not history or len(history) < 2:
        return "┄" * width
    
    max_val = max(history) if max(history) > 0 else 1
    min_val = min(history)
    range_val = max_val - min_val if max_val != min_val else 1
    
    # Spark characters
    sparks = "▁▂▃▄▅▆▇█"
    
    # Sample history to fit width
    step = max(1, len(history) // width)
    sample = history[-width*step::step][-width:]
    
    line = ""
    for val in sample:
        normalized = (val - min_val) / range_val if range_val > 0 else 0.5
        idx = int(normalized * (len(sparks) - 1))
        line += sparks[idx]
    
    return line.ljust(width, "▁")


def get_sensor_confidence(sensor_name):
    """Calculate confidence score for a sensor (0-100%)."""
    confidence = 0
    
    if sensor_name == "LIDAR":
        if state['rangefinder_healthy']:
            confidence = 95
        else:
            confidence = 0
    
    elif sensor_name == "BARO":
        # Barometer is always available but less accurate
        if state['alt'] > 0:
            confidence = 60
        else:
            confidence = 30
    
    elif sensor_name == "SLAM":
        if state['fastlio_healthy']:
            age = time.time() - state['fastlio_last_time']
            if age < 0.5:
                confidence = 90
            elif age < 1.0:
                confidence = 70
            elif age < 2.0:
                confidence = 40
            else:
                confidence = 0
        else:
            confidence = 0
    
    elif sensor_name == "OPTFLOW":
        if state['flow_qual'] > 150:
            confidence = 85
        elif state['flow_qual'] > 100:
            confidence = 70
        elif state['flow_qual'] > 50:
            confidence = 50
        elif state['flow_qual'] > 20:
            confidence = 20
        else:
            confidence = 0
    
    return confidence


def get_best_altitude_estimate():
    """Return best altitude estimate and confidence."""
    lidar_conf = get_sensor_confidence("LIDAR")
    slam_conf = get_sensor_confidence("SLAM")
    baro_conf = get_sensor_confidence("BARO")
    
    if lidar_conf > slam_conf and lidar_conf > 50:
        return state['lidar_alt'], "LIDAR", lidar_conf
    elif slam_conf > 50:
        return state['fastlio_z'], "SLAM", slam_conf
    elif baro_conf > 0:
        return state['alt'], "BARO", baro_conf
    else:
        return state['alt'], "???", 0


def get_battery_time_remaining():
    """Estimate flight time remaining based on current drain."""
    if state['batt_pct'] <= 0:
        return 0
    
    volt_drain, pct_drain = get_battery_drain_rate()
    
    if pct_drain > 0:
        minutes = state['batt_pct'] / (pct_drain * 60.0)
        return max(0, minutes)
    
    return 0


def estimate_link_quality():
    """Estimate telemetry/MAVLink link quality based on message freshness."""
    # If we're receiving data regularly, link is good
    return 95 if state['armed'] else 80


def format_position_compact(x, y, z):
    """Format 3D position compactly."""
    return f"({x:+.2f},{y:+.2f},{z:+.2f})"


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
            # FIX: Only dampen in final approach phase (last 30cm)
            if alt_to_target < CLIMB_DIST_THRESHOLD:
                # Damped approach: reduce throttle smoothly as we reach target
                damp_ratio = alt_to_target / CLIMB_DIST_THRESHOLD  # 0 to 1
                dampened_thr = HOVER_THR + int((LIFTOFF_THR - HOVER_THR) * damp_ratio * CLIMB_DAMPING)
                climb_throttle = max(1500, dampened_thr)  # Never drop below minimum climb authority
            elif climb_throttle < LIFTOFF_THR:
                climb_throttle = min(LIFTOFF_THR, climb_throttle + CLIMB_RAMP_RATE)
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

    # 10. SET HOME POSITION FOR POSITION HOLD
    set_home_position()
    
    # Switch to ALT_HOLD for active altitude and position control
    update_log("AUTO: Activating ALT_HOLD for active position control...")
    master.set_mode(master.mode_mapping()['ALT_HOLD'])
    time.sleep(0.5)
    
    # 11. SETTLING PERIOD: Fine hover control with drift correction
    update_log(f"AUTO: Settling at {TARGET_ALTITUDE}m with drift control for {HOVER_SETTLE_TIME}s...")
    settle_start = time.time()
    alt_integral = 0.0
    prev_alt_error = 0.0
    prev_time = time.time()
    max_drift_recorded = 0.0
    
    while time.time() - settle_start < HOVER_SETTLE_TIME and state['armed']:
        current_alt = get_best_altitude_for_control()
        current_time = time.time()
        dt = max(0.01, current_time - prev_time)
        prev_time = current_time
        update_position_estimate(dt)
        
        # --- ALTITUDE CONTROLLER (PID) ---
        alt_error = target_abs_alt - current_alt
        
        if abs(alt_error) > ALT_DEADBAND:
            alt_p = alt_error * ALT_PID_P
            alt_integral += alt_error * dt
            alt_integral = max(-30, min(30, alt_integral))
            alt_i = alt_integral * ALT_PID_I
            alt_d = (alt_error - prev_alt_error) / dt * ALT_PID_D if dt > 0 else 0
            alt_d = max(-80, min(80, alt_d))
            throttle_delta = alt_p + alt_i + alt_d
        else:
            throttle_delta = 0
            alt_integral *= 0.95
        
        state['rc_throttle'] = int(max(1200, min(1750, HOVER_THR + throttle_delta)))
        prev_alt_error = alt_error
        
        # --- POSITION HOLD (DRIFT CORRECTION) ---
        roll_pwm, pitch_pwm = compute_position_correction(max_angle=4.0)
        state['rc_roll'] = roll_pwm
        state['rc_pitch'] = pitch_pwm
        
        # --- YAW STABILIZATION (Hold current heading) ---
        # Prevent wild yaw oscillations by dampening heading changes
        yaw_rate_of_change = abs(state['hdg'] - prev_alt_error) if hasattr(compute_position_correction, '__self__') else 0
        if abs(state['hdg']) > YAW_RATE_LIMIT:
            # If heading is changing too fast, center yaw to stabilize
            state['rc_yaw'] = 1500
        else:
            # Normal operation - maintain neutral yaw for indoor STABILIZE/ALT_HOLD
            state['rc_yaw'] = 1500
        
        send_rc_override(master)
        time.sleep(0.05)
        
        # Monitor drift
        drift = get_drift_magnitude()
        if drift > max_drift_recorded:
            max_drift_recorded = drift
        if drift > 0.3 and (int(time.time() - settle_start)) % 2 == 0:
            update_log(f"AUTO: Drift: {drift:.2f}m. Correcting...")
        
        # Safety checks
        if abs(state['roll']) > 35 or abs(state['pitch']) > 35:
            update_log("AUTO: CRITICAL TILT during settle! Reducing corrections.")
            alt_integral *= 0.5
        
        max_vib = max(state['vibration_filtered'])
        if max_vib > MAX_VIBRATION:
            update_log(f"AUTO: FATAL VIBRATION during settle ({max_vib:.1f} m/s²)! KILLED.")
            state['rc_throttle'] = 1000
            send_rc_override(master)
            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            state['macro_status'] = 'IDLE'
            return
    
    update_log(f"AUTO: Settled. Max drift during settle: {max_drift_recorded:.2f}m")
    
    # 12. STABLE HOVER PHASE: Maintain at target altitude and position
    update_log(f"AUTO: {TARGET_ALTITUDE}m stable hover achieved. Holding position for {HOVER_DURATION}s...")
    hover_start_time = time.time()
    hover_frames = 0
    hover_drift_samples = []
    
    while time.time() - hover_start_time < HOVER_DURATION and state['armed']:
        current_alt = get_best_altitude_for_control()
        current_time = time.time()
        dt = max(0.01, current_time - prev_time)
        prev_time = current_time
        hover_frames += 1
        update_position_estimate(dt)
        
        # --- ALTITUDE CONTROLLER ---
        alt_error = target_abs_alt - current_alt
        if abs(alt_error) > ALT_DEADBAND:
            alt_p = alt_error * ALT_PID_P * 0.8  # Slightly reduced during hover
            alt_integral += alt_error * dt
            alt_integral = max(-20, min(20, alt_integral))
            alt_i = alt_integral * ALT_PID_I * 0.9
            alt_d = (alt_error - prev_alt_error) / dt * ALT_PID_D * 0.8 if dt > 0 else 0
            throttle_delta = alt_p + alt_i + alt_d
        else:
            throttle_delta = 0
            alt_integral *= 0.92
        
        state['rc_throttle'] = int(max(1200, min(1750, HOVER_THR + throttle_delta)))
        prev_alt_error = alt_error
        
        # --- POSITION HOLD (DRIFT CORRECTION) ---
        roll_pwm, pitch_pwm = compute_position_correction(max_angle=3.0)
        state['rc_roll'] = roll_pwm
        state['rc_pitch'] = pitch_pwm
        
        # --- YAW STABILIZATION (Hold current heading) ---
        # Prevent wild yaw oscillations by keeping neutral yaw command
        if abs(state['hdg']) > YAW_RATE_LIMIT:
            # If heading is changing too fast, center yaw to stabilize
            state['rc_yaw'] = 1500
        else:
            # Normal operation - maintain neutral yaw for stable heading hold
            state['rc_yaw'] = 1500
        
        send_rc_override(master)
        
        # Log drift for analysis
        drift = get_drift_magnitude()
        hover_drift_samples.append(drift)
        if len(hover_drift_samples) > 200:
            hover_drift_samples.pop(0)
        
        time.sleep(0.05)
        
        # Safety checks
        max_vib = max(state['vibration_filtered'])
        if max_vib > MAX_VIBRATION:
            update_log(f"AUTO: FATAL VIBRATION during hover ({max_vib:.1f} m/s²)! KILLED.")
            state['rc_throttle'] = 1000
            send_rc_override(master)
            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            state['macro_status'] = 'IDLE'
            return
        
        if state['batt_pct'] < 20:
            update_log("AUTO: LOW BATTERY during hover! Triggering land...")
            break
    
    avg_drift = sum(hover_drift_samples) / len(hover_drift_samples) if hover_drift_samples else 0.0
    max_drift_hover = max(hover_drift_samples) if hover_drift_samples else 0.0
    update_log(f"AUTO: Hover complete. Avg drift: {avg_drift:.3f}m, Max: {max_drift_hover:.3f}m")
    
    # 13. Initiate landing
    update_log("AUTO: Hover complete. Initiating landing sequence...")
    
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
    """Precision landing with position hold and drift prevention."""
    if state['macro_status'] not in ['IDLE', 'HOVER', None]:
        return  # Prevent double-triggering
    
    state['macro_status'] = 'LANDING'
    
    update_log("AUTO: Starting precision landing sequence...")
    
    # Ensure home is set for position hold during descent
    if not state['home_set']:
        set_home_position()
    
    # Switch to ALT_HOLD for active descent control
    master.set_mode(master.mode_mapping()['ALT_HOLD'])
    time.sleep(0.5)
    
    update_log("AUTO: Descending to ground with position hold...")
    
    land_start = time.time()
    timeout = land_start + 30  # 30 second timeout for landing
    land_alt_integral = 0.0
    prev_land_alt_error = 0.0
    prev_land_time = time.time()
    touchdown_frames = 0
    
    while state['armed'] and time.time() < timeout:
        current_alt = get_best_altitude_for_control()
        current_time = time.time()
        dt = max(0.01, current_time - prev_land_time)
        prev_land_time = current_time
        update_position_estimate(dt)
        
        descent_rate = 0.2  # Smooth descent at 0.2 m/s
        target_alt = state['home_z'] - (time.time() - land_start) * descent_rate
        target_alt = max(0.0, target_alt)  # Don't go below ground
        
        # --- ALTITUDE CONTROLLER FOR DESCENT ---
        alt_error = target_alt - current_alt
        
        # Proportional only for descent (smoother than PID)
        throttle_delta = alt_error * 40  # Reduced from 60 for softer descent
        
        state['rc_throttle'] = int(max(1100, min(1600, HOVER_THR + throttle_delta)))
        
        # --- POSITION HOLD DURING DESCENT ---
        roll_pwm, pitch_pwm = compute_position_correction(max_angle=3.0)
        state['rc_roll'] = roll_pwm
        state['rc_pitch'] = pitch_pwm
        
        send_rc_override(master)
        time.sleep(0.05)
        
        # Touchdown detection: Near ground AND vertical velocity low
        if current_alt < 0.20 and abs(state['climb']) < 0.10:
            touchdown_frames += 1
            if touchdown_frames % 5 == 0:  # Log every 0.25 seconds
                update_log(f"AUTO: Near ground. Velocity: {state['climb']:.2f} m/s")
        else:
            touchdown_frames = 0
        
        # Confirm touchdown: 0.5 seconds of zero vertical movement near ground
        if touchdown_frames > 10:
            update_log("AUTO: Touchdown confirmed. Disarming...")
            break
        
        # Safety: don't land indefinitely
        if time.time() - land_start > 25:
            update_log("AUTO: Landing timeout. Force disarming...")
            break
    
    # Final disarm
    state['rc_throttle'] = 1000
    state['rc_roll'] = 1500
    state['rc_pitch'] = 1500
    send_rc_override(master)
    time.sleep(0.5)
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    
    update_log("AUTO: Disarmed. Landing complete.")
    state['macro_status'] = 'IDLE'
    state['home_set'] = False  # Reset home after landing


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
            update_telemetry_metrics()  # Update trending metrics
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
    curses.init_pair(1, curses.COLOR_GREEN,  -1)   # Normal data
    curses.init_pair(2, curses.COLOR_RED,    -1)   # Warnings / armed
    curses.init_pair(3, curses.COLOR_CYAN,   -1)   # Borders / headings
    curses.init_pair(4, curses.COLOR_YELLOW, -1)   # Section labels / keys
    curses.init_pair(5, curses.COLOR_MAGENTA,-1)   # Livox / FAST-LIO highlights
    curses.init_pair(6, curses.COLOR_WHITE,  -1)   # Dim/neutral info
    curses.init_pair(7, curses.COLOR_BLUE,   -1)   # Alternate highlight

    while True:
        stdscr.erase()
        rows, cols = stdscr.getmaxyx()
        c_ok  = curses.color_pair(1) | curses.A_BOLD  # Green
        c_crit = curses.color_pair(2) | curses.A_BOLD # Red
        c_info = curses.color_pair(3) | curses.A_BOLD # Cyan
        c_warn = curses.color_pair(4) | curses.A_BOLD # Yellow
        c_slam = curses.color_pair(5) | curses.A_BOLD # Magenta
        c_neu  = curses.color_pair(6)                  # White

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

        W = max(cols - 1, 1)
        sep = "=" * W
        mid = "─" * W

        row = 0
        
        # ============ HEADER ============
        safe_add(row, 0, sep, c_info)
        row += 1
        header = "ASCEND v8.0 MISSION CONTROL | FAST-LIO Edition"
        safe_add(row, (W - len(header)) // 2, header, c_ok)
        row += 1

        # ============ CRITICAL ALERTS ============
        vib_max = max(state['vibration_filtered'])
        alerts = []
        
        if state['armed']:
            alerts.append(("🔴 ARMED LIVE", c_crit))
        
        if state['batt_v'] < 11.0:
            alerts.append(("⚠ CRITICAL BATTERY", c_crit))
        elif state['batt_v'] < MIN_SAFE_VOLTAGE:
            alerts.append(("⚠ LOW BATTERY", c_warn))
        
        if vib_max > MAX_VIBRATION:
            alerts.append(("⚠ EXCESSIVE VIBRATION", c_crit))
        elif vib_max > MAX_GROUND_VIBE:
            alerts.append(("⚠ HIGH VIBRATION", c_warn))
        
        if state['macro_status'] != 'IDLE':
            alerts.append((f"🔄 {state['macro_status']}", c_warn))
        
        if alerts:
            for alert_text, alert_color in alerts:
                safe_add(row, 2, alert_text, alert_color)
                row += 1
            row += 1

        # ============ MISSION STATUS BAR ============
        safe_add(row, 0, mid, c_info)
        row += 1
        
        best_alt, alt_source, alt_conf = get_best_altitude_estimate()
        remaining_time = get_battery_time_remaining()
        
        status_bar = f"MODE:{state['mode']:<12} | PHASE:{state['macro_status']:<8} | EST:{alt_source}({alt_conf:3d}%) | TIME:{remaining_time:5.1f}min | LINK:{estimate_link_quality():3d}%"
        safe_add(row, 2, status_bar, c_info)
        row += 1

        # ============ PRIMARY FLIGHT INSTRUMENTS ============
        safe_add(row, 0, mid, c_info)
        safe_add(row, 2, "[ PRIMARY INSTRUMENTS ]", c_info)
        row += 1
        
        # Altitude readout with all estimates
        alt_color = c_ok if abs(state['lidar_alt'] - state['alt']) < 0.3 else c_warn
        alt_line = f"ALT: {best_alt:7.2f}m [{alt_source}]  LIDAR:{state['lidar_alt']:6.2f}m  BARO:{state['alt']:6.2f}m  SLAM:{state['fastlio_z']:6.2f}m  Δ:{abs(state['lidar_alt']-state['alt']):6.3f}m"
        safe_add(row, 2, alt_line, alt_color)
        row += 1
        
        # Climb rate with trend sparkline
        climb_spark = create_sparkline([abs(c) for c in state['climb_history'][-10:]])
        climb_line = f"CLM: {state['climb']:+7.2f}m/s {format_trend_arrow(get_avg_climb_rate())} [trend:{climb_spark}]  AVG:{get_avg_climb_rate():+6.2f}m/s"
        safe_add(row, 2, climb_line, c_ok)
        row += 1
        
        # Attitude
        att_line = f"ATT: Roll {state['roll']:+7.1f}°  Pitch {state['pitch']:+7.1f}°  Yaw {state['hdg']:7.1f}°  | Accel limits: Roll {int(abs(state['roll'])//5):2d}/7, Pitch {int(abs(state['pitch'])//5):2d}/7"
        att_color = c_crit if abs(state['roll']) > 45 or abs(state['pitch']) > 45 else c_ok
        safe_add(row, 2, att_line, att_color)
        row += 1

        # ============ NAVIGATION & POSITIONING ============
        safe_add(row, 0, mid, c_info)
        safe_add(row, 2, "[ NAVIGATION ]", c_info)
        row += 1
        
        slam_conf = get_sensor_confidence("SLAM")
        slam_status = get_sensor_health_char(state['fastlio_healthy'])
        slam_line1 = f"SLAM: {slam_status} {state['fastlio_hz']:.0f}Hz  Pos{format_position_compact(state['fastlio_x'], state['fastlio_y'], state['fastlio_z'])}  Conf:{slam_conf:3d}%  Pts:{state['fastlio_points']:6d}"
        slam_color = c_ok if slam_conf > 50 else c_warn if slam_conf > 0 else c_neu
        safe_add(row, 2, slam_line1, slam_color)
        row += 1
        
        slam_vel_mag = math.sqrt(state['fastlio_vx']**2 + state['fastlio_vy']**2 + state['fastlio_vz']**2)
        slam_line2 = f"      Vel{format_position_compact(state['fastlio_vx'], state['fastlio_vy'], state['fastlio_vz'])} (mag:{slam_vel_mag:6.3f}m/s)  Ori: R{state['fastlio_roll']:+6.1f}° P{state['fastlio_pitch']:+6.1f}° Y{state['fastlio_yaw']:+6.1f}°"
        safe_add(row, 2, slam_line2, slam_color)
        row += 1

        # ============ SENSOR HEALTH MATRIX ============
        safe_add(row, 0, mid, c_info)
        safe_add(row, 2, "[ SENSOR HEALTH ]", c_info)
        row += 1
        
        lidar_conf = get_sensor_confidence("LIDAR")
        baro_conf = get_sensor_confidence("BARO")
        optflow_conf = get_sensor_confidence("OPTFLOW")
        
        lidar_stat = f"LIDAR {get_sensor_health_char(state['rangefinder_healthy'])} {lidar_conf:3d}%"
        baro_stat = f"BARO  {get_sensor_health_char(True)} {baro_conf:3d}%"
        optflow_stat = f"OPTFLOW {get_sensor_health_char(state['flow_qual'] > 50)} {optflow_conf:3d}% Q:{state['flow_qual']:3d}/255"
        slam_stat = f"SLAM {get_sensor_health_char(state['fastlio_healthy'])} {slam_conf:3d}%"
        livox_stat = f"LIVOX {get_sensor_health_char(state['livox_obs'] > 0)} Obs:{state['livox_obs']:5.2f}m Sectors:{state['livox_sectors']:2d}"
        
        health_color_lidar = c_ok if lidar_conf > 70 else c_warn if lidar_conf > 0 else c_neu
        health_color_baro = c_ok
        health_color_opt = c_ok if optflow_conf > 70 else c_warn if optflow_conf > 0 else c_neu
        health_color_slam = c_ok if slam_conf > 70 else c_warn if slam_conf > 0 else c_neu
        health_color_livox = c_ok if state['livox_obs'] > 0 else c_neu
        
        safe_add(row, 2, lidar_stat, health_color_lidar)
        safe_add(row, 22, baro_stat, health_color_baro)
        safe_add(row, 38, optflow_stat, health_color_opt)
        safe_add(row, 60, slam_stat, health_color_slam)
        row += 1
        
        safe_add(row, 2, livox_stat, health_color_livox)
        safe_add(row, 30, f"EKF: 0x{state['ekf_flags']:04X}  Vision: {format_position_compact(state['vision_x'], state['vision_y'], state['vision_z'])}", c_neu)
        row += 1

        # ============ VIBRATION & PERFORMANCE ============
        safe_add(row, 0, mid, c_info)
        safe_add(row, 2, "[ SYSTEM PERFORMANCE ]", c_info)
        row += 1
        
        vib_spark = create_sparkline([v for v in state['vib_max_history'][-10:]])
        cpu_spark = create_sparkline([min(c, 100) for c in state['cpu_load_history'][-10:]])
        
        vib_color = c_crit if vib_max > MAX_VIBRATION else c_warn if vib_max > MAX_GROUND_VIBE else c_ok
        vib_line = f"VIB: X{state['vibration_filtered'][0]:6.2f} Y{state['vibration_filtered'][1]:6.2f} Z{state['vibration_filtered'][2]:6.2f} m/s² | Max:{vib_max:6.2f} | Record:{state['max_vib_recorded']:6.2f} | Trend:[{vib_spark}]"
        safe_add(row, 2, vib_line, vib_color)
        row += 1
        
        cpu_color = c_warn if state['cpu_load'] > 80 else c_ok
        cpu_line = f"CPU: {state['cpu_load']:5.1f}% | Trend:[{cpu_spark}]  |  Throttle: {state['rc_throttle']:4d}({(state['rc_throttle']-1000)/10:5.1f}%)  |  OptFlow Vel: {state['flow_magnitude']:6.3f}m/s"
        safe_add(row, 2, cpu_line, cpu_color)
        row += 1

        # ============ BATTERY & ENERGY ============
        safe_add(row, 0, mid, c_info)
        safe_add(row, 2, "[ BATTERY & ENERGY ]", c_info)
        row += 1
        
        volt_drain, pct_drain = get_battery_drain_rate()
        batt_spark = create_sparkline(state['batt_pct_history'][-10:])
        
        batt_color = c_crit if state['batt_v'] < 11.0 else c_warn if state['batt_v'] < MIN_SAFE_VOLTAGE else c_ok
        batt_line = f"V: {state['batt_v']:5.2f}V  I: {state['batt_curr']:+6.1f}A  Pct: {state['batt_pct']:3d}%  Drain: {abs(volt_drain):6.3f}V/s ({abs(pct_drain):6.2f}%/s)  Remain: {remaining_time:5.1f}min  Trend:[{batt_spark}]"
        safe_add(row, 2, batt_line, batt_color)
        row += 1

        # ============ OBSTACLE AVOIDANCE / SAFETY ============
        safe_add(row, 0, mid, c_info)
        safe_add(row, 2, "[ OBSTACLE AVOIDANCE ]", c_info)
        row += 1
        
        obs_color = c_crit if state['livox_obs'] < 0.5 else c_warn if state['livox_obs'] < 1.0 else c_ok
        obs_line = f"Min Distance: {state['livox_obs']:6.2f}m  Sectors Active: {state['livox_sectors']:2d}  Safety: {'🔴 CRITICAL' if state['livox_obs'] < 0.3 else '🟠 WARNING' if state['livox_obs'] < 0.8 else '🟢 SAFE'}"
        safe_add(row, 2, obs_line, obs_color)
        row += 2

        # ============ RC CONTROL ============
        safe_add(row, 0, mid, c_info)
        safe_add(row, 2, "[RC CONTROL & COMMANDS]", c_info)
        row += 1
        
        thr_pct = (state['rc_throttle'] - 1000) / 10.0
        thr_bar = "█" * int(thr_pct / 5) + "░" * (20 - int(thr_pct / 5))
        rc_line = f"THR: [{thr_bar}] {min(100,thr_pct):5.1f}%  PITCH: {int(state['rc_pitch']):4d}  ROLL: {int(state['rc_roll']):4d}  YAW: {int(state['rc_yaw']):4d}"
        safe_add(row, 2, rc_line, c_info)
        row += 1
        
        safe_add(row, 2, "[MISSION] k=Takeoff  j=Land  [MODE] h=AltHold  l=Loiter  [CONTROL] a=Arm  d=Disarm  x=KILL  q=Quit", c_warn)
        row += 1
        
        safe_add(row, 2, "[MANUAL] W/S=Pitch  A/D=Roll  ↑↓=Throttle  [DEBUG] r=ROS2  e=Export  c=CalGyr  o=Origin  SPACE=Hover", c_warn)
        row += 2

        # ============ EVENT LOG ============
        safe_add(row, 0, mid, c_info)
        safe_add(row, 2, "[ EVENT LOG ]", c_info)
        row += 1
        
        for i, log_msg in enumerate(state['log']):
            log_row = row + i
            if log_row < rows - 1:
                # Truncate very long messages
                display_msg = log_msg if len(log_msg) < W - 4 else log_msg[:W-7] + "..."
                safe_add(log_row, 2, f"  {display_msg}", c_ok)

        stdscr.refresh()
        
        # Safety check for emergency RTL conditions
        check_emergency_rtl(master)
        
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
            elif key == ord('n'):
                # Navigate home (RTL)
                trigger_rtl(master)
            elif key == ord('w'):
                state['rc_pitch'] = 1420   # Pitch forward
            elif key == ord('s'):
                state['rc_pitch'] = 1580   # Pitch backward
            elif key in [ord('A'), ord('a')] and not curses.ascii.isalpha(key):  # Avoid conflict with arm
                state['rc_roll'] = 1420
            elif key in [ord('D'), ord('d')] and not curses.ascii.isalpha(key):  # Avoid conflict with disarm
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
                # ROS2 diagnostics & telemetry summary
                if ROS2_AVAILABLE:
                    if state['ros2_active']:
                        fastlio_age = time.time() - state['fastlio_last_time'] if state['fastlio_last_time'] > 0 else 999
                        update_log(f"ROS2: ACTIVE {state['fastlio_hz']:.0f}Hz | SLAM age: {fastlio_age:.1f}s | Pts: {state['fastlio_points']}")
                    else:
                        update_log("ROS2: NOT ACTIVE - Check `ros2 node list` and `ros2 topic list`")
                else:
                    update_log("ROS2: Not installed. Run: pip install rclpy")
            elif key == ord('e'):
                # Export comprehensive diagnostics
                export_log = f"diag_{session_id}.txt"
                with open(export_log, 'a') as f:
                    f.write(f"\n{'='*70}\n")
                    f.write(f"DIAGNOSTIC SNAPSHOT: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write(f"{'='*70}\n")
                    f.write(f"FLIGHT STATE:\n")
                    f.write(f"  Mode: {state['mode']} | Armed: {state['armed']} | Macro: {state['macro_status']}\n")
                    f.write(f"  Flight Time: {state['flight_time']:.1f}s | Max Alt: {state['max_alt_achieved']:.2f}m\n")
                    f.write(f"\nBATTERY & POWER:\n")
                    volt_dr, pct_dr = get_battery_drain_rate()
                    f.write(f"  Voltage: {state['batt_v']:.2f}V | Current: {state['batt_curr']:.1f}A | Capacity: {state['batt_pct']}%\n")
                    f.write(f"  Drain Rate: {abs(volt_dr):.3f}V/s ({abs(pct_dr):.1f}%/s)\n")
                    f.write(f"\nESTIMATES & SENSORS:\n")
                    f.write(f"  Barometer Alt: {state['alt']:.2f}m | Lidar Alt: {state['lidar_alt']:.2f}m | Error: {state['pos_error_lidar_vs_baro']:.3f}m\n")
                    f.write(f"  Climb Rate: {state['climb']:.2f}m/s | Avg: {get_avg_climb_rate():.2f}m/s\n")
                    f.write(f"  Attitude: R{state['roll']:.1f}° P{state['pitch']:.1f}° Y{state['hdg']:.0f}°\n")
                    f.write(f"\nVIBRATION & PERFORMANCE:\n")
                    f.write(f"  Raw: X{state['vibration'][0]:.3f} Y{state['vibration'][1]:.3f} Z{state['vibration'][2]:.3f} m/s²\n")
                    f.write(f"  Filtered: X{state['vibration_filtered'][0]:.2f} Y{state['vibration_filtered'][1]:.2f} Z{state['vibration_filtered'][2]:.2f} m/s²\n")
                    f.write(f"  Max Recorded: {state['max_vib_recorded']:.2f}m/s² | CPU Load: {state['cpu_load']:.1f}%\n")
                    f.write(f"\nSENSORS:\n")
                    f.write(f"  LIDAR: {'OK' if state['rangefinder_healthy'] else 'FAIL'} | Livox: {state['livox_obs']:.2f}m ({state['livox_sectors']} sectors)\n")
                    f.write(f"  OptFlow Quality: {state['flow_qual']}/255 | Mag: {state['flow_magnitude']:.3f}m/s\n")
                    f.write(f"  Vision: X{state['vision_x']:.2f} Y{state['vision_y']:.2f} Z{state['vision_z']:.2f}m\n")
                    f.write(f"  EKF Flags: 0x{state['ekf_flags']:04X}\n")
                    f.write(f"\nFAST-LIO SLAM:\n")
                    f.write(f"  Status: {'HEALTHY' if state['fastlio_healthy'] else 'INACTIVE'} | Hz: {state['fastlio_hz']:.1f}\n")
                    f.write(f"  Position: X{state['fastlio_x']:.3f} Y{state['fastlio_y']:.3f} Z{state['fastlio_z']:.3f}m\n")
                    f.write(f"  Velocity: Vx{state['fastlio_vx']:.3f} Vy{state['fastlio_vy']:.3f} Vz{state['fastlio_vz']:.3f}m/s\n")
                    f.write(f"  Orientation: R{state['fastlio_roll']:.1f}° P{state['fastlio_pitch']:.1f}° Y{state['fastlio_yaw']:.1f}°\n")
                    f.write(f"  Point Cloud: {state['fastlio_points']:,} pts/scan\n")
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
