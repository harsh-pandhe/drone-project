import time
import threading
import curses
import csv
import math
import struct
from datetime import datetime
from pymavlink import mavutil

# --- CONDITIONAL ROS2 IMPORT ---
ROS2_AVAILABLE = False
LIVOX_CUSTOM_AVAILABLE = False
ROS2_IMPORT_ERROR = ""
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, Imu
    try:
        from livox_ros_driver2.msg import CustomMsg
        LIVOX_CUSTOM_AVAILABLE = True
    except ImportError:
        CustomMsg = None
    ROS2_AVAILABLE = True
except ImportError as e:
    ROS2_IMPORT_ERROR = str(e)

# --- MISSION PARAMETERS ---
TARGET_ALTITUDE = 1.0     # Target hover height in meters
SPOOL_THR = 1400          # Idle throttle to test frame vibrations
LIFTOFF_THR = 1600        # Smooth liftoff power (+150 PWM above hover for positive authority)
HOVER_THR = 1500          # Neutral throttle for ALT_HOLD (1500 = FC holds altitude)
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

# --- FAST-LIO & SLAM ---
FASTLIO_TIMEOUT = 3.0     # Seconds before FAST-LIO is considered unhealthy
SLAM_WARN_INTERVAL = 5.0  # Minimum seconds between SLAM warning messages

# --- LOG-DRIVEN SAFETY FILTERS ---
MIN_PREFLIGHT_FLOW_QUAL = 80   # Require stable flow before autonomous takeoff
FLOW_SPIKE_REJECT_MPS = 1.2    # Reject unrealistic optical-flow velocity spikes
FLOW_LPF_ALPHA = 0.35          # Optical-flow low-pass filter coefficient
LIDAR_LPF_ALPHA = 0.35         # Rangefinder low-pass filter coefficient
MAX_LIDAR_STEP_M = 0.35        # Reject sudden lidar jumps larger than this
MIN_TAKEOFF_CLIMB_RATE = 0.12  # m/s minimum expected climb during ascent
NO_CLIMB_ABORT_SEC = 2.5       # Abort if no climb response for this long

# --- SENSOR USAGE POLICY ---
# Flight control must remain Rangefinder + Optical Flow only.
# Livox/FAST-LIO is recorded for ML and monitoring, not used for control decisions.
USE_FASTLIO_FOR_CONTROL = False

# --- LOGGING INITIALIZATION ---
session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f"telemetry_{session_id}.csv"
txt_filename = f"events_{session_id}.txt"
livox_pointcloud_filename = f"livox_pointcloud_{session_id}.bin"
livox_imu_filename = f"livox_imu_{session_id}.csv"

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

# Livox point cloud binary file - stores raw XYZ points with timestamps
livox_pc_file = open(livox_pointcloud_filename, "wb")
# Write header: magic bytes + version
livox_pc_file.write(b'LIVOXPC1')  # 8-byte magic header
livox_pc_file.flush()

# Livox IMU CSV file - stores accelerometer and gyroscope data
livox_imu_file = open(livox_imu_filename, "w", newline="")
livox_imu_writer = csv.writer(livox_imu_file)
livox_imu_writer.writerow([
    "Timestamp", "Accel_X", "Accel_Y", "Accel_Z",
    "Gyro_X", "Gyro_Y", "Gyro_Z",
    "Orient_X", "Orient_Y", "Orient_Z", "Orient_W"
])

# --- GLOBAL STATE ---
state = {
    'alt': 0.0, 'lidar_alt': 0.0, 'rangefinder_healthy': False, 'climb': 0.0,
    'lidar_alt_raw': 0.0, 'lidar_last_update': 0.0,
    'hdg': 0, 'roll': 0.0, 'pitch': 0.0,
    'batt_v': 0.0, 'batt_pct': 0, 'batt_curr': 0.0,
    'mode': 'CONNECTING...', 'armed': False, 'macro_status': 'IDLE',
    'log': [],
    'rc_throttle': 1000, 'rc_pitch': 1500, 'rc_roll': 1500, 'rc_yaw': 1500,
    'ekf_flags': 0, 'flow_qual': 0, 'flow_vx': 0.0, 'flow_vy': 0.0,
    'flow_vx_raw': 0.0, 'flow_vy_raw': 0.0, 'flow_last_update': 0.0,
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
    # Livox Mid-360 ML Training Data Collection
    'livox_pc_frames': 0, 'livox_pc_last_pts': 0,  # Point cloud frame count and last point count
    'livox_imu_samples': 0,  # IMU sample count
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
    'slam_last_warn_time': 0.0,  # Rate-limit SLAM warnings
}


def low_pass(prev_value, new_value, alpha):
    """Simple first-order low-pass filter."""
    return (alpha * new_value) + ((1.0 - alpha) * prev_value)

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


def emergency_disarm(master):
    """Emergency disarm - cut throttle and disarm immediately."""
    state['rc_throttle'] = 1000
    state['rc_roll'] = 1500
    state['rc_pitch'] = 1500
    state['rc_yaw'] = 1500
    send_rc_override(master)
    time.sleep(0.1)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    state['macro_status'] = 'IDLE'
    state['home_set'] = False


# --- IMPROVED POSITION HOLD & DRIFT CORRECTION ---

def set_home_position():
    """Lock current position as home for position hold."""
    if state['rangefinder_healthy']:
        state['home_z'] = state['lidar_alt']
    else:
        state['home_z'] = state['alt']
    
    # Flight control policy: do not use FAST-LIO for control/home reference.
    # Home XY is maintained in the local optical-flow frame only.
    if USE_FASTLIO_FOR_CONTROL and state['fastlio_healthy']:
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
    # Staleness guard for rangefinder stream.
    if state['lidar_last_update'] > 0 and (time.time() - state['lidar_last_update']) > 0.6:
        state['rangefinder_healthy'] = False

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
    """Monitor for emergency conditions that should trigger RTL.
    
    CRITICAL: NEVER trigger RTL if KILL switch was pressed (macro_status == IDLE
    and we were just in a flight macro). RTL re-engages motors which is deadly
    if the drone is already crashed/flipped.
    """
    if not state['armed'] or state['rtl_active']:
        return  # Not armed or already in RTL
    
    # NEVER override a KILL switch — if macro_status is IDLE and drone is
    # tilted, it means the user killed it. DO NOT re-engage motors.
    # Only monitor during active autonomous flight.
    if state['macro_status'] == 'IDLE':
        return
    
    # Low battery emergency
    if state['batt_pct'] < 10 and state['batt_v'] < 10.5:
        update_log("CRITICAL: Battery < 10%! Emergency disarm!")
        emergency_disarm(master)
        return
    
    # Excessive vibration — disarm, don't RTL (RTL can make things worse)
    if max(state['vibration_filtered']) > MAX_VIBRATION * 1.5:
        update_log("CRITICAL: Extreme vibrations! Emergency disarm!")
        emergency_disarm(master)
        return
    
    # Excessive tilt (loss of control) — disarm immediately, NOT RTL
    # RTL tried to fly the drone while it was flipped → caused the crash
    if abs(state['roll']) > 45 or abs(state['pitch']) > 45:
        update_log(f"CRITICAL: Tilt {state['roll']:.0f}/{state['pitch']:.0f}! Emergency disarm!")
        emergency_disarm(master)
        return
    
    # SLAM failure during flight - only warn if SLAM was previously active
    if state['home_set'] and state['fastlio_last_time'] > 0 and state['fastlio_healthy'] is False:
        if time.time() - state['fastlio_last_time'] > 5.0 and state['lidar_alt'] < 5.0:
            now = time.time()
            if now - state['slam_last_warn_time'] > SLAM_WARN_INTERVAL:
                update_log("⚠ WARNING: SLAM lost at low altitude")
                state['slam_last_warn_time'] = now
    
    # Loss of rangefinder at low altitude - RATE LIMITED
    if not state['rangefinder_healthy'] and state['lidar_alt'] < 1.0:
        if state['alt'] < 0.5:
            now = time.time()
            if now - state.get('rangefinder_last_warn_time', 0) > SLAM_WARN_INTERVAL:
                update_log("⚠ WARNING: Rangefinder lost near ground")
                state['rangefinder_last_warn_time'] = now


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
            
            try:
                # Subscribe to raw Livox point cloud for ML training.
                # msg_MID360_launch publishes livox_ros_driver2/CustomMsg on /livox/lidar.
                if LIVOX_CUSTOM_AVAILABLE:
                    self.livox_cloud_sub = self.create_subscription(
                        CustomMsg, '/livox/lidar', self.livox_custom_cloud_callback, 5)
                    self.get_logger().info("Subscribed to /livox/lidar (CustomMsg raw points)")
                else:
                    # Fallback for pointcloud2 launch variants.
                    self.livox_cloud_sub = self.create_subscription(
                        PointCloud2, '/livox/lidar', self.livox_cloud_callback, 5)
                    self.get_logger().info("Subscribed to /livox/lidar (PointCloud2 raw points)")
            except Exception as e:
                self.get_logger().warn(f"Failed to subscribe to /livox/lidar: {e}")
                self.livox_cloud_sub = None
            
            try:
                # Subscribe to Livox IMU data
                self.livox_imu_sub = self.create_subscription(
                    Imu, '/livox/imu', self.livox_imu_callback, 50)
                self.get_logger().info("Subscribed to /livox/imu topic")
            except Exception as e:
                self.get_logger().warn(f"Failed to subscribe to /livox/imu: {e}")
                self.livox_imu_sub = None
            
            # Point cloud frame counter for logging
            self.pc_frame_count = 0
            self.imu_frame_count = 0
                
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

        def livox_cloud_callback(self, msg):
            """Store full raw Livox point cloud to binary file for ML training."""
            try:
                timestamp = time.time()
                num_points = (msg.row_step * msg.height) // msg.point_step if msg.point_step > 0 else 0
                
                if num_points > 0:
                    # Write frame to binary file:
                    # Header: timestamp + point_count + point_step
                    # Data: raw PointCloud2 payload
                    livox_pc_file.write(struct.pack('<d', timestamp))  # timestamp
                    livox_pc_file.write(struct.pack('<I', num_points))  # num points
                    livox_pc_file.write(struct.pack('<H', msg.point_step))  # bytes per point
                    livox_pc_file.write(msg.data)  # raw point cloud data
                    
                    self.pc_frame_count += 1
                    
                    # Flush every 10 frames to prevent data loss
                    if self.pc_frame_count % 10 == 0:
                        livox_pc_file.flush()
                    
                    # Update state for display
                    state['livox_pc_frames'] = self.pc_frame_count
                    state['livox_pc_last_pts'] = num_points
                    
            except Exception as e:
                self.get_logger().error(f"Livox cloud callback error: {e}")

        def livox_custom_cloud_callback(self, msg):
            """Store full livox_ros_driver2/CustomMsg cloud data for Mid-360 ML training."""
            try:
                timestamp = time.time()
                num_points = int(msg.point_num)

                if num_points > 0:
                    # Binary frame:
                    # [float64 ts][uint32 point_num][uint16 point_step=19][uint64 timebase]
                    # then repeated points as <IfffBBB>
                    #   offset_time(uint32), x(float32), y(float32), z(float32),
                    #   reflectivity(uint8), tag(uint8), line(uint8)
                    livox_pc_file.write(struct.pack('<d', timestamp))
                    livox_pc_file.write(struct.pack('<I', num_points))
                    livox_pc_file.write(struct.pack('<H', 19))
                    livox_pc_file.write(struct.pack('<Q', int(msg.timebase)))

                    for p in msg.points:
                        livox_pc_file.write(struct.pack(
                            '<IfffBBB',
                            int(p.offset_time),
                            float(p.x),
                            float(p.y),
                            float(p.z),
                            int(p.reflectivity),
                            int(p.tag),
                            int(p.line)
                        ))

                    self.pc_frame_count += 1
                    if self.pc_frame_count % 10 == 0:
                        livox_pc_file.flush()

                    state['livox_pc_frames'] = self.pc_frame_count
                    state['livox_pc_last_pts'] = num_points

            except Exception as e:
                self.get_logger().error(f"Livox CustomMsg callback error: {e}")

        def livox_imu_callback(self, msg):
            """Store Livox IMU data to CSV for ML training."""
            try:
                timestamp = time.time()
                
                # Extract IMU data
                accel_x = msg.linear_acceleration.x
                accel_y = msg.linear_acceleration.y
                accel_z = msg.linear_acceleration.z
                gyro_x = msg.angular_velocity.x
                gyro_y = msg.angular_velocity.y
                gyro_z = msg.angular_velocity.z
                orient_x = msg.orientation.x
                orient_y = msg.orientation.y
                orient_z = msg.orientation.z
                orient_w = msg.orientation.w
                
                # Write to CSV
                livox_imu_writer.writerow([
                    f"{timestamp:.6f}",
                    f"{accel_x:.6f}", f"{accel_y:.6f}", f"{accel_z:.6f}",
                    f"{gyro_x:.6f}", f"{gyro_y:.6f}", f"{gyro_z:.6f}",
                    f"{orient_x:.6f}", f"{orient_y:.6f}", f"{orient_z:.6f}", f"{orient_w:.6f}"
                ])
                
                self.imu_frame_count += 1
                
                # Flush every 100 samples
                if self.imu_frame_count % 100 == 0:
                    livox_imu_file.flush()
                
                # Update state for display
                state['livox_imu_samples'] = self.imu_frame_count
                
            except Exception as e:
                self.get_logger().error(f"Livox IMU callback error: {e}")


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
    oldest = recent[0]
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
    slam_conf = get_sensor_confidence("SLAM") if USE_FASTLIO_FOR_CONTROL else 0
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
        
    if state['flow_qual'] < MIN_PREFLIGHT_FLOW_QUAL:
        update_log(
            f"DIAG: FAIL - Optical Flow quality too low ({state['flow_qual']}). "
            f"Need >= {MIN_PREFLIGHT_FLOW_QUAL}."
        )
        return False

    if not state['rangefinder_healthy']:
        update_log("DIAG: FAIL - Rangefinder unavailable. Autonomous takeoff blocked.")
        return False

    # FAST-LIO health check
    if state['ros2_active']:
        check_fastlio_health()
        if state['fastlio_healthy']:
            update_log(f"DIAG: FAST-LIO OK ({state['fastlio_hz']:.0f} Hz) [logging-only]")
        else:
            update_log("DIAG: WARN - FAST-LIO not publishing. ML logging unavailable.")
    else:
        update_log("DIAG: INFO - ROS2 not active. Livox ML logging unavailable.")

    update_log("DIAG: Control source = Rangefinder + Optical Flow (FAST-LIO disabled for control)")
        
    update_log("DIAG: PASS - Systems nominal. Ready for flight.")
    return True

# --- FLIGHT AUTOMATION ---

def auto_takeoff_sequence(master, target_alt=None):
    """Automatic takeoff to specified altitude with stable position hold.
    
    Args:
        target_alt: Altitude in meters (default: TARGET_ALTITUDE=1.0m)
    
    KEY DESIGN: Trust ArduCopter's ALT_HOLD controller for altitude.
    - Throttle 1500 = hold altitude (FC PID handles internally)
    - Throttle > 1500 = climb (rate proportional to stick deflection)
    - Throttle < 1500 = descend
    - Pitch/Roll 1500 = level/hold attitude (FC stabilization)
    
    DO NOT layer Python PID on top of ArduCopter's PID - this causes
    oscillations and fighting between two controllers.
    """
    if target_alt is None:
        target_alt = TARGET_ALTITUDE
    
    if state['macro_status'] != 'IDLE': return
    state['macro_status'] = 'TAKEOFF'
    
    if not run_preflight_diagnostics():
        state['macro_status'] = 'IDLE'
        return

    # 1. Disable arming checks and force EKF origin
    # Send RC keepalives throughout to prevent "Radio Failsafe" on FC
    update_log("AUTO: Anchoring EKF Origin...")
    state['rc_throttle'] = 1000
    state['rc_pitch'] = 1500
    state['rc_roll'] = 1500
    state['rc_yaw'] = 1500
    master.mav.param_set_send(master.target_system, master.target_component,
                              b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    for _ in range(15):  # 1.5s with RC keepalive every 0.1s
        send_rc_override(master)
        time.sleep(0.1)
    
    for _ in range(5):
        master.mav.set_gps_global_origin_send(master.target_system, int(19.033 * 1e7), int(73.029 * 1e7), 0)
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0, 0, 0, 0, 0, 0)
        send_rc_override(master)
        time.sleep(0.3)
    for _ in range(10):  # 1.0s with RC keepalive
        send_rc_override(master)
        time.sleep(0.1)

    # 2. Enter STABILIZE for arming (most permissive)
    update_log("AUTO: Entering STABILIZE mode for arming...")
    master.set_mode(master.mode_mapping()['STABILIZE'])
    for _ in range(10):  # 1.0s with RC keepalive (prevents Radio Failsafe)
        send_rc_override(master)
        time.sleep(0.1)

    # 3. Capture ground altitude reference
    ground_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
    abs_target = ground_alt + target_alt
    update_log(f"AUTO: Ground: {ground_alt:.2f}m. Target: {abs_target:.2f}m ({target_alt:.0f}m AGL)")

    # 4. Arm with retry (RC keepalive during waits)
    # ALWAYS send arm command even if state['armed'] appears True —
    # FC may have auto-disarmed (DISARM_DELAY=10s) and HEARTBEAT is stale.
    update_log("AUTO: Attempting arm...")
    for arm_attempt in range(3):
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                     0, 1, 21196, 0, 0, 0, 0, 0)
        for _ in range(15):  # 1.5s with RC keepalive
            send_rc_override(master)
            time.sleep(0.1)
        if state['armed']:
            break
        update_log(f"AUTO: Arm attempt {arm_attempt+1}/3 failed, retrying...")
    
    if not state['armed']:
        update_log("AUTO: Arming FAILED. Safety switch or other issue.")
        state['macro_status'] = 'IDLE'
        return
    
    update_log("AUTO: Armed. Switching to ALT_HOLD...")

    # 5. Switch to ALT_HOLD
    master.set_mode(master.mode_mapping()['ALT_HOLD'])
    for _ in range(10):  # 1.0s with RC keepalive
        send_rc_override(master)
        time.sleep(0.1)

    # 6. Ground spool-up with vibration test
    update_log(f"AUTO: Spooling to {SPOOL_THR} for vibration test...")
    for thr in range(1100, SPOOL_THR, 15):
        state['rc_throttle'] = thr
        state['rc_pitch'] = 1500
        state['rc_roll'] = 1500
        state['rc_yaw'] = 1500
        send_rc_override(master)
        time.sleep(0.05)
    
    update_log("AUTO: Holding spool for 3.0s vibration check...")
    max_spool_vib = 0.0
    for i in range(30):
        time.sleep(0.1)
        send_rc_override(master)
        current_max_vib = max(state['vibration_filtered'])
        max_spool_vib = max(max_spool_vib, current_max_vib)
        if i % 10 == 0:
            update_log(f"AUTO: Spool [{i*100//30}%] vib: {max_spool_vib:.2f} m/s²")
        if current_max_vib > MAX_GROUND_VIBE:
            update_log(f"AUTO: ABORT! Ground vib {current_max_vib:.1f} > {MAX_GROUND_VIBE}")
            emergency_disarm(master)
            return

    # Check that motors are actually spinning (vibration too low = FC disarmed silently)
    if max_spool_vib < 0.5:
        update_log(f"AUTO: ABORT! Vib only {max_spool_vib:.2f} — motors not spinning. FC may have disarmed.")
        emergency_disarm(master)
        return
    
    # Re-verify armed state before committing to climb
    if not state['armed']:
        update_log("AUTO: ABORT! FC disarmed during spool-up.")
        state['macro_status'] = 'IDLE'
        return
    
    update_log(f"AUTO: Vib PASS (max {max_spool_vib:.2f}). Climbing to {target_alt:.0f}m...")

    # 7. CLIMB PHASE
    # In ALT_HOLD: throttle > 1500 = climb at rate proportional to deflection
    # 1700 = strong climb (~40% of max range), 1580 = gentle approach
    CLIMB_THR = 1700       # Strong climb command
    APPROACH_THR = 1580    # Gentler climb for final 30cm
    FINE_THR = 1530        # Very gentle for last 10cm
    CLIMB_TIMEOUT = 15.0 + target_alt * 5.0  # More time for higher altitudes
    
    climb_start = time.time()
    
    # Ramp from spool throttle to full climb throttle
    for thr in range(SPOOL_THR, CLIMB_THR, 10):
        state['rc_throttle'] = thr
        state['rc_pitch'] = 1500
        state['rc_roll'] = 1500
        state['rc_yaw'] = 1500
        send_rc_override(master)
        time.sleep(0.02)
    
    update_log("AUTO: Full climb power. Monitoring altitude...")
    last_alt_log = time.time()
    no_climb_since = None
    
    while time.time() - climb_start < CLIMB_TIMEOUT and state['armed']:
        current_alt = get_best_altitude_for_control()
        alt_remaining = abs_target - current_alt
        
        # Staged throttle: reduce as we approach target and damp with climb-rate feedback.
        if alt_remaining < 0.10:
            state['rc_throttle'] = 1520
        elif alt_remaining < 0.30:
            state['rc_throttle'] = 1560 if state['climb'] < 0.25 else 1535
        else:
            state['rc_throttle'] = 1680 if state['climb'] < 0.15 else 1620
        
        state['rc_pitch'] = 1500
        state['rc_roll'] = 1500
        state['rc_yaw'] = 1500
        send_rc_override(master)
        
        # Check if target reached (within 5cm)
        if current_alt >= abs_target - 0.05:
            update_log(f"AUTO: Target reached at {current_alt:.2f}m")
            break

        # Abort if FC is not responding to climb command despite enough margin.
        if alt_remaining > 0.25 and state['climb'] < MIN_TAKEOFF_CLIMB_RATE:
            if no_climb_since is None:
                no_climb_since = time.time()
            elif time.time() - no_climb_since > NO_CLIMB_ABORT_SEC:
                update_log(
                    f"AUTO: ABORT! No climb response for {NO_CLIMB_ABORT_SEC:.1f}s "
                    f"(climb={state['climb']:.2f}m/s)."
                )
                emergency_disarm(master)
                return
        else:
            no_climb_since = None
        
        # Periodic altitude log
        if time.time() - last_alt_log > 1.0:
            update_log(
                f"AUTO: Climbing... {current_alt:.2f}m / {abs_target:.2f}m "
                f"(remaining: {alt_remaining:.2f}m, climb: {state['climb']:.2f}m/s)"
            )
            last_alt_log = time.time()
        
        time.sleep(0.05)
        
        # Safety: tilt check
        if abs(state['roll']) > 35 or abs(state['pitch']) > 35:
            update_log("AUTO: CRITICAL TILT during climb! KILL.")
            emergency_disarm(master)
            return
        # Safety: vibration check
        if max(state['vibration_filtered']) > MAX_VIBRATION:
            update_log(f"AUTO: FATAL VIB during climb ({max(state['vibration_filtered']):.1f})! KILL.")
            emergency_disarm(master)
            return
    else:
        # Loop ended via timeout (while condition became false)
        current_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
        update_log(f"AUTO: Climb timeout at {current_alt:.2f}m. Proceeding to hover.")

    if not state['armed']:
        state['macro_status'] = 'IDLE'
        return

    # 8. HOVER PHASE - Use LOITER for FC-native position hold
    # LOITER mode uses EKF position estimate (from optical flow + rangefinder)
    # to hold both altitude AND horizontal position. All sticks at 1500 = hold.
    # This is MUCH better than ALT_HOLD (altitude only, no position hold).
    total_hover_time = HOVER_SETTLE_TIME + HOVER_DURATION
    update_log(f"AUTO: Switching to LOITER for position hold at {target_alt:.0f}m...")
    
    # Retry LOITER mode switch up to 3 times
    loiter_ok = False
    for attempt in range(3):
        master.set_mode(master.mode_mapping()['LOITER'])
        # Keep sending RC overrides while waiting for mode switch
        for _ in range(10):  # 0.5s with keepalive
            state['rc_throttle'] = HOVER_THR
            state['rc_pitch'] = 1500
            state['rc_roll'] = 1500
            state['rc_yaw'] = 1500
            send_rc_override(master)
            time.sleep(0.05)
        
        if 'LOITER' in state['mode'].upper():
            loiter_ok = True
            break
        update_log(f"AUTO: LOITER attempt {attempt+1}/3 failed, retrying...")
    
    if not loiter_ok:
        update_log("AUTO: LOITER REJECTED by FC! No position hold available. Landing for safety.")
        auto_land_sequence(master)
        return
    
    update_log(f"AUTO: LOITER engaged. FC holding position for {total_hover_time:.0f}s...")
    
    state['rc_throttle'] = HOVER_THR  # 1500 = neutral = hold altitude
    state['rc_pitch'] = 1500          # In LOITER: 1500 = hold position
    state['rc_roll'] = 1500           # In LOITER: 1500 = hold position
    state['rc_yaw'] = 1500
    send_rc_override(master)
    
    set_home_position()
    state['macro_status'] = 'HOVER'
    
    hover_start = time.time()
    hover_drift_samples = []
    last_log_time = time.time()
    poor_flow_since = None
    LOG_INTERVAL = 2.0
    
    while time.time() - hover_start < total_hover_time and state['armed'] and state['macro_status'] == 'HOVER':
        current_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
        
        # ALL STICKS NEUTRAL - FC handles everything in LOITER
        # In LOITER: pitch/roll 1500 = hold position (FC uses optical flow)
        # In ALT_HOLD: pitch/roll 1500 = level (no position hold)
        state['rc_throttle'] = HOVER_THR  # 1500
        state['rc_pitch'] = 1500
        state['rc_roll'] = 1500
        state['rc_yaw'] = 1500
        send_rc_override(master)
        
        # Track drift for reporting
        update_position_estimate(0.05)
        drift = get_drift_magnitude()
        hover_drift_samples.append(drift)
        if len(hover_drift_samples) > 500:
            hover_drift_samples.pop(0)
        
        # Periodic logging
        if time.time() - last_log_time > LOG_INTERVAL:
            elapsed = time.time() - hover_start
            vib_now = max(state['vibration_filtered'])
            phase = "SETTLE" if elapsed < HOVER_SETTLE_TIME else "HOVER"
            update_log(f"{phase}: [{elapsed:.0f}s] Alt={current_alt:.2f}m Drift={drift:.2f}m Vib={vib_now:.1f}")
            last_log_time = time.time()

        # Position hold depends on optical-flow quality; land safely on sustained dropout.
        if state['flow_qual'] < 40:
            if poor_flow_since is None:
                poor_flow_since = time.time()
            elif time.time() - poor_flow_since > 2.0:
                update_log("AUTO: Optical-flow degraded during hover. Landing for safety.")
                auto_land_sequence(master)
                return
        else:
            poor_flow_since = None
        
        time.sleep(0.05)
        
        # Safety: vibration
        if max(state['vibration_filtered']) > MAX_VIBRATION:
            update_log(f"AUTO: FATAL VIB during hover ({max(state['vibration_filtered']):.1f})! KILL.")
            emergency_disarm(master)
            return
        # Safety: tilt
        if abs(state['roll']) > 40 or abs(state['pitch']) > 40:
            update_log("AUTO: CRITICAL TILT during hover! KILL.")
            emergency_disarm(master)
            return
        # Safety: battery - only trigger on actual dangerously low voltage
        # NOT on percentage (12% at 12V is normal for a 4S LiPo)
        if state['batt_v'] > 1.0 and state['batt_v'] < MIN_SAFE_VOLTAGE:
            update_log(f"AUTO: CRITICAL VOLTAGE {state['batt_v']:.1f}V! Landing now.")
            break
    
    avg_drift = sum(hover_drift_samples) / max(1, len(hover_drift_samples))
    max_drift = max(hover_drift_samples) if hover_drift_samples else 0
    update_log(f"AUTO: Hover complete. Avg drift: {avg_drift:.3f}m, Max: {max_drift:.3f}m")
    
    # 9. Transition to landing (only if not already started by 'j' key)
    if state['macro_status'] == 'HOVER':
        update_log("AUTO: Initiating landing sequence...")
        auto_land_sequence(master)


def auto_land_sequence(master):
    """Controlled landing using ArduCopter's LAND mode.
    
    LAND mode handles everything internally:
    - Descends at LAND_SPEED (FC parameter, default 50 cm/s)
    - Maintains horizontal position hold (like LOITER)
    - Handles ground effect and rangefinder fusion via EKF
    - Auto-detects touchdown and disarms
    
    Previous attempts to manually descend in LOITER failed because:
    - Throttle 1440 was inside FC's THR_DZ deadband (±100 around 1500)
    - Rangefinder spikes caused FC altitude overcorrection
    - Python descent braking fought FC's altitude controller
    
    SAFETY: Loop exits immediately if KILL switch sets macro_status to IDLE.
    """
    # Guard: only allow landing from HOVER or IDLE states
    if state['macro_status'] not in ('IDLE', 'HOVER'):
        update_log("LAND: Cannot land during " + state['macro_status'] + ". Use 'x' to abort.")
        return
    
    # Guard: must be armed and airborne
    if not state['armed']:
        update_log("LAND: Not armed. Nothing to do.")
        return
    
    current_check_alt = get_best_altitude_for_control()
    if current_check_alt < 0.10:
        update_log("LAND: Already on ground. Disarming.")
        emergency_disarm(master)
        return
    
    state['macro_status'] = 'LANDING'
    
    update_log("AUTO: Switching to LAND mode...")
    
    # Switch to LAND mode — FC handles descent, position hold, and touchdown
    master.set_mode(master.mode_mapping()['LAND'])
    
    # All sticks neutral — FC controls everything in LAND mode
    state['rc_throttle'] = 1500
    state['rc_pitch'] = 1500
    state['rc_roll'] = 1500
    state['rc_yaw'] = 1500
    send_rc_override(master)
    
    # Wait for mode switch confirmation
    for _ in range(10):  # 0.5s with RC keepalive
        send_rc_override(master)
        time.sleep(0.05)
    
    if 'LAND' in state['mode'].upper():
        update_log("AUTO: LAND mode engaged. FC handling descent...")
    else:
        update_log(f"AUTO: LAND mode requested (current: {state['mode']}). Monitoring...")
    
    land_start = time.time()
    LAND_TIMEOUT = 30.0
    last_log_time = time.time()
    
    while state['armed'] and time.time() - land_start < LAND_TIMEOUT:
        # EXIT IMMEDIATELY if KILL switch was pressed
        if state['macro_status'] != 'LANDING':
            update_log("LAND: Aborted — KILL switch or mode change.")
            return
        
        current_alt = get_best_altitude_for_control()
        
        # TILT SAFETY: if drone tilts > 30° during landing, emergency disarm
        if abs(state['roll']) > 30 or abs(state['pitch']) > 30:
            update_log(f"LAND: TILT ABORT Roll={state['roll']:.0f}° Pitch={state['pitch']:.0f}°")
            emergency_disarm(master)
            return
        
        # All sticks neutral — let FC handle everything
        state['rc_throttle'] = 1500
        state['rc_pitch'] = 1500
        state['rc_roll'] = 1500
        state['rc_yaw'] = 1500
        send_rc_override(master)
        
        # Periodic logging
        if time.time() - last_log_time > 2.0:
            update_log(f"LAND: Alt={current_alt:.2f}m Mode={state['mode']} Climb={state['climb']:.2f}")
            last_log_time = time.time()
        
        time.sleep(0.05)
    
    # EXIT IMMEDIATELY if KILL was pressed while in loop
    if state['macro_status'] != 'LANDING':
        return
    
    if not state['armed']:
        # FC auto-disarmed on touchdown — success!
        update_log("AUTO: Touchdown! FC auto-disarmed. Landing complete.")
        state['macro_status'] = 'IDLE'
        state['home_set'] = False
    elif time.time() - land_start >= LAND_TIMEOUT:
        update_log("AUTO: Landing timeout. Force disarming...")
        emergency_disarm(master)
    
    update_log("AUTO: Landing sequence finished.")


def auto_move_sequence(master, direction):
    """Move ~1 meter in specified direction using LOITER stick deflection.
    
    In LOITER mode, pitch/roll stick deflection from 1500 commands a velocity.
    We tilt the stick for a calibrated duration, then return to neutral and let
    the FC (LOITER) brake and hold the new position.
    
    direction: 'front', 'back', 'left', 'right'
    
    Only works when in HOVER state (armed + LOITER mode).
    """
    if state['macro_status'] != 'HOVER':
        update_log(f"MOVE: Cannot move — must be in HOVER state (current: {state['macro_status']})")
        return
    
    if not state['armed']:
        update_log("MOVE: Not armed.")
        return
    
    if 'LOITER' not in state['mode'].upper():
        update_log(f"MOVE: Not in LOITER mode (current: {state['mode']}). Cannot move.")
        return
    
    # Stick deflection and duration to produce ~1m movement
    # LOITER interprets stick deflection as velocity command.
    # Moderate deflection (~100 PWM from center) for a short duration.
    MOVE_DEFLECTION = 100   # PWM units from 1500 center
    MOVE_DURATION = 0.8     # Seconds of stick input (tune on real drone)
    BRAKE_SETTLE = 1.0      # Seconds to let FC brake and settle
    
    direction_map = {
        'front': ('rc_pitch', 1500 - MOVE_DEFLECTION),   # Pitch forward = lower PWM
        'back':  ('rc_pitch', 1500 + MOVE_DEFLECTION),   # Pitch back = higher PWM
        'left':  ('rc_roll',  1500 - MOVE_DEFLECTION),   # Roll left = lower PWM
        'right': ('rc_roll',  1500 + MOVE_DEFLECTION),   # Roll right = higher PWM
    }
    
    if direction not in direction_map:
        update_log(f"MOVE: Unknown direction '{direction}'")
        return
    
    channel, pwm_value = direction_map[direction]
    state['macro_status'] = 'MOVING'
    update_log(f"MOVE: Moving {direction} ~1m...")
    
    # Apply stick deflection for MOVE_DURATION
    move_start = time.time()
    while time.time() - move_start < MOVE_DURATION:
        if state['macro_status'] == 'IDLE':  # KILL switch
            return
        
        state[channel] = pwm_value
        state['rc_throttle'] = 1500  # Maintain altitude
        state['rc_yaw'] = 1500
        # Keep the other axis neutral
        if channel == 'rc_pitch':
            state['rc_roll'] = 1500
        else:
            state['rc_pitch'] = 1500
        send_rc_override(master)
        
        # Safety: tilt check
        if abs(state['roll']) > 35 or abs(state['pitch']) > 35:
            update_log("MOVE: TILT ABORT! Returning to hover.")
            break
        
        time.sleep(0.05)
    
    # Return all sticks to neutral — FC will brake in LOITER
    state['rc_pitch'] = 1500
    state['rc_roll'] = 1500
    state['rc_throttle'] = 1500
    state['rc_yaw'] = 1500
    send_rc_override(master)
    
    update_log(f"MOVE: Stick released. FC braking...")
    
    # Let FC settle at new position
    settle_start = time.time()
    while time.time() - settle_start < BRAKE_SETTLE:
        if state['macro_status'] == 'IDLE':
            return
        state['rc_pitch'] = 1500
        state['rc_roll'] = 1500
        state['rc_throttle'] = 1500
        state['rc_yaw'] = 1500
        send_rc_override(master)
        time.sleep(0.05)
    
    current_alt = get_best_altitude_for_control()
    update_log(f"MOVE: {direction} complete. Alt: {current_alt:.2f}m")
    state['macro_status'] = 'HOVER'


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
                new_alt = round(dist_cm / 100.0, 2)

                # Reject abrupt lidar jumps, then low-pass to reduce control jitter.
                prev_raw = state['lidar_alt_raw']
                if prev_raw > 0 and abs(new_alt - prev_raw) > MAX_LIDAR_STEP_M:
                    state['lidar_spike_count'] = state.get('lidar_spike_count', 0) + 1
                    if state['lidar_spike_count'] < 3:
                        new_alt = prev_raw
                    else:
                        state['lidar_spike_count'] = 0
                else:
                    state['lidar_spike_count'] = 0

                state['lidar_alt_raw'] = new_alt
                if state['lidar_alt'] <= 0:
                    state['lidar_alt'] = new_alt
                else:
                    state['lidar_alt'] = round(low_pass(state['lidar_alt'], new_alt, LIDAR_LPF_ALPHA), 3)

                state['lidar_last_update'] = time.time()
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
            raw_vx = float(msg.flow_comp_m_x)
            raw_vy = float(msg.flow_comp_m_y)
            state['flow_vx_raw'] = round(raw_vx, 3)
            state['flow_vy_raw'] = round(raw_vy, 3)
            state['flow_last_update'] = time.time()

            if state['flow_qual'] < 30:
                state['flow_vx'] = 0.0
                state['flow_vy'] = 0.0
            else:
                # Clamp unrealistic bursts observed in logs while keeping valid motion.
                clamped_vx = max(-FLOW_SPIKE_REJECT_MPS, min(FLOW_SPIKE_REJECT_MPS, raw_vx))
                clamped_vy = max(-FLOW_SPIKE_REJECT_MPS, min(FLOW_SPIKE_REJECT_MPS, raw_vy))

                if abs(raw_vx) > FLOW_SPIKE_REJECT_MPS or abs(raw_vy) > FLOW_SPIKE_REJECT_MPS:
                    state['flow_spike_count'] = state.get('flow_spike_count', 0) + 1
                else:
                    state['flow_spike_count'] = 0

                state['flow_vx'] = round(low_pass(state['flow_vx'], clamped_vx, FLOW_LPF_ALPHA), 3)
                state['flow_vy'] = round(low_pass(state['flow_vy'], clamped_vy, FLOW_LPF_ALPHA), 3)

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
            # Filter repetitive FC messages that flood the event log
            text = msg.text
            if 'Field Elevation Set' not in text:
                update_log(f"SYS: {text}")


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
        
        # Livox ML Data Collection Status
        livox_ml_stat = f"ML DATA: PC_Frames:{state['livox_pc_frames']:6d}  Last:{state['livox_pc_last_pts']:6d}pts  IMU:{state['livox_imu_samples']:8d}smpls"
        ml_color = c_ok if state['livox_pc_frames'] > 0 else c_neu
        safe_add(row, 2, livox_ml_stat, ml_color)
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
        
        safe_add(row, 2, "[TAKEOFF] 1=1m  2=2m  j=Land  [MOVE] i=Fwd  ,=Back  ;=Left  '=Right  [CTRL] a=Arm  d=Disarm  x=KILL  q=Quit", c_warn)
        row += 1
        
        safe_add(row, 2, "[MODE] h=AltHold  l=Loiter  [MANUAL] W/S=Pitch  ↑↓=Thr  SPACE=Hover  [DBG] r=ROS2  e=Export  c=Cal  o=Origin", c_warn)
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

        # Reset sticks to neutral ONLY when not in autonomous mode
        # During autonomous flight, the flight thread controls all sticks
        if state['macro_status'] == 'IDLE':
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
            elif key == ord('1'):
                threading.Thread(target=auto_takeoff_sequence, args=(master, 1.0), daemon=True).start()
            elif key == ord('2'):
                threading.Thread(target=auto_takeoff_sequence, args=(master, 2.0), daemon=True).start()
            elif key == ord('i'):
                threading.Thread(target=auto_move_sequence, args=(master, 'front'), daemon=True).start()
            elif key == ord(','):
                threading.Thread(target=auto_move_sequence, args=(master, 'back'), daemon=True).start()
            elif key == ord(';'):
                threading.Thread(target=auto_move_sequence, args=(master, 'left'), daemon=True).start()
            elif key == ord("'"):
                threading.Thread(target=auto_move_sequence, args=(master, 'right'), daemon=True).start()
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
                update_log("CMD: *** EMERGENCY KILL SWITCH TRIGGERED ***")
                state['macro_status'] = 'IDLE'  # Stop all autonomous threads FIRST
                state['rtl_active'] = False      # Cancel any RTL
                emergency_disarm(master)
                # Send force-disarm multiple times to ensure FC complies
                for _ in range(5):
                    master.mav.command_long_send(
                        master.target_system, master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0
                    )
                    time.sleep(0.05)
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

        # Only send RC overrides from dashboard when not in autonomous mode
        if state['macro_status'] == 'IDLE':
            send_rc_override(master)
        time.sleep(0.05)


# --- ENTRY POINT ---

if __name__ == '__main__':
    try:
        print("\n" + "="*70)
        print("  ASCEND MISSION CONTROL v8.0 - Rangefinder + Optical Flow")
        print("="*70)
        
        # Start ROS2 FAST-LIO subscriber if available (optional)
        if ROS2_AVAILABLE:
            print("\n[*] ROS2 detected. Starting FAST-LIO subscriber (optional)...")
            ros2_thread = threading.Thread(target=run_ros2_thread, daemon=True)
            ros2_thread.start()
            time.sleep(2.0)
        else:
            print("\n[!] ROS2 Python modules not available in current shell.")
            print("    FAST-LIO/Livox logging will be disabled for this run.")
            if ROS2_IMPORT_ERROR:
                print(f"    Import error: {ROS2_IMPORT_ERROR}")
            print("    Source env before launch:")
            print("    source /home/iic5/.venv/bin/activate")
            print("    source /home/iic5/ros2_humble/install/setup.bash")
            print("    source /home/iic5/slam_ws/install/setup.bash")
            print("\n[*] Running with Rangefinder + Optical Flow (no SLAM).")
            time.sleep(0.5)

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
        # Close Livox data files
        livox_pc_file.close()
        livox_imu_file.close()
        print(f"[*] Livox data saved: {state['livox_pc_frames']} point cloud frames, {state['livox_imu_samples']} IMU samples")
        if ROS2_AVAILABLE:
            state['ros2_active'] = False
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except:
                pass
        print("[*] Goodbye!")



