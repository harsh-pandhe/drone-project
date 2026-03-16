import time
import threading
import curses
import csv
import math
import struct
import argparse
import os
import re
import glob
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
HOVER_DURATION = 10.0     # Timed hover window for repeatable 1m test, then auto-land
CLIMB_RAMP_RATE = 5       # PWM per cycle (increased from 2 for faster climb ramp)
TAKEOFF_REACH_MARGIN = 0.18  # Acceptable shortfall below target altitude before entering hold

# --- CLIMB SAFETY & STABILITY ---
CLIMB_TILT_DERATE_START_DEG = 12.0
CLIMB_TILT_DERATE_HARD_DEG = 20.0
CLIMB_TILT_LAND_DEG = 28.0
CLIMB_TILT_LAND_HOLD_SEC = 0.4
CLIMB_TILT_KILL_DEG = 45.0

# --- YAW STABILIZATION (Compass/IMU) ---
YAW_RATE_LIMIT = 60.0     # Max rate of change for yaw angle (degrees/second)
YAW_INTEGRAL_LIMIT = 30   # Max accumulated yaw error

# --- FAST-LIO & SLAM ---
FASTLIO_TIMEOUT = 3.0     # Seconds before FAST-LIO is considered unhealthy
SLAM_WARN_INTERVAL = 5.0  # Minimum seconds between SLAM warning messages
FASTLIO_MIN_HEALTHY_HZ = 5.0
FASTLIO_STALE_WARN_SEC = 1.5
SLAM_MAX_ALT_DIFF = 1.0

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
# Enable weighted multi-sensor fusion for altitude supervision during autonomy.
# This still favors rangefinder + flow, but adds SLAM/baro to reject spikes.
USE_MULTISENSOR_CONTROL = True
MAX_ALT_SENSOR_DISAGREE = 0.8
LIVOX_MIN_SAFE_DIST = 0.8

# LOITER engagement safety gates (optical-flow/rangefinder only operations)
LOITER_ENTRY_MAX_TILT_DEG = 15.0
PRE_LOITER_STABLE_TILT_DEG = 10.0
PRE_LOITER_STABLE_CLIMB_MPS = 0.25
PRE_LOITER_STABLE_SEC = 1.2
PRE_LOITER_TIMEOUT_SEC = 4.0
LOITER_ENTRY_CONFIRM_TILT_DEG = 12.0
LOITER_ENTRY_CONFIRM_CLIMB_MPS = 0.25
LOITER_ENTRY_CONFIRM_SEC = 1.2
LOITER_ENTRY_CONFIRM_TIMEOUT_SEC = 3.0
LOITER_ENTRY_ABORT_TILT_DEG = 25.0
LOITER_GUARD_SEC = 2.0
LOITER_GUARD_LAND_TILT_DEG = 30.0
LOITER_GUARD_KILL_TILT_DEG = 55.0
MIN_LOITER_ENTRY_AGL = 0.45

# --- ARCHIVE DATA FUSION ---
ARCHIVE_REFRESH_SEC = 8.0
LFS_POINTER_HEADER = "version https://git-lfs.github.com/spec/v1"
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DATA_ROOT = os.path.join(PROJECT_ROOT, "data")
TELEMETRY_DIR = os.path.join(DATA_ROOT, "telemetry")
EVENTS_DIR = os.path.join(DATA_ROOT, "events")
LIVOX_DIR = os.path.join(DATA_ROOT, "livox")

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
    'fastlio_last_odom_time': 0.0, 'fastlio_dt': 0.0,
    'slam_age': 999.0, 'slam_quality': 0, 'slam_status': 'LOST',
    'slam_dropouts': 0, 'slam_pose_error': 0.0,
    'slam_hz_history': [], 'slam_error_history': [],
    'ros2_active': False, 'ros2_init_time': 0.0, 'ros2_error': '',
    # Real-time fused control state
    'fused_alt': 0.0, 'fused_climb': 0.0, 'fused_conf': 0,
    'fused_weights': {'lidar': 0.0, 'slam': 0.0, 'baro': 1.0},
    'fused_source': 'BARO',
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
    'land_requested': False,  # Queue landing request while autonomous takeoff is active
    'rtl_active': False,  # Return-to-launch active
    'sensor_status_log': [],  # Track sensor health during flight
    'slam_last_warn_time': 0.0,  # Rate-limit SLAM warnings
    # Archive fusion status
    'archive_summary': {},
    'archive_rates': {},
    'archive_hotspots': [],
    'test_rig_mode': False,
    'session_study': {},
    'archive_last_scan': 0.0,
    'archive_last_error': '',
    'ops_todo': [],
}


def low_pass(prev_value, new_value, alpha):
    """Simple first-order low-pass filter."""
    return (alpha * new_value) + ((1.0 - alpha) * prev_value)


def _safe_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _is_lfs_pointer(file_path):
    """Detect git-lfs pointer files so we can warn the operator clearly."""
    try:
        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            first_line = f.readline().strip()
            return first_line == LFS_POINTER_HEADER
    except OSError:
        return False


def _read_lfs_declared_size(file_path):
    try:
        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                if line.startswith("size "):
                    return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        pass
    return 0


def _extract_session_id(file_name):
    match = re.search(r"(\d{8}_\d{6})", file_name)
    return match.group(1) if match else ""


def _candidate_files(pattern, include_root_pattern=None):
    files = []
    files.extend(glob.glob(pattern))
    if include_root_pattern:
        files.extend(glob.glob(include_root_pattern))
    unique_files = sorted(set(files), key=os.path.basename)
    return unique_files


def summarize_telemetry_file(file_path):
    summary = {
        'rows': 0,
        'max_alt': 0.0,
        'min_batt_v': 999.0,
        'max_vib': 0.0,
        'armed_rows': 0,
        'max_tilt': 0.0,
        'max_alt_delta': 0.0,
        'is_lfs': False,
        'lfs_size': 0,
    }

    if _is_lfs_pointer(file_path):
        summary['is_lfs'] = True
        summary['lfs_size'] = _read_lfs_declared_size(file_path)
        return summary

    try:
        with open(file_path, "r", newline="", encoding="utf-8", errors="ignore") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                return summary
            for row in reader:
                summary['rows'] += 1
                if row.get('Armed', '') == 'True':
                    summary['armed_rows'] += 1

                alt = _safe_float(
                    row.get('Alt_m', row.get('Lidar_Alt_m', row.get('Baro_Alt_m', 0.0))),
                    0.0
                )
                summary['max_alt'] = max(summary['max_alt'], alt)
                lidar_alt = _safe_float(row.get('Lidar_Alt_m', 0.0), 0.0)
                if lidar_alt > 0.05:
                    summary['max_alt_delta'] = max(summary['max_alt_delta'], abs(alt - lidar_alt))

                roll = abs(_safe_float(row.get('Roll_deg', 0.0), 0.0))
                pitch = abs(_safe_float(row.get('Pitch_deg', 0.0), 0.0))
                summary['max_tilt'] = max(summary['max_tilt'], roll, pitch)

                batt = _safe_float(row.get('Batt_V', row.get('Batt_Voltage', 0.0)), 0.0)
                if batt > 0.0:
                    summary['min_batt_v'] = min(summary['min_batt_v'], batt)

                vib_x = _safe_float(row.get('Vib_X', row.get('Vibration_X', 0.0)), 0.0)
                vib_y = _safe_float(row.get('Vib_Y', row.get('Vibration_Y', 0.0)), 0.0)
                vib_z = _safe_float(row.get('Vib_Z', row.get('Vibration_Z', 0.0)), 0.0)
                summary['max_vib'] = max(summary['max_vib'], vib_x, vib_y, vib_z)

        if summary['min_batt_v'] == 999.0:
            summary['min_batt_v'] = 0.0
    except OSError:
        pass

    return summary


def summarize_event_file(file_path):
    summary = {
        'lines': 0,
        'warn': 0,
        'critical': 0,
        'radio_failsafe': 0,
        'battery_failsafe': 0,
        'mag_anomaly': 0,
        'ekf_aiding_flip': 0,
        'crash_disarm': 0,
        'tilt_critical': 0,
        'loiter_unstable': 0,
        'climb_timeout': 0,
        'touchdown': 0,
        'kill': 0,
        'rtl': 0,
        'land_mode': 0,
        'arm_events': 0,
        'is_lfs': False,
        'lfs_size': 0,
    }

    if _is_lfs_pointer(file_path):
        summary['is_lfs'] = True
        summary['lfs_size'] = _read_lfs_declared_size(file_path)
        return summary

    try:
        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                stripped = line.strip()
                if not stripped:
                    continue
                summary['lines'] += 1
                upper = stripped.upper()
                if "CRITICAL" in upper or "ABORT" in upper or "FAIL" in upper:
                    summary['critical'] += 1
                elif "WARN" in upper:
                    summary['warn'] += 1

                if "RADIO FAILSAFE" in upper:
                    summary['radio_failsafe'] += 1
                if "BATTERY FAILSAFE" in upper or ("FAILSAFE" in upper and "BATTERY" in upper):
                    summary['battery_failsafe'] += 1
                if (
                    ("MAG" in upper and ("ANOMAL" in upper or "INCONSISTENT" in upper or "INTERFERENCE" in upper))
                    or "PREFLIGHT FAIL: COMPASS" in upper
                    or "PREFLIGHT FAIL: MAG" in upper
                ):
                    summary['mag_anomaly'] += 1
                if "EKF" in upper and "AIDING" in upper and ("STOP" in upper or "START" in upper):
                    summary['ekf_aiding_flip'] += 1
                if "CRASH DISARM" in upper or ("DISARM" in upper and "CRASH" in upper):
                    summary['crash_disarm'] += 1
                if "CRITICAL: TILT" in upper or "EXTREME TILT" in upper or "CRITICAL TILT" in upper:
                    summary['tilt_critical'] += 1
                if "LOITER" in upper and "UNSTABLE" in upper:
                    summary['loiter_unstable'] += 1
                if "CLIMB TIMEOUT" in upper:
                    summary['climb_timeout'] += 1
                if "TOUCHDOWN" in upper and "LANDING COMPLETE" in upper:
                    summary['touchdown'] += 1
                if "KILL" in upper:
                    summary['kill'] += 1
                if "RTL" in upper:
                    summary['rtl'] += 1
                if "LAND MODE ENGAGED" in upper or "SWITCHING TO LAND MODE" in upper:
                    summary['land_mode'] += 1
                if "ARM REQUESTED" in upper or "AUTO: ARMED." in upper:
                    summary['arm_events'] += 1
    except OSError:
        pass

    return summary


def compute_archive_rates(summary):
    """Derive compact rates from archive totals for quick risk assessment."""
    telemetry_files = max(1, int(summary.get('telemetry_files', 0)))
    event_files = max(1, int(summary.get('event_files', 0)))
    armed_sessions = int(summary.get('telemetry_sessions_armed', 0))

    rates = {
        'armed_session_rate_pct': round(100.0 * armed_sessions / telemetry_files, 1),
        'target_08_rate_pct': round(100.0 * int(summary.get('telemetry_sessions_target_08', 0)) / telemetry_files, 1),
        'target_10_rate_pct': round(100.0 * int(summary.get('telemetry_sessions_target_10', 0)) / telemetry_files, 1),
        'critical_event_file_rate_pct': round(100.0 * int(summary.get('event_sessions_critical', 0)) / event_files, 1),
        'failsafe_session_rate_pct': round(100.0 * int(summary.get('sessions_with_failsafe', 0)) / event_files, 1),
        'battery_failsafe_session_rate_pct': round(100.0 * int(summary.get('sessions_with_battery_failsafe', 0)) / event_files, 1),
        'mag_anomaly_session_rate_pct': round(100.0 * int(summary.get('sessions_with_mag_anomaly', 0)) / event_files, 1),
        'ekf_aiding_flip_session_rate_pct': round(100.0 * int(summary.get('sessions_with_ekf_aiding_flip', 0)) / event_files, 1),
        'crash_disarm_session_rate_pct': round(100.0 * int(summary.get('sessions_with_crash_disarm', 0)) / event_files, 1),
        'kill_session_rate_pct': round(100.0 * int(summary.get('sessions_with_kill', 0)) / event_files, 1),
        'rig_pattern_session_rate_pct': round(100.0 * int(summary.get('telemetry_sessions_rig_pattern', 0)) / telemetry_files, 1),
        'touchdown_per_armed_pct': round(
            100.0 * int(summary.get('touchdown_total', 0)) / max(1, armed_sessions), 1
        ),
    }
    return rates


def build_failure_hotspots(summary):
    """Rank recurring issue families for dashboard display."""
    hotspots = [
        ("EKF aiding flips", int(summary.get('ekf_aiding_flip_total', 0))),
        ("Radio failsafe", int(summary.get('radio_failsafe_total', 0))),
        ("Kill events", int(summary.get('kill_total', 0))),
        ("Mag anomalies", int(summary.get('mag_anomaly_total', 0))),
        ("Battery failsafe", int(summary.get('battery_failsafe_total', 0))),
        ("Crash disarm", int(summary.get('crash_disarm_total', 0))),
        ("Climb timeout", int(summary.get('climb_timeout_total', 0))),
        ("Tilt critical", int(summary.get('tilt_critical_total', 0))),
        ("LOITER unstable", int(summary.get('loiter_unstable_total', 0))),
        ("RTL events", int(summary.get('rtl_total', 0))),
    ]
    hotspots.sort(key=lambda item: item[1], reverse=True)
    return hotspots


def summarize_livox_imu_file(file_path):
    summary = {'samples': 0, 'max_accel': 0.0, 'is_lfs': False, 'lfs_size': 0}

    if _is_lfs_pointer(file_path):
        summary['is_lfs'] = True
        summary['lfs_size'] = _read_lfs_declared_size(file_path)
        return summary

    try:
        with open(file_path, "r", newline="", encoding="utf-8", errors="ignore") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                return summary
            for row in reader:
                ax = _safe_float(row.get('Accel_X', 0.0), 0.0)
                ay = _safe_float(row.get('Accel_Y', 0.0), 0.0)
                az = _safe_float(row.get('Accel_Z', 0.0), 0.0)
                amag = math.sqrt(ax * ax + ay * ay + az * az)
                summary['samples'] += 1
                summary['max_accel'] = max(summary['max_accel'], amag)
    except OSError:
        pass

    return summary


def summarize_livox_pointcloud_file(file_path):
    summary = {'frames': 0, 'points': 0, 'bytes': 0, 'is_lfs': False, 'lfs_size': 0}

    if _is_lfs_pointer(file_path):
        summary['is_lfs'] = True
        summary['lfs_size'] = _read_lfs_declared_size(file_path)
        return summary

    try:
        summary['bytes'] = os.path.getsize(file_path)
        with open(file_path, "rb") as f:
            if f.read(8) != b'LIVOXPC1':
                return summary

            file_size = summary['bytes']
            while True:
                hdr = f.read(14)
                if len(hdr) < 14:
                    break
                _, point_num, point_step = struct.unpack('<dIH', hdr)

                if point_step == 19:
                    probe = f.read(8)
                    if len(probe) < 8:
                        break
                    candidate_timebase = struct.unpack('<Q', probe)[0]
                    if candidate_timebase < 1_000_000_000:
                        f.seek(-8, os.SEEK_CUR)

                payload_bytes = int(point_num) * int(point_step)
                remaining = file_size - f.tell()
                if payload_bytes < 0 or payload_bytes > remaining:
                    break

                f.seek(payload_bytes, os.SEEK_CUR)
                summary['frames'] += 1
                summary['points'] += int(point_num)
    except OSError:
        pass

    return summary


def collect_archive_summary():
    telemetry_files = _candidate_files(
        os.path.join(TELEMETRY_DIR, 'telemetry_*.csv'),
        os.path.join(PROJECT_ROOT, 'telemetry_*.csv')
    )
    event_files = _candidate_files(
        os.path.join(EVENTS_DIR, 'events_*.txt'),
        os.path.join(PROJECT_ROOT, 'events_*.txt')
    )
    imu_files = _candidate_files(
        os.path.join(LIVOX_DIR, 'livox_imu_*.csv'),
        os.path.join(PROJECT_ROOT, 'livox_imu_*.csv')
    )
    pc_files = _candidate_files(
        os.path.join(LIVOX_DIR, 'livox_pointcloud_*.bin'),
        os.path.join(PROJECT_ROOT, 'livox_pointcloud_*.bin')
    )

    summary = {
        'telemetry_files': len(telemetry_files),
        'event_files': len(event_files),
        'imu_files': len(imu_files),
        'pc_files': len(pc_files),
        'telemetry_rows_total': 0,
        'event_lines_total': 0,
        'event_warn_total': 0,
        'event_critical_total': 0,
        'event_sessions_nonempty': 0,
        'event_sessions_critical': 0,
        'imu_samples_total': 0,
        'pc_frames_total': 0,
        'pc_points_total': 0,
        'max_alt_all': 0.0,
        'max_alt_reasonable': 0.0,
        'max_vib_all': 0.0,
        'min_batt_all': 999.0,
        'max_imu_accel': 0.0,
        'telemetry_sessions_armed': 0,
        'telemetry_sessions_target_08': 0,
        'telemetry_sessions_target_10': 0,
        'telemetry_sessions_outlier_alt': 0,
        'telemetry_sessions_high_tilt': 0,
        'telemetry_sessions_alt_diverge': 0,
        'telemetry_sessions_rig_pattern': 0,
        'radio_failsafe_total': 0,
        'battery_failsafe_total': 0,
        'mag_anomaly_total': 0,
        'ekf_aiding_flip_total': 0,
        'crash_disarm_total': 0,
        'tilt_critical_total': 0,
        'loiter_unstable_total': 0,
        'climb_timeout_total': 0,
        'touchdown_total': 0,
        'kill_total': 0,
        'rtl_total': 0,
        'land_mode_total': 0,
        'arm_events_total': 0,
        'sessions_with_failsafe': 0,
        'sessions_with_battery_failsafe': 0,
        'sessions_with_mag_anomaly': 0,
        'sessions_with_ekf_aiding_flip': 0,
        'sessions_with_crash_disarm': 0,
        'sessions_with_tilt': 0,
        'sessions_with_kill': 0,
        'sessions_with_touchdown': 0,
        'sessions_with_loiter_unstable': 0,
        'lfs_placeholders': 0,
        'lfs_declared_bytes': 0,
        'correlated_sessions': 0,
        'latest_session': '',
        'latest_correlated_session': '',
    }

    tele_sessions = set()
    evt_sessions = set()
    imu_sessions = set()
    pc_sessions = set()

    for fp in telemetry_files:
        ts = summarize_telemetry_file(fp)
        if ts['is_lfs']:
            summary['lfs_placeholders'] += 1
            summary['lfs_declared_bytes'] += ts['lfs_size']
        else:
            summary['telemetry_rows_total'] += ts['rows']
            summary['max_alt_all'] = max(summary['max_alt_all'], ts['max_alt'])
            if ts['max_alt'] <= 15.0:
                summary['max_alt_reasonable'] = max(summary['max_alt_reasonable'], ts['max_alt'])
            else:
                summary['telemetry_sessions_outlier_alt'] += 1
            summary['max_vib_all'] = max(summary['max_vib_all'], ts['max_vib'])
            if ts.get('armed_rows', 0) > 0:
                summary['telemetry_sessions_armed'] += 1
            if ts.get('max_tilt', 0.0) >= 45.0:
                summary['telemetry_sessions_high_tilt'] += 1
            if ts.get('max_alt_delta', 0.0) >= 0.8:
                summary['telemetry_sessions_alt_diverge'] += 1
            if ts.get('max_tilt', 0.0) >= 45.0 and ts.get('max_alt_delta', 0.0) >= 0.8:
                summary['telemetry_sessions_rig_pattern'] += 1
            if ts['max_alt'] >= 0.8:
                summary['telemetry_sessions_target_08'] += 1
            if ts['max_alt'] >= 1.0:
                summary['telemetry_sessions_target_10'] += 1
            if ts['min_batt_v'] > 0.0:
                summary['min_batt_all'] = min(summary['min_batt_all'], ts['min_batt_v'])
        sid = _extract_session_id(os.path.basename(fp))
        if sid:
            tele_sessions.add(sid)

    for fp in event_files:
        es = summarize_event_file(fp)
        if es['is_lfs']:
            summary['lfs_placeholders'] += 1
            summary['lfs_declared_bytes'] += es['lfs_size']
        else:
            summary['event_lines_total'] += es['lines']
            summary['event_warn_total'] += es['warn']
            summary['event_critical_total'] += es['critical']
            if es['lines'] > 0:
                summary['event_sessions_nonempty'] += 1
            if es['critical'] > 0:
                summary['event_sessions_critical'] += 1

            summary['radio_failsafe_total'] += es.get('radio_failsafe', 0)
            summary['battery_failsafe_total'] += es.get('battery_failsafe', 0)
            summary['mag_anomaly_total'] += es.get('mag_anomaly', 0)
            summary['ekf_aiding_flip_total'] += es.get('ekf_aiding_flip', 0)
            summary['crash_disarm_total'] += es.get('crash_disarm', 0)
            summary['tilt_critical_total'] += es.get('tilt_critical', 0)
            summary['loiter_unstable_total'] += es.get('loiter_unstable', 0)
            summary['climb_timeout_total'] += es.get('climb_timeout', 0)
            summary['touchdown_total'] += es.get('touchdown', 0)
            summary['kill_total'] += es.get('kill', 0)
            summary['rtl_total'] += es.get('rtl', 0)
            summary['land_mode_total'] += es.get('land_mode', 0)
            summary['arm_events_total'] += es.get('arm_events', 0)

            if es.get('radio_failsafe', 0) > 0:
                summary['sessions_with_failsafe'] += 1
            if es.get('battery_failsafe', 0) > 0:
                summary['sessions_with_battery_failsafe'] += 1
            if es.get('mag_anomaly', 0) > 0:
                summary['sessions_with_mag_anomaly'] += 1
            if es.get('ekf_aiding_flip', 0) > 0:
                summary['sessions_with_ekf_aiding_flip'] += 1
            if es.get('crash_disarm', 0) > 0:
                summary['sessions_with_crash_disarm'] += 1
            if es.get('tilt_critical', 0) > 0:
                summary['sessions_with_tilt'] += 1
            if es.get('kill', 0) > 0:
                summary['sessions_with_kill'] += 1
            if es.get('touchdown', 0) > 0:
                summary['sessions_with_touchdown'] += 1
            if es.get('loiter_unstable', 0) > 0:
                summary['sessions_with_loiter_unstable'] += 1
        sid = _extract_session_id(os.path.basename(fp))
        if sid:
            evt_sessions.add(sid)

    for fp in imu_files:
        ims = summarize_livox_imu_file(fp)
        if ims['is_lfs']:
            summary['lfs_placeholders'] += 1
            summary['lfs_declared_bytes'] += ims['lfs_size']
        else:
            summary['imu_samples_total'] += ims['samples']
            summary['max_imu_accel'] = max(summary['max_imu_accel'], ims['max_accel'])
        sid = _extract_session_id(os.path.basename(fp))
        if sid:
            imu_sessions.add(sid)

    for fp in pc_files:
        pcs = summarize_livox_pointcloud_file(fp)
        if pcs['is_lfs']:
            summary['lfs_placeholders'] += 1
            summary['lfs_declared_bytes'] += pcs['lfs_size']
        else:
            summary['pc_frames_total'] += pcs['frames']
            summary['pc_points_total'] += pcs['points']
        sid = _extract_session_id(os.path.basename(fp))
        if sid:
            pc_sessions.add(sid)

    if summary['min_batt_all'] == 999.0:
        summary['min_batt_all'] = 0.0

    shared = tele_sessions & evt_sessions & imu_sessions & pc_sessions
    summary['correlated_sessions'] = len(shared)
    if shared:
        summary['latest_correlated_session'] = sorted(shared)[-1]

    all_sessions = sorted(tele_sessions | evt_sessions | imu_sessions | pc_sessions)
    if all_sessions:
        summary['latest_session'] = all_sessions[-1]

    return summary


def collect_session_bundle(session_id):
    """Collect file paths for a specific session id across all sources."""
    if not session_id:
        return {}

    return {
        'telemetry': _candidate_files(
            os.path.join(TELEMETRY_DIR, f'telemetry_{session_id}.csv'),
            os.path.join(PROJECT_ROOT, f'telemetry_{session_id}.csv')
        ),
        'events': _candidate_files(
            os.path.join(EVENTS_DIR, f'events_{session_id}.txt'),
            os.path.join(PROJECT_ROOT, f'events_{session_id}.txt')
        ),
        'imu': _candidate_files(
            os.path.join(LIVOX_DIR, f'livox_imu_{session_id}.csv'),
            os.path.join(PROJECT_ROOT, f'livox_imu_{session_id}.csv')
        ),
        'pointcloud': _candidate_files(
            os.path.join(LIVOX_DIR, f'livox_pointcloud_{session_id}.bin'),
            os.path.join(PROJECT_ROOT, f'livox_pointcloud_{session_id}.bin')
        )
    }


def summarize_latest_session(session_id):
    """Build a compact study summary for the latest correlated session."""
    study = {
        'session_id': session_id,
        'duration_s': 0.0,
        'rows': 0,
        'event_lines': 0,
        'event_warn': 0,
        'event_critical': 0,
        'imu_samples': 0,
        'imu_peak_accel': 0.0,
        'pc_frames': 0,
        'pc_points': 0,
        'max_alt': 0.0,
        'min_batt_v': 0.0,
        'max_vib': 0.0,
        'avg_flow_qual': 0.0,
        'avg_fastlio_hz': 0.0,
        'flight_mode_last': 'N/A',
    }

    bundle = collect_session_bundle(session_id)
    tele = bundle.get('telemetry', [])
    evt = bundle.get('events', [])
    imu = bundle.get('imu', [])
    pc = bundle.get('pointcloud', [])

    if tele:
        fp = tele[-1]
        ts = summarize_telemetry_file(fp)
        study['rows'] = ts.get('rows', 0)
        study['max_alt'] = ts.get('max_alt', 0.0)
        study['min_batt_v'] = ts.get('min_batt_v', 0.0)
        study['max_vib'] = ts.get('max_vib', 0.0)

        if not ts.get('is_lfs', False):
            first_secs = None
            last_secs = None
            flow_sum = 0.0
            flow_count = 0
            hz_sum = 0.0
            hz_count = 0
            last_mode = 'N/A'
            try:
                with open(fp, 'r', newline='', encoding='utf-8', errors='ignore') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        ts_text = row.get('Timestamp', '')
                        try:
                            t = datetime.strptime(ts_text, "%H:%M:%S.%f")
                            sec = t.hour * 3600.0 + t.minute * 60.0 + t.second + t.microsecond / 1e6
                            if first_secs is None:
                                first_secs = sec
                            last_secs = sec
                        except (ValueError, TypeError):
                            pass

                        fq = _safe_float(row.get('OptFlow_Qual', 0.0), 0.0)
                        flow_sum += fq
                        flow_count += 1

                        hz = _safe_float(row.get('FastLIO_Hz', 0.0), 0.0)
                        hz_sum += hz
                        hz_count += 1

                        mode = row.get('Flight_Mode', '').strip()
                        if mode:
                            last_mode = mode

                if first_secs is not None and last_secs is not None:
                    if last_secs >= first_secs:
                        study['duration_s'] = round(last_secs - first_secs, 1)
                    else:
                        study['duration_s'] = 0.0
                if flow_count > 0:
                    study['avg_flow_qual'] = round(flow_sum / flow_count, 1)
                if hz_count > 0:
                    study['avg_fastlio_hz'] = round(hz_sum / hz_count, 1)
                study['flight_mode_last'] = last_mode
            except OSError:
                pass

    if evt:
        es = summarize_event_file(evt[-1])
        study['event_lines'] = es.get('lines', 0)
        study['event_warn'] = es.get('warn', 0)
        study['event_critical'] = es.get('critical', 0)

    if imu:
        ims = summarize_livox_imu_file(imu[-1])
        study['imu_samples'] = ims.get('samples', 0)
        study['imu_peak_accel'] = ims.get('max_accel', 0.0)

    if pc:
        pcs = summarize_livox_pointcloud_file(pc[-1])
        study['pc_frames'] = pcs.get('frames', 0)
        study['pc_points'] = pcs.get('points', 0)

    return study


def collect_correlated_session_ids():
    """Return sorted session ids that have telemetry + events + Livox IMU + pointcloud."""
    tele = {
        _extract_session_id(os.path.basename(fp))
        for fp in _candidate_files(
            os.path.join(TELEMETRY_DIR, 'telemetry_*.csv'),
            os.path.join(PROJECT_ROOT, 'telemetry_*.csv')
        )
    }
    evt = {
        _extract_session_id(os.path.basename(fp))
        for fp in _candidate_files(
            os.path.join(EVENTS_DIR, 'events_*.txt'),
            os.path.join(PROJECT_ROOT, 'events_*.txt')
        )
    }
    imu = {
        _extract_session_id(os.path.basename(fp))
        for fp in _candidate_files(
            os.path.join(LIVOX_DIR, 'livox_imu_*.csv'),
            os.path.join(PROJECT_ROOT, 'livox_imu_*.csv')
        )
    }
    pc = {
        _extract_session_id(os.path.basename(fp))
        for fp in _candidate_files(
            os.path.join(LIVOX_DIR, 'livox_pointcloud_*.bin'),
            os.path.join(PROJECT_ROOT, 'livox_pointcloud_*.bin')
        )
    }

    shared = (tele & evt & imu & pc)
    shared.discard('')
    return sorted(shared)


def read_event_tail(session_id, max_lines=6):
    """Read final significant lines for a session event file."""
    bundle = collect_session_bundle(session_id)
    event_files = bundle.get('events', [])
    if not event_files:
        return []

    fp = event_files[-1]
    if _is_lfs_pointer(fp):
        return ["Event file is still an LFS pointer"]

    lines = []
    try:
        with open(fp, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                s = line.strip()
                if s:
                    lines.append(s)
    except OSError:
        return ["Failed to read event file"]

    return lines[-max_lines:]


def build_session_sparklines(session_id):
    """Create compact trend sparklines for a session telemetry file."""
    out = {
        'alt': "n/a",
        'batt': "n/a",
        'vib': "n/a",
        'flow': "n/a",
        'slam_hz': "n/a",
    }

    bundle = collect_session_bundle(session_id)
    tele = bundle.get('telemetry', [])
    if not tele:
        return out

    fp = tele[-1]
    if _is_lfs_pointer(fp):
        return out

    alt_hist = []
    batt_hist = []
    vib_hist = []
    flow_hist = []
    slam_hz_hist = []

    try:
        with open(fp, 'r', newline='', encoding='utf-8', errors='ignore') as f:
            reader = csv.DictReader(f)
            for row in reader:
                alt_hist.append(_safe_float(row.get('Lidar_Alt_m', row.get('Alt_m', 0.0)), 0.0))
                batt_hist.append(_safe_float(row.get('Batt_V', 0.0), 0.0))

                vx = _safe_float(row.get('Vib_X', 0.0), 0.0)
                vy = _safe_float(row.get('Vib_Y', 0.0), 0.0)
                vz = _safe_float(row.get('Vib_Z', 0.0), 0.0)
                vib_hist.append(max(vx, vy, vz))

                fq = _safe_float(row.get('OptFlow_Qual', 0.0), 0.0)
                flow_hist.append(fq)

                slam_hz_hist.append(_safe_float(row.get('FastLIO_Hz', 0.0), 0.0))
    except OSError:
        return out

    if alt_hist:
        out['alt'] = create_sparkline(alt_hist[-80:], width=24)
    if batt_hist:
        out['batt'] = create_sparkline(batt_hist[-80:], width=24)
    if vib_hist:
        out['vib'] = create_sparkline(vib_hist[-80:], width=24)
    if flow_hist:
        out['flow'] = create_sparkline(flow_hist[-80:], width=24)
    if slam_hz_hist:
        out['slam_hz'] = create_sparkline(slam_hz_hist[-80:], width=24)

    return out


def build_ops_todo(summary):
    todos = []
    rates = compute_archive_rates(summary)

    if summary.get('lfs_placeholders', 0) > 0:
        todos.append("Fetch Git LFS data to unlock archived telemetry/events/Livox logs")

    if summary.get('telemetry_sessions_outlier_alt', 0) > 0:
        todos.append(
            f"Validate altitude outliers in {summary.get('telemetry_sessions_outlier_alt', 0)} telemetry sessions (sensor spike filtering)"
        )

    if summary.get('telemetry_sessions_rig_pattern', 0) > 0:
        todos.append(
            f"RIG signature detected in {summary.get('telemetry_sessions_rig_pattern', 0)} sessions (high tilt + baro/lidar divergence); tag these as constrained tests"
        )

    if summary.get('telemetry_sessions_high_tilt', 0) > 0:
        todos.append(
            f"Study high-tilt episodes ({summary.get('telemetry_sessions_high_tilt', 0)} sessions): align rope tension timing with attitude spikes"
        )

    if summary.get('telemetry_sessions_alt_diverge', 0) > 0:
        todos.append(
            f"Investigate baro vs lidar divergence in {summary.get('telemetry_sessions_alt_diverge', 0)} sessions (ground effect + tether bias)"
        )

    if summary.get('correlated_sessions', 0) == 0:
        todos.append("Record one full session containing telemetry + events + Livox + IMU")

    if summary.get('event_critical_total', 0) > 0:
        todos.append(
            f"Review {summary['event_critical_total']} critical/abort events ({rates['critical_event_file_rate_pct']:.1f}% event-file incidence)"
        )

    if summary.get('sessions_with_failsafe', 0) > 0:
        todos.append(
            f"Reduce radio failsafe episodes: {summary.get('radio_failsafe_total', 0)} detections across {summary.get('sessions_with_failsafe', 0)} sessions"
        )

    if summary.get('sessions_with_battery_failsafe', 0) > 0:
        todos.append(
            f"Battery failsafe seen in {summary.get('sessions_with_battery_failsafe', 0)} sessions ({summary.get('battery_failsafe_total', 0)} events): verify voltage sag and failsafe thresholds"
        )

    if summary.get('sessions_with_mag_anomaly', 0) > 0:
        todos.append(
            f"Mag anomalies in {summary.get('sessions_with_mag_anomaly', 0)} sessions ({summary.get('mag_anomaly_total', 0)} events): recalibrate compass and review magnetic interference near rig"
        )

    if summary.get('sessions_with_ekf_aiding_flip', 0) > 0:
        todos.append(
            f"EKF aiding flips in {summary.get('sessions_with_ekf_aiding_flip', 0)} sessions ({summary.get('ekf_aiding_flip_total', 0)} events): stabilize sensor aiding transitions before LOITER"
        )

    if summary.get('sessions_with_crash_disarm', 0) > 0:
        todos.append(
            f"Crash-disarm detected in {summary.get('sessions_with_crash_disarm', 0)} sessions ({summary.get('crash_disarm_total', 0)} events): align kill policy and post-crash motor inhibit"
        )

    if summary.get('sessions_with_kill', 0) > 0:
        todos.append(
            f"Audit kill-path triggers: {summary.get('kill_total', 0)} kill events across {summary.get('sessions_with_kill', 0)} sessions"
        )

    if summary.get('sessions_with_tilt', 0) > 0:
        todos.append(
            f"Tune climb/transition attitude safety: tilt-critical appeared in {summary.get('sessions_with_tilt', 0)} sessions"
        )

    if summary.get('climb_timeout_total', 0) > 0:
        todos.append(
            f"Reduce climb timeout count ({summary.get('climb_timeout_total', 0)} total): adjust ascent profile and climb-response gating"
        )

    if summary.get('loiter_unstable_total', 0) > 0:
        todos.append(
            f"Track LOITER-entry instability ({summary.get('loiter_unstable_total', 0)} events) and tighten pre-entry stabilization"
        )

    armed_sessions = summary.get('telemetry_sessions_armed', 0)
    if armed_sessions == 0:
        todos.append("No armed telemetry sessions detected. Resolve arming/failsafe path first")
    else:
        todos.append(
            f"Mission completion proxy (touchdown per armed): {rates['touchdown_per_armed_pct']:.1f}% ({summary.get('touchdown_total', 0)}/{armed_sessions})"
        )
        todos.append(
            f"1m readiness: sessions reaching >=0.8m = {summary.get('telemetry_sessions_target_08', 0)} ({rates['target_08_rate_pct']:.1f}%)"
        )
        todos.append(
            f"1m strict hit-rate: sessions reaching >=1.0m = {summary.get('telemetry_sessions_target_10', 0)} ({rates['target_10_rate_pct']:.1f}%)"
        )

    if summary.get('max_vib_all', 0.0) > MAX_GROUND_VIBE:
        todos.append(f"Investigate high vibration archive peaks ({summary['max_vib_all']:.1f} m/s^2)")

    if summary.get('min_batt_all', 99.0) > 0.0 and summary.get('min_batt_all', 99.0) < MIN_SAFE_VOLTAGE:
        todos.append(f"Archive shows low-voltage floor {summary['min_batt_all']:.2f}V; retune power budget")

    if summary.get('imu_samples_total', 0) == 0:
        todos.append("Verify /livox/imu stream and IMU CSV logging")

    if state['ros2_active'] and state.get('slam_quality', 0) < 50:
        todos.append(
            f"SLAM quality low ({state.get('slam_quality', 0)}%). Check /Odometry and /cloud_registered health"
        )

    if state['ros2_active'] and state.get('slam_dropouts', 0) > 20:
        todos.append("Reduce SLAM dropouts: check ROS2 CPU load/QoS/network timing")

    if state['flow_qual'] < MIN_PREFLIGHT_FLOW_QUAL:
        todos.append(f"Improve optical-flow quality to >={MIN_PREFLIGHT_FLOW_QUAL} before AUTO TAKEOFF")

    if not state['rangefinder_healthy']:
        todos.append("Rangefinder unhealthy; fix lidar/range stream before autonomy")

    todos.append(
        "Run next validation sequence: 5 consecutive 1m autonomous flights with no manual intervention and compare completion metrics"
    )

    todos.append(
        "Run rope-rig protocol: three 1m flights with controlled rope restraint at hover, then compare drift/tilt signatures against free-flight baseline"
    )

    todos.append(
        "For each rope-rig run, annotate operator action timestamps (rope pull/release, kill press) in event notes for supervised model labeling"
    )

    todos.append(
        "Export diagnostics (`e`) and TODO (`y`) after each run to keep trend history and verify whether fixes improve rates"
    )

    if not todos:
        todos.append("No high-priority blockers detected")

    return todos[:16]


def refresh_archive_summary(force=False):
    now = time.time()
    if not force and (now - state['archive_last_scan'] < ARCHIVE_REFRESH_SEC):
        return

    try:
        summary = collect_archive_summary()
        state['archive_summary'] = summary
        state['archive_rates'] = compute_archive_rates(summary)
        state['archive_hotspots'] = build_failure_hotspots(summary)
        latest_corr = summary.get('latest_correlated_session', '')
        state['session_study'] = summarize_latest_session(latest_corr) if latest_corr else {}
        state['ops_todo'] = build_ops_todo(summary)
        state['archive_last_error'] = ''
    except Exception as e:
        state['archive_last_error'] = str(e)[:120]
    finally:
        state['archive_last_scan'] = now


def export_ops_todo():
    """Save a focused TODO snapshot for offline follow-up."""
    out_path = f"ops_todo_{session_id}.txt"
    summary = state.get('archive_summary', {})
    rates = state.get('archive_rates', {})
    hotspots = state.get('archive_hotspots', [])
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("ASCEND Ops TODO Snapshot\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Session: {session_id}\n\n")
        f.write("Archive Summary\n")
        f.write(f"- Telemetry files: {summary.get('telemetry_files', 0)}\n")
        f.write(f"- Event files: {summary.get('event_files', 0)}\n")
        f.write(f"- Livox IMU files: {summary.get('imu_files', 0)}\n")
        f.write(f"- Livox PC files: {summary.get('pc_files', 0)}\n")
        f.write(f"- Correlated sessions: {summary.get('correlated_sessions', 0)}\n")
        f.write(f"- LFS placeholders: {summary.get('lfs_placeholders', 0)}\n\n")
        f.write("Archive Rates\n")
        f.write(f"- Armed session rate: {rates.get('armed_session_rate_pct', 0.0)}%\n")
        f.write(f"- >=0.8m session rate: {rates.get('target_08_rate_pct', 0.0)}%\n")
        f.write(f"- >=1.0m session rate: {rates.get('target_10_rate_pct', 0.0)}%\n")
        f.write(f"- Critical event-file rate: {rates.get('critical_event_file_rate_pct', 0.0)}%\n")
        f.write(f"- Failsafe session rate: {rates.get('failsafe_session_rate_pct', 0.0)}%\n")
        f.write(f"- Battery failsafe session rate: {rates.get('battery_failsafe_session_rate_pct', 0.0)}%\n")
        f.write(f"- Mag anomaly session rate: {rates.get('mag_anomaly_session_rate_pct', 0.0)}%\n")
        f.write(f"- EKF aiding flip session rate: {rates.get('ekf_aiding_flip_session_rate_pct', 0.0)}%\n")
        f.write(f"- Crash disarm session rate: {rates.get('crash_disarm_session_rate_pct', 0.0)}%\n")
        f.write(f"- Touchdown per armed: {rates.get('touchdown_per_armed_pct', 0.0)}%\n\n")
        f.write("Failure Hotspots\n")
        for label, count in hotspots[:6]:
            f.write(f"- {label}: {count}\n")
        f.write("\n")
        f.write("TODO\n")
        for idx, item in enumerate(state.get('ops_todo', []), start=1):
            f.write(f"{idx}. {item}\n")
    return out_path

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
    try:
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            int(state['rc_roll']), int(state['rc_pitch']), int(state['rc_throttle']), int(state['rc_yaw']),
            0, 0, 0, 0
        )
    except Exception as e:
        update_log(f"WARN: RC override send error: {e}")


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


def get_fused_altitude_for_control():
    """Weighted fusion of lidar, SLAM, and baro altitude for robust supervision."""
    now = time.time()
    lidar_ok = state['rangefinder_healthy'] and state['lidar_alt'] > 0
    slam_ok = (
        state['fastlio_healthy']
        and state.get('slam_quality', 0) >= 40
        and state.get('slam_age', 999.0) < FASTLIO_TIMEOUT
    )
    baro_ok = state['alt'] != 0.0

    w_lidar = 0.0
    w_slam = 0.0
    w_baro = 0.0

    if lidar_ok:
        lidar_age = now - state.get('lidar_last_update', 0.0) if state.get('lidar_last_update', 0.0) > 0 else 999.0
        lidar_fresh = 1.0 if lidar_age < 0.3 else 0.6 if lidar_age < 0.7 else 0.2
        w_lidar = 0.65 * lidar_fresh

    if slam_ok:
        w_slam = 0.25 * (state.get('slam_quality', 0) / 100.0)

    if baro_ok:
        w_baro = 0.10

    # Barometer can drift heavily near ground test benches; downweight hard outliers.
    if lidar_ok and baro_ok and abs(state['lidar_alt'] - state['alt']) > 3.0:
        w_baro *= 0.05

    # Reduce trust on sensors that disagree heavily.
    if lidar_ok and slam_ok:
        disagreement = abs(state['lidar_alt'] - state['fastlio_z'])
        if disagreement > MAX_ALT_SENSOR_DISAGREE:
            if state.get('slam_quality', 0) >= 70:
                w_lidar *= 0.5
            else:
                w_slam *= 0.4
        if disagreement > 2.0:
            # Extreme mismatch: keep SLAM as secondary guard rail, not primary altitude source.
            w_slam *= 0.3

    total = w_lidar + w_slam + w_baro
    if total <= 1e-6:
        fused = get_best_altitude_for_control()
        state['fused_weights'] = {'lidar': 0.0, 'slam': 0.0, 'baro': 1.0}
        state['fused_conf'] = 20
        state['fused_source'] = 'BARO'
    else:
        wl = w_lidar / total
        ws = w_slam / total
        wb = w_baro / total
        fused = (
            wl * state['lidar_alt'] +
            ws * state['fastlio_z'] +
            wb * state['alt']
        )
        state['fused_weights'] = {'lidar': round(wl, 3), 'slam': round(ws, 3), 'baro': round(wb, 3)}
        state['fused_conf'] = int(min(100, (wl * 100) + (ws * state.get('slam_quality', 0)) + (wb * 50)))

        if wl >= ws and wl >= wb:
            state['fused_source'] = 'LIDAR'
        elif ws >= wb:
            state['fused_source'] = 'SLAM'
        else:
            state['fused_source'] = 'BARO'

    # Blend for smoother control handoff.
    prev = state.get('fused_alt', 0.0)
    if prev <= 0.0:
        state['fused_alt'] = round(fused, 3)
    else:
        state['fused_alt'] = round(low_pass(prev, fused, 0.45), 3)

    return state['fused_alt']


def get_fused_climb_rate():
    """Fuse FC climb with fused-altitude slope for smoother vertical decisions."""
    if len(state['alt_history']) >= 8:
        slope = get_altitude_slope()
    else:
        slope = state.get('climb', 0.0)

    # Blend MAVLink climb and estimator slope.
    fused_climb = (0.6 * state.get('climb', 0.0)) + (0.4 * slope)
    state['fused_climb'] = round(fused_climb, 3)
    return state['fused_climb']


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
                prev_odom_time = state['fastlio_last_odom_time']
                if prev_odom_time > 0:
                    dt = now - prev_odom_time
                    state['fastlio_dt'] = dt
                    if dt > 0.35:
                        state['slam_dropouts'] += 1
                state['fastlio_last_odom_time'] = now
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
        state['slam_age'] = age
        state['fastlio_healthy'] = age < FASTLIO_TIMEOUT
    else:
        state['slam_age'] = 999.0
        state['fastlio_healthy'] = False


def update_slam_metrics():
    """Compute SLAM quality/status metrics for robust operator visibility."""
    now = time.time()
    age = now - state['fastlio_last_time'] if state['fastlio_last_time'] > 0 else 999.0
    state['slam_age'] = age

    hz = float(state.get('fastlio_hz', 0.0))
    state['slam_hz_history'].append(hz)
    if len(state['slam_hz_history']) > 60:
        state['slam_hz_history'].pop(0)

    if state['rangefinder_healthy']:
        pose_error = abs(state['fastlio_z'] - state['lidar_alt'])
    else:
        pose_error = 0.0
    state['slam_pose_error'] = pose_error

    state['slam_error_history'].append(pose_error)
    if len(state['slam_error_history']) > 60:
        state['slam_error_history'].pop(0)

    quality = 0
    if state['ros2_active']:
        quality += 10

    if age < FASTLIO_TIMEOUT:
        quality += 30

    if hz >= FASTLIO_MIN_HEALTHY_HZ:
        quality += 20
    elif hz > 0:
        quality += int((hz / FASTLIO_MIN_HEALTHY_HZ) * 20)

    points = max(int(state.get('fastlio_points', 0)), int(state.get('livox_pc_last_pts', 0)))
    if points > 0:
        quality += min(20, int(points / 1500))

    if state['rangefinder_healthy']:
        if pose_error < 0.25:
            quality += 20
        elif pose_error < SLAM_MAX_ALT_DIFF:
            quality += 10
        else:
            quality -= 20

    if age > FASTLIO_STALE_WARN_SEC:
        quality -= 15

    if state.get('slam_dropouts', 0) > 10:
        quality -= 10

    quality = max(0, min(100, int(quality)))
    state['slam_quality'] = quality

    if not state['ros2_active'] or age >= FASTLIO_TIMEOUT:
        state['slam_status'] = 'LOST'
    elif quality >= 75:
        state['slam_status'] = 'GOOD'
    elif quality >= 45:
        state['slam_status'] = 'DEGRADED'
    else:
        state['slam_status'] = 'WEAK'

    if USE_FASTLIO_FOR_CONTROL and state['armed'] and age > FASTLIO_STALE_WARN_SEC:
        if now - state['slam_last_warn_time'] > SLAM_WARN_INTERVAL:
            update_log(f"⚠ WARNING: SLAM stale ({age:.1f}s)")
            state['slam_last_warn_time'] = now


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
    current_best_alt = get_fused_altitude_for_control() if USE_MULTISENSOR_CONTROL else (
        state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
    )
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
        confidence = int(state.get('slam_quality', 0))
    
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
        update_log("DIAG: FAIL - Rangefinder not healthy. Required for optical-flow-only flight.")
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

    fused_alt = get_fused_altitude_for_control()
    update_log(
        "DIAG: Control fusion active "
        f"(src={state.get('fused_source', 'N/A')}, conf={state.get('fused_conf', 0)}%, alt={fused_alt:.2f}m)"
    )
        
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
    state['land_requested'] = False
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
    ground_alt = get_fused_altitude_for_control() if USE_MULTISENSOR_CONTROL else (
        state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
    )
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
    # Keep climb commands conservative; aggressive throttle correlated with tilt spikes.
    CLIMB_THR = 1660       # Reduced from 1700 to lower pitch-up impulse
    APPROACH_THR = 1560    # Gentler climb for final 30cm
    FINE_THR = 1525        # Very gentle for last 10cm
    CLIMB_TIMEOUT = 15.0 + target_alt * 5.0  # More time for higher altitudes
    
    climb_start = time.time()
    
    # Ramp from spool throttle to full climb throttle
    for thr in range(SPOOL_THR, CLIMB_THR, max(2, CLIMB_RAMP_RATE)):
        state['rc_throttle'] = thr
        state['rc_pitch'] = 1500
        state['rc_roll'] = 1500
        state['rc_yaw'] = 1500
        send_rc_override(master)
        time.sleep(0.03)
    
    update_log("AUTO: Full climb power. Monitoring altitude...")
    last_alt_log = time.time()
    no_climb_since = None
    tilt_unstable_since = None
    
    climb_exit_reason = 'timeout'

    while time.time() - climb_start < CLIMB_TIMEOUT:
        if not state['armed']:
            climb_exit_reason = 'disarmed'
            break

        current_alt = get_fused_altitude_for_control() if USE_MULTISENSOR_CONTROL else get_best_altitude_for_control()
        fused_climb = get_fused_climb_rate()
        alt_remaining = abs_target - current_alt
        
        # Staged throttle: reduce as we approach target and damp with climb-rate feedback.
        if alt_remaining < 0.10:
            state['rc_throttle'] = FINE_THR
        elif alt_remaining < 0.30:
            state['rc_throttle'] = APPROACH_THR if fused_climb < 0.22 else 1530
        else:
            state['rc_throttle'] = CLIMB_THR if fused_climb < 0.15 else 1600

        # Attitude-aware derating: reduce climb authority when tilt starts diverging.
        climb_tilt = max(abs(state['roll']), abs(state['pitch']))
        if climb_tilt > CLIMB_TILT_DERATE_START_DEG:
            if climb_tilt >= CLIMB_TILT_DERATE_HARD_DEG:
                state['rc_throttle'] = min(state['rc_throttle'], 1520)
            else:
                state['rc_throttle'] = min(state['rc_throttle'], 1560)

        # If tilt remains high during climb, hand over to LAND before it escalates.
        if climb_tilt > CLIMB_TILT_LAND_DEG:
            if tilt_unstable_since is None:
                tilt_unstable_since = time.time()
            elif time.time() - tilt_unstable_since > CLIMB_TILT_LAND_HOLD_SEC:
                update_log(
                    f"AUTO: Climb unstable (tilt {climb_tilt:.1f}deg). Switching to LAND for safety."
                )
                state['macro_status'] = 'HOVER'
                auto_land_sequence(master)
                return
        else:
            tilt_unstable_since = None

        # Dynamic confidence adjustment only when multisensor fusion is active.
        if USE_MULTISENSOR_CONTROL and state.get('fused_conf', 0) < 45:
            state['rc_throttle'] = min(state['rc_throttle'], 1600)

        # Livox obstacle gate during ascent.
        if state['livox_obs'] > 0 and state['livox_obs'] < LIVOX_MIN_SAFE_DIST:
            update_log(f"AUTO: ABORT! Livox obstacle {state['livox_obs']:.2f}m during takeoff")
            auto_land_sequence(master)
            return
        
        state['rc_pitch'] = 1500
        state['rc_roll'] = 1500
        state['rc_yaw'] = 1500
        send_rc_override(master)
        
        # Check if target reached within practical test margin.
        if current_alt >= abs_target - TAKEOFF_REACH_MARGIN:
            climb_exit_reason = 'reached'
            update_log(f"AUTO: Target reached at {current_alt:.2f}m")
            break

        if state.get('land_requested'):
            climb_exit_reason = 'land_requested'
            update_log("AUTO: Landing requested during takeoff. Exiting climb phase.")
            break

        # Abort if FC is not responding to climb command despite enough margin.
        if alt_remaining > 0.25 and fused_climb < MIN_TAKEOFF_CLIMB_RATE:
            if no_climb_since is None:
                no_climb_since = time.time()
            elif time.time() - no_climb_since > NO_CLIMB_ABORT_SEC:
                update_log(
                    f"AUTO: ABORT! No climb response for {NO_CLIMB_ABORT_SEC:.1f}s "
                    f"(climb={fused_climb:.2f}m/s)."
                )
                emergency_disarm(master)
                return
        else:
            no_climb_since = None
        
        # Periodic altitude log
        if time.time() - last_alt_log > 1.0:
            update_log(
                f"AUTO: Climbing... {current_alt:.2f}m / {abs_target:.2f}m "
                f"(remaining: {alt_remaining:.2f}m, climb: {fused_climb:.2f}m/s, tilt:{climb_tilt:.1f}deg, "
                f"thr:{int(state['rc_throttle'])}, conf:{state.get('fused_conf', 0)}%)"
            )
            last_alt_log = time.time()
        
        time.sleep(0.05)
        
        # Safety: tilt check
        if abs(state['roll']) > CLIMB_TILT_KILL_DEG or abs(state['pitch']) > CLIMB_TILT_KILL_DEG:
            update_log("AUTO: CRITICAL TILT during climb! KILL.")
            emergency_disarm(master)
            return
        # Safety: vibration check
        if max(state['vibration_filtered']) > MAX_VIBRATION:
            update_log(f"AUTO: FATAL VIB during climb ({max(state['vibration_filtered']):.1f})! KILL.")
            emergency_disarm(master)
            return
    current_alt = get_fused_altitude_for_control() if USE_MULTISENSOR_CONTROL else (
        state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
    )

    if climb_exit_reason == 'disarmed':
        update_log(f"AUTO: ABORT! FC disarmed during climb at {current_alt:.2f}m.")
        state['macro_status'] = 'IDLE'
        return
    if climb_exit_reason == 'land_requested':
        current_agl = current_alt - ground_alt
        if current_agl < MIN_LOITER_ENTRY_AGL:
            update_log(
                f"AUTO: Landing requested but altitude too low ({current_agl:.2f}m AGL). Keeping FC control, then landing."
            )
        state['macro_status'] = 'HOVER'
        auto_land_sequence(master)
        return
    if climb_exit_reason == 'timeout':
        update_log(f"AUTO: Climb timeout at {current_alt:.2f}m. Proceeding to hold/land checks.")

    current_agl = current_alt - ground_alt
    if current_agl < MIN_LOITER_ENTRY_AGL:
        update_log(
            f"AUTO: ABORT! Altitude too low for LOITER hold ({current_agl:.2f}m AGL). Landing."
        )
        state['macro_status'] = 'HOVER'  # Allow auto_land_sequence to run
        auto_land_sequence(master)
        return

    if not state['armed']:
        state['macro_status'] = 'IDLE'
        return

    # 8. Pre-LOITER stabilization in ALT_HOLD.
    # Require a short continuous stable window before switching modes.
    update_log("AUTO: Stabilizing before LOITER...")
    stable_since = None
    settle_start = time.time()
    while state['armed'] and state['macro_status'] == 'TAKEOFF':
        elapsed = time.time() - settle_start
        if elapsed > PRE_LOITER_TIMEOUT_SEC:
            update_log("AUTO: Pre-LOITER stabilization timeout. Landing for safety.")
            state['macro_status'] = 'HOVER'
            auto_land_sequence(master)
            return

        if state.get('land_requested'):
            update_log("AUTO: Landing requested during pre-LOITER stabilization.")
            state['macro_status'] = 'HOVER'
            auto_land_sequence(master)
            return

        current_alt = get_fused_altitude_for_control() if USE_MULTISENSOR_CONTROL else get_best_altitude_for_control()
        current_agl = current_alt - ground_alt
        fused_climb = abs(get_fused_climb_rate())
        tilt_ok = (
            abs(state['roll']) <= PRE_LOITER_STABLE_TILT_DEG and
            abs(state['pitch']) <= PRE_LOITER_STABLE_TILT_DEG
        )
        climb_ok = fused_climb <= PRE_LOITER_STABLE_CLIMB_MPS
        agl_ok = current_agl >= MIN_LOITER_ENTRY_AGL

        state['rc_throttle'] = HOVER_THR
        state['rc_pitch'] = 1500
        state['rc_roll'] = 1500
        state['rc_yaw'] = 1500
        send_rc_override(master)

        if tilt_ok and climb_ok and agl_ok:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= PRE_LOITER_STABLE_SEC:
                update_log("AUTO: Pre-LOITER stabilization PASS.")
                break
        else:
            stable_since = None

        time.sleep(0.05)

    if not state['armed']:
        state['macro_status'] = 'IDLE'
        return

    # 9. HOVER PHASE - Use LOITER for FC-native position hold
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
        state['macro_status'] = 'HOVER'  # Allow auto_land_sequence to run
        auto_land_sequence(master)
        return
    
    update_log(f"AUTO: LOITER engaged. FC holding position for {total_hover_time:.0f}s...")

    # Guard LOITER entry with a short confirmation window.
    # This avoids false starts where attitude spikes immediately after mode switch.
    confirm_start = time.time()
    stable_since = None
    while state['armed'] and state['macro_status'] == 'TAKEOFF':
        elapsed = time.time() - confirm_start
        if elapsed > LOITER_ENTRY_CONFIRM_TIMEOUT_SEC:
            update_log("AUTO: LOITER entry did not stabilize in time. Landing.")
            state['macro_status'] = 'HOVER'
            auto_land_sequence(master)
            return

        if state.get('land_requested'):
            update_log("AUTO: Landing requested during LOITER entry confirm.")
            state['macro_status'] = 'HOVER'
            auto_land_sequence(master)
            return

        if abs(state['roll']) > LOITER_ENTRY_ABORT_TILT_DEG or abs(state['pitch']) > LOITER_ENTRY_ABORT_TILT_DEG:
            update_log(
                f"AUTO: LOITER entry unstable (R={state['roll']:.1f} P={state['pitch']:.1f}). Landing."
            )
            state['macro_status'] = 'HOVER'
            auto_land_sequence(master)
            return

        fused_climb = abs(get_fused_climb_rate())
        tilt_ok = (
            abs(state['roll']) <= LOITER_ENTRY_CONFIRM_TILT_DEG and
            abs(state['pitch']) <= LOITER_ENTRY_CONFIRM_TILT_DEG
        )
        climb_ok = fused_climb <= LOITER_ENTRY_CONFIRM_CLIMB_MPS

        state['rc_throttle'] = HOVER_THR
        state['rc_pitch'] = 1500
        state['rc_roll'] = 1500
        state['rc_yaw'] = 1500
        send_rc_override(master)

        if tilt_ok and climb_ok:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= LOITER_ENTRY_CONFIRM_SEC:
                break
        else:
            stable_since = None

        time.sleep(0.05)
    
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
        current_alt = get_fused_altitude_for_control() if USE_MULTISENSOR_CONTROL else (
            state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
        )
        elapsed = time.time() - hover_start

        if state.get('land_requested'):
            update_log("AUTO: Landing requested. Leaving hover and starting LAND.")
            break
        
        # ALL STICKS NEUTRAL — LOITER mode owns altitude + position.
        # Throttle 1500 = neutral; FC holds altitude via rangefinder EKF.
        # Do NOT nudge throttle — it fights ArduCopter's LOITER altitude PID.
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

        if state['livox_obs'] > 0 and state['livox_obs'] < 0.6:
            update_log(f"AUTO: Livox obstacle {state['livox_obs']:.2f}m during hover. Landing.")
            auto_land_sequence(master)
            return
        
        time.sleep(0.05)
        
        # Safety: vibration
        if max(state['vibration_filtered']) > MAX_VIBRATION:
            update_log(f"AUTO: FATAL VIB during hover ({max(state['vibration_filtered']):.1f})! KILL.")
            emergency_disarm(master)
            return
        # Safety: LOITER guard window right after engagement.
        # If attitude starts to diverge, switch to LAND first; keep KILL for extreme tilt.
        if elapsed < LOITER_GUARD_SEC:
            if abs(state['roll']) > LOITER_GUARD_KILL_TILT_DEG or abs(state['pitch']) > LOITER_GUARD_KILL_TILT_DEG:
                update_log("AUTO: EXTREME TILT during LOITER engage! KILL.")
                emergency_disarm(master)
                return
            if abs(state['roll']) > LOITER_GUARD_LAND_TILT_DEG or abs(state['pitch']) > LOITER_GUARD_LAND_TILT_DEG:
                update_log("AUTO: LOITER unstable during engage. Switching to LAND.")
                state['macro_status'] = 'HOVER'  # Allow auto_land_sequence to run
                auto_land_sequence(master)
                return

        # Safety: tilt (outside engage guard)
        if abs(state['roll']) > LOITER_GUARD_KILL_TILT_DEG or abs(state['pitch']) > LOITER_GUARD_KILL_TILT_DEG:
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
    
    # 10. Transition to landing (only if not already started by 'j' key)
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

    state['land_requested'] = False
    
    # Guard: must be armed and airborne
    if not state['armed']:
        update_log("LAND: Not armed. Nothing to do.")
        return
    
    current_check_alt = get_fused_altitude_for_control() if USE_MULTISENSOR_CONTROL else get_best_altitude_for_control()
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
        
        current_alt = get_fused_altitude_for_control() if USE_MULTISENSOR_CONTROL else get_best_altitude_for_control()
        
        # TILT SAFETY: disarm only on extreme attitude during landing.
        if abs(state['roll']) > 55 or abs(state['pitch']) > 55:
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
            update_log(
                f"LAND: Alt={current_alt:.2f}m Mode={state['mode']} "
                f"Climb={get_fused_climb_rate():.2f} Conf={state.get('fused_conf', 0)}%"
            )
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
            update_slam_metrics()
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
            # Accept FC heartbeats even when target_component is broadcast (0),
            # but continue ignoring companion/GCS components to prevent mode flicker.
            src_sys = msg.get_srcSystem() if hasattr(msg, 'get_srcSystem') else None
            src_comp = msg.get_srcComponent() if hasattr(msg, 'get_srcComponent') else None
            target_comp = int(getattr(master, 'target_component', 0) or 0)

            is_fc_heartbeat = (
                src_sys == master.target_system and
                (src_comp == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 or target_comp == 0 or src_comp == target_comp)
            )

            if is_fc_heartbeat:
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

    refresh_archive_summary(force=True)

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
        refresh_archive_summary(force=False)
        
        # ============ HEADER ============
        safe_add(row, 0, sep, c_info)
        row += 1
        header = "ASCEND v8.0 MISSION CONTROL | Pixhawk 6C OF+Rangefinder"
        safe_add(row, (W - len(header)) // 2, header, c_ok)
        row += 1
        rig_hdr = "TEST PROFILE: RIG-CONSTRAINED" if state.get('test_rig_mode') else "TEST PROFILE: FREE-FLIGHT"
        safe_add(row, (W - len(rig_hdr)) // 2, rig_hdr, c_warn if state.get('test_rig_mode') else c_neu)
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
        
        lidar_ok_icon = "●" if state['rangefinder_healthy'] else "○ WARN"
        flow_ok_icon  = "●" if state['flow_qual'] >= 80 else ("~ LOW" if state['flow_qual'] >= 40 else "○ FAIL")
        status_bar = (
            f"MODE:{state['mode']:<10} PHASE:{state['macro_status']:<8} "
            f"CTL:LIDAR+FLOW  "
            f"TEST:1m/{HOVER_DURATION:.0f}s  "
            f"LIDAR:{state['lidar_alt']:.2f}m {lidar_ok_icon}  "
            f"FLOW:Q{state['flow_qual']:3d} {flow_ok_icon}  "
            f"BATT:{state['batt_v']:.1f}V {state['batt_pct']:2d}%  "
            f"TIME:{remaining_time:4.1f}min"
        )
        safe_add(row, 2, status_bar, c_info)
        row += 1

        # ============ ARCHIVE DATA FUSION ==========
        archive = state.get('archive_summary', {})
        rates = state.get('archive_rates', {})
        hotspots = state.get('archive_hotspots', [])
        if archive:
            safe_add(row, 0, mid, c_info)
            row += 1
            archive_line = (
                f"ARCHIVE: TLM {archive.get('telemetry_files', 0):3d} | EVT {archive.get('event_files', 0):3d} "
                f"| IMU {archive.get('imu_files', 0):3d} | PC {archive.get('pc_files', 0):3d} "
                f"| MATCHED {archive.get('correlated_sessions', 0):3d}"
            )
            safe_add(row, 2, archive_line, c_info)
            row += 1

            aggregate_line = (
                f"TOTALS: rows {archive.get('telemetry_rows_total', 0):7d} | events {archive.get('event_lines_total', 0):6d} "
                f"| imu {archive.get('imu_samples_total', 0):8d} | pc_frames {archive.get('pc_frames_total', 0):6d}"
            )
            safe_add(row, 2, aggregate_line, c_neu)
            row += 1

            quality_line = (
                f"PEAKS: max_alt {archive.get('max_alt_all', 0.0):6.2f}m | max_vib {archive.get('max_vib_all', 0.0):6.2f} "
                f"| min_batt {archive.get('min_batt_all', 0.0):5.2f}V | imu_accel {archive.get('max_imu_accel', 0.0):6.2f}"
            )
            safe_add(row, 2, quality_line, c_neu)
            row += 1

            lfs_count = archive.get('lfs_placeholders', 0)
            lfs_line = f"LATEST: {archive.get('latest_session', 'n/a')} | LFS placeholders: {lfs_count}"
            safe_add(row, 2, lfs_line, c_warn if lfs_count else c_ok)
            row += 1

            rates_line = (
                f"RATES: armed {rates.get('armed_session_rate_pct', 0.0):5.1f}% | >=0.8m {rates.get('target_08_rate_pct', 0.0):5.1f}% "
                f"| >=1.0m {rates.get('target_10_rate_pct', 0.0):5.1f}% | critical_evt {rates.get('critical_event_file_rate_pct', 0.0):5.1f}% "
                f"| failsafe_evt {rates.get('failsafe_session_rate_pct', 0.0):5.1f}%"
            )
            safe_add(row, 2, rates_line, c_warn)
            row += 1

            anomaly_line = (
                f"ANOMALY: batt_fs {rates.get('battery_failsafe_session_rate_pct', 0.0):5.1f}% | mag {rates.get('mag_anomaly_session_rate_pct', 0.0):5.1f}% "
                f"| ekf_flip {rates.get('ekf_aiding_flip_session_rate_pct', 0.0):5.1f}% | crash_disarm {rates.get('crash_disarm_session_rate_pct', 0.0):5.1f}% "
                f"| rig_pat {rates.get('rig_pattern_session_rate_pct', 0.0):5.1f}%"
            )
            safe_add(row, 2, anomaly_line, c_warn)
            row += 1

            if hotspots:
                hot_text = " | ".join([f"{label}:{count}" for label, count in hotspots[:3]])
                safe_add(row, 2, f"HOTSPOTS: {hot_text}", c_crit)
                row += 1

            rig_line = (
                f"RIG-ANALYSIS: high_tilt_sessions {archive.get('telemetry_sessions_high_tilt', 0):3d} | "
                f"alt_diverge_sessions {archive.get('telemetry_sessions_alt_diverge', 0):3d} | "
                f"rig_pattern_sessions {archive.get('telemetry_sessions_rig_pattern', 0):3d}"
            )
            safe_add(row, 2, rig_line, c_warn)
            row += 1

            study = state.get('session_study', {})
            if study:
                study_line1 = (
                    f"STUDY: session {study.get('session_id', 'n/a')} | dur {study.get('duration_s', 0.0):6.1f}s | "
                    f"rows {study.get('rows', 0):6d} | mode {study.get('flight_mode_last', 'N/A'):<12}"
                )
                safe_add(row, 2, study_line1, c_info)
                row += 1

                study_line2 = (
                    f"       EVT w/c {study.get('event_warn', 0):3d}/{study.get('event_critical', 0):3d} | "
                    f"IMU {study.get('imu_samples', 0):7d} pk {study.get('imu_peak_accel', 0.0):5.2f} | "
                    f"PC {study.get('pc_frames', 0):5d}/{study.get('pc_points', 0):8d} | "
                    f"FlowQ {study.get('avg_flow_qual', 0.0):5.1f}"
                )
                safe_add(row, 2, study_line2, c_neu)
                row += 1

            if state.get('archive_last_error'):
                safe_add(row, 2, f"ARCHIVE ERROR: {state['archive_last_error']}", c_crit)
                row += 1

            for idx, todo_text in enumerate(state.get('ops_todo', [])[:8], start=1):
                safe_add(row, 2, f"TODO {idx}: {todo_text}", c_warn)
                row += 1

        # ============ PRIMARY FLIGHT INSTRUMENTS ============
        safe_add(row, 0, mid, c_info)
        safe_add(row, 2, "[ PRIMARY INSTRUMENTS ]", c_info)
        row += 1
        
        # Altitude readout with all estimates
        lb_diff = abs(state['lidar_alt'] - state['alt'])
        alt_color = c_ok if state['rangefinder_healthy'] else c_crit
        lidar_icon = "●" if state['rangefinder_healthy'] else "○"
        alt_line = (
            f"ALT: LIDAR {lidar_icon} {state['lidar_alt']:7.3f}m   "
            f"BARO {state['alt']:7.2f}m   ΔLB:{lb_diff:5.3f}m   "
            f"FLOW Vx:{state['flow_vx']:+.3f} Vy:{state['flow_vy']:+.3f} Q:{state['flow_qual']:3d}/255"
        )
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
        safe_add(row, 2, "[ NAVIGATION — SLAM/LIVOX DISABLED (LIDAR+FLOW only) ]", c_info)
        row += 1

        slam_conf = 0  # SLAM not used for control
        flow_spark = create_sparkline(state['flow_history'][-20:] if state.get('flow_history') else [], width=12)
        nav_line = (
            f"OptFlow Vx:{state['flow_vx']:+6.3f} Vy:{state['flow_vy']:+6.3f} m/s  "
            f"Mag:{state['flow_magnitude']:.3f}m/s  "
            f"Drift:{get_drift_magnitude():.3f}m  "
            f"Trend:[{flow_spark}]"
        )
        flow_nav_color = c_ok if state['flow_qual'] >= 80 else c_warn if state['flow_qual'] >= 40 else c_crit
        safe_add(row, 2, nav_line, flow_nav_color)
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
        health_color_lidar = c_ok if lidar_conf > 70 else c_warn if lidar_conf > 0 else c_crit
        health_color_baro  = c_ok
        health_color_opt   = c_ok if optflow_conf > 70 else c_warn if optflow_conf > 0 else c_crit

        safe_add(row, 2, lidar_stat,   health_color_lidar)
        safe_add(row, 22, baro_stat,   health_color_baro)
        safe_add(row, 38, optflow_stat, health_color_opt)
        safe_add(row, 60, "NAV:FLOW+RNG", c_neu)
        row += 1

        safe_add(row, 2,  "EXT LOG:OFF", c_neu)
        safe_add(row, 20, f"EKF: 0x{state['ekf_flags']:04X}", c_neu)
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
        safe_add(row, 2, "[ SAFETY ZONE ]", c_info)
        row += 1
        safe_add(row, 2, "Maintain visual line-of-sight and clear flight area before arming/takeoff.", c_neu)
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
        
        safe_add(row, 2, "[TEST] 1=1m hold+land  2=2m hold+land  z=RigProfile toggle  j=Land  [MOVE] i=Fwd  ,=Back  ;=Left  '=Right  [CTRL] a=Arm  d=Disarm  x=KILL  q=Quit", c_warn)
        row += 1
        
        safe_add(row, 2, "[MODE] h=AltHold  l=Loiter  [MANUAL] W/S=Pitch  ↑↓=Thr  SPACE=Hover  [TOOLS] u=Study  e=Export  t=Rescan  y=TODO  c=Cal  o=Origin", c_warn)
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
                if state['macro_status'] == 'TAKEOFF':
                    state['land_requested'] = True
                    update_log("CMD: Land requested. Will transition to LAND after takeoff phase.")
                elif state['macro_status'] == 'LANDING':
                    update_log("CMD: LAND already active.")
                else:
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
                        update_log(
                            f"ROS2: ACTIVE {state['fastlio_hz']:.0f}Hz | "
                            f"SLAM {state['slam_status']} {state['slam_quality']}% | "
                            f"age {fastlio_age:.1f}s | Pts {state['fastlio_points']}"
                        )
                    else:
                        update_log("ROS2: NOT ACTIVE - Check `ros2 node list` and `ros2 topic list`")
                else:
                    update_log("ROS2: Not installed. Run: pip install rclpy")
            elif key == ord('g'):
                update_log(
                    f"SLAM: {state['slam_status']} Q={state['slam_quality']}% "
                    f"Hz={state['fastlio_hz']:.1f} Age={state['slam_age']:.2f}s "
                    f"dZ={state['slam_pose_error']:.2f}m Drop={state['slam_dropouts']}"
                )
            elif key == ord('u'):
                study = state.get('session_study', {})
                if study:
                    update_log(
                        f"STUDY[{study.get('session_id', 'n/a')}]: dur {study.get('duration_s', 0.0):.1f}s "
                        f"rows {study.get('rows', 0)} evt {study.get('event_warn', 0)}/{study.get('event_critical', 0)} "
                        f"imu {study.get('imu_samples', 0)} pc {study.get('pc_frames', 0)}"
                    )
                else:
                    update_log("STUDY: No correlated session available yet")
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
            elif key == ord('t'):
                refresh_archive_summary(force=True)
                update_log("ARCHIVE: Rescan complete")
            elif key == ord('y'):
                out = export_ops_todo()
                update_log(f"TODO exported to {out}")
            elif key == ord('z'):
                state['test_rig_mode'] = not state.get('test_rig_mode', False)
                mode_str = "RIG-CONSTRAINED" if state['test_rig_mode'] else "FREE-FLIGHT"
                update_log(f"TEST PROFILE -> {mode_str}")

        # Only send RC overrides from dashboard when not in autonomous mode
        if state['macro_status'] == 'IDLE':
            send_rc_override(master)
        time.sleep(0.05)


def draw_study_dashboard(stdscr):
    """Offline dashboard focused on archived telemetry/event/Livox study on PC."""
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.keypad(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN,  -1)
    curses.init_pair(2, curses.COLOR_RED,    -1)
    curses.init_pair(3, curses.COLOR_CYAN,   -1)
    curses.init_pair(4, curses.COLOR_YELLOW, -1)
    curses.init_pair(5, curses.COLOR_WHITE,  -1)

    messages = []

    def push_msg(msg):
        timestamp = datetime.now().strftime("%H:%M:%S")
        messages.append(f"[{timestamp}] {msg}")
        if len(messages) > 8:
            messages.pop(0)

    refresh_archive_summary(force=True)
    sessions = collect_correlated_session_ids()
    index = len(sessions) - 1 if sessions else 0
    current_session = sessions[index] if sessions else ""
    current_study = summarize_latest_session(current_session) if current_session else {}
    current_sparks = build_session_sparklines(current_session) if current_session else {}
    current_events = read_event_tail(current_session, max_lines=6) if current_session else []

    push_msg("Study mode active (offline, no MAVLink)")

    while True:
        stdscr.erase()
        rows, cols = stdscr.getmaxyx()
        c_ok = curses.color_pair(1) | curses.A_BOLD
        c_crit = curses.color_pair(2) | curses.A_BOLD
        c_info = curses.color_pair(3) | curses.A_BOLD
        c_warn = curses.color_pair(4) | curses.A_BOLD
        c_neu = curses.color_pair(5)

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

        archive = state.get('archive_summary', {})
        rates = state.get('archive_rates', {})
        hotspots = state.get('archive_hotspots', [])

        safe_add(row, 0, sep, c_info)
        row += 1
        title = "ASCEND DATA STUDY MODE | TELEMETRY + EVENTS + LIVOX"
        safe_add(row, max((W - len(title)) // 2, 0), title, c_ok)
        row += 1
        safe_add(row, 0, mid, c_info)
        row += 1

        safe_add(
            row,
            2,
            (
                f"ARCHIVE: TLM {archive.get('telemetry_files', 0):3d} | EVT {archive.get('event_files', 0):3d} | "
                f"IMU {archive.get('imu_files', 0):3d} | PC {archive.get('pc_files', 0):3d} | "
                f"CORR {archive.get('correlated_sessions', 0):3d}"
            ),
            c_info,
        )
        row += 1
        safe_add(
            row,
            2,
            (
                f"TOTALS: rows {archive.get('telemetry_rows_total', 0):8d} | events {archive.get('event_lines_total', 0):6d} | "
                f"imu {archive.get('imu_samples_total', 0):8d} | pc_frames {archive.get('pc_frames_total', 0):6d}"
            ),
            c_neu,
        )
        row += 1
        safe_add(
            row,
            2,
            (
                f"RATES: armed {rates.get('armed_session_rate_pct', 0.0):5.1f}% | >=0.8m {rates.get('target_08_rate_pct', 0.0):5.1f}% "
                f"| >=1.0m {rates.get('target_10_rate_pct', 0.0):5.1f}% | critical {rates.get('critical_event_file_rate_pct', 0.0):5.1f}% "
                f"| failsafe {rates.get('failsafe_session_rate_pct', 0.0):5.1f}%"
            ),
            c_warn,
        )
        row += 1

        safe_add(
            row,
            2,
            (
                f"ANOMALY: batt_fs {rates.get('battery_failsafe_session_rate_pct', 0.0):5.1f}% | mag {rates.get('mag_anomaly_session_rate_pct', 0.0):5.1f}% "
                f"| ekf_flip {rates.get('ekf_aiding_flip_session_rate_pct', 0.0):5.1f}% | crash_disarm {rates.get('crash_disarm_session_rate_pct', 0.0):5.1f}% "
                f"| rig_pat {rates.get('rig_pattern_session_rate_pct', 0.0):5.1f}%"
            ),
            c_warn,
        )
        row += 1

        if hotspots:
            top_hotspots = " | ".join([f"{label}:{count}" for label, count in hotspots[:4]])
            safe_add(row, 2, f"HOTSPOTS: {top_hotspots}", c_crit)
            row += 1

        safe_add(
            row,
            2,
            (
                f"RIG-ANALYSIS: high_tilt_sessions {archive.get('telemetry_sessions_high_tilt', 0):3d} | "
                f"alt_diverge_sessions {archive.get('telemetry_sessions_alt_diverge', 0):3d} | "
                f"rig_pattern_sessions {archive.get('telemetry_sessions_rig_pattern', 0):3d}"
            ),
            c_warn,
        )
        row += 1

        safe_add(row, 0, mid, c_info)
        row += 1
        if current_session:
            safe_add(
                row,
                2,
                (
                    f"SESSION [{index + 1}/{max(1, len(sessions))}]: {current_session} | "
                    f"dur {current_study.get('duration_s', 0.0):6.1f}s | rows {current_study.get('rows', 0):6d} | "
                    f"mode {current_study.get('flight_mode_last', 'N/A')}"
                ),
                c_ok,
            )
            row += 1

            safe_add(
                row,
                2,
                (
                    f"METRICS: alt_max {current_study.get('max_alt', 0.0):6.2f}m | batt_min {current_study.get('min_batt_v', 0.0):5.2f}V | "
                    f"vib_max {current_study.get('max_vib', 0.0):6.2f} | flow_q {current_study.get('avg_flow_qual', 0.0):5.1f} | "
                    f"slam_hz {current_study.get('avg_fastlio_hz', 0.0):5.1f}"
                ),
                c_neu,
            )
            row += 1

            safe_add(
                row,
                2,
                (
                    f"LIVOX: imu {current_study.get('imu_samples', 0):8d} peak_acc {current_study.get('imu_peak_accel', 0.0):5.2f} | "
                    f"pointcloud frames {current_study.get('pc_frames', 0):6d} points {current_study.get('pc_points', 0):9d} | "
                    f"events W/C {current_study.get('event_warn', 0):3d}/{current_study.get('event_critical', 0):3d}"
                ),
                c_neu,
            )
            row += 1
        else:
            safe_add(row, 2, "No correlated session found yet (need telemetry + events + imu + pointcloud)", c_warn)
            row += 1

        safe_add(row, 0, mid, c_info)
        row += 1
        safe_add(row, 2, f"TREND ALT   [{current_sparks.get('alt', 'n/a')}]", c_neu)
        row += 1
        safe_add(row, 2, f"TREND BATT  [{current_sparks.get('batt', 'n/a')}]", c_neu)
        row += 1
        safe_add(row, 2, f"TREND VIB   [{current_sparks.get('vib', 'n/a')}]", c_neu)
        row += 1
        safe_add(row, 2, f"TREND FLOWQ [{current_sparks.get('flow', 'n/a')}]", c_neu)
        row += 1
        safe_add(row, 2, f"TREND SLAM  [{current_sparks.get('slam_hz', 'n/a')}]", c_neu)
        row += 1

        safe_add(row, 0, mid, c_info)
        row += 1
        safe_add(row, 2, "[ EVENT TAIL ]", c_info)
        row += 1
        for line in current_events[-5:]:
            color = c_crit if "CRITICAL" in line.upper() or "ABORT" in line.upper() else c_neu
            safe_add(row, 2, line, color)
            row += 1
            if row >= rows - 3:
                break

        safe_add(row, 0, mid, c_info)
        row += 1
        safe_add(row, 2, "[ LONG TODO PREVIEW ]", c_info)
        row += 1
        for idx, todo_text in enumerate(state.get('ops_todo', [])[:8], start=1):
            color = c_warn if idx < 5 else c_neu
            safe_add(row, 2, f"{idx:02d}. {todo_text}", color)
            row += 1
            if row >= rows - 4:
                break

        safe_add(row, 0, mid, c_info)
        row += 1
        safe_add(row, 2, "[KEYS] n=next session  p=prev session  r=rescan data  y=export todo  q=quit", c_warn)
        row += 1
        safe_add(row, 2, "[ STATUS ]", c_info)
        row += 1
        for msg in messages[-2:]:
            safe_add(row, 2, msg, c_ok)
            row += 1

        stdscr.refresh()
        key = stdscr.getch()

        if key == ord('q'):
            break
        elif key == ord('r'):
            refresh_archive_summary(force=True)
            sessions = collect_correlated_session_ids()
            if not sessions:
                current_session = ""
                index = 0
                current_study = {}
                current_sparks = {}
                current_events = []
            else:
                index = min(index, len(sessions) - 1)
                current_session = sessions[index]
                current_study = summarize_latest_session(current_session)
                current_sparks = build_session_sparklines(current_session)
                current_events = read_event_tail(current_session, max_lines=6)
            push_msg("Archive rescanned")
        elif key == ord('n') and sessions:
            index = (index + 1) % len(sessions)
            current_session = sessions[index]
            current_study = summarize_latest_session(current_session)
            current_sparks = build_session_sparklines(current_session)
            current_events = read_event_tail(current_session, max_lines=6)
            push_msg(f"Session -> {current_session}")
        elif key == ord('p') and sessions:
            index = (index - 1) % len(sessions)
            current_session = sessions[index]
            current_study = summarize_latest_session(current_session)
            current_sparks = build_session_sparklines(current_session)
            current_events = read_event_tail(current_session, max_lines=6)
            push_msg(f"Session -> {current_session}")
        elif key == ord('y'):
            out = export_ops_todo()
            push_msg(f"TODO exported to {out}")

        time.sleep(0.05)


# --- ENTRY POINT ---

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ASCEND Mission Control / Study TUI')
    parser.add_argument('--mode', choices=['auto', 'study', 'live'], default='auto')
    parser.add_argument('--port', default='/dev/ttyAMA0', help='MAVLink serial port for live mode')
    parser.add_argument('--baud', type=int, default=921600, help='MAVLink serial baud rate')
    args = parser.parse_args()

    try:
        print("\n" + "="*70)
        print("  ASCEND MISSION CONTROL v8.0 - Rangefinder + Optical Flow")
        print("="*70)

        if args.mode == 'study':
            print("\n[*] Starting OFFLINE STUDY MODE (no MAVLink)...")
            refresh_archive_summary(force=True)
            curses.wrapper(draw_study_dashboard)
            raise SystemExit(0)

        if args.mode == 'auto' and not os.path.exists(args.port):
            print(f"\n[*] Port {args.port} not found. Falling back to OFFLINE STUDY MODE.")
            refresh_archive_summary(force=True)
            curses.wrapper(draw_study_dashboard)
            raise SystemExit(0)
        
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

        print(f"\n[*] Connecting to Pixhawk on {args.port} @ {args.baud} baud...")
        master_conn = mavutil.mavlink_connection(args.port, baud=args.baud)
        
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




