#!/usr/bin/env python3
"""
ASCEND MISSION CONTROL v9.0  —  ascend_tui_v2.py
Improved PC-side TUI: 6 full-screen tabs, session browser, cross-session
analysis, Livox tab, SLAM-burst deduplication, and --offline mode.

Hardware: Raspberry Pi 5 + Pixhawk 6C + Livox Mid-360 + OptFlow + Rangefinder
Run on PC:  python3 src/ascend_tui_v2.py [--offline] [--port /dev/ttyUSB0]
"""

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
import sys
from datetime import datetime
from collections import deque

# ─────────────────────────────────────────────────────────────────────────────
# CONDITIONAL IMPORTS (gracefully absent in offline mode)
# ─────────────────────────────────────────────────────────────────────────────
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

MAVLINK_AVAILABLE = False
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    pass

# ─────────────────────────────────────────────────────────────────────────────
# MISSION PARAMETERS (unchanged from v1)
# ─────────────────────────────────────────────────────────────────────────────
TARGET_ALTITUDE   = 1.0
SPOOL_THR         = 1400
LIFTOFF_THR       = 1600
HOVER_THR         = 1500
MIN_SAFE_VOLTAGE  = 10.5
MAX_VIBRATION     = 25.0
MAX_GROUND_VIBE   = 18.0
VIBRATION_FILTER_SIZE = 10

ALT_PID_P   = 8.0;   ALT_PID_I = 0.5;  ALT_PID_D   = 2.5
ALT_DEADBAND = 0.08; CLIMB_DAMPING = 0.6
CLIMB_DIST_THRESHOLD = 0.30

POS_P_GAIN  = 50.0;  POS_I_GAIN = 1.5;  POS_DEADBAND = 0.02
POS_MAX_ANGLE = 3.0

HOVER_SETTLE_TIME = 6.0;  HOVER_DURATION = 5.0;  CLIMB_RAMP_RATE = 5
YAW_RATE_LIMIT = 60.0;    YAW_INTEGRAL_LIMIT = 30

FASTLIO_TIMEOUT        = 3.0
SLAM_WARN_INTERVAL     = 5.0
FASTLIO_MIN_HEALTHY_HZ = 5.0
FASTLIO_STALE_WARN_SEC = 1.5
SLAM_MAX_ALT_DIFF      = 1.0

MIN_PREFLIGHT_FLOW_QUAL   = 80
FLOW_SPIKE_REJECT_MPS     = 1.2
FLOW_LPF_ALPHA            = 0.35
LIDAR_LPF_ALPHA           = 0.35
MAX_LIDAR_STEP_M          = 0.35
MIN_TAKEOFF_CLIMB_RATE    = 0.12
NO_CLIMB_ABORT_SEC        = 2.5

USE_FASTLIO_FOR_CONTROL   = False
USE_MULTISENSOR_CONTROL   = True
MAX_ALT_SENSOR_DISAGREE   = 0.8
LIVOX_MIN_SAFE_DIST       = 0.8

ARCHIVE_REFRESH_SEC       = 12.0
LFS_POINTER_HEADER        = "version https://git-lfs.github.com/spec/v1"

PROJECT_ROOT  = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DATA_ROOT     = os.path.join(PROJECT_ROOT, "data")
TELEMETRY_DIR = os.path.join(DATA_ROOT, "telemetry")
EVENTS_DIR    = os.path.join(DATA_ROOT, "events")
LIVOX_DIR     = os.path.join(DATA_ROOT, "livox")

# ─────────────────────────────────────────────────────────────────────────────
# SESSION LOGGING (created even in offline mode so we can import helpers)
# ─────────────────────────────────────────────────────────────────────────────
session_id            = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename          = os.path.join(TELEMETRY_DIR, f"telemetry_{session_id}.csv")
txt_filename          = os.path.join(EVENTS_DIR,    f"events_{session_id}.txt")
livox_pc_filename     = os.path.join(LIVOX_DIR,     f"livox_pointcloud_{session_id}.bin")
livox_imu_filename    = os.path.join(LIVOX_DIR,     f"livox_imu_{session_id}.csv")

_OFFLINE_MODE = False   # set by argparse before opening files

def _open_log_files():
    """Open logging file handles (skipped in offline mode)."""
    global telemetry_file, csv_writer, event_file, livox_pc_file
    global livox_imu_file, livox_imu_writer
    os.makedirs(TELEMETRY_DIR, exist_ok=True)
    os.makedirs(EVENTS_DIR,    exist_ok=True)
    os.makedirs(LIVOX_DIR,     exist_ok=True)

    telemetry_file = open(csv_filename, "w", newline="")
    csv_writer = csv.writer(telemetry_file)
    csv_writer.writerow([
        "Timestamp","Flight_Mode","Armed","Alt_m","Lidar_Alt_m","Climb_ms",
        "Roll_deg","Pitch_deg","Yaw_deg","Batt_V","Batt_A","Batt_Pct",
        "Thr_PWM","OptFlow_Qual","OptFlow_Vx","OptFlow_Vy",
        "Vib_X","Vib_Y","Vib_Z","CPU_Load_Pct",
        "Livox_MinObs_m","Livox_Sectors","Vision_X","Vision_Y","Vision_Z",
        "FastLIO_X","FastLIO_Y","FastLIO_Z",
        "FastLIO_Vx","FastLIO_Vy","FastLIO_Vz",
        "FastLIO_Roll","FastLIO_Pitch","FastLIO_Yaw",
        "FastLIO_Hz","FastLIO_Points",
    ])

    event_file = open(txt_filename, "w")
    event_file.write(f"=== ASCEND MISSION CONTROL v9.0 SESSION: {session_id} ===\n")

    livox_pc_file = open(livox_pc_filename, "wb")
    livox_pc_file.write(b"LIVOXPC1")
    livox_pc_file.flush()

    livox_imu_file   = open(livox_imu_filename, "w", newline="")
    livox_imu_writer = csv.writer(livox_imu_file)
    livox_imu_writer.writerow([
        "Timestamp","Accel_X","Accel_Y","Accel_Z",
        "Gyro_X","Gyro_Y","Gyro_Z",
        "Orient_X","Orient_Y","Orient_Z","Orient_W",
    ])

# Stubs so module-level references survive offline mode
telemetry_file = None;  csv_writer = None
event_file     = None;  livox_pc_file = None
livox_imu_file = None;  livox_imu_writer = None

# ─────────────────────────────────────────────────────────────────────────────
# GLOBAL STATE
# ─────────────────────────────────────────────────────────────────────────────
state = {
    "alt": 0.0, "lidar_alt": 0.0, "rangefinder_healthy": False, "climb": 0.0,
    "lidar_alt_raw": 0.0, "lidar_last_update": 0.0,
    "hdg": 0, "roll": 0.0, "pitch": 0.0,
    "batt_v": 0.0, "batt_pct": 0, "batt_curr": 0.0,
    "mode": "CONNECTING...", "armed": False, "macro_status": "IDLE",
    "log": [],          # list of (timestamp_str, msg, color_key)
    "rc_throttle": 1000, "rc_pitch": 1500, "rc_roll": 1500, "rc_yaw": 1500,
    "ekf_flags": 0, "flow_qual": 0, "flow_vx": 0.0, "flow_vy": 0.0,
    "flow_vx_raw": 0.0, "flow_vy_raw": 0.0, "flow_last_update": 0.0,
    "vibration": [0.0,0.0,0.0], "vibration_filtered": [0.0,0.0,0.0],
    "vibration_history": [[],[],[]], "cpu_load": 0,
    "livox_obs": 0.0, "livox_sectors": 0,
    "vision_x": 0.0, "vision_y": 0.0, "vision_z": 0.0,
    "fastlio_x": 0.0, "fastlio_y": 0.0, "fastlio_z": 0.0,
    "fastlio_vx": 0.0, "fastlio_vy": 0.0, "fastlio_vz": 0.0,
    "fastlio_roll": 0.0, "fastlio_pitch": 0.0, "fastlio_yaw": 0.0,
    "fastlio_points": 0, "fastlio_healthy": False, "fastlio_hz": 0.0,
    "fastlio_last_time": 0.0, "fastlio_msg_count": 0, "fastlio_hz_timer": 0.0,
    "fastlio_last_odom_time": 0.0, "fastlio_dt": 0.0,
    "slam_age": 999.0, "slam_quality": 0, "slam_status": "LOST",
    "slam_dropouts": 0, "slam_pose_error": 0.0,
    "slam_hz_history": [], "slam_error_history": [],
    "ros2_active": False, "ros2_init_time": 0.0, "ros2_error": "",
    "fused_alt": 0.0, "fused_climb": 0.0, "fused_conf": 0,
    "fused_weights": {"lidar": 0.0, "slam": 0.0, "baro": 1.0},
    "fused_source": "BARO",
    "livox_pc_frames": 0, "livox_pc_last_pts": 0, "livox_imu_samples": 0,
    "batt_v_history": [], "batt_pct_history": [], "climb_history": [],
    "vib_max_history": [], "alt_history": [],
    "session_start_time": 0.0, "flight_time": 0.0,
    "max_alt_achieved": 0.0, "max_vib_recorded": 0.0,
    "pos_error_lidar_vs_baro": 0.0, "pos_error_fastlio_vs_lidar": 0.0,
    "flow_magnitude": 0.0, "flow_history": [], "cpu_load_history": [],
    "home_x": 0.0, "home_y": 0.0, "home_z": 0.0, "home_set": False,
    "pos_x": 0.0, "pos_y": 0.0, "pos_x_drift": 0.0, "pos_y_drift": 0.0,
    "flow_integral_x": 0.0, "flow_integral_y": 0.0,
    "desired_pos_x": 0.0, "desired_pos_y": 0.0,
    "rtl_active": False, "sensor_status_log": [],
    "slam_last_warn_time": 0.0,
    # NEW: SLAM burst deduplication
    "slam_lost_burst": 0,       # consecutive suppressed SLAM-lost messages
    "slam_last_dedup_time": 0.0,
    # Archive
    "archive_summary": {}, "session_study": {},
    "archive_last_scan": 0.0, "archive_last_error": "",
    "ops_todo": [],
    # NEW: All-sessions table (list of dicts)
    "all_sessions": [],
    "all_sessions_loaded": False,
    "all_sessions_load_ts": 0.0,
    # NEW: Selected session for SESSION / LIVOX tabs
    "selected_session_idx": 0,
    "selected_session_detail": {},
    "selected_session_sparklines": {},
    "selected_session_events": [],
    # NEW: Cross-session trend data
    "trend_data": {},
    # Current tab
    "active_tab": 0,
    # Archive table scroll
    "archive_scroll": 0,
    # Connection
    "mav": None,
    "connected": False,
    "mav_hz": 0.0,
    "mav_msg_count": 0,
    "mav_hz_timer": 0.0,
}

# ─────────────────────────────────────────────────────────────────────────────
# UTILITY FUNCTIONS
# ─────────────────────────────────────────────────────────────────────────────
def low_pass(prev, new, alpha):
    return alpha * new + (1.0 - alpha) * prev

def _safe_float(v, d=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d

def _is_lfs_pointer(path):
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            return f.readline().strip() == LFS_POINTER_HEADER
    except OSError:
        return False

def _read_lfs_size(path):
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                if line.startswith("size "):
                    return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        pass
    return 0

def _extract_session_id(fname):
    m = re.search(r"(\d{8}_\d{6})", fname)
    return m.group(1) if m else ""

def _candidate_files(pattern, alt_pattern=None):
    files = list(glob.glob(pattern))
    if alt_pattern:
        files.extend(glob.glob(alt_pattern))
    return sorted(set(files), key=os.path.basename)

def _fmt_size(n):
    if n >= 1_073_741_824:
        return f"{n/1_073_741_824:.1f} GB"
    if n >= 1_048_576:
        return f"{n/1_048_576:.1f} MB"
    if n >= 1024:
        return f"{n/1024:.1f} KB"
    return f"{n} B"

def _fmt_dur(seconds):
    if seconds < 60:
        return f"{seconds:.0f}s"
    m, s = divmod(int(seconds), 60)
    if m < 60:
        return f"{m}m{s:02d}s"
    h, m = divmod(m, 60)
    return f"{h}h{m:02d}m"

def create_sparkline(data, width=20):
    """Unicode block sparkline from a list of numbers, returns string of `width` chars."""
    BLOCKS = " ▁▂▃▄▅▆▇█"
    if not data:
        return "─" * width
    mn, mx = min(data), max(data)
    if mx == mn:
        return "▄" * width
    step = len(data) / width
    chars = []
    for i in range(width):
        idx = min(int(i * step), len(data) - 1)
        v = data[idx]
        level = int(8 * (v - mn) / (mx - mn))
        chars.append(BLOCKS[max(0, min(8, level))])
    return "".join(chars)

def update_log(msg):
    """Add a line to the event log with automatic colour tagging."""
    ts = datetime.now().strftime("%H:%M:%S.%f")[:12]
    upper = msg.upper()
    if "CRITICAL" in upper or "ABORT" in upper or "FAIL" in upper or "KILL" in upper:
        key = "RED_BOLD"
    elif "WARN" in upper or "WARNING" in upper:
        key = "YELLOW"
    elif msg.startswith("[") and any(t in upper for t in ["AUTO:", "CMD:"]):
        key = "CYAN"
    elif any(t in upper for t in ["SYS:", "DIAG:", "SETTLE:", "HOVER:", "LAND:"]):
        key = "WHITE"
    else:
        key = "NORMAL"
    state["log"].append((ts, msg, key))
    if len(state["log"]) > 500:
        state["log"] = state["log"][-500:]
    if event_file:
        event_file.write(f"[{ts}] {msg}\n")
        event_file.flush()

def _dedup_slam_warn(msg):
    """Return True if this SLAM-lost message should be suppressed (burst dedup)."""
    if "SLAM lost" not in msg and "slam lost" not in msg.lower():
        return False
    now = time.time()
    state["slam_lost_burst"] += 1
    dt = now - state["slam_last_dedup_time"]
    if state["slam_lost_burst"] == 1:
        state["slam_last_dedup_time"] = now
        return False    # let first one through
    if dt > 3.0:
        # flush the count as a summary, reset
        n = state["slam_lost_burst"]
        state["slam_lost_burst"] = 0
        state["slam_last_dedup_time"] = now
        update_log(f"⚠ SLAM lost ×{n} (burst suppressed)")
    return True         # suppress this duplicate

def update_vibration_filter(vib):
    h = state["vibration_history"]
    for i in range(3):
        h[i].append(vib[i])
        if len(h[i]) > VIBRATION_FILTER_SIZE:
            h[i].pop(0)
    state["vibration_filtered"] = [
        sum(h[i]) / len(h[i]) if h[i] else 0.0 for i in range(3)
    ]

# ─────────────────────────────────────────────────────────────────────────────
# ARCHIVE / SESSION ANALYSIS ENGINE
# ─────────────────────────────────────────────────────────────────────────────
def summarize_telemetry_file(path):
    s = {"rows": 0, "max_alt": 0.0, "min_batt_v": 999.0, "max_vib": 0.0,
         "avg_flow_qual": 0.0, "avg_fastlio_hz": 0.0, "duration_s": 0.0,
         "last_mode": "N/A", "is_lfs": False, "lfs_size": 0}
    if _is_lfs_pointer(path):
        s["is_lfs"] = True; s["lfs_size"] = _read_lfs_size(path); return s
    try:
        first_t = last_t = None
        fq_sum = fq_n = hz_sum = hz_n = 0
        with open(path, "r", newline="", encoding="utf-8", errors="ignore") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                return s
            for row in reader:
                s["rows"] += 1
                alt = _safe_float(row.get("Alt_m", row.get("Lidar_Alt_m", 0)), 0.0)
                s["max_alt"] = max(s["max_alt"], alt)
                bv = _safe_float(row.get("Batt_V", 0), 0.0)
                if bv > 0: s["min_batt_v"] = min(s["min_batt_v"], bv)
                vx = _safe_float(row.get("Vib_X", 0), 0.0)
                vy = _safe_float(row.get("Vib_Y", 0), 0.0)
                vz = _safe_float(row.get("Vib_Z", 0), 0.0)
                s["max_vib"] = max(s["max_vib"], vx, vy, vz)
                fq = _safe_float(row.get("OptFlow_Qual", 0), 0.0)
                fq_sum += fq; fq_n += 1
                hz = _safe_float(row.get("FastLIO_Hz", 0), 0.0)
                hz_sum += hz; hz_n += 1
                mode = (row.get("Flight_Mode") or "").strip()
                if mode:
                    s["last_mode"] = mode
                ts_text = row.get("Timestamp", "")
                try:
                    t = datetime.strptime(ts_text, "%H:%M:%S.%f")
                    sec = t.hour*3600 + t.minute*60 + t.second + t.microsecond/1e6
                    if first_t is None: first_t = sec
                    last_t = sec
                except (ValueError, TypeError):
                    pass
        if first_t is not None and last_t is not None and last_t >= first_t:
            s["duration_s"] = round(last_t - first_t, 1)
        if fq_n > 0: s["avg_flow_qual"] = round(fq_sum / fq_n, 1)
        if hz_n > 0: s["avg_fastlio_hz"] = round(hz_sum / hz_n, 1)
        if s["min_batt_v"] == 999.0: s["min_batt_v"] = 0.0
    except OSError:
        pass
    return s

def summarize_event_file(path):
    s = {"lines": 0, "warn": 0, "critical": 0, "is_lfs": False, "lfs_size": 0}
    if _is_lfs_pointer(path):
        s["is_lfs"] = True; s["lfs_size"] = _read_lfs_size(path); return s
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                stripped = line.strip()
                if not stripped: continue
                s["lines"] += 1
                u = stripped.upper()
                if any(k in u for k in ("CRITICAL","ABORT","FAIL","KILL")):
                    s["critical"] += 1
                elif "WARN" in u:
                    s["warn"] += 1
    except OSError:
        pass
    return s

def summarize_livox_imu_file(path):
    s = {"samples": 0, "max_accel": 0.0, "is_lfs": False, "lfs_size": 0}
    if _is_lfs_pointer(path):
        s["is_lfs"] = True; s["lfs_size"] = _read_lfs_size(path); return s
    try:
        with open(path, "r", newline="", encoding="utf-8", errors="ignore") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames: return s
            for row in reader:
                ax = _safe_float(row.get("Accel_X",0),0.0)
                ay = _safe_float(row.get("Accel_Y",0),0.0)
                az = _safe_float(row.get("Accel_Z",0),0.0)
                amag = math.sqrt(ax*ax + ay*ay + az*az)
                s["samples"] += 1
                s["max_accel"] = max(s["max_accel"], amag)
    except OSError:
        pass
    return s

def summarize_livox_pc_file(path):
    s = {"frames": 0, "points": 0, "bytes": 0, "is_lfs": False, "lfs_size": 0}
    if _is_lfs_pointer(path):
        s["is_lfs"] = True; s["lfs_size"] = _read_lfs_size(path); return s
    try:
        s["bytes"] = os.path.getsize(path)
        with open(path, "rb") as f:
            if f.read(8) != b"LIVOXPC1":
                return s
            fsz = s["bytes"]
            while True:
                hdr = f.read(14)
                if len(hdr) < 14: break
                _, pn, ps = struct.unpack("<dIH", hdr)
                if ps == 19:
                    probe = f.read(8)
                    if len(probe) < 8: break
                    if struct.unpack("<Q", probe)[0] < 1_000_000_000:
                        f.seek(-8, os.SEEK_CUR)
                pay = int(pn) * int(ps)
                rem = fsz - f.tell()
                if pay < 0 or pay > rem: break
                f.seek(pay, os.SEEK_CUR)
                s["frames"] += 1; s["points"] += int(pn)
    except OSError:
        pass
    return s

def read_event_tail(session_id_str, max_lines=12):
    """Return last max_lines non-empty lines from the event file."""
    paths = _candidate_files(
        os.path.join(EVENTS_DIR, f"events_{session_id_str}.txt"),
        os.path.join(PROJECT_ROOT, f"events_{session_id_str}.txt"),
    )
    if not paths: return []
    if _is_lfs_pointer(paths[-1]): return ["[LFS pointer — run git lfs pull]"]
    lines = []
    try:
        with open(paths[-1], "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                s = line.strip()
                if s: lines.append(s)
    except OSError:
        return ["[Cannot read event file]"]
    return lines[-max_lines:]

def build_session_sparklines(session_id_str):
    out = {k: "n/a" for k in ("alt","batt","vib","flow","slam_hz","imu_accel")}
    paths = _candidate_files(
        os.path.join(TELEMETRY_DIR, f"telemetry_{session_id_str}.csv"),
        os.path.join(PROJECT_ROOT, f"telemetry_{session_id_str}.csv"),
    )
    if not paths or _is_lfs_pointer(paths[-1]):
        return out
    alt_h=[]; batt_h=[]; vib_h=[]; flow_h=[]; hz_h=[]
    try:
        with open(paths[-1], "r", newline="", encoding="utf-8", errors="ignore") as f:
            reader = csv.DictReader(f)
            for row in reader:
                alt_h.append(_safe_float(row.get("Lidar_Alt_m", row.get("Alt_m",0)),0.0))
                batt_h.append(_safe_float(row.get("Batt_V",0),0.0))
                vib_h.append(max(
                    _safe_float(row.get("Vib_X",0),0.0),
                    _safe_float(row.get("Vib_Y",0),0.0),
                    _safe_float(row.get("Vib_Z",0),0.0)))
                flow_h.append(_safe_float(row.get("OptFlow_Qual",0),0.0))
                hz_h.append(_safe_float(row.get("FastLIO_Hz",0),0.0))
    except OSError:
        return out
    W = 28
    if alt_h:  out["alt"]     = create_sparkline(alt_h[-120:],  W)
    if batt_h: out["batt"]    = create_sparkline(batt_h[-120:], W)
    if vib_h:  out["vib"]     = create_sparkline(vib_h[-120:],  W)
    if flow_h: out["flow"]    = create_sparkline(flow_h[-120:], W)
    if hz_h:   out["slam_hz"] = create_sparkline(hz_h[-120:],   W)
    # livox IMU
    imu_paths = _candidate_files(
        os.path.join(LIVOX_DIR, f"livox_imu_{session_id_str}.csv"),
        os.path.join(PROJECT_ROOT, f"livox_imu_{session_id_str}.csv"),
    )
    if imu_paths and not _is_lfs_pointer(imu_paths[-1]):
        ai=[]
        try:
            with open(imu_paths[-1], "r", newline="", encoding="utf-8", errors="ignore") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    ax=_safe_float(row.get("Accel_X",0),0.0)
                    ay=_safe_float(row.get("Accel_Y",0),0.0)
                    az=_safe_float(row.get("Accel_Z",0),0.0)
                    ai.append(math.sqrt(ax*ax+ay*ay+az*az))
        except OSError:
            pass
        if ai: out["imu_accel"] = create_sparkline(ai[-120:], W)
    return out

def _collect_all_sessions():
    """Scan all data directories and build sorted session list."""
    tele_map = {}
    for fp in _candidate_files(
        os.path.join(TELEMETRY_DIR,"telemetry_*.csv"),
        os.path.join(PROJECT_ROOT,"telemetry_*.csv")):
        sid = _extract_session_id(os.path.basename(fp))
        if sid: tele_map[sid] = fp

    # sessions with at least a telemetry file
    sessions = []
    for sid in sorted(tele_map.keys(), reverse=True):
        ts = summarize_telemetry_file(tele_map[sid])
        ev_paths = _candidate_files(
            os.path.join(EVENTS_DIR, f"events_{sid}.txt"),
            os.path.join(PROJECT_ROOT, f"events_{sid}.txt"))
        ev = summarize_event_file(ev_paths[-1]) if ev_paths else {"lines":0,"warn":0,"critical":0,"is_lfs":False}
        imu_paths = _candidate_files(
            os.path.join(LIVOX_DIR, f"livox_imu_{sid}.csv"),
            os.path.join(PROJECT_ROOT, f"livox_imu_{sid}.csv"))
        imu = summarize_livox_imu_file(imu_paths[-1]) if imu_paths else {"samples":0,"max_accel":0.0}
        pc_paths = _candidate_files(
            os.path.join(LIVOX_DIR, f"livox_pointcloud_{sid}.bin"),
            os.path.join(PROJECT_ROOT, f"livox_pointcloud_{sid}.bin"))
        pc = summarize_livox_pc_file(pc_paths[-1]) if pc_paths else {"frames":0,"points":0,"bytes":0}
        sessions.append({
            "sid": sid,
            "rows": ts["rows"], "max_alt": ts["max_alt"],
            "min_batt_v": ts["min_batt_v"], "max_vib": ts["max_vib"],
            "avg_flow_qual": ts["avg_flow_qual"],
            "avg_fastlio_hz": ts["avg_fastlio_hz"],
            "duration_s": ts["duration_s"], "last_mode": ts["last_mode"],
            "ev_lines": ev["lines"], "ev_warn": ev["warn"], "ev_crit": ev["critical"],
            "imu_samples": imu["samples"], "imu_peak_accel": imu["max_accel"],
            "pc_frames": pc["frames"], "pc_points": pc["points"], "pc_bytes": pc["bytes"],
            "is_lfs": ts.get("is_lfs", False),
            "has_livox": bool(imu_paths),
            "has_pc": bool(pc_paths),
        })
    return sessions

def _build_trend_data(sessions):
    """Build cross-session trend arrays (newest-last)."""
    sids=[]; max_alts=[]; min_batts=[]; max_vibs=[]; flow_quals=[]; slam_hzs=[]
    for ss in reversed(sessions):
        if ss["rows"] < 5: continue
        sids.append(ss["sid"][-6:])
        max_alts.append(ss["max_alt"])
        min_batts.append(ss["min_batt_v"])
        max_vibs.append(ss["max_vib"])
        flow_quals.append(ss["avg_flow_qual"])
        slam_hzs.append(ss["avg_fastlio_hz"])
    return {"sids":sids,"max_alts":max_alts,"min_batts":min_batts,
            "max_vibs":max_vibs,"flow_quals":flow_quals,"slam_hzs":slam_hzs}

def _collect_archive_totals(sessions):
    tot = {"sessions":len(sessions),"rows":0,"ev_lines":0,"ev_warn":0,"ev_crit":0,
           "imu_samples":0,"pc_frames":0,"pc_points":0,"pc_bytes":0,
           "max_alt":0.0,"max_vib":0.0,"min_batt":999.0,"lfs":0}
    for ss in sessions:
        tot["rows"]       += ss["rows"]
        tot["ev_lines"]   += ss["ev_lines"]
        tot["ev_warn"]    += ss["ev_warn"]
        tot["ev_crit"]    += ss["ev_crit"]
        tot["imu_samples"]+= ss["imu_samples"]
        tot["pc_frames"]  += ss["pc_frames"]
        tot["pc_points"]  += ss["pc_points"]
        tot["pc_bytes"]   += ss["pc_bytes"]
        tot["max_alt"]     = max(tot["max_alt"], ss["max_alt"])
        tot["max_vib"]     = max(tot["max_vib"], ss["max_vib"])
        if ss["min_batt_v"] > 0:
            tot["min_batt"] = min(tot["min_batt"], ss["min_batt_v"])
        if ss["is_lfs"]: tot["lfs"] += 1
    if tot["min_batt"] == 999.0: tot["min_batt"] = 0.0
    return tot

def _build_ops_todos(sessions, totals):
    todos = []
    if totals["ev_crit"] > 0:
        todos.append(f"Review {totals['ev_crit']} CRITICAL/ABORT events across all sessions")
    if totals["lfs"] > 0:
        todos.append(f"{totals['lfs']} LFS placeholders — run 'git lfs pull' to unlock data")
    if totals["max_vib"] > MAX_GROUND_VIBE:
        todos.append(f"Peak vibration {totals['max_vib']:.1f} m/s² detected — check motor mounts")
    if totals["min_batt"] > 0 and totals["min_batt"] < MIN_SAFE_VOLTAGE:
        todos.append(f"Lowest battery seen: {totals['min_batt']:.2f} V — below safety cutoff {MIN_SAFE_VOLTAGE} V")
    if totals["imu_samples"] == 0:
        todos.append("No Livox IMU data found — verify /livox/imu ROS2 stream")
    if state["flow_qual"] < MIN_PREFLIGHT_FLOW_QUAL and not _OFFLINE_MODE:
        todos.append(f"OptFlow quality {state['flow_qual']} < {MIN_PREFLIGHT_FLOW_QUAL} — poor surface/lighting")
    if not state["rangefinder_healthy"] and not _OFFLINE_MODE:
        todos.append("Rangefinder unhealthy — fix before autonomous flight")
    recent = [s for s in sessions[:5] if s["rows"] > 10]
    if recent:
        avg_vib5 = sum(s["max_vib"] for s in recent) / len(recent)
        if avg_vib5 > 15:
            todos.append(f"Recent avg peak vib {avg_vib5:.1f} m/s² — investigate vibration source")
    if not todos:
        todos.append("No blockers detected — system looks healthy 🟢")
    return todos[:8]

def refresh_all_sessions_bg():
    """Background thread: re-scan all sessions, update state."""
    while True:
        time.sleep(15)
        _do_refresh_sessions()

def _do_refresh_sessions():
    try:
        sessions = _collect_all_sessions()
        totals   = _collect_archive_totals(sessions)
        trend    = _build_trend_data(sessions)
        todos    = _build_ops_todos(sessions, totals)
        state["all_sessions"]        = sessions
        state["all_sessions_loaded"] = True
        state["all_sessions_load_ts"]= time.time()
        state["archive_summary"]     = totals
        state["trend_data"]          = trend
        state["ops_todo"]            = todos
        if sessions:
            idx = min(state["selected_session_idx"], len(sessions)-1)
            _load_session_detail(sessions[idx]["sid"])
    except Exception as e:
        state["archive_last_error"] = str(e)[:120]

def _load_session_detail(sid):
    state["selected_session_detail"]    = next(
        (s for s in state["all_sessions"] if s["sid"] == sid), {})
    state["selected_session_sparklines"]= build_session_sparklines(sid)
    state["selected_session_events"]    = read_event_tail(sid, max_lines=16)

# ─────────────────────────────────────────────────────────────────────────────
# CURSES COLOUR SETUP
# ─────────────────────────────────────────────────────────────────────────────
# Colour pair IDs
C_NORMAL    = 1
C_HEADER    = 2
C_GREEN     = 3
C_YELLOW    = 4
C_RED       = 5
C_CYAN      = 6
C_BLUE      = 7
C_MAGENTA   = 8
C_RED_BOLD  = 9   # used as pair id; bold added via attr
C_DIM       = 10
C_WHITE_BLD = 11

TAB_NAMES = ["1:LIVE", "2:ARCHIVE", "3:SESSION", "4:LIVOX", "5:ANALYSIS", "6:OPS"]

def init_colours():
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(C_NORMAL,   curses.COLOR_WHITE,   -1)
    curses.init_pair(C_HEADER,   curses.COLOR_BLACK,   curses.COLOR_CYAN)
    curses.init_pair(C_GREEN,    curses.COLOR_GREEN,   -1)
    curses.init_pair(C_YELLOW,   curses.COLOR_YELLOW,  -1)
    curses.init_pair(C_RED,      curses.COLOR_RED,     -1)
    curses.init_pair(C_CYAN,     curses.COLOR_CYAN,    -1)
    curses.init_pair(C_BLUE,     curses.COLOR_BLUE,    -1)
    curses.init_pair(C_MAGENTA,  curses.COLOR_MAGENTA, -1)
    curses.init_pair(C_RED_BOLD, curses.COLOR_RED,     -1)
    curses.init_pair(C_DIM,      curses.COLOR_WHITE,   -1)
    curses.init_pair(C_WHITE_BLD,curses.COLOR_WHITE,   -1)

def _safe_addstr(win, y, x, text, attr=0):
    """addstr that silently swallows out-of-bound errors."""
    h, w = win.getmaxyx()
    if y < 0 or y >= h or x < 0 or x >= w:
        return
    max_len = w - x
    if max_len <= 0:
        return
    try:
        win.addstr(y, x, text[:max_len], attr)
    except curses.error:
        pass

def _draw_tabbar(win, active):
    h, w = win.getmaxyx()
    x = 0
    for i, name in enumerate(TAB_NAMES):
        label = f" {name} "
        attr = (curses.color_pair(C_HEADER) | curses.A_BOLD) if i == active \
               else curses.color_pair(C_DIM)
        _safe_addstr(win, 0, x, label, attr)
        x += len(label) + 1
    # right-aligned: session id + time
    ts = datetime.now().strftime("%H:%M:%S")
    info = f" {session_id}  {ts} "
    _safe_addstr(win, 0, max(0, w - len(info)), info, curses.color_pair(C_DIM))

def _hline(win, y, attr=0):
    h, w = win.getmaxyx()
    if 0 <= y < h:
        try:
            win.hline(y, 0, curses.ACS_HLINE, w, attr)
        except curses.error:
            pass

def _bar(val, mx, width=15, fill="█", empty="░"):
    n = int(width * max(0.0, min(1.0, val / mx))) if mx > 0 else 0
    return fill * n + empty * (width - n)

def _color_for_log_key(key):
    return {
        "RED_BOLD": curses.color_pair(C_RED) | curses.A_BOLD,
        "YELLOW":   curses.color_pair(C_YELLOW),
        "CYAN":     curses.color_pair(C_CYAN),
        "WHITE":    curses.color_pair(C_WHITE_BLD) | curses.A_BOLD,
        "NORMAL":   curses.color_pair(C_NORMAL),
    }.get(key, curses.color_pair(C_NORMAL))

# ─────────────────────────────────────────────────────────────────────────────
# TAB 0 — LIVE
# ─────────────────────────────────────────────────────────────────────────────
def draw_live(win):
    h, w = win.getmaxyx()
    win.erase()
    _draw_tabbar(win, 0)
    row = 1

    # ── Mode / Arm banner ────────────────────────────────────────────────────
    mode    = state["mode"]
    armed   = state["armed"]
    macro   = state["macro_status"]
    arm_col = curses.color_pair(C_RED) | curses.A_BOLD if armed \
              else curses.color_pair(C_GREEN)
    arm_txt = "ARMED ●" if armed else "DISARMED ○"
    banner  = f"  {arm_txt}   MODE: {mode:<14}  PHASE: {macro}"
    _safe_addstr(win, row, 0, banner, arm_col)
    row += 1
    _hline(win, row); row += 1

    # ── Three column layout ──────────────────────────────────────────────────
    col1 = 0
    col2 = max(0, w // 3)
    col3 = max(0, 2 * w // 3)
    cw   = w // 3 - 2   # usable width per column

    # ─ LEFT: Altitude & heading ─
    alt_b = state["alt"]
    lid_a = state["lidar_alt"]
    flio_z= state["fastlio_z"]
    fused = state["fused_alt"]
    src   = state["fused_source"]
    _safe_addstr(win, row,   col1, "── ALTITUDE ─────────────────",  curses.color_pair(C_CYAN)|curses.A_BOLD)
    _safe_addstr(win, row+1, col1, f"  Baro  : {alt_b:7.3f} m")
    lid_col = curses.color_pair(C_GREEN) if state["rangefinder_healthy"] else curses.color_pair(C_RED)
    _safe_addstr(win, row+2, col1, f"  Lidar : {lid_a:7.3f} m", lid_col)
    _safe_addstr(win, row+3, col1, f"  FLIO  : {flio_z:7.3f} m", curses.color_pair(C_MAGENTA))
    _safe_addstr(win, row+4, col1, f"  Fused : {fused:7.3f} m  [{src}]", curses.color_pair(C_WHITE_BLD)|curses.A_BOLD)

    climb = state["climb"]
    clr = curses.color_pair(C_GREEN) if climb > 0 else \
          curses.color_pair(C_RED) if climb < -0.05 else curses.color_pair(C_NORMAL)
    _safe_addstr(win, row+5, col1, f"  Climb : {climb:+7.3f} m/s", clr)
    _safe_addstr(win, row+6, col1, f"  Hdg   : {state['hdg']:5d}°")
    row_after_alt = row + 8

    _safe_addstr(win, row_after_alt, col1, "── ATTITUDE ──────────────────", curses.color_pair(C_CYAN)|curses.A_BOLD)
    _safe_addstr(win, row_after_alt+1, col1, f"  Roll  : {state['roll']:+7.2f}°")
    _safe_addstr(win, row_after_alt+2, col1, f"  Pitch : {state['pitch']:+7.2f}°")

    # ─ CENTRE: Sensors ─
    _safe_addstr(win, row,   col2, "── OPTICAL FLOW ─────────────", curses.color_pair(C_CYAN)|curses.A_BOLD)
    fq = state["flow_qual"]
    fq_col = curses.color_pair(C_GREEN) if fq >= 80 else \
             curses.color_pair(C_YELLOW) if fq >= 50 else curses.color_pair(C_RED)
    _safe_addstr(win, row+1, col2, f"  Qual  : {int(fq):3d}  {_bar(fq,255,16)}", fq_col)
    _safe_addstr(win, row+2, col2, f"  Vx    : {state['flow_vx']:+7.3f} m/s  (raw {state['flow_vx_raw']:+.3f})")
    _safe_addstr(win, row+3, col2, f"  Vy    : {state['flow_vy']:+7.3f} m/s  (raw {state['flow_vy_raw']:+.3f})")
    _safe_addstr(win, row+4, col2, f"  |flow|: {state['flow_magnitude']:.3f} m/s")

    vf = state["vibration_filtered"]
    vmax = max(vf)
    vib_col = curses.color_pair(C_RED)|curses.A_BOLD if vmax > MAX_VIBRATION else \
              curses.color_pair(C_YELLOW) if vmax > MAX_GROUND_VIBE else curses.color_pair(C_GREEN)
    _safe_addstr(win, row+6, col2, "── VIBRATION ────────────────", curses.color_pair(C_CYAN)|curses.A_BOLD)
    _safe_addstr(win, row+7, col2, f"  X: {vf[0]:6.2f}  {_bar(vf[0],30,12)}", vib_col)
    _safe_addstr(win, row+8, col2, f"  Y: {vf[1]:6.2f}  {_bar(vf[1],30,12)}", vib_col)
    _safe_addstr(win, row+9, col2, f"  Z: {vf[2]:6.2f}  {_bar(vf[2],30,12)}", vib_col)

    bv  = state["batt_v"];  ba = state["batt_curr"];  bp = state["batt_pct"]
    brow = row + 11
    bv_col = curses.color_pair(C_RED)|curses.A_BOLD if bv < MIN_SAFE_VOLTAGE and bv > 0 else \
             curses.color_pair(C_YELLOW) if bv < MIN_SAFE_VOLTAGE + 0.5 and bv > 0 else \
             curses.color_pair(C_GREEN)
    _safe_addstr(win, brow,   col2, "── BATTERY ──────────────────", curses.color_pair(C_CYAN)|curses.A_BOLD)
    _safe_addstr(win, brow+1, col2, f"  Volt  : {bv:6.2f} V  {_bar(bv,16.8,14)}", bv_col)
    _safe_addstr(win, brow+2, col2, f"  Curr  : {ba:6.2f} A")
    _safe_addstr(win, brow+3, col2, f"  State : {int(bp):3d}%  {_bar(bp,100,14)}")
    _safe_addstr(win, brow+4, col2, f"  CPU   : {int(state['cpu_load']):3d}%  {_bar(state['cpu_load'],100,14)}")

    batt_spark = create_sparkline(state["batt_v_history"][-80:], 28)
    _safe_addstr(win, brow+5, col2, f"  V▸     {batt_spark}", curses.color_pair(C_DIM))

    # ─ RIGHT: FAST-LIO / SLAM ─
    _safe_addstr(win, row, col3, "── FAST-LIO / SLAM ──────────", curses.color_pair(C_CYAN)|curses.A_BOLD)
    slam_st = state["slam_status"]
    scolor = curses.color_pair(C_GREEN)|curses.A_BOLD if slam_st == "HEALTHY" else \
             curses.color_pair(C_YELLOW)|curses.A_BOLD if slam_st == "WARN"   else \
             curses.color_pair(C_RED)|curses.A_BOLD
    _safe_addstr(win, row+1, col3, f"  Status: {slam_st:<10}", scolor)
    _safe_addstr(win, row+2, col3, f"  Age   : {state['slam_age']:.1f} s")
    _safe_addstr(win, row+3, col3, f"  Hz    : {state['fastlio_hz']:.1f} Hz   Pts:{state['fastlio_points']}")
    _safe_addstr(win, row+4, col3, f"  Drops : {state['slam_dropouts']}")
    _safe_addstr(win, row+5, col3, f"  Pos X : {state['fastlio_x']:+8.3f} m")
    _safe_addstr(win, row+6, col3, f"  Pos Y : {state['fastlio_y']:+8.3f} m")
    _safe_addstr(win, row+7, col3, f"  Pos Z : {state['fastlio_z']:+8.3f} m")
    _safe_addstr(win, row+8, col3, f"  Vel X : {state['fastlio_vx']:+7.3f} m/s")
    _safe_addstr(win, row+9, col3, f"  Vel Y : {state['fastlio_vy']:+7.3f} m/s")
    _safe_addstr(win, row+10,col3, f"  Vel Z : {state['fastlio_vz']:+7.3f} m/s")
    _safe_addstr(win, row+11,col3, f"  Roll  : {state['fastlio_roll']:+7.2f}°")
    _safe_addstr(win, row+12,col3, f"  Pitch : {state['fastlio_pitch']:+7.2f}°")
    _safe_addstr(win, row+13,col3, f"  Yaw   : {state['fastlio_yaw']:+7.2f}°")
    ros_ok = state["ros2_active"]
    ros_col = curses.color_pair(C_GREEN) if ros_ok else curses.color_pair(C_RED)
    _safe_addstr(win, row+14,col3, f"  ROS2  : {'ACTIVE' if ros_ok else 'DOWN'}", ros_col)
    _safe_addstr(win, row+15,col3, f"  Livox : {state['livox_pc_frames']} frames  {state['livox_imu_samples']} IMU")

    # ── Commands Help ────────────────────────────────────────────────────────
    crow = row + 14
    _safe_addstr(win, crow, col1, "── COMMANDS ──────────────────", curses.color_pair(C_CYAN)|curses.A_BOLD)
    _safe_addstr(win, crow+1, col1, " [a] ARM    [d] DISARM  [x] KILL", curses.color_pair(C_NORMAL))
    _safe_addstr(win, crow+2, col1, " [1] TO 1m  [2] TO 2m   [j] LAND", curses.color_pair(C_NORMAL))
    _safe_addstr(win, crow+3, col1, " [i/./;/'] Nudge F/B/L/R", curses.color_pair(C_NORMAL))
    _safe_addstr(win, crow+4, col1, " [h] AltHld [l] Loiter", curses.color_pair(C_NORMAL))
    _safe_addstr(win, crow+5, col1, " [c] CAL    [o] SET HOME", curses.color_pair(C_NORMAL))

    # ── Event log ────────────────────────────────────────────────────────────
    log_top = h - 13
    _hline(win, log_top)
    _safe_addstr(win, log_top, 0, "── EVENT LOG ", curses.color_pair(C_CYAN)|curses.A_BOLD)
    conn_txt = f" MAVLink {'●' if state['connected'] else '○'} {state['mav_hz']:.0f}Hz "
    _safe_addstr(win, log_top, max(0, w - len(conn_txt)), conn_txt,
                 curses.color_pair(C_GREEN) if state["connected"] else curses.color_pair(C_RED))
    log_lines = state["log"]
    visible   = h - log_top - 2
    shown     = log_lines[-visible:] if visible > 0 else []
    for i, (ts, msg, key) in enumerate(shown):
        _safe_addstr(win, log_top + 1 + i, 0,
                     f"  {ts} {msg}", _color_for_log_key(key))

    # ── Footer ───────────────────────────────────────────────────────────────
    ftr = " [q]Quit  [1-6]Tabs  [r]Refresh  [TAB]Next tab "
    _safe_addstr(win, h-1, 0, ftr, curses.color_pair(C_HEADER))


# ─────────────────────────────────────────────────────────────────────────────
# TAB 1 — ARCHIVE
# ─────────────────────────────────────────────────────────────────────────────
def draw_archive(win):
    h, w = win.getmaxyx()
    win.erase()
    _draw_tabbar(win, 1)
    row = 1

    tot = state.get("archive_summary", {})
    _hline(win, row); row += 1
    hdr = (f"  Sessions:{tot.get('sessions',0)}  Rows:{tot.get('rows',0):,}  "
           f"Events:{tot.get('ev_lines',0):,}  Warns:{tot.get('ev_warn',0)}  "
           f"Crits:{tot.get('ev_crit',0)}  "
           f"IMU:{tot.get('imu_samples',0):,}  "
           f"PC-frames:{tot.get('pc_frames',0):,}  "
           f"PC-pts:{_fmt_size(tot.get('pc_bytes',0))}  "
           f"MaxAlt:{tot.get('max_alt',0):.2f}m  "
           f"MaxVib:{tot.get('max_vib',0):.1f}")
    _safe_addstr(win, row, 0, hdr, curses.color_pair(C_CYAN))
    row += 1
    _hline(win, row); row += 1

    if not state["all_sessions_loaded"]:
        _safe_addstr(win, row, 2, "⏳ Loading sessions…", curses.color_pair(C_YELLOW))
        _safe_addstr(win, h-1, 0, " [q]Quit  [r]Force refresh  [↑↓]Scroll  [Enter]Open session ",
                     curses.color_pair(C_HEADER))
        return

    sessions = state["all_sessions"]
    # Table header
    COL = ["Session ID      ", "Dur   ", "Alt   ", "Bat   ", "Vib   ",
           "Flow  ", "SLAM  ", "Rows  ", "Evts  ", "Wrn","Crt", "Livox"]
    hdr_str = "".join(COL)
    _safe_addstr(win, row, 0, "  " + hdr_str, curses.color_pair(C_WHITE_BLD)|curses.A_BOLD)
    row += 1

    visible     = h - row - 3
    scroll      = state["archive_scroll"]
    sel_idx     = state["selected_session_idx"]
    page_start  = max(0, min(scroll, len(sessions) - visible))

    for i, ss in enumerate(sessions[page_start : page_start + visible]):
        abs_i = page_start + i
        sid   = ss["sid"]
        is_sel= abs_i == sel_idx
        bg    = curses.A_REVERSE if is_sel else 0
        crit  = ss["ev_crit"] > 0
        warn  = ss["ev_warn"] > 0
        lfs   = ss["is_lfs"]
        if crit:   col = curses.color_pair(C_RED)
        elif lfs:  col = curses.color_pair(C_DIM)
        elif warn: col = curses.color_pair(C_YELLOW)
        else:      col = curses.color_pair(C_NORMAL)

        bv   = f"{ss['min_batt_v']:5.2f}V" if ss["min_batt_v"] > 0 else "  n/a "
        line = (
            f"  {sid:<16}"
            f"{_fmt_dur(ss['duration_s']):<6}"
            f"{ss['max_alt']:5.2f}m "
            f"{bv:<6}"
            f"{ss['max_vib']:5.1f} "
            f"{ss['avg_flow_qual']:5.0f} "
            f"{ss['avg_fastlio_hz']:5.1f} "
            f"{ss['rows']:>6} "
            f"{ss['ev_lines']:>5} "
            f"{ss['ev_warn']:>3} "
            f"{ss['ev_crit']:>3} "
            f"{'✓' if ss['has_livox'] else '·'}"
        )
        _safe_addstr(win, row + i, 0, line, col | bg)

    # scrollbar hint
    pos_pct = int(100 * page_start / max(1, len(sessions)))
    _safe_addstr(win, h-2, 0, f"  ↑↓ scroll  {pos_pct}% of {len(sessions)} sessions  "
                 f"[Enter] open in SESSION tab  [r] refresh",
                 curses.color_pair(C_DIM))
    _safe_addstr(win, h-1, 0, " [q]Quit  [1-6]Tabs  [TAB]Next tab ",
                 curses.color_pair(C_HEADER))


# ─────────────────────────────────────────────────────────────────────────────
# TAB 2 — SESSION
# ─────────────────────────────────────────────────────────────────────────────
def draw_session(win):
    h, w = win.getmaxyx()
    win.erase()
    _draw_tabbar(win, 2)
    row = 1

    ss  = state.get("selected_session_detail", {})
    sls = state.get("selected_session_sparklines", {})
    evs = state.get("selected_session_events", [])

    if not ss:
        _safe_addstr(win, row+1, 2, "No session selected. Go to ARCHIVE tab and press Enter.",
                     curses.color_pair(C_YELLOW))
        _safe_addstr(win, h-1, 0, " [F2]Archive  [q]Quit ", curses.color_pair(C_HEADER))
        return

    sid = ss.get("sid","?")
    _safe_addstr(win, row, 0, f"  SESSION: {sid}   Duration: {_fmt_dur(ss.get('duration_s',0))}   "
                 f"Last mode: {ss.get('last_mode','?')}   Rows: {ss.get('rows',0)}",
                 curses.color_pair(C_CYAN)|curses.A_BOLD)
    row += 1
    _hline(win, row); row += 1

    # ── Summary cards ────────────────────────────────────────────────────────
    cards = [
        ("Max Altitude",   f"{ss.get('max_alt',0):.3f} m"),
        ("Min Battery",    f"{ss.get('min_batt_v',0):.2f} V"),
        ("Max Vibration",  f"{ss.get('max_vib',0):.1f} m/s²"),
        ("Avg OptFlow",    f"{ss.get('avg_flow_qual',0):.0f}/255"),
        ("Avg SLAM Hz",    f"{ss.get('avg_fastlio_hz',0):.1f} Hz"),
        ("Event lines",    str(ss.get("ev_lines",0))),
        ("Warns",          str(ss.get("ev_warn",0))),
        ("Crits",          str(ss.get("ev_crit",0))),
        ("IMU samples",    str(ss.get("imu_samples",0))),
        ("IMU peak accel", f"{ss.get('imu_peak_accel',0):.2f} m/s²"),
        ("PC frames",      str(ss.get("pc_frames",0))),
        ("PC points",      f"{ss.get('pc_points',0):,}"),
    ]
    cw = max(1, w // 4)
    for idx, (label, val) in enumerate(cards):
        cx = (idx % 4) * cw
        cy = row + (idx // 4) * 2
        _safe_addstr(win, cy,   cx, f"  {label}", curses.color_pair(C_DIM))
        _safe_addstr(win, cy+1, cx, f"  {val}",   curses.color_pair(C_WHITE_BLD)|curses.A_BOLD)
    row += (len(cards) // 4) * 2 + 2
    _hline(win, row); row += 1

    # ── Sparklines ───────────────────────────────────────────────────────────
    _safe_addstr(win, row, 0, "  TRENDS ──────────────────────────────────────────",
                 curses.color_pair(C_CYAN)|curses.A_BOLD)
    row += 1
    for label, key in [("Alt (m)   ", "alt"), ("Batt (V)  ", "batt"),
                        ("Vib       ", "vib"), ("OptFlow Q ", "flow"),
                        ("SLAM Hz   ", "slam_hz"), ("Livox IMU ", "imu_accel")]:
        spark = sls.get(key, "n/a")
        _safe_addstr(win, row, 2, f"{label} {spark}", curses.color_pair(C_NORMAL))
        row += 1
        if row >= h - 10: break

    _hline(win, row); row += 1

    # ── Event tail ───────────────────────────────────────────────────────────
    _safe_addstr(win, row, 0, "  RECENT EVENTS ─────────────────────────────────",
                 curses.color_pair(C_CYAN)|curses.A_BOLD)
    row += 1
    avail = h - row - 2
    for line in evs[-avail:]:
        u = line.upper()
        if any(k in u for k in ("CRITICAL","ABORT","FAIL","KILL")):
            col = curses.color_pair(C_RED)|curses.A_BOLD
        elif "WARN" in u:
            col = curses.color_pair(C_YELLOW)
        else:
            col = curses.color_pair(C_NORMAL)
        _safe_addstr(win, row, 2, line, col)
        row += 1
        if row >= h - 2: break

    _safe_addstr(win, h-1, 0, " [2]Archive  [4]Livox  [q]Quit ", curses.color_pair(C_HEADER))


# ─────────────────────────────────────────────────────────────────────────────
# TAB 3 — LIVOX
# ─────────────────────────────────────────────────────────────────────────────
def draw_livox(win):
    h, w = win.getmaxyx()
    win.erase()
    _draw_tabbar(win, 3)
    row = 1

    ss  = state.get("selected_session_detail", {})
    sls = state.get("selected_session_sparklines", {})

    _safe_addstr(win, row, 0,
        "  LIVOX MID-360  —  recording policy: SLAM/Livox for ML training only, NOT used for flight control",
        curses.color_pair(C_CYAN)|curses.A_BOLD)
    row += 1
    _hline(win, row); row += 1

    sid = ss.get("sid","?") if ss else "?"
    _safe_addstr(win, row, 0, f"  Selected session: {sid}", curses.color_pair(C_NORMAL))
    row += 2

    # ── IMU summary ──────────────────────────────────────────────────────────
    _safe_addstr(win, row, 0, "  IMU (Accel + Gyro from /livox/imu)", curses.color_pair(C_YELLOW)|curses.A_BOLD)
    row += 1
    imu_s  = ss.get("imu_samples", 0)   if ss else 0
    imu_pk = ss.get("imu_peak_accel",0) if ss else 0.0
    _safe_addstr(win, row,   2, f"Samples     : {imu_s}")
    _safe_addstr(win, row+1, 2, f"Peak |accel|: {imu_pk:.3f} m/s²")
    row += 3

    spark_imu = sls.get("imu_accel", "n/a")
    _safe_addstr(win, row, 2, f"Accel mag over time: {spark_imu}", curses.color_pair(C_NORMAL))
    row += 2

    # ── Point cloud summary ──────────────────────────────────────────────────
    _safe_addstr(win, row, 0, "  POINT CLOUD (/cloud_registered  →  livox_pointcloud_*.bin)",
                 curses.color_pair(C_YELLOW)|curses.A_BOLD)
    row += 1
    pc_f = ss.get("pc_frames",0)  if ss else 0
    pc_p = ss.get("pc_points",0)  if ss else 0
    pc_b = ss.get("pc_bytes",0)   if ss else 0
    is_lfs = ss.get("is_lfs",False) if ss else False
    if is_lfs:
        _safe_addstr(win, row, 2, "⚠ LFS placeholder — run 'git lfs pull' to fetch real data",
                     curses.color_pair(C_YELLOW))
    else:
        _safe_addstr(win, row,   2, f"Frames : {pc_f}")
        _safe_addstr(win, row+1, 2, f"Points : {pc_p:,}")
        _safe_addstr(win, row+2, 2, f"Size   : {_fmt_size(pc_b)}")
    row += 4

    # ── Cross-session IMU trend ──────────────────────────────────────────────
    _hline(win, row); row += 1
    _safe_addstr(win, row, 0, "  ALL-SESSION IMU PEAK TREND (newest right)",
                 curses.color_pair(C_CYAN)|curses.A_BOLD)
    row += 1
    sessions = state.get("all_sessions", [])
    imu_vals = [ss2["imu_peak_accel"] for ss2 in reversed(sessions) if ss2.get("imu_samples",0)>0]
    if imu_vals:
        spark = create_sparkline(imu_vals, min(w - 4, 80))
        _safe_addstr(win, row, 2, spark, curses.color_pair(C_MAGENTA))
        row += 1
        _safe_addstr(win, row, 2, f"min={min(imu_vals):.2f}  max={max(imu_vals):.2f}  "
                     f"avg={sum(imu_vals)/len(imu_vals):.2f}  ({len(imu_vals)} sessions with Livox IMU)",
                     curses.color_pair(C_DIM))
    else:
        _safe_addstr(win, row, 2, "No Livox IMU data found across sessions.",
                     curses.color_pair(C_YELLOW))
    row += 2

    # ── Cross-session Livox PC frames ────────────────────────────────────────
    _safe_addstr(win, row, 0, "  ALL-SESSION POINT-CLOUD FRAMES (newest right)",
                 curses.color_pair(C_CYAN)|curses.A_BOLD)
    row += 1
    pc_vals = [ss2["pc_frames"] for ss2 in reversed(sessions) if ss2["pc_frames"] > 0]
    if pc_vals:
        spark2 = create_sparkline(pc_vals, min(w - 4, 80))
        _safe_addstr(win, row, 2, spark2, curses.color_pair(C_BLUE))
        row += 1
        _safe_addstr(win, row, 2, f"min={min(pc_vals)}  max={max(pc_vals)}  "
                     f"total={sum(pc_vals):,}  ({len(pc_vals)} sessions with point cloud)",
                     curses.color_pair(C_DIM))
    else:
        _safe_addstr(win, row, 2, "No point cloud data found across sessions.",
                     curses.color_pair(C_YELLOW))

    _safe_addstr(win, h-1, 0, " [2]Archive  [3]Session  [q]Quit ", curses.color_pair(C_HEADER))


# ─────────────────────────────────────────────────────────────────────────────
# TAB 4 — ANALYSIS
# ─────────────────────────────────────────────────────────────────────────────
def _draw_trend_chart(win, row, col, label, values, unit, width, color, hi_bad=None, lo_bad=None):
    """Draw a labelled sparkline trend chart row."""
    if not values:
        _safe_addstr(win, row, col, f"  {label:<18} no data", curses.color_pair(C_DIM))
        return
    spark = create_sparkline(values, width)
    mn = min(values); mx = max(values); avg = sum(values)/len(values)
    warn = (hi_bad is not None and mx > hi_bad) or (lo_bad is not None and mn < lo_bad)
    attr = (curses.color_pair(C_YELLOW)|curses.A_BOLD) if warn else color
    _safe_addstr(win, row, col,
                 f"  {label:<18} {spark} min={mn:.1f} max={mx:.1f} avg={avg:.1f}{unit}", attr)

def draw_analysis(win):
    h, w = win.getmaxyx()
    win.erase()
    _draw_tabbar(win, 4)
    row = 1

    td = state.get("trend_data", {})
    sessions = state.get("all_sessions", [])

    _safe_addstr(win, row, 0,
        f"  CROSS-SESSION ANALYSIS  —  {len(sessions)} sessions loaded (newest right in charts)",
        curses.color_pair(C_CYAN)|curses.A_BOLD)
    row += 1
    _hline(win, row); row += 1

    if not state["all_sessions_loaded"]:
        _safe_addstr(win, row, 2, "⏳ Loading sessions…", curses.color_pair(C_YELLOW))
        _safe_addstr(win, h-1, 0, " [q]Quit  [r]Refresh ", curses.color_pair(C_HEADER)); return

    cw = min(w - 45, 60)   # sparkline width

    _safe_addstr(win, row, 0, "  ALTITUDE PROFILE", curses.color_pair(C_YELLOW)|curses.A_BOLD); row+=1
    _draw_trend_chart(win, row, 0, "Peak alt (m)",    td.get("max_alts",[]),  " m",  cw, curses.color_pair(C_GREEN))
    row += 1

    _safe_addstr(win, row, 0, "  BATTERY HEALTH", curses.color_pair(C_YELLOW)|curses.A_BOLD); row+=1
    _draw_trend_chart(win, row, 0, "Min batt (V)",    td.get("min_batts",[]), " V",  cw, curses.color_pair(C_GREEN), lo_bad=MIN_SAFE_VOLTAGE+0.3)
    row += 1

    _safe_addstr(win, row, 0, "  VIBRATION TREND", curses.color_pair(C_YELLOW)|curses.A_BOLD); row+=1
    _draw_trend_chart(win, row, 0, "Peak vib (m/s²)", td.get("max_vibs",[]),  "",   cw, curses.color_pair(C_MAGENTA), hi_bad=MAX_GROUND_VIBE)
    row += 1

    _safe_addstr(win, row, 0, "  OPTICAL FLOW QUALITY", curses.color_pair(C_YELLOW)|curses.A_BOLD); row+=1
    _draw_trend_chart(win, row, 0, "Avg flow qual",   td.get("flow_quals",[]),"",   cw, curses.color_pair(C_CYAN), lo_bad=MIN_PREFLIGHT_FLOW_QUAL)
    row += 1

    _safe_addstr(win, row, 0, "  SLAM / FAST-LIO HEALTH", curses.color_pair(C_YELLOW)|curses.A_BOLD); row+=1
    _draw_trend_chart(win, row, 0, "Avg SLAM Hz",     td.get("slam_hzs",[]),  " Hz",cw, curses.color_pair(C_BLUE), lo_bad=FASTLIO_MIN_HEALTHY_HZ)
    row += 2

    # ── Quick statistics table ────────────────────────────────────────────────
    _hline(win, row); row += 1
    _safe_addstr(win, row, 0, "  BEST / WORST SESSIONS", curses.color_pair(C_CYAN)|curses.A_BOLD); row+=1

    valid = [s for s in sessions if s["rows"] > 10]
    if valid:
        best_alt  = max(valid, key=lambda s: s["max_alt"])
        worst_vib = max(valid, key=lambda s: s["max_vib"])
        best_flow = max(valid, key=lambda s: s["avg_flow_qual"])
        worst_bat = min((s for s in valid if s["min_batt_v"] > 0),
                        key=lambda s: s["min_batt_v"], default=None)

        _safe_addstr(win, row,   2, f"Highest altitude : {best_alt['sid']}  →  {best_alt['max_alt']:.2f} m", curses.color_pair(C_GREEN))
        _safe_addstr(win, row+1, 2, f"Worst vibration  : {worst_vib['sid']}  →  {worst_vib['max_vib']:.1f} m/s²", curses.color_pair(C_RED))
        _safe_addstr(win, row+2, 2, f"Best OptFlow     : {best_flow['sid']}  →  {best_flow['avg_flow_qual']:.0f}/255", curses.color_pair(C_CYAN))
        if worst_bat:
            _safe_addstr(win, row+3, 2, f"Lowest battery   : {worst_bat['sid']}  →  {worst_bat['min_batt_v']:.2f} V", curses.color_pair(C_YELLOW))

    _safe_addstr(win, h-1, 0, " [2]Archive  [r]Refresh  [q]Quit ", curses.color_pair(C_HEADER))


# ─────────────────────────────────────────────────────────────────────────────
# TAB 5 — OPS
# ─────────────────────────────────────────────────────────────────────────────
def draw_ops(win):
    h, w = win.getmaxyx()
    win.erase()
    _draw_tabbar(win, 5)
    row = 1

    _safe_addstr(win, row, 0, "  OPS BOARD — Pre-flight checklist & data-driven action items",
                 curses.color_pair(C_CYAN)|curses.A_BOLD)
    row += 1
    _hline(win, row); row += 1

    todos = state.get("ops_todo", ["Loading…"])
    for i, todo in enumerate(todos):
        upper = todo.upper()
        if "CRITICAL" in upper or "ABORT" in upper or "FAIL" in upper:
            col = curses.color_pair(C_RED)|curses.A_BOLD
            icon = "✖ "
        elif "WARN" in upper or "INVESTIGATE" in upper or "REVIEW" in upper or "FIX" in upper:
            col = curses.color_pair(C_YELLOW)
            icon = "⚠ "
        elif "HEALTHY" in upper or "BLOCKER" in upper or "🟢" in todo:
            col = curses.color_pair(C_GREEN)|curses.A_BOLD
            icon = "✔ "
        else:
            col = curses.color_pair(C_NORMAL)
            icon = "▸ "
        _safe_addstr(win, row, 2, f"{icon}{todo}", col)
        row += 1

    row += 1
    _hline(win, row); row += 1

    # ── System readiness checklist ───────────────────────────────────────────
    _safe_addstr(win, row, 0, "  SYSTEM READINESS", curses.color_pair(C_YELLOW)|curses.A_BOLD)
    row += 1
    checks = [
        ("MAVLink connection",  state["connected"]),
        ("Rangefinder healthy", state["rangefinder_healthy"]),
        ("OptFlow qual ≥ 80",   state["flow_qual"] >= MIN_PREFLIGHT_FLOW_QUAL),
        ("FAST-LIO healthy",    state["fastlio_healthy"]),
        ("ROS2 active",         state["ros2_active"]),
        ("Battery > 10.5 V",    state["batt_v"] > MIN_SAFE_VOLTAGE),
        ("Vib < 18 m/s²",       max(state["vibration_filtered"]) < MAX_GROUND_VIBE),
    ]
    for label, ok in checks:
        col   = curses.color_pair(C_GREEN)|curses.A_BOLD if ok else curses.color_pair(C_RED)
        mark  = "✔" if ok else "✖"
        mode_note = ""
        if _OFFLINE_MODE and label in ("MAVLink connection","Rangefinder healthy",
                                       "ROS2 active","Battery > 10.5 V","Vib < 18 m/s²"):
            col = curses.color_pair(C_DIM)
            mark = "~"
            mode_note = " (offline)"
        _safe_addstr(win, row, 4, f"[{mark}] {label}{mode_note}", col)
        row += 1

    if _OFFLINE_MODE:
        _safe_addstr(win, row+1, 4, "ℹ Running in OFFLINE mode — live sensor checks skipped",
                     curses.color_pair(C_DIM))

    _safe_addstr(win, h-1, 0, " [2]Archive  [r]Refresh todos  [q]Quit ", curses.color_pair(C_HEADER))


# ─────────────────────────────────────────────────────────────────────────────
# MAVLINK MESSAGE HANDLERS  (matched to v1 ascend_tui.py)
# ─────────────────────────────────────────────────────────────────────────────
def handle_mavlink_message(msg):
    mtype = msg.get_type()
    now   = time.time()

    if mtype == "HEARTBEAT":
        state["connected"] = True
        state["mav_msg_count"] += 1
        if now - state["mav_hz_timer"] >= 1.0:
            state["mav_hz"]        = state["mav_msg_count"] / (now - state["mav_hz_timer"] + 1e-9)
            state["mav_msg_count"] = 0
            state["mav_hz_timer"]  = now
        state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        state["mode"]  = mavutil.mode_string_v10(msg)

    elif mtype == "DISTANCE_SENSOR":
        dist_cm = msg.current_distance
        max_cm  = msg.max_distance
        if dist_cm < max_cm:
            new_alt = round(dist_cm / 100.0, 2)
            prev_raw = state["lidar_alt_raw"]
            if prev_raw > 0 and abs(new_alt - prev_raw) > MAX_LIDAR_STEP_M:
                state["lidar_spike_count"] = state.get("lidar_spike_count", 0) + 1
                if state["lidar_spike_count"] < 3:
                    new_alt = prev_raw
                else:
                    state["lidar_spike_count"] = 0
            else:
                state["lidar_spike_count"] = 0
            state["lidar_alt_raw"] = new_alt
            if state["lidar_alt"] <= 0:
                state["lidar_alt"] = new_alt
            else:
                state["lidar_alt"] = round(low_pass(state["lidar_alt"], new_alt, LIDAR_LPF_ALPHA), 3)
            state["lidar_last_update"]  = now
            state["rangefinder_healthy"] = True
        else:
            state["rangefinder_healthy"] = False

    elif mtype == "OBSTACLE_DISTANCE":
        distances = [d for d in msg.distances if d < 65535]
        if distances:
            state["livox_obs"]     = round(min(distances) / 100.0, 2)
            state["livox_sectors"] = len(distances)
        else:
            state["livox_obs"]     = 0.0
            state["livox_sectors"] = 0

    elif mtype == "OPTICAL_FLOW":
        state["flow_qual"] = msg.quality
        raw_vx = float(msg.flow_comp_m_x)
        raw_vy = float(msg.flow_comp_m_y)
        state["flow_vx_raw"] = round(raw_vx, 3)
        state["flow_vy_raw"] = round(raw_vy, 3)
        state["flow_last_update"] = now

        if state["flow_qual"] < 30:
            state["flow_vx"] = 0.0
            state["flow_vy"] = 0.0
        else:
            clamped_vx = max(-FLOW_SPIKE_REJECT_MPS, min(FLOW_SPIKE_REJECT_MPS, raw_vx))
            clamped_vy = max(-FLOW_SPIKE_REJECT_MPS, min(FLOW_SPIKE_REJECT_MPS, raw_vy))
            state["flow_vx"] = round(low_pass(state["flow_vx"], clamped_vx, FLOW_LPF_ALPHA), 3)
            state["flow_vy"] = round(low_pass(state["flow_vy"], clamped_vy, FLOW_LPF_ALPHA), 3)
        state["flow_magnitude"] = math.sqrt(state["flow_vx"]**2 + state["flow_vy"]**2)

    elif mtype == "VISION_POSITION_ESTIMATE":
        state["vision_x"] = round(msg.x, 2)
        state["vision_y"] = round(msg.y, 2)
        state["vision_z"] = round(msg.z, 2)

    elif mtype == "GLOBAL_POSITION_INT":
        state["alt"] = round(msg.relative_alt / 1000.0, 2)
        state["hdg"] = msg.hdg // 100
        _fuse_altitude()
        _telemetry_log_row()

    elif mtype == "VFR_HUD":
        state["climb"] = round(msg.climb, 2)

    elif mtype == "ATTITUDE":
        state["roll"]  = round(msg.roll  * 57.3, 1)
        state["pitch"] = round(msg.pitch * 57.3, 1)

    elif mtype == "SYS_STATUS":
        state["batt_v"]    = round(msg.voltage_battery / 1000.0, 2)
        state["batt_pct"]  = msg.battery_remaining
        state["batt_curr"] = msg.current_battery / 100.0
        state["cpu_load"]  = msg.load / 10.0
        state["batt_v_history"].append(state["batt_v"])
        if len(state["batt_v_history"]) > 600:
            state["batt_v_history"].pop(0)

    elif mtype == "EKF_STATUS_REPORT":
        state["ekf_flags"] = msg.flags

    elif mtype == "VIBRATION":
        state["vibration"] = [
            round(msg.vibration_x, 3),
            round(msg.vibration_y, 3),
            round(msg.vibration_z, 3),
        ]
        update_vibration_filter(state["vibration"])
        vmax = max(state["vibration"])
        state["vib_max_history"].append(vmax)
        if len(state["vib_max_history"]) > 600:
            state["vib_max_history"].pop(0)
        if vmax > state["max_vib_recorded"]:
            state["max_vib_recorded"] = vmax

    elif mtype == "STATUSTEXT":
        text = msg.text
        if "Field Elevation Set" not in text:
            update_log(f"FC: {text}")

    elif mtype == "SYSTEM_TIME":
        pass


# ─────────────────────────────────────────────────────────────────────────────
# CONTROL LOGIC & HELPERS (Matched to v1)
# ─────────────────────────────────────────────────────────────────────────────

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
        update_log(f"WARN: RC override error: {e}")

def emergency_disarm(master):
    """Emergency disarm - cut throttle and disarm immediately."""
    state['rc_throttle'] = 1000
    state['rc_roll'] = 1500
    state['rc_pitch'] = 1500
    state['rc_yaw'] = 1500
    send_rc_override(master)
    time.sleep(0.1)
    if master:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )
    state['macro_status'] = 'IDLE'
    state['home_set'] = False
    update_log("CMD: EMERGENCY DISARM SENT")

def set_home_position():
    """Lock current position as home for position hold."""
    state['home_z'] = state['fused_alt']
    state['home_x'] = 0.0
    state['home_y'] = 0.0
    state['pos_x'] = 0.0
    state['pos_y'] = 0.0
    state['desired_pos_x'] = 0.0
    state['desired_pos_y'] = 0.0
    state['flow_integral_x'] = 0.0
    state['flow_integral_y'] = 0.0
    state['home_set'] = True
    update_log(f"HOME locked: ({state['home_x']:.1f}, {state['home_y']:.1f}, {state['home_z']:.1f})")

def update_position_estimate(dt):
    """Update position estimate using optical flow integration."""
    if not state['home_set']: return
    if state['flow_qual'] > 50:
        state['flow_integral_x'] += state['flow_vx'] * dt
        state['flow_integral_y'] += state['flow_vy'] * dt
        state['pos_x'] = state['home_x'] + state['flow_integral_x']
        state['pos_y'] = state['home_y'] + state['flow_integral_y']

def get_drift_magnitude():
    if not state['home_set']: return 0.0
    dx = state['pos_x'] - state['home_x']
    dy = state['pos_y'] - state['home_y']
    return math.sqrt(dx**2 + dy**2)

def get_fused_climb_rate():
    """Fuse FC climb with altitude slope for smoother vertical decisions."""
    slope = get_altitude_slope()
    fclimb = (0.6 * state['climb']) + (0.4 * slope)
    state['fused_climb'] = round(fclimb, 3)
    return state['fused_climb']

def get_altitude_slope():
    if len(state['alt_history']) < 10: return 0.0
    recent = state['alt_history'][-10:]
    return (recent[-1] - recent[0]) / 0.5  # approx 10 samples @ 20Hz

def get_avg_climb_rate():
    if len(state['climb_history']) < 5: return 0.0
    return sum(state['climb_history'][-10:]) / max(1, len(state['climb_history'][-10:]))

def get_avg_vibration():
    if not state['vib_max_history']: return 0.0
    return sum(state['vib_max_history'][-20:]) / max(1, len(state['vib_max_history'][-20:]))

def format_trend_arrow(val, th_hi=0.05, th_lo=-0.05):
    if val > th_hi: return "↑"
    if val < th_lo: return "↓"
    return "→"

def get_sensor_health_char(cond):
    return "●" if cond else "○"

def get_sensor_confidence(name):
    if name == "LIDAR":   return 95 if state['rangefinder_healthy'] else 0
    if name == "BARO":    return 60 if state['alt'] > 0 else 30
    if name == "SLAM":    return int(state['slam_quality'])
    if name == "OPTFLOW":
        q = state['flow_qual']
        return 85 if q > 150 else 70 if q > 100 else 50 if q > 50 else 20 if q > 20 else 0
    return 0

def estimate_link_quality():
    return 95 if state['connected'] else 0

def get_battery_drain_rate():
    if len(state['batt_pct_history']) < 10: return 0.0, 0.0
    pd = state['batt_pct_history'][0] - state['batt_pct_history'][-1]
    vd = state['batt_v_history'][0] - state['batt_v_history'][-1]
    return vd/0.5, pd/0.5

def get_battery_time_remaining():
    vd, pd = get_battery_drain_rate()
    if pd > 0: return state['batt_pct'] / (pd * 60.0)
    return 0.0

def check_emergency_rtl(master):
    if not state['armed'] or state['macro_status'] == 'IDLE': return
    if state['batt_v'] > 1.0 and state['batt_v'] < 10.5:
        update_log("CRITICAL: Low Battery! Emergency Disarm!")
        emergency_disarm(master)
        return
    if abs(state['roll']) > 45 or abs(state['pitch']) > 45:
        update_log("CRITICAL: Excessive Tilt! Emergency Disarm!")
        emergency_disarm(master)
        return

def run_preflight_diagnostics():
    update_log("SYS: Running Pre-Flight Diagnostics...")
    if state['batt_v'] > 1.0 and state['batt_v'] < MIN_SAFE_VOLTAGE:
        update_log(f"DIAG: FAIL - Battery too low ({state['batt_v']}V)")
        return False
    if state['flow_qual'] < MIN_PREFLIGHT_FLOW_QUAL:
        update_log(f"DIAG: FAIL - Flow quality low ({state['flow_qual']})")
        return False
    if not state['rangefinder_healthy']:
        update_log("DIAG: FAIL - Rangefinder unavailable")
        return False
    update_log(f"DIAG: PASS - Systems Nominal. FusedAlt: {state['fused_alt']:.2f}m")
    return True

# ─────────────────────────────────────────────────────────────────────────────
# FLIGHT AUTOMATION SEQUENCES (Matched to v1)
# ─────────────────────────────────────────────────────────────────────────────

def auto_takeoff_sequence(master, target_alt=1.0):
    if state['macro_status'] != 'IDLE': return
    state['macro_status'] = 'TAKEOFF'
    
    if not run_preflight_diagnostics():
        state['macro_status'] = 'IDLE'
        return

    update_log(f"AUTO: Takeoff to {target_alt}m initiation...")
    
    # Force EKF Origin / Home
    if master:
        master.mav.param_set_send(master.target_system, master.target_component, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0, 0, 0, 0, 0, 0)
    
    time.sleep(1.0)
    update_log("AUTO: Mode -> STABILIZE (Arming)")
    if master: master.set_mode(master.mode_mapping()['STABILIZE'])
    
    # Arming
    update_log("AUTO: Sending ARM command...")
    if master:
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    
    for _ in range(30):
        if state['armed']: break
        time.sleep(0.1)
    
    if not state['armed']:
        update_log("AUTO: ARM FAILED. Aborting.")
        state['macro_status'] = 'IDLE'
        return

    update_log("AUTO: Armed. Spooling up...")
    state['rc_throttle'] = SPOOL_THR
    send_rc_override(master)
    time.sleep(1.5)

    update_log("AUTO: Mode -> ALT_HOLD (Liftoff)")
    if master: master.set_mode(master.mode_mapping()['ALT_HOLD'])
    
    start_alt = state['fused_alt']
    abs_target = start_alt + target_alt
    takeoff_start = time.time()
    
    while time.time() - takeoff_start < 15.0 and state['armed']:
        curr_alt = state['fused_alt']
        dist_to_go = abs_target - curr_alt
        
        # Adaptive climb rate
        if dist_to_go < 0.1:
            state['rc_throttle'] = 1515
        elif dist_to_go < 0.3:
            state['rc_throttle'] = 1560
        else:
            state['rc_throttle'] = 1650
            
        send_rc_override(master)
        
        if curr_alt >= abs_target - 0.05:
            update_log(f"AUTO: Target {target_alt}m reached.")
            break
            
        if abs(state['roll']) > 35 or abs(state['pitch']) > 35:
            update_log("AUTO: CRITICAL TILT! KILL.")
            emergency_disarm(master)
            return
            
        time.sleep(0.05)

    update_log("AUTO: Transitioning to LOITER (Hover)...")
    if master: master.set_mode(master.mode_mapping()['LOITER'])
    state['rc_throttle'] = HOVER_THR
    send_rc_override(master)
    set_home_position()
    state['macro_status'] = 'HOVER'

def auto_land_sequence(master):
    if state['macro_status'] not in ('IDLE', 'HOVER', 'TAKEOFF'): return
    if not state['armed']: return
    
    update_log("AUTO: Landing. Mode -> LAND")
    state['macro_status'] = 'LANDING'
    if master: master.set_mode(master.mode_mapping()['LAND'])
    
    state['rc_throttle'] = 1500
    state['rc_pitch'] = 1500
    state['rc_roll'] = 1500
    send_rc_override(master)
    
    land_start = time.time()
    while state['armed'] and time.time() - land_start < 25.0:
        if state['macro_status'] != 'LANDING': break
        send_rc_override(master)
        time.sleep(0.05)
        
    if not state['armed']:
        update_log("AUTO: Disarmed. Landing complete.")
    state['macro_status'] = 'IDLE'

def auto_move_sequence(master, direction):
    if state['macro_status'] != 'HOVER':
        update_log("MOVE: Must be in HOVER to nudge")
        return
    
    MOVE_PWM = 120
    DUR = 0.8
    
    orig_p = state['rc_pitch']
    orig_r = state['rc_roll']
    
    update_log(f"MOVE: Nudge {direction}...")
    state['macro_status'] = 'MOVING'
    
    if direction == 'front': state['rc_pitch'] = 1500 - MOVE_PWM
    if direction == 'back':  state['rc_pitch'] = 1500 + MOVE_PWM
    if direction == 'left':  state['rc_roll']  = 1500 - MOVE_PWM
    if direction == 'right': state['rc_roll']  = 1500 + MOVE_PWM
    
    start = time.time()
    while time.time() - start < DUR:
        send_rc_override(master)
        time.sleep(0.05)
        
    state['rc_pitch'] = 1500
    state['rc_roll']  = 1500
    send_rc_override(master)
    update_log("MOVE: Nudge complete.")
    state['macro_status'] = 'HOVER'


def _fuse_altitude():
    """Weighted multi-sensor altitude fusion (rangefinder favoured)."""
    lidar_ok = state["rangefinder_healthy"]
    slam_ok  = state["fastlio_healthy"]
    baro     = state["alt"]
    lidar    = state["lidar_alt"]
    slam_z   = state["fastlio_z"]

    if lidar_ok:
        w_lidar, w_slam, w_baro = 0.75, 0.15 if slam_ok else 0.0, 0.10
        src = "LIDAR+SLAM" if slam_ok else "LIDAR"
    elif slam_ok:
        w_lidar, w_slam, w_baro = 0.0, 0.60, 0.40
        src = "SLAM+BARO"
    else:
        w_lidar, w_slam, w_baro = 0.0, 0.0, 1.0
        src = "BARO"

    total = w_lidar + w_slam + w_baro
    if total > 0:
        fused = (w_lidar*lidar + w_slam*slam_z + w_baro*baro) / total
    else:
        fused = baro

    state["fused_alt"]     = low_pass(state["fused_alt"], fused, 0.4)
    state["fused_weights"] = {"lidar": w_lidar, "slam": w_slam, "baro": w_baro}
    state["fused_source"]  = src


def _telemetry_log_row():
    row = [
        datetime.now().strftime("%H:%M:%S.%f")[:12],
        state["mode"], state["armed"],
        round(state["alt"], 4), round(state["lidar_alt"], 4), round(state["climb"], 4),
        round(state["roll"], 3), round(state["pitch"], 3), state["hdg"],
        round(state["batt_v"], 3), round(state["batt_curr"], 3), state["batt_pct"],
        state["rc_throttle"],
        state["flow_qual"], round(state["flow_vx"], 4), round(state["flow_vy"], 4),
        round(state["vibration_filtered"][0], 4),
        round(state["vibration_filtered"][1], 4),
        round(state["vibration_filtered"][2], 4),
        state["cpu_load"],
        round(state["livox_obs"], 3), state["livox_sectors"],
        round(state["vision_x"], 4), round(state["vision_y"], 4), round(state["vision_z"], 4),
        round(state["fastlio_x"], 4), round(state["fastlio_y"], 4), round(state["fastlio_z"], 4),
        round(state["fastlio_vx"], 4), round(state["fastlio_vy"], 4), round(state["fastlio_vz"], 4),
        round(state["fastlio_roll"], 3), round(state["fastlio_pitch"], 3), round(state["fastlio_yaw"], 3),
        round(state["fastlio_hz"], 2), state["fastlio_points"],
    ]
    if csv_writer:
        csv_writer.writerow(row)
        telemetry_file.flush()


# ─────────────────────────────────────────────────────────────────────────────
# MAVLINK RECEIVE THREAD
# ─────────────────────────────────────────────────────────────────────────────
def mavlink_thread(connection_str):
    if not MAVLINK_AVAILABLE:
        update_log("SYS: MAVLink library not available — skipping connection")
        return
    update_log(f"SYS: Connecting to {connection_str} …")
    try:
        mav = mavutil.mavlink_connection(connection_str, baud=921600)
        mav.wait_heartbeat(timeout=10)
        state["mav"]       = mav
        state["connected"] = True
        update_log(f"SYS: Connected   sysid={mav.target_system}")
    except Exception as e:
        update_log(f"WARN: MAVLink connect failed: {e}")
        return

    # Request all data streams at 10 Hz (matches v1)
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )

    while True:
        try:
            msg = mav.recv_match(blocking=True, timeout=0.1)
            if msg:
                handle_mavlink_message(msg)
        except Exception as e:
            update_log(f"WARN: MAVLink read error: {e}")
            time.sleep(1)


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 / FAST-LIO THREAD (optional)
# ─────────────────────────────────────────────────────────────────────────────
def ros2_thread():
    if not ROS2_AVAILABLE:
        state["ros2_error"] = "rclpy not found"
        return
    try:
        rclpy.init()
        node = rclpy.create_node("ascend_tui_v2")

        def _odom_cb(msg):
            pos = msg.pose.pose.position
            vel = msg.twist.twist.linear
            ori = msg.pose.pose.orientation
            import math as _m
            # quaternion → euler yaw
            siny = 2*(ori.w*ori.z + ori.x*ori.y)
            cosy = 1 - 2*(ori.y*ori.y + ori.z*ori.z)
            sinp = 2*(ori.w*ori.y - ori.z*ori.x)
            sinr = 2*(ori.w*ori.x + ori.y*ori.z)
            cosr = 1 - 2*(ori.x*ori.x + ori.y*ori.y)

            now = time.time()
            dt  = now - state["fastlio_last_odom_time"] if state["fastlio_last_odom_time"] > 0 else 0.033
            state["fastlio_last_odom_time"] = now; state["fastlio_dt"] = dt

            state["fastlio_x"] = pos.x; state["fastlio_y"] = pos.y; state["fastlio_z"] = pos.z
            state["fastlio_vx"]= vel.x; state["fastlio_vy"]= vel.y; state["fastlio_vz"]= vel.z
            state["fastlio_roll"]  = _m.degrees(_m.atan2(sinr, cosr))
            state["fastlio_pitch"] = _m.degrees(_m.asin(max(-1,min(1,sinp))))
            state["fastlio_yaw"]   = _m.degrees(_m.atan2(siny, cosy))
            state["fastlio_msg_count"] += 1

            if now - state["fastlio_hz_timer"] >= 1.0:
                state["fastlio_hz"]      = state["fastlio_msg_count"] / (now-state["fastlio_hz_timer"]+1e-9)
                state["fastlio_msg_count"] = 0
                state["fastlio_hz_timer"]  = now

            state["fastlio_last_time"] = now
            state["slam_age"]          = 0.0
            age_limit = 2.0
            state["fastlio_healthy"]   = (now - state["fastlio_last_time"]) < age_limit
            state["slam_status"] = ("HEALTHY" if state["fastlio_hz"] >= FASTLIO_MIN_HEALTHY_HZ
                                    else "WARN" if state["fastlio_healthy"] else "LOST")

        def _cloud_cb(msg):
            state["fastlio_points"] = msg.width * msg.height

        node.create_subscription(Odometry,    "/Odometry",         _odom_cb,  10)
        node.create_subscription(PointCloud2, "/cloud_registered", _cloud_cb, 2)
        state["ros2_active"]    = True
        state["ros2_init_time"] = time.time()
        update_log("SYS: ROS2 active — FAST-LIO subscriptions created")
        rclpy.spin(node)
    except Exception as e:
        state["ros2_error"] = str(e)[:80]
        update_log(f"WARN: ROS2 init failed: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# SLAM AGE WATCHDOG
# ─────────────────────────────────────────────────────────────────────────────
def slam_watchdog_thread():
    while True:
        time.sleep(0.5)
        now = time.time()
        if state["fastlio_last_time"] > 0:
            state["slam_age"] = now - state["fastlio_last_time"]
        if state["slam_age"] > FASTLIO_TIMEOUT:
            if state["fastlio_healthy"]:
                state["fastlio_healthy"] = False
                state["slam_dropouts"]  += 1
                state["slam_status"]     = "LOST"
                msg = f"WARN: SLAM lost at alt {state['lidar_alt']:.2f} m"
                if not _dedup_slam_warn(msg):
                    update_log(msg)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN CURSES LOOP
# ─────────────────────────────────────────────────────────────────────────────
DRAW_FNS = [draw_live, draw_archive, draw_session, draw_livox, draw_analysis, draw_ops]

def curses_main(stdscr, args):
    global _OFFLINE_MODE
    _OFFLINE_MODE = args.offline

    curses.curs_set(0)
    stdscr.timeout(150)   # ms — controls refresh rate
    init_colours()

    state["session_start_time"] = time.time()
    state["mav_hz_timer"]       = time.time()
    state["fastlio_hz_timer"]   = time.time()

    # Initial archive load in foreground (fast; just lists files)
    _do_refresh_sessions()

    while True:
        tab = state["active_tab"]
        try:
            DRAW_FNS[tab](stdscr)
            stdscr.refresh()
        except curses.error:
            pass

        if not _OFFLINE_MODE and state['mav']:
            # Periodic control link tasks (Arm safety, RC overrides)
            check_emergency_rtl(state['mav'])
            if state['macro_status'] != 'IDLE':
                # Sequences handle their own RC overrides, but we ensure link keepalive
                pass
            else:
                # Manual stick reset in IDLE
                state['rc_pitch'] = 1500
                state['rc_roll']  = 1500
                state['rc_yaw']   = 1500
                send_rc_override(state['mav'])

        # ── Key handling ─────────────────────────────────────────────────────
        ch = stdscr.getch()
        if ch == ord("q") or ch == ord("Q"):
            break

        elif ch == ord("1"):  state["active_tab"] = 0
        elif ch == ord("2"):  state["active_tab"] = 1
        elif ch == ord("3"):  state["active_tab"] = 2
        elif ch == ord("4"):  state["active_tab"] = 3
        elif ch == ord("5"):  state["active_tab"] = 4
        elif ch == ord("6"):  state["active_tab"] = 5

        elif ch == ord("\t"):  # Tab → next tab
            state["active_tab"] = (tab + 1) % len(TAB_NAMES)

        elif ch == curses.KEY_BTAB:  # Shift-Tab → prev tab
            state["active_tab"] = (tab - 1) % len(TAB_NAMES)

        elif ch == ord("r") or ch == ord("R"):
            # Force refresh archive data
            threading.Thread(target=_do_refresh_sessions, daemon=True).start()

        # ─ LIVE COMMANDS ─
        elif not _OFFLINE_MODE and tab == 0:
            master = state['mav']
            if ch == ord('a'):
                update_log("CMD: Arming requested")
                if master: master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
            elif ch == ord('d'):
                update_log("CMD: Disarming requested")
                if master: master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            elif ch == ord('1'):
                threading.Thread(target=auto_takeoff_sequence, args=(master, 1.0), daemon=True).start()
            elif ch == ord('2'):
                threading.Thread(target=auto_takeoff_sequence, args=(master, 2.0), daemon=True).start()
            elif ch == ord('j'):
                threading.Thread(target=auto_land_sequence, args=(master,), daemon=True).start()
            elif ch == ord('i'):
                threading.Thread(target=auto_move_sequence, args=(master, 'front'), daemon=True).start()
            elif ch == ord(','):
                threading.Thread(target=auto_move_sequence, args=(master, 'back'), daemon=True).start()
            elif ch == ord(';'):
                threading.Thread(target=auto_move_sequence, args=(master, 'left'), daemon=True).start()
            elif ch == ord("'"):
                threading.Thread(target=auto_move_sequence, args=(master, 'right'), daemon=True).start()
            elif ch == ord('x'):
                update_log("CMD: *** EMERGENCY KILL ***")
                emergency_disarm(master)
            elif ch == ord('h'):
                if master: master.set_mode(master.mode_mapping()['ALT_HOLD'])
                update_log("CMD: Mode -> ALT_HOLD")
            elif ch == ord('l'):
                if master: master.set_mode(master.mode_mapping()['LOITER'])
                update_log("CMD: Mode -> LOITER")
            elif ch == ord('c'):
                update_log("CMD: Calibration requested")
                if master: master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 1, 0, 0, 0, 0)
            elif ch == ord('o'):
                update_log("CMD: Set Home / Origin")
                if master:
                    master.mav.set_gps_global_origin_send(master.target_system, int(19.033 * 1e7), int(73.029 * 1e7), 0)
                    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0, 0, 0, 0, 0, 0)

        elif ch == curses.KEY_UP:
            if tab == 1:  # ARCHIVE
                sess = state["all_sessions"]
                idx  = max(0, state["selected_session_idx"] - 1)
                state["selected_session_idx"] = idx
                state["archive_scroll"] = max(0, min(state["archive_scroll"], idx))
                if sess:
                    threading.Thread(target=_load_session_detail,
                                     args=(sess[idx]["sid"],), daemon=True).start()
            elif tab == 0 and not _OFFLINE_MODE:
                state['rc_throttle'] = min(state['rc_throttle'] + 25, 2000)

        elif ch == curses.KEY_DOWN:
            if tab == 1:
                sess = state["all_sessions"]
                if sess:
                    idx = min(len(sess)-1, state["selected_session_idx"] + 1)
                    state["selected_session_idx"] = idx
                    # auto-scroll
                    h, _ = stdscr.getmaxyx()
                    visible = h - 8
                    if idx >= state["archive_scroll"] + visible:
                        state["archive_scroll"] = idx - visible + 1
                    threading.Thread(target=_load_session_detail,
                                     args=(sess[idx]["sid"],), daemon=True).start()
            elif tab == 0 and not _OFFLINE_MODE:
                state['rc_throttle'] = max(state['rc_throttle'] - 25, 1000)

        elif ch == ord(" "):
            if tab == 0 and not _OFFLINE_MODE:
                state['rc_throttle'] = HOVER_THR
                update_log("CMD: Throttle -> HOVER (1500)")

        elif ch == curses.KEY_PPAGE:  # PageUp
            if tab == 1:
                h, _ = stdscr.getmaxyx()
                page = max(6, h - 10)
                state["selected_session_idx"] = max(0, state["selected_session_idx"] - page)
                state["archive_scroll"]        = max(0, state["archive_scroll"] - page)

        elif ch == curses.KEY_NPAGE:  # PageDown
            if tab == 1:
                sess = state["all_sessions"]
                if sess:
                    h, _ = stdscr.getmaxyx()
                    page = max(6, h - 10)
                    idx  = min(len(sess)-1, state["selected_session_idx"] + page)
                    state["selected_session_idx"] = idx
                    state["archive_scroll"]        = max(0, idx - (h-10) + 1)

        elif ch in (curses.KEY_ENTER, ord("\n"), ord("\r")):
            if tab == 1:  # ARCHIVE → open in SESSION tab
                state["active_tab"] = 2

        elif ch == curses.KEY_RESIZE:
            stdscr.clear()


def main():
    parser = argparse.ArgumentParser(description="Ascend Mission Control TUI v9.0")
    parser.add_argument("--offline", action="store_true",
                        help="Run without MAVLink/ROS2 (archive analysis only)")
    parser.add_argument("--port",    default="/dev/ttyAMA0",
                        help="MAVLink connection string (default: /dev/ttyAMA0)")
    parser.add_argument("--baud",    type=int, default=921600,
                        help="Serial baud rate if using serial port")
    args = parser.parse_args()

    if not args.offline:
        _open_log_files()

    # Background threads
    if not args.offline:
        threading.Thread(target=mavlink_thread,      args=(args.port,), daemon=True).start()
        threading.Thread(target=ros2_thread,                             daemon=True).start()
        threading.Thread(target=slam_watchdog_thread,                    daemon=True).start()

    threading.Thread(target=refresh_all_sessions_bg, daemon=True).start()

    try:
        curses.wrapper(curses_main, args)
    except KeyboardInterrupt:
        pass
    finally:
        # Close log files cleanly
        for f in (telemetry_file, event_file, livox_pc_file, livox_imu_file):
            try:
                if f: f.close()
            except Exception:
                pass
        print("Ascend TUI v9.0 — session closed.")
        if not args.offline:
            print(f"  Telemetry : {csv_filename}")
            print(f"  Events    : {txt_filename}")
            print(f"  Livox PC  : {livox_pc_filename}")
            print(f"  Livox IMU : {livox_imu_filename}")


if __name__ == "__main__":
    main()
