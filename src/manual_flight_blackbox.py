#!/usr/bin/env python3
"""Manual RC flight blackbox logger.

Outputs per session:
- telemetry_snapshot_<session>.csv: fixed-rate state snapshot for quick analysis.
- mavlink_raw_<session>.jsonl: every MAVLink message payload for full replay/audit.
- session_meta_<session>.json: run configuration and environment details.
"""

import argparse
import csv
import json
import math
import os
import signal
import threading
import time
from datetime import datetime
from typing import Any, Dict

from pymavlink import mavutil


def now_ts() -> float:
    return time.time()


def iso_now() -> str:
    return datetime.utcnow().isoformat(timespec="milliseconds") + "Z"


def sanitize_for_json(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, (list, tuple)):
        return [sanitize_for_json(v) for v in value]
    if isinstance(value, dict):
        return {str(k): sanitize_for_json(v) for k, v in value.items()}
    return str(value)


class RecorderState:
    def __init__(self):
        self.lock = threading.Lock()
        self.data: Dict[str, Any] = {
            "session_time": 0.0,
            "flight_mode": "UNKNOWN",
            "armed": False,
            "cpu_load_pct": 0.0,
            "ekf_flags": 0,
            "gps_fix_type": 0,
            "gps_satellites": 0,
            "gps_hdop": 0.0,
            "lat_deg": 0.0,
            "lon_deg": 0.0,
            "alt_msl_m": 0.0,
            "rel_alt_m": 0.0,
            "groundspeed_ms": 0.0,
            "airspeed_ms": 0.0,
            "climb_ms": 0.0,
            "throttle_pct": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
            "heading_deg": 0.0,
            "rollspeed": 0.0,
            "pitchspeed": 0.0,
            "yawspeed": 0.0,
            "local_x_m": 0.0,
            "local_y_m": 0.0,
            "local_z_m": 0.0,
            "local_vx_ms": 0.0,
            "local_vy_ms": 0.0,
            "local_vz_ms": 0.0,
            "flow_quality": 0,
            "flow_x_ms": 0.0,
            "flow_y_ms": 0.0,
            "flow_ground_distance_m": 0.0,
            "rangefinder_m": 0.0,
            "rangefinder_type": -1,
            "obstacle_min_m": 0.0,
            "obstacle_valid_bins": 0,
            "baro_alt_m": 0.0,
            "batt_voltage_v": 0.0,
            "batt_current_a": 0.0,
            "batt_remaining_pct": 0,
            "battery_temp_c": 0.0,
            "vib_x": 0.0,
            "vib_y": 0.0,
            "vib_z": 0.0,
            "imu_ax": 0.0,
            "imu_ay": 0.0,
            "imu_az": 0.0,
            "imu_gx": 0.0,
            "imu_gy": 0.0,
            "imu_gz": 0.0,
            "rc1": 0,
            "rc2": 0,
            "rc3": 0,
            "rc4": 0,
            "rc5": 0,
            "rc6": 0,
            "rc7": 0,
            "rc8": 0,
            "servo1": 0,
            "servo2": 0,
            "servo3": 0,
            "servo4": 0,
            "servo5": 0,
            "servo6": 0,
            "servo7": 0,
            "servo8": 0,
        }

    def update(self, **kwargs):
        with self.lock:
            self.data.update(kwargs)

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return dict(self.data)


class ManualFlightBlackbox:

    def __init__(self, args):
        self.args = args
        self.state = RecorderState()
        self.stop_event = threading.Event()
        self.start_ts = now_ts()

        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(args.out_dir, f"manual_flight_{self.session_id}")
        os.makedirs(self.session_dir, exist_ok=True)

        self.paths = {
            "telemetry_csv": os.path.join(self.session_dir, f"telemetry_snapshot_{self.session_id}.csv"),
            "mavlink_jsonl": os.path.join(self.session_dir, f"mavlink_raw_{self.session_id}.jsonl"),
            "meta_json": os.path.join(self.session_dir, f"session_meta_{self.session_id}.json"),
        }

        self.telemetry_fp = open(self.paths["telemetry_csv"], "w", newline="", encoding="utf-8")
        self.telemetry_writer = None

        self.mav_jsonl_fp = open(self.paths["mavlink_jsonl"], "w", encoding="utf-8")

        self.master = None
        self.meta_started_utc = iso_now()

    def write_meta(self):
        snapshot = self.state.snapshot()
        meta = {
            "session_id": self.session_id,
            "started_utc": self.meta_started_utc,
            "mavlink_port": self.args.port,
            "mavlink_baud": self.args.baud,
            "snapshot_hz": self.args.snapshot_hz,
            "request_stream_hz": self.args.stream_hz,
            "output_files": self.paths,
        }
        with open(self.paths["meta_json"], "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)

    def connect_mavlink(self):
        print(f"[*] Connecting to Pixhawk on {self.args.port} @ {self.args.baud}...")
        self.master = mavutil.mavlink_connection(self.args.port, baud=self.args.baud)
        self.master.wait_heartbeat(timeout=20)
        print("[*] Heartbeat received. Logging started.")

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            int(self.args.stream_hz),
            1,
        )

    def message_to_jsonl(self, msg):
        payload = sanitize_for_json(msg.to_dict())
        rec = {
            "ts": now_ts(),
            "msg_type": msg.get_type(),
            "src_sys": msg.get_srcSystem() if hasattr(msg, "get_srcSystem") else None,
            "src_comp": msg.get_srcComponent() if hasattr(msg, "get_srcComponent") else None,
            "payload": payload,
        }
        self.mav_jsonl_fp.write(json.dumps(rec, separators=(",", ":")) + "\n")

    def update_state_from_mavlink(self, msg):
        m_type = msg.get_type()

        if m_type == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.state.update(armed=armed, flight_mode=mavutil.mode_string_v10(msg))

        elif m_type == "ATTITUDE":
            self.state.update(
                roll_deg=round(math.degrees(msg.roll), 3),
                pitch_deg=round(math.degrees(msg.pitch), 3),
                yaw_deg=round(math.degrees(msg.yaw), 3),
                rollspeed=round(float(msg.rollspeed), 5),
                pitchspeed=round(float(msg.pitchspeed), 5),
                yawspeed=round(float(msg.yawspeed), 5),
            )

        elif m_type == "VFR_HUD":
            self.state.update(
                groundspeed_ms=round(float(msg.groundspeed), 3),
                airspeed_ms=round(float(msg.airspeed), 3),
                baro_alt_m=round(float(msg.alt), 3),
                climb_ms=round(float(msg.climb), 3),
                heading_deg=round(float(msg.heading), 2),
                throttle_pct=round(float(msg.throttle), 2),
            )

        elif m_type == "GLOBAL_POSITION_INT":
            self.state.update(
                lat_deg=round(msg.lat / 1e7, 8),
                lon_deg=round(msg.lon / 1e7, 8),
                alt_msl_m=round(msg.alt / 1000.0, 3),
                rel_alt_m=round(msg.relative_alt / 1000.0, 3),
            )

        elif m_type == "GPS_RAW_INT":
            self.state.update(
                gps_fix_type=int(msg.fix_type),
                gps_satellites=int(msg.satellites_visible),
                gps_hdop=round(msg.eph / 100.0, 2) if msg.eph != 65535 else 0.0,
            )

        elif m_type == "LOCAL_POSITION_NED":
            self.state.update(
                local_x_m=round(float(msg.x), 4),
                local_y_m=round(float(msg.y), 4),
                local_z_m=round(float(msg.z), 4),
                local_vx_ms=round(float(msg.vx), 4),
                local_vy_ms=round(float(msg.vy), 4),
                local_vz_ms=round(float(msg.vz), 4),
            )

        elif m_type == "OPTICAL_FLOW":
            self.state.update(
                flow_quality=int(msg.quality),
                flow_x_ms=round(float(msg.flow_comp_m_x), 4),
                flow_y_ms=round(float(msg.flow_comp_m_y), 4),
                flow_ground_distance_m=round(float(msg.ground_distance), 4),
            )

        elif m_type == "DISTANCE_SENSOR":
            self.state.update(
                rangefinder_m=round(float(msg.current_distance) / 100.0, 4),
                rangefinder_type=int(msg.type),
            )

        elif m_type == "OBSTACLE_DISTANCE":
            valid = [d for d in msg.distances if d < 65535]
            min_m = min(valid) / 100.0 if valid else 0.0
            self.state.update(obstacle_min_m=round(min_m, 4), obstacle_valid_bins=len(valid))

        elif m_type == "SYS_STATUS":
            self.state.update(
                batt_voltage_v=round(float(msg.voltage_battery) / 1000.0, 4),
                batt_current_a=round(float(msg.current_battery) / 100.0, 4),
                batt_remaining_pct=int(msg.battery_remaining),
                cpu_load_pct=round(float(msg.load) / 10.0, 2),
            )

        elif m_type == "BATTERY_STATUS":
            temp_c = 0.0
            if hasattr(msg, "temperature") and int(msg.temperature) != 32767:
                temp_c = round(float(msg.temperature) / 100.0, 2)
            self.state.update(battery_temp_c=temp_c)

        elif m_type == "VIBRATION":
            self.state.update(
                vib_x=round(float(msg.vibration_x), 4),
                vib_y=round(float(msg.vibration_y), 4),
                vib_z=round(float(msg.vibration_z), 4),
            )

        elif m_type in ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"):
            self.state.update(
                imu_ax=round(float(msg.xacc), 4),
                imu_ay=round(float(msg.yacc), 4),
                imu_az=round(float(msg.zacc), 4),
                imu_gx=round(float(msg.xgyro), 4),
                imu_gy=round(float(msg.ygyro), 4),
                imu_gz=round(float(msg.zgyro), 4),
            )

        elif m_type == "RC_CHANNELS":
            self.state.update(
                rc1=int(msg.chan1_raw),
                rc2=int(msg.chan2_raw),
                rc3=int(msg.chan3_raw),
                rc4=int(msg.chan4_raw),
                rc5=int(msg.chan5_raw),
                rc6=int(msg.chan6_raw),
                rc7=int(msg.chan7_raw),
                rc8=int(msg.chan8_raw),
            )

        elif m_type in ("SERVO_OUTPUT_RAW", "ACTUATOR_OUTPUT_STATUS"):
            if m_type == "SERVO_OUTPUT_RAW":
                self.state.update(
                    servo1=int(msg.servo1_raw),
                    servo2=int(msg.servo2_raw),
                    servo3=int(msg.servo3_raw),
                    servo4=int(msg.servo4_raw),
                    servo5=int(msg.servo5_raw),
                    servo6=int(msg.servo6_raw),
                    servo7=int(msg.servo7_raw),
                    servo8=int(msg.servo8_raw),
                )

        elif m_type == "EKF_STATUS_REPORT":
            self.state.update(ekf_flags=int(msg.flags))

    def write_snapshot(self):
        snap = self.state.snapshot()
        snap["session_time"] = round(now_ts() - self.start_ts, 3)

        if self.telemetry_writer is None:
            headers = list(snap.keys())
            self.telemetry_writer = csv.DictWriter(self.telemetry_fp, fieldnames=headers)
            self.telemetry_writer.writeheader()

        self.telemetry_writer.writerow(snap)

    def run(self):
        self.write_meta()
        self.connect_mavlink()

        snapshot_interval = 1.0 / max(1.0, float(self.args.snapshot_hz))
        last_snapshot = 0.0
        last_flush = 0.0

        print("[*] Logging in progress. Press Ctrl+C to stop.")
        while not self.stop_event.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                try:
                    self.message_to_jsonl(msg)
                    self.update_state_from_mavlink(msg)
                except Exception as exc:
                    print(f"[!] MAV parse/write warning: {exc}")

            t = now_ts()
            if (t - last_snapshot) >= snapshot_interval:
                self.write_snapshot()
                last_snapshot = t

            if (t - last_flush) >= 1.0:
                self.telemetry_fp.flush()
                self.mav_jsonl_fp.flush()
                last_flush = t

    def stop(self):
        self.stop_event.set()

    def close(self):
        self.write_meta()
        self.telemetry_fp.flush()
        self.telemetry_fp.close()
        self.mav_jsonl_fp.flush()
        self.mav_jsonl_fp.close()


def parse_args():
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    default_out = os.path.join(project_root, "data", "manual_logs")

    parser = argparse.ArgumentParser(description="Manual RC flight all-parameter recorder")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="MAVLink serial port")
    parser.add_argument("--baud", type=int, default=921600, help="MAVLink serial baud")
    parser.add_argument("--stream-hz", type=int, default=20, help="Requested MAVLink stream rate")
    parser.add_argument("--snapshot-hz", type=float, default=10.0, help="CSV state snapshot rate")
    parser.add_argument("--out-dir", default=default_out, help="Session output directory")
    return parser.parse_args()


def main():
    args = parse_args()
    recorder = ManualFlightBlackbox(args)

    def _handle_signal(signum, _frame):
        print(f"\n[*] Signal {signum} received. Stopping logger...")
        recorder.stop()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        recorder.run()
    except KeyboardInterrupt:
        recorder.stop()
    except RuntimeError as exc:
        recorder.stop()
        print(f"[!] {exc}")
    finally:
        recorder.close()
        print("[*] Logs saved in:")
        print(f"    {recorder.session_dir}")
        for name, path in recorder.paths.items():
            print(f"    - {name}: {path}")


if __name__ == "__main__":
    main()
