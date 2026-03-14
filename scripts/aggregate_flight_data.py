#!/usr/bin/env python3
"""Aggregate archived flight data into single telemetry/event/lidar artifacts."""

import argparse
import csv
import os
import re
from datetime import datetime
from glob import glob

LFS_POINTER_HEADER = "version https://git-lfs.github.com/spec/v1"
LIVOX_MAGIC = b"LIVOXPC1"


def extract_session_id(path):
    name = os.path.basename(path)
    match = re.search(r"(\d{8}_\d{6})", name)
    return match.group(1) if match else ""


def is_lfs_pointer(path):
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            return f.readline().strip() == LFS_POINTER_HEADER
    except OSError:
        return False


def discover_files(root):
    telemetry = set(glob(os.path.join(root, "data/telemetry/telemetry_*.csv")))
    telemetry.update(glob(os.path.join(root, "telemetry_*.csv")))

    events = set(glob(os.path.join(root, "data/events/events_*.txt")))
    events.update(glob(os.path.join(root, "events_*.txt")))

    lidar = set(glob(os.path.join(root, "data/livox/livox_pointcloud_*.bin")))
    lidar.update(glob(os.path.join(root, "livox_pointcloud_*.bin")))

    key = lambda p: (extract_session_id(p), os.path.basename(p))
    return sorted(telemetry, key=key), sorted(events, key=key), sorted(lidar, key=key)


def aggregate_telemetry(files, out_csv):
    if not files:
        return 0, 0

    union_fields = []
    seen = set()
    valid_files = []

    for path in files:
        if is_lfs_pointer(path):
            continue
        try:
            with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
                reader = csv.reader(f)
                header = next(reader, None)
                if not header:
                    continue
                for col in header:
                    if col not in seen:
                        seen.add(col)
                        union_fields.append(col)
                valid_files.append(path)
        except OSError:
            continue

    if not valid_files:
        return 0, 0

    out_fields = ["Session_ID", "Source_File"] + union_fields
    total_rows = 0

    with open(out_csv, "w", encoding="utf-8", newline="") as fout:
        writer = csv.DictWriter(fout, fieldnames=out_fields)
        writer.writeheader()

        for path in valid_files:
            session_id = extract_session_id(path)
            with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    out_row = {field: "" for field in out_fields}
                    out_row["Session_ID"] = session_id
                    out_row["Source_File"] = os.path.relpath(path)
                    for key, val in row.items():
                        if key in out_row:
                            out_row[key] = val
                    writer.writerow(out_row)
                    total_rows += 1

    return len(valid_files), total_rows


def aggregate_events(files, out_txt):
    total_lines = 0
    valid = 0

    with open(out_txt, "w", encoding="utf-8") as fout:
        fout.write("=== AGGREGATED EVENTS LOG ===\n")
        fout.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        for path in files:
            if is_lfs_pointer(path):
                continue
            try:
                with open(path, "r", encoding="utf-8", errors="ignore") as f:
                    lines = f.readlines()
            except OSError:
                continue

            valid += 1
            fout.write(f"\n--- SOURCE: {os.path.relpath(path)} ---\n")
            for line in lines:
                if line.strip():
                    fout.write(line.rstrip("\n") + "\n")
                    total_lines += 1

    return valid, total_lines


def aggregate_lidar(files, out_bin, out_manifest):
    appended_files = 0
    total_payload_bytes = 0

    with open(out_bin, "wb") as fout, open(out_manifest, "w", encoding="utf-8", newline="") as mf:
        writer = csv.writer(mf)
        writer.writerow(["Source_File", "Session_ID", "Input_Bytes", "Payload_Bytes_Appended"])

        fout.write(LIVOX_MAGIC)

        for path in files:
            if is_lfs_pointer(path):
                continue
            try:
                with open(path, "rb") as f:
                    data = f.read()
            except OSError:
                continue

            if not data:
                continue

            if data.startswith(LIVOX_MAGIC):
                payload = data[len(LIVOX_MAGIC):]
            else:
                # Unexpected format; skip to avoid corrupting merged file.
                payload = b""

            if not payload:
                writer.writerow([os.path.relpath(path), extract_session_id(path), len(data), 0])
                continue

            fout.write(payload)
            appended_files += 1
            total_payload_bytes += len(payload)
            writer.writerow([
                os.path.relpath(path),
                extract_session_id(path),
                len(data),
                len(payload),
            ])

    return appended_files, total_payload_bytes


def main():
    parser = argparse.ArgumentParser(description="Aggregate telemetry/events/lidar into single files.")
    parser.add_argument("--root", default=".", help="Project root directory")
    parser.add_argument("--out-dir", default="", help="Output directory (default: data/aggregated/<timestamp>)")
    args = parser.parse_args()

    root = os.path.abspath(args.root)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out_dir or os.path.join(root, "data", "aggregated", ts)
    os.makedirs(out_dir, exist_ok=True)

    telemetry_files, event_files, lidar_files = discover_files(root)

    telemetry_out = os.path.join(out_dir, "telemetry_all.csv")
    events_out = os.path.join(out_dir, "events_all.txt")
    lidar_out = os.path.join(out_dir, "lidar_all.bin")
    lidar_manifest_out = os.path.join(out_dir, "lidar_manifest.csv")
    summary_out = os.path.join(out_dir, "summary.txt")

    tele_valid, tele_rows = aggregate_telemetry(telemetry_files, telemetry_out)
    evt_valid, evt_lines = aggregate_events(event_files, events_out)
    lidar_valid, lidar_payload_bytes = aggregate_lidar(lidar_files, lidar_out, lidar_manifest_out)

    with open(summary_out, "w", encoding="utf-8") as sf:
        sf.write("ASCEND Aggregation Summary\n")
        sf.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        sf.write(f"Output: {out_dir}\n\n")
        sf.write(f"Telemetry files discovered: {len(telemetry_files)}\n")
        sf.write(f"Telemetry files merged: {tele_valid}\n")
        sf.write(f"Telemetry rows merged: {tele_rows}\n\n")
        sf.write(f"Event files discovered: {len(event_files)}\n")
        sf.write(f"Event files merged: {evt_valid}\n")
        sf.write(f"Event lines merged: {evt_lines}\n\n")
        sf.write(f"Lidar files discovered: {len(lidar_files)}\n")
        sf.write(f"Lidar files merged: {lidar_valid}\n")
        sf.write(f"Lidar payload bytes merged: {lidar_payload_bytes}\n")

    print(f"[DONE] Aggregated data written to: {out_dir}")
    print(f" - telemetry: {telemetry_out}")
    print(f" - events:    {events_out}")
    print(f" - lidar:     {lidar_out}")
    print(f" - manifest:  {lidar_manifest_out}")
    print(f" - summary:   {summary_out}")


if __name__ == "__main__":
    main()
