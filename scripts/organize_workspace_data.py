#!/usr/bin/env python3
"""Organize generated flight artifacts into the data folder structure."""

from __future__ import annotations

import os
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
DATA = ROOT / "data"

PATTERNS = [
    (re.compile(r"^telemetry_\d{8}_\d{6}\.csv$"), DATA / "telemetry"),
    (re.compile(r"^events_\d{8}_\d{6}\.txt$"), DATA / "events"),
    (re.compile(r"^livox_pointcloud_\d{8}_\d{6}\.bin$"), DATA / "livox"),
    (re.compile(r"^livox_imu_\d{8}_\d{6}\.csv$"), DATA / "livox"),
    (re.compile(r"^diag_\d{8}_\d{6}\.txt$"), DATA / "raw"),
    (re.compile(r"^ops_todo_\d{8}_\d{6}\.txt$"), DATA / "raw"),
]


def destination_for(name: str) -> Path | None:
    for pattern, dst in PATTERNS:
        if pattern.match(name):
            return dst
    return None


def unique_target(path: Path) -> Path:
    if not path.exists():
        return path
    stem = path.stem
    suffix = path.suffix
    i = 1
    while True:
        candidate = path.with_name(f"{stem}_dup{i}{suffix}")
        if not candidate.exists():
            return candidate
        i += 1


def move_generated(source_dir: Path, moved: list[tuple[Path, Path]]) -> None:
    if not source_dir.exists():
        return

    for item in sorted(source_dir.iterdir(), key=lambda p: p.name):
        if not item.is_file():
            continue
        target_dir = destination_for(item.name)
        if target_dir is None:
            continue
        target_dir.mkdir(parents=True, exist_ok=True)
        target = unique_target(target_dir / item.name)
        shutil.move(str(item), str(target))
        moved.append((item, target))


def main() -> None:
    moved: list[tuple[Path, Path]] = []

    move_generated(ROOT, moved)
    move_generated(SRC, moved)

    print(f"[DONE] Organized {len(moved)} files")
    for src, dst in moved:
        print(f" - {src.relative_to(ROOT)} -> {dst.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
