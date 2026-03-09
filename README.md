# Drone Project

This repository contains scripts and data for controlling and logging a drone.

## Structure

- **ascend_env/** - Python virtual environment used by the project.
- **src/** - Application code and utility scripts.
  - Most of the `.py` files from the project have been moved here.
  - Includes shell tools such as `sys_check.sh`.
- **tests/** - Test scripts and sanity checks.
- **config/** - Configuration files for the flight controller (e.g. `mav.parm`, `mav.tlog`).
- **data/** - Flight data and logs organized by type:
  - `telemetry/` &ndash; CSV telemetry dumps.
  - `events/` &ndash; text event logs.
  - `livox/` &ndash; LiDAR scan outputs and related CSVs.
  - `raw/` &ndash; other raw log files.
- **models/** - Binary and model files (e.g. TensorFlow Lite network).

## Usage

Run scripts from the project root, for example:

```bash
cd /home/iic5/drone_project
python3 src/ascend_server.py
```

Data files reside under `data/`; update any script paths if you move them again.

---

This reorganization is intended to make it easier to locate code versus data and to
cleanly separate tests and configurations. Feel free to adjust further as the
project evolves.