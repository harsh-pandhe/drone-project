# Setup Guide

## Hardware Setup

### Prerequisites
- Pixhawk flight controller (ArduCopter compatible)
- Raspberry Pi (4B recommended, 8GB RAM)
- USB serial cable or FTDI adapter (for Pixhawk ↔ Pi communication)
- Power distribution board and battery
- Optional: Livox LiDAR, Edge TPU accelerator

### Connections
1. **Pixhawk ↔ Raspberry Pi**:
   - Connect Pixhawk `TELEM1` or `SERIAL3` to Pi GPIO serial port
   - Common baud rates: 57600, 115200, or 921600
   - Check [`config/mav.parm`](config/mav.parm) for your setup

2. **Power**:
   - Supply Pi via USB-C or GPIO pins from a stable 5V source
   - Connect battery to Pixhawk power module

## Software Setup

### 1. On Raspberry Pi

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python and dependencies
sudo apt install -y python3-pip python3-venv git

# Clone this repository
git clone https://github.com/harsh-pandhe/drone-project.git
cd drone-project

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 2. MAVProxy (Optional but Recommended)

```bash
pip install MAVProxy
```

Use MAVProxy to test Pixhawk connection before running the full system:
```bash
mavproxy.py --master=/dev/ttyAMA0:57600 --baudrate=57600
```

### 3. Verify Connection

Run the connection check:
```bash
python3 src/check_connection.py
```

Expected output:
```
Checking connection to: serial:///dev/serial0:921600...
Waiting for heartbeat (Timeout: 5s)...
SUCCESS: Drone is connected!
```

## Running the Application

### Start the Server
```bash
python3 src/ascend_server.py
```
- Serves at `http://localhost:5000`
- Provides real-time telemetry via SocketIO

### Launch Terminal UI
```bash
python3 src/ascend_tui.py
```
- Interactive terminal interface for arm/disarm/mode changes

### Record Flight Data
```bash
python3 src/log_flight_data.py
```
- Saves telemetry to `data/telemetry/` as CSV
- Logs events to `data/events/`

### Process LiDAR Data (if equipped)
```bash
python3 src/record_livox.py
python3 src/plot_ascend_log.py
```

## Troubleshooting

### "Connection timed out"
- Check USB cable and serial port: `ls /dev/tty*`
- Verify baud rate matches Pixhawk config
- Use `mavproxy.py` to diagnose

### "Heartbeat not received"
- Ensure Pixhawk is powered and booted
- Check MAVLink is enabled in Pixhawk firmware
- Look for error messages: `tail data/events/*.txt`

### Permission denied on serial port
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### ROS2 Humble Setup (Optional)

If using ROS2 integration:
```bash
sudo apt install -y ros-humble-ros-core
source /opt/ros/humble/setup.bash
pip install rclpy
```

## Configuration Files

- **`config/mav.parm`** – Pixhawk parameter export
- **`config/mav.tlog`** – MAVLink binary telemetry log
- **`.gitignore`** – Excludes data/ and venv/ from version control

## Next Steps

1. Run tests: `pytest tests/`
2. Review example scripts in `src/`
3. Check telemetry: view CSV files in `data/telemetry/`
4. Explore the TUI: `python3 src/ascend_tui.py`

---

For issues or questions, visit the [GitHub Issues](https://github.com/harsh-pandhe/drone-project/issues) page.
