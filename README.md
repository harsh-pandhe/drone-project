# Drone Project

Autonomous drone control platform for quadcopters using MAVLink protocol and Pixhawk flight controllers. Includes real-time telemetry monitoring, LiDAR integration, and edge ML inference capabilities.

## Features

- 🚁 **MAVLink Protocol Integration** – Direct communication with Pixhawk flight controller
- 📊 **Real-Time Telemetry** – Web dashboard with SocketIO streaming
- 📹 **LiDAR Integration** – Livox LiDAR support for obstacle detection
- 🤖 **Edge ML** – TensorFlow Lite model inference on Raspberry Pi
- 📝 **Flight Logging** – Comprehensive telemetry and event logging
- 🎮 **Terminal UI** – Interactive control interface for arm/disarm/mode changes
- 🧪 **Test Suite** – Hardware health checks and communication validation

## Quick Start

### Prerequisites
- Pixhawk flight controller (ArduCopter)
- Raspberry Pi 4B+ (8GB recommended)
- Python 3.9+
- USB serial adapter or GPIO serial connection

### Installation

```bash
git clone https://github.com/harsh-pandhe/drone-project.git
cd drone-project

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### Verify Connection

```bash
python3 src/check_connection.py
```

### Run Server

```bash
python3 src/ascend_server.py
# Visit http://localhost:5000
```

## Directory Structure

```
drone-project/
├── src/                    # Main application code
│   ├── ascend_server.py    # Flask/SocketIO web server
│   ├── ascend_tui.py       # Terminal user interface
│   ├── ascend_autoflight.py # Autonomous flight logic
│   ├── log_flight_data.py  # Data logging utilities
│   └── ...                 # Other utilities
├── tests/                  # Test suites
├── config/                 # Pixhawk parameters & logs
├── data/                   # Flight data (gitignored)
│   ├── telemetry/          # CSV telemetry dumps
│   ├── events/             # Text event logs
│   ├── livox/              # LiDAR scan data
│   └── raw/                # Other raw logs
├── models/                 # ML models (e.g. TFLite)
├── README.md               # This file
├── SETUP.md                # Detailed setup guide
├── CONTRIBUTING.md         # Contribution guidelines
├── requirements.txt        # Python dependencies
└── LICENSE                 # MIT License
```

## Key Scripts

| Script | Purpose |
|--------|---------|
| `src/ascend_server.py` | Web dashboard server on port 5000 |
| `src/ascend_tui.py` | Interactive terminal interface |
| `src/log_flight_data.py` | Record telemetry to CSV |
| `src/check_connection.py` | Verify Pixhawk connectivity |
| `src/pi_data_streamer.py` | Stream sensor data from Pi |
| `src/record_livox.py` | Capture LiDAR scans |

## Documentation

- **[SETUP.md](SETUP.md)** – Hardware wiring, software install, troubleshooting
- **[CONTRIBUTING.md](CONTRIBUTING.md)** – Development guidelines, how to contribute
- **[config/mav.parm](config/mav.parm)** – Pixhawk parameter export

## MAVLink Connection

Default serial configuration:
- **Port**: `/dev/ttyAMA0` (or `/dev/serial0`)
- **Baud Rate**: 57600, 115200, or 921600 (check firmware)
- **Protocol**: MAVLink v2

## Testing

```bash
# Run all tests
pytest tests/

# Run with coverage
pytest --cov=src tests/
```

## Architecture

```
┌─────────────────────────────────────────────┐
│         Web Browser (http://localhost:5000)  │
├─────────────────────────────────────────────┤
│ Flask/SocketIO Server (ascend_server.py)    │
├─────────────────────────────────────────────┤
│ MAVLink Protocol (pymavlink/mavsdk)         │
├─────────────────────────────────────────────┤
│ Pixhawk Flight Controller (via serial)      │
├─────────────────────────────────────────────┤
│ Sensors: GPS, IMU, Barometer, Compass       │
└─────────────────────────────────────────────┘

Optional components:
- Livox LiDAR (obstacle detection)
- Edge TPU (ML inference)
- ROS2 Humble (for advanced autonomy)
```

## License

MIT License – See [LICENSE](LICENSE) file for details.

## Support

- 📖 Check [SETUP.md](SETUP.md) for hardware/software guides
- 🐛 Report issues on [GitHub Issues](https://github.com/harsh-pandhe/drone-project/issues)
- 💬 See [CONTRIBUTING.md](CONTRIBUTING.md) for development questions

---

**Author**: harsh-pandhe  
**Email**: harshpandhehome@gmail.com  
**Repository**: https://github.com/harsh-pandhe/drone-project