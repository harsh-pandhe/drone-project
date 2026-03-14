# 🚁 Drone Project

The Drone Project is a comprehensive platform for autonomous quadcopter control, real-time telemetry visualization, and LiDAR-based obstacle detection. It leverages the MAVLink protocol for communication with Pixhawk flight controllers and integrates edge ML capabilities.

---

## 🚀 Quick Start (In 30 Seconds)

1.  **Launch the Telemetry Analyzer:**
    ```bash
    ./scripts/run_analyzer.sh
    ```
2.  **Access the Dashboard:** Open `http://localhost:8501` in your browser.
3.  **Explore Data:** Use the sidebar to load the included `data/sample_flight_log.csv`.

---

## ✨ Features

- 🛰️ **MAVLink Integration**: Direct interface with ArduCopter/Pixhawk.
- 📊 **Telemetry Dashboard**: High-performance visualization with Streamlit.
- 📡 **LiDAR Support**: Real-time pointcloud recording and obstacle avoidance.
- 🤖 **Edge ML**: Onboard inference for autonomous mission logic.
- 🎮 **Hybrid Control**: TUI (Terminal UI) and Web Dashboard.

---

## 📂 Project Structure

```text
drone-project/
├── src/            # Core application source code
│   ├── ascend_tui.py        # Main Terminal Mission Control (v1)
│   ├── ascend_tui_v2.py     # Modern Terminal Mission Control (v2)
│   └── ...                  # MAVLink and Sensor drivers
├── docs/           # Comprehensive documentation & guides
│   ├── SETUP.md             # Hardware/Software install guide
│   ├── QUICKSTART.md        # Detailed dashboard walkthrough
│   └── ...                  # Flight analysis & reports
├── tools/          # Analysis and utility tools
│   └── telemetry_analyzer.py # Streamlit Dashboard
├── scripts/        # Automation and launcher scripts
│   └── run_analyzer.sh       # Telemetry app launcher
├── data/           # Telemetry logs and datasets
└── tests/          # Unit and integration tests
```

---

## 🛠️ Hardware Requirements

- **Flight Controller**: Pixhawk (Cube, 6C, or similar) running ArduCopter.
- **Companion Computer**: Raspberry Pi 4B (4GB+ recommended).
- **Sensors**: Livox Mid-360 LiDAR (Optional), Optical Flow.

---

## 💻 Installation

```bash
# Clone the repository
git clone https://github.com/harsh-pandhe/drone-project.git
cd drone-project

# Setup environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

---

## 🎮 Basic Usage

### Verify Pixhawk Connection
```bash
python3 src/check_connection.py
```

### Start Mission Control (TUI)
```bash
python3 src/ascend_tui_v2.py
```

### Analyze Flight Logs
```bash
./scripts/run_analyzer.sh
```

---

## 📚 Documentation Hub

Explore the `docs/` folder for detailed guides:

- 🏗️ **[SETUP.md](docs/SETUP.md)**: Hardware wiring and software initialization.
- 📈 **[QUICKSTART.md](docs/QUICKSTART.md)**: Using the Telemetry Visualizer.
- 🚁 **[READY_TO_FLY.md](docs/READY_TO_FLY.md)**: Pre-flight checklist and 1m hover test.
- 🛠️ **[FIXES_APPLIED.md](docs/FIXES_APPLIED.md)**: Technical history of implemented improvements.

---

## 🤝 Contributing

Contributions are welcome! Please see **[CONTRIBUTING.md](CONTRIBUTING.md)** for development standards and pull request processes.

## 📄 License

This project is licensed under the MIT License - see the **[LICENSE](LICENSE)** file for details.

---
**Author**: [Harsh Pandhe](https://github.com/harsh-pandhe)  
**Email**: harshpandhehome@gmail.com