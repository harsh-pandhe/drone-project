# Architecture

## System Overview

The Drone Project is a modular autonomous flight system built around:

1. **Pixhawk Flight Controller** – Runs ArduCopter firmware
2. **Raspberry Pi Companion Computer** – Runs main application logic
3. **MAVLink Protocol** – Serial communication between Pixhawk and Pi
4. **Flask Web Server** – Provides dashboard and API
5. **Optional Sensors** – LiDAR (Livox), Edge TPU (for ML)

## Communication Flow

```
┌──────────────────────────────────────────────────────────┐
│                   Web Browser / Client                   │
│                  (http://localhost:5000)                 │
└──────────────────┬───────────────────────────────────────┘
                   │ HTTP / WebSocket
┌──────────────────▼───────────────────────────────────────┐
│         Flask Application (ascend_server.py)             │
│  - Event emission via SocketIO                           │
│  - Command routing to flight controller                  │
│  - Telemetry aggregation                                 │
└──────────────────┬───────────────────────────────────────┘
                   │ Threading / Async
┌──────────────────▼───────────────────────────────────────┐
│    MAVLink Interface (pymavlink / mavsdk)                │
│  - Serialization / deserialization                       │
│  - Message queuing                                       │
└──────────────────┬───────────────────────────────────────┘
                   │ Serial (57600-921600 baud)
┌──────────────────▼───────────────────────────────────────┐
│      Pixhawk / Flight Controller (ArduCopter)            │
│  - Motor control                                         │
│  - Sensor fusion (GPS, IMU, barometer)                   │
│  - Stabilization & flight modes                          │
└──────────────────────────────────────────────────────────┘
```

## Module Responsibilities

### **src/ascend_server.py**
- Provides Flask web server on port 5000
- Manages SocketIO connections for real-time telemetry
- Bridges web commands to MAVLink protocol
- Emits telemetry events (VFR_HUD, ATTITUDE, SYS_STATUS, etc.)

### **src/ascend_tui.py**
- Interactive terminal interface using curses
- Local control without web access
- Arm/disarm/mode operations via TUI

### **src/ascend_autoflight.py**
- Autonomous flight logic
- Sensor monitoring (optical flow, distance)
- Autonomous takeoff/landing

### **src/log_flight_data.py**
- Captures telemetry to CSV files
- Records events and errors
- Data archival for post-flight analysis

### **src/check_connection.py**
- Validates Pixhawk connectivity
- Tests serial communication
- Useful for debugging setup issues

### **src/record_livox.py** (Optional)
- Records LiDAR scans from Livox sensor
- Enables obstacle detection

### **src/pi_data_streamer.py** (Optional)
- Streams Pi sensor data (temperature, voltage)
- Monitoring of companion computer health

## Data Flow

### Telemetry Capture Loop

```
Pixhawk (sends MAVLink messages)
        ↓
Serial RX (pymavlink.recv_match)
        ↓
Message parsing (VFR_HUD, ATTITUDE, SYS_STATUS, etc.)
        ↓
SocketIO event emission
        ↓
Web client (real-time dashboard)
```

### Command Execution Path

```
Web/TUI command (ARM, DISARM, TAKEOFF)
        ↓
Flask route handler / TUI input
        ↓
MAVLink command generation
        ↓
Serial TX (pymavlink.mav.command_long_send)
        ↓
Pixhawk executes command
        ↓
Confirmation in telemetry loop
```

## Threading Model

- **Main Thread**: Flask web server (blocking)
- **Daemon Thread**: Telemetry loop (continuous polling)
  - Listens for MAVLink messages
  - Emits SocketIO events
  - Non-blocking with 0.05s sleep between iterations

```python
threading.Thread(target=telemetry_loop, daemon=True).start()
socketio.run(app, host='0.0.0.0', port=5000)
```

## File Organization

```
src/
├── ascend_*.py          # Main application (server, TUI, autoflight, etc.)
├── check_*.py           # Diagnostic/validation scripts
├── drone_server.py      # Alternative server implementation
├── log_flight_data.py   # Data logging
├── pi_*.py              # Pi companion utilities
├── record_*.py          # Sensor recording
└── ...

data/
├── telemetry/           # CSV telemetry dumps
├── events/              # Event logs
├── livox/               # LiDAR scans
└── raw/                 # Other raw data

tests/
├── test_*.py            # Unit tests
├── final_*.py           # Integration tests
└── ...

config/
├── mav.parm             # Pixhawk parameters
├── mav.tlog             # MAVLink binary log
└── mav.tlog.raw         # Raw MAVLink log
```

## MAVLink Message Types Used

| Message | Source | Purpose |
|---------|--------|---------|
| HEARTBEAT | Pixhawk | Connection keep-alive |
| VFR_HUD | Pixhawk | Altitude, speed, heading, throttle |
| ATTITUDE | Pixhawk | Roll, pitch, yaw angles |
| SYS_STATUS | Pixhawk | Battery, voltage, current |
| GPS_RAW_INT | Pixhawk | GPS position and altitude |
| STATUSTEXT | Pixhawk | System messages, errors |
| COMMAND_LONG | Pi → Pixhawk | Arm, disarm, takeoff, mode changes |

## Extension Points

### Adding New Telemetry
1. Add message type to `recv_match()` in `ascend_server.py`
2. Parse message and extract fields
3. Emit via `socketio.emit()` to web client

### Adding New Commands
1. Add route in Flask app (`@app.route()` or `@socketio.on()`)
2. Convert command to MAVLink `command_long_send()`
3. Verify execution in telemetry loop

### Adding Sensors
1. Create new script in `src/` (e.g., `src/record_sensor.py`)
2. Log data to `data/` directory
3. Optionally emit to SocketIO for real-time display

## Performance Considerations

- **Serial Baud Rate**: Higher rates (921600) improve responsiveness but require stable connection
- **Telemetry Loop Rate**: 0.05s sleep = 20Hz polling frequency
- **SocketIO Burst**: Multiple events per second may overload low-bandwidth connections
- **Data File I/O**: CSV writes every telemetry update; consider buffering for high-frequency logging

## Security Notes

- ⚠️ Flask server runs on `0.0.0.0:5000` – exposed to network
- No authentication by default (suitable for LAN only)
- Consider firewall rules for production deployments
- Avoid exposing to untrusted networks

## Future Improvements

- [ ] REST API for telemetry queries
- [ ] Database backend (SQLite/PostgreSQL) for telemetry storage
- [ ] ROS2 integration for advanced autonomy
- [ ] Kalman filter for state estimation
- [ ] Failsafe and geofencing logic
- [ ] Multi-drone coordination
