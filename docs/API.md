# API Reference

## Web Server

The main web server runs at `http://localhost:5000` and provides:
- Static HTML dashboard
- WebSocket SocketIO for real-time telemetry
- Command endpoints for flight control

### Starting the Server

```bash
python3 src/ascend_server.py
```

Connects to Pixhawk on `/dev/ttyAMA0` at 57600 baud (configurable in code).

## SocketIO Events

### Client Receives (emit)

#### `vfr_data`
Velocity, flight readiness, heading data.

```json
{
  "alt": 12.34,      // altitude in meters
  "climb": 1.2,      // climb rate (m/s)
  "hdg": 180,        // heading in degrees
  "throttle": 50     // throttle percentage
}
```

#### `attitude`
Aircraft orientation.

```json
{
  "roll": 5.2,       // roll angle in degrees
  "pitch": -2.1      // pitch angle in degrees
}
```

#### `sys_status`
System and battery status.

```json
{
  "voltage": 12.5,   // battery voltage (V)
  "current": 3.2,    // current draw (A)
  "level": 85        // battery remaining (%)
}
```

#### `state`
Flight controller state.

```json
{
  "armed": true,     // arm status
  "mode": "GUIDED"   // flight mode
}
```

#### `console`
System messages and errors.

```json
{
  "text": "Pre-arm check: GPS not ready",
  "severity": 1
}
```

### Client Sends (on)

#### `command`
Execute a flight control command.

```json
{
  "type": "ARM"      // Command type
}
```

**Supported Commands:**

- `"ARM"` – Arm the drone
- `"DISARM"` – Disarm the drone
- `"TAKEOFF"` – Take off to specified altitude
  ```json
  {"type": "TAKEOFF", "alt": 5}  // alt in meters
  ```
- `"SET_MODE"` – Change flight mode
  ```json
  {"type": "SET_MODE", "mode": "LOITER"}
  ```

**Example (JavaScript):**

```javascript
const socket = io('http://localhost:5000');

// Listen for telemetry
socket.on('vfr_data', (data) => {
  console.log(`Altitude: ${data.alt}m, Heading: ${data.hdg}°`);
});

// Send command
socket.emit('command', {type: 'ARM'});

// Takeoff to 10m
socket.emit('command', {type: 'TAKEOFF', alt: 10});
```

## Data Logging

### Telemetry CSV Format

Saved to `data/telemetry/telemetry_YYYYMMDD_HHMMSS.csv`

```csv
timestamp,altitude,climb_rate,heading,throttle,roll,pitch,voltage,current,battery_pct
2026-03-09 12:30:45.123,5.2,1.1,180,50,2.3,-1.2,12.5,3.2,85
```

### Event Log Format

Saved to `data/events/events_YYYYMMDD_HHMMSS.txt`

```
2026-03-09 12:30:45.123 | INFO | Systems ready
2026-03-09 12:30:46.000 | ARM  | Drone armed
2026-03-09 12:30:48.000 | WARN | GPS signal weak
```

## Command-Line Utilities

### Check Connection

```bash
python3 src/check_connection.py
```

Output:
```
Checking connection to: serial:///dev/serial0:921600...
Waiting for heartbeat (Timeout: 5s)...
SUCCESS: Drone is connected!
```

### Check Heartbeat

```bash
python3 src/check_heartbeat.py
```

Continuous heartbeat monitoring.

### Log Flight Data

```bash
python3 src/log_flight_data.py
```

Records telemetry to CSV in real-time.

### System Check

```bash
bash src/sys_check.sh
```

Hardware and system diagnostics.

## Terminal UI

Interactive terminal interface for local control:

```bash
python3 src/ascend_tui.py
```

**Keyboard Controls:**
- `a` – Arm
- `d` – Disarm
- `t` – Takeoff
- `l` – Land
- `q` – Quit

## LiDAR Integration (Optional)

### Record Livox Scans

```bash
python3 src/record_livox.py
```

Saves scans to `data/livox/livox_*.csv`

### Plot Log Data

```bash
python3 src/plot_ascend_log.py
```

Generates visualization of flight telemetry.

## Configuration

### Pixhawk Parameters (_mav.parm_)

Located in `config/mav.parm`:

```
SERIAL1_BAUD 57600        # Serial port baud rate
SYSID_MYGCS 255           # GCS system ID
GPS_TYPE 1                # GPS module type
```

### Serial Connection

Default configuration in `ascend_server.py`:

```python
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
```

**To change:**

1. Edit the file: `nano src/ascend_server.py`
2. Modify connection string:
   ```python
   master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
   ```
3. Restart server

## Error Handling

### Connection Errors

| Error | Likely Cause | Solution |
|-------|--------------|----------|
| "Connection timed out" | No heartbeat | Check serial cable, Pixhawk power |
| "Permission denied" | Serial port access | Run with `sudo` or add user to `dialout` group |
| "Baud rate mismatch" | Wrong baud rate | Check Pixhawk firmware settings |

### Flight Errors

Check `data/events/` logs for:
- Pre-arm check failures
- Battery low warnings
- GPS signal loss
- Sensor errors

View recent events:
```bash
tail data/events/*.txt
```

## Performance Metrics

### Expected Telemetry Rate

With 0.05s loop interval:
- ~20 messages/second from Pixhawk
- ~20 SocketIO events/second to clients
- ~5KB/min CSV logging (varies by message frequency)

### Bandwidth

- Serial: ~10-50 KB/min (depending on baud)
- WebSocket: ~50-200 KB/min (many clients)
- Local file I/O: ~5-10 KB/min (telemetry CSV)

## Troubleshooting

### No telemetry appearing

```bash
# Check serial connection
ls -la /dev/tty*

# Test with MAVProxy
mavproxy.py --master=/dev/ttyAMA0:57600
```

### Web server not responding

```bash
# Check if port is in use
lsof -i :5000

# Test locally
curl http://localhost:5000
```

### Python import errors

```bash
# Verify dependencies
pip list | grep -E "pymavlink|flask"

# Reinstall
pip install -r requirements.txt
```

## Advanced Usage

### Custom MAVLink Messages

To handle additional message types:

```python
# In ascend_server.py, add to recv_match():
msg = master.recv_match(type=['CUSTOM_MESSAGE'], ...)
```

### Extending Commands

Add new command types:

```python
@socketio.on('command')
def handle_command(data):
    cmd = data['type']
    if cmd == 'MY_CUSTOM_CMD':
        # Your logic here
```

### Multi-Drone Support

The architecture supports:
1. Multiple serial connections to different Pixhawks
2. Separate SocketIO namespaces per drone
3. Independent telemetry logging per drone

See `ARCHITECTURE.md` for details.

---

**Need help?** Check [SETUP.md](../SETUP.md) or [CONTRIBUTING.md](../CONTRIBUTING.md).
