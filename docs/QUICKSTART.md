# 🚁 ASCEND Telemetry Visualizer - Quick Start Guide

Your telemetry visualization dashboard is ready! Here's how to get started immediately.

## ⚡ Quick Start (30 seconds)

### Option 1: Using the Launcher Script
```bash
cd /home/iic/Desktop/GitHub/drone-project
./run_analyzer.sh
```

### Option 2: Direct Command
```bash
cd /home/iic/Desktop/GitHub/drone-project
streamlit run telemetry_analyzer.py
```

Then open your browser to: **http://localhost:8501**

## 📊 What You Get

The dashboard displays 7 sections of comprehensive telemetry analysis:

### 1. **Flight Summary** 📈
- Total flight duration
- Maximum altitude
- Minimum battery voltage
- Number of data points

### 2. **Position & Flight Path** 📍
- **Altitude Profile**: Height over time with fill visualization
- **Vertical Velocity**: Climb rate and descent tracking  
- **2D Trajectory**: Optical flow path + LiDAR position in X-Y plane

### 3. **Vehicle Dynamics** 🎛️
- **Attitude Angles**: Roll, Pitch, Yaw over time
- **Vibration Analysis**: Acceleration signature on X, Y, Z axes

### 4. **System Health** 🔋
- **Battery Status**: Voltage and remaining capacity curves
- **Power Draw**: Current consumption over time
- **System Load**: CPU/processor utilization

### 5. **Control Inputs** 🎮
- **RC Channels**: PWM values for Roll, Pitch, Throttle, Yaw

### 6. **Raw Data Explorer** 🔍
- Search specific columns
- View raw CSV data
- Export subsets

### 7. **Statistics Summary** 📈
- Mean, standard deviation, min/max for any parameter
- Identify data variability and anomalies

## 🧪 Try It Now

A **sample flight log** is included: `sample_flight_log.csv`

The dashboard will automatically load the most recent log file or you can:
- **Upload** a CSV file using the sidebar
- **Select** from available log files
- **Generate** new logs with: `python src/ascend_blackbox.py`

## 📂 Your Log Files

These scripts **generate telemetry CSV files**:
```
src/ascend_blackbox.py      → full_flight_log_<timestamp>.csv ⭐ Recommended
src/log_flight_data.py       → ascend_flight_log.csv
src/ascend_tui.py            → telemetry_<timestamp>.csv
src/pc_data_visualizer.py    → stage1_telemetry_log.csv
```

Generate a log:
```bash
cd /home/iic/Desktop/GitHub/drone-project/src
python ascend_blackbox.py  # Requires Pixhawk connected to /dev/ttyAMA0
```

## 🔧 Dashboard Features

### Interactive Charts
- 🖱️ **Hover** to see exact values
- 📏 **Zoom** to focus on time ranges
- 🏠 **Reset** to original view
- 📥 **Download** charts as PNG

### Customization
- **Search columns** in the Data Explorer
- **Select statistics** for custom analysis
- **Adjust visualizations** (toggle, filter by time)

### Export Data
Add this to `telemetry_analyzer.py` (after line 500) for CSV export:
```python
if st.checkbox("📥 Export Data"):
    selected_cols = st.multiselect("Choose columns:", df.columns)
    if selected_cols:
        csv = df[selected_cols].to_csv(index=False)
        st.download_button("Download CSV", csv, f"telemetry_{pd.Timestamp.now().strftime('%Y%m%d_%H%M%S')}.csv", "text/csv")
```

## 📋 Log File Format

The analyzer supports CSV files with these column names (case-sensitive):

**Position Data:**
- `Timestamp` - Unix time
- `Baro_Alt_m`, `Lidar_Dist_m` - Altitude
- `Climb_Rate_ms` - Vertical velocity
- `Opt_Flow_X`, `Opt_Flow_Y`, `Opt_Flow_Qual` - Optical flow

**Attitude:**
- `Roll_deg`, `Pitch_deg`, `Yaw_deg` - Angles

**System:**
- `Batt_Voltage`, `Batt_Current_A`, `Batt_Remain_Pct` - Power
- `CPU_Load_Pct` - System health
- `Flight_Mode`, `Armed` - Status

**Ctrl:**
- `RC1_Roll`, `RC2_Pitch`, `RC3_Throttle`, `RC4_Yaw` - Control inputs

**Dynamics:**
- `Vibration_X`, `Vibration_Y`, `Vibration_Z` - Acceleration

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| "Port 8501 already in use" | Kill previous Streamlit: `pkill streamlit` |
| "No log files found" | Generate one: `python src/ascend_blackbox.py` |
| "ModuleNotFoundError" | Run: `pip install -r requirements.txt` |
| Charts not showing | Check CSV column names match expected format |
| Slow performance | Reduce data size or filter large CSV before uploading |

## 🚀 Advanced Usage

### Remote Access
```bash
streamlit run telemetry_analyzer.py --server.address 0.0.0.0 --server.port 8501
```
Then access at: `http://<your-ip>:8501`

### Development Mode
```bash
streamlit run telemetry_analyzer.py --logger.level=debug
```

### Configuration File
Create `.streamlit/config.toml`:
```toml
[server]
port = 8501
headless = true

[browser]
gatherUsageStats = false
```

## 📚 Resources

- 📖 [Streamlit Docs](https://docs.streamlit.io/)
- 📊 [Plotly API](https://plotly.com/python/)
- 🐼 [Pandas Tutorial](https://pandas.pydata.org/docs/)
- 📡 [MAVLink Protocol](https://mavlink.io/)

## 🎯 Next Steps

1. **Try it now**: `./run_analyzer.sh` or `streamlit run telemetry_analyzer.py`
2. **Upload a flight log**: Use sidebar file selector
3. **Explore sections**: Navigate through all 7 analysis tabs
4. **Customize**: Edit `telemetry_analyzer.py` to add custom metrics
5. **Share**: Export charts by clicking the camera icon

---

**Enjoy analyzing your drone telemetry! 🚁📊**

Need help? Check `TELEMETRY_ANALYZER.md` for detailed documentation.
