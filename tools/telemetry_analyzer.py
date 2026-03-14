"""
ASCEND Telemetry Analyzer - Post-Flight Data Visualization
Interactive dashboard for analyzing logged flight telemetry data
"""

import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import numpy as np
from pathlib import Path
import os

# Set page config
st.set_page_config(
    page_title="ASCEND Telemetry Analyzer",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

st.title("🚁 ASCEND Flight Telemetry Analyzer")
st.markdown("_Post-flight analysis dashboard for comprehensive telemetry visualization_")

# ============================================================================
# SIDEBAR: File Selection & Configuration
# ============================================================================
with st.sidebar:
    st.header("📁 Data Source")
    
    # Auto-detect log files in project directory
    src_dir = Path(__file__).parent / "src"
    project_root = Path(__file__).parent
    
    log_files = []
    for directory in [project_root, src_dir]:
        if directory.exists():
            log_files.extend(directory.glob("*flight*.csv"))
            log_files.extend(directory.glob("*telemetry*.csv"))
            log_files.extend(directory.glob("*_log.csv"))
    
    log_files = sorted(set(log_files), key=lambda x: x.stat().st_mtime, reverse=True)
    
    # File selection options
    col1, col2 = st.columns([3, 1])
    with col1:
        use_recent = st.checkbox("Use recent file", value=True, help="Automatically load the most recent log file")
    with col2:
        if st.button("🔄 Refresh", key="refresh_files"):
            st.rerun()
    
    if use_recent and log_files:
        selected_file = log_files[0]
        st.info(f"📄 {selected_file.name}")
    elif log_files:
        selected_file = Path(st.selectbox("Select a log file:", log_files, format_func=lambda x: x.name))
    else:
        st.warning("No log files found. Upload a CSV file below.")
        selected_file = None
    
    # File upload option
    st.divider()
    uploaded_file = st.file_uploader("Or upload a CSV file:", type=['csv'], key='csv_uploader')
    
    if uploaded_file:
        selected_file = uploaded_file

# ============================================================================
# MAIN CONTENT: Load and Parse Data
# ============================================================================
if selected_file is None:
    st.error("❌ No telemetry file selected. Please select or upload a CSV file.")
    st.stop()

try:
    # Load CSV
    if isinstance(selected_file, str) or isinstance(selected_file, Path):
        df = pd.read_csv(selected_file)
        filename = Path(selected_file).name
    else:
        df = pd.read_csv(selected_file)
        filename = "uploaded_file.csv"
    
    # Normalize timestamp to start from 0
    if 'Timestamp' in df.columns:
        df['Time'] = df['Timestamp'] - df['Timestamp'].iloc[0]
        time_col = 'Time'
    elif 'timestamp' in df.columns:
        df['Time'] = df['timestamp'] - df['timestamp'].iloc[0]
        time_col = 'Time'
    else:
        df['Time'] = np.arange(len(df))
        time_col = 'Time'
    
    st.success(f"✅ Loaded: {filename} ({len(df)} records)")
    
except Exception as e:
    st.error(f"Error loading file: {e}")
    st.stop()

# ============================================================================
# SECTION 1: Summary Statistics
# ============================================================================
st.header("📊 Flight Summary")

col1, col2, col3, col4 = st.columns(4)

# Extract key metrics
metrics = {}

# Flight time
if time_col in df.columns:
    flight_time = df[time_col].max()
    metrics['Flight Time'] = f"{flight_time:.1f}s ({flight_time/60:.1f}m)"
    col1.metric("⏱️ Flight Duration", metrics['Flight Time'])

# Altitude
alt_cols = [col for col in df.columns if 'alt' in col.lower() or 'Alt' in col]
if alt_cols:
    alt_col = alt_cols[0]
    max_alt = df[alt_col].max()
    metrics['Max Altitude'] = f"{max_alt:.2f}m"
    col2.metric("🏔️ Max Altitude", metrics['Max Altitude'])

# Battery
batt_cols = [col for col in df.columns if 'batt' in col.lower() and ('volt' in col.lower() or 'V' in col)]
if batt_cols:
    batt_col = batt_cols[0]
    min_batt = df[batt_col].min()
    metrics['Min Battery'] = f"{min_batt:.2f}V"
    col3.metric("🔋 Min Battery", metrics['Min Battery'])

# Records
col4.metric("📈 Data Points", f"{len(df):,}")

st.divider()

# ============================================================================
# SECTION 2: Position & Flight Path
# ============================================================================
st.header("📍 Position & Flight Path")

col1, col2 = st.columns(2)

with col1:
    st.subheader("Altitude Profile")
    alt_cols = [col for col in df.columns if 'alt' in col.lower() or 'Alt' in col or 'Baro_Alt' in col]
    
    if alt_cols:
        alt_col = alt_cols[0]
        fig_alt = go.Figure()
        fig_alt.add_trace(go.Scatter(
            x=df[time_col], y=df[alt_col],
            mode='lines', name='Altitude',
            line=dict(color='green', width=2),
            fill='tozeroy', fillcolor='rgba(0,255,0,0.1)'
        ))
        fig_alt.update_layout(
            title="Altitude vs Time",
            xaxis_title="Time (s)",
            yaxis_title="Altitude (m)",
            hovermode='x unified',
            height=400
        )
        st.plotly_chart(fig_alt, use_container_width=True)
    else:
        st.warning("Altitude data not found")

with col2:
    st.subheader("Vertical Velocity")
    climb_cols = [col for col in df.columns if 'climb' in col.lower() or 'climbrate' in col.lower()]
    
    if climb_cols:
        climb_col = climb_cols[0]
        fig_climb = go.Figure()
        fig_climb.add_trace(go.Scatter(
            x=df[time_col], y=df[climb_col],
            mode='lines', name='Climb Rate',
            line=dict(color='blue', width=2)
        ))
        fig_climb.add_hline(y=0, line_dash="dash", line_color="gray", annotation_text="Zero")
        fig_climb.update_layout(
            title="Vertical Velocity vs Time",
            xaxis_title="Time (s)",
            yaxis_title="Climb Rate (m/s)",
            hovermode='x unified',
            height=400
        )
        st.plotly_chart(fig_climb, use_container_width=True)
    else:
        st.info("Climb rate data not available")

# Optical Flow Trajectory
st.subheader("2D Position (Optical Flow Trajectory)")

flow_x_cols = [col for col in df.columns if 'Flow_X' in col or 'flow_x' in col or 'Opt_Flow_X' in col]
flow_y_cols = [col for col in df.columns if 'Flow_Y' in col or 'flow_y' in col or 'Opt_Flow_Y' in col]
lidar_x_cols = [col for col in df.columns if 'LiDAR_X' in col or 'lidar_x' in col]
lidar_y_cols = [col for col in df.columns if 'LiDAR_Y' in col or 'lidar_y' in col]

fig_2d = go.Figure()

if flow_x_cols and flow_y_cols:
    flow_x, flow_y = df[flow_x_cols[0]], df[flow_y_cols[0]]
    fig_2d.add_trace(go.Scatter(
        x=flow_x, y=flow_y, mode='lines+markers',
        name='Optical Flow Path',
        line=dict(color='orange', width=2),
        marker=dict(size=4)
    ))

if lidar_x_cols and lidar_y_cols:
    lidar_x, lidar_y = df[lidar_x_cols[0]], df[lidar_y_cols[0]]
    # Filter out outliers for better visualization
    q1_x, q3_x = lidar_x.quantile([0.25, 0.75])
    q1_y, q3_y = lidar_y.quantile([0.25, 0.75])
    mask = (lidar_x >= q1_x - 1.5*(q3_x-q1_x)) & (lidar_x <= q3_x + 1.5*(q3_x-q1_x)) & \
           (lidar_y >= q1_y - 1.5*(q3_y-q1_y)) & (lidar_y <= q3_y + 1.5*(q3_y-q1_y))
    
    fig_2d.add_trace(go.Scatter(
        x=lidar_x[mask], y=lidar_y[mask], mode='lines',
        name='LiDAR Position',
        line=dict(color='purple', width=2)
    ))

fig_2d.update_layout(
    title="2D Flight Path (X vs Y)",
    xaxis_title="X Position",
    yaxis_title="Y Position",
    hovermode='closest',
    height=500,
    showlegend=True
)
st.plotly_chart(fig_2d, use_container_width=True)

st.divider()

# ============================================================================
# SECTION 3: Vehicle Dynamics (Attitude & Vibration)
# ============================================================================
st.header("🎛️ Vehicle Dynamics")

col1, col2 = st.columns(2)

with col1:
    st.subheader("Attitude Angles")
    
    roll_cols = [col for col in df.columns if 'roll' in col.lower()]
    pitch_cols = [col for col in df.columns if 'pitch' in col.lower()]
    yaw_cols = [col for col in df.columns if 'yaw' in col.lower() or 'hdg' in col.lower() or 'heading' in col.lower()]
    
    fig_attitude = go.Figure()
    
    if roll_cols:
        fig_attitude.add_trace(go.Scatter(
            x=df[time_col], y=df[roll_cols[0]], name='Roll',
            line=dict(color='red')
        ))
    if pitch_cols:
        fig_attitude.add_trace(go.Scatter(
            x=df[time_col], y=df[pitch_cols[0]], name='Pitch',
            line=dict(color='green')
        ))
    if yaw_cols:
        fig_attitude.add_trace(go.Scatter(
            x=df[time_col], y=df[yaw_cols[0]], name='Yaw',
            line=dict(color='blue')
        ))
    
    fig_attitude.update_layout(
        title="Attitude (Roll, Pitch, Yaw) vs Time",
        xaxis_title="Time (s)",
        yaxis_title="Angle (degrees)",
        hovermode='x unified',
        height=400
    )
    st.plotly_chart(fig_attitude, use_container_width=True)

with col2:
    st.subheader("Vibration Analysis")
    
    vib_x = [col for col in df.columns if 'vibration' in col.lower() and 'x' in col.lower()]
    vib_y = [col for col in df.columns if 'vibration' in col.lower() and 'y' in col.lower()]
    vib_z = [col for col in df.columns if 'vibration' in col.lower() and 'z' in col.lower()]
    
    if vib_x or vib_y or vib_z:
        fig_vib = go.Figure()
        if vib_x:
            fig_vib.add_trace(go.Scatter(x=df[time_col], y=df[vib_x[0]], name='Vib X', line=dict(color='red')))
        if vib_y:
            fig_vib.add_trace(go.Scatter(x=df[time_col], y=df[vib_y[0]], name='Vib Y', line=dict(color='green')))
        if vib_z:
            fig_vib.add_trace(go.Scatter(x=df[time_col], y=df[vib_z[0]], name='Vib Z', line=dict(color='blue')))
        
        fig_vib.update_layout(
            title="Vibration Levels (X, Y, Z)",
            xaxis_title="Time (s)",
            yaxis_title="Acceleration (m/s²)",
            hovermode='x unified',
            height=400
        )
        st.plotly_chart(fig_vib, use_container_width=True)
    else:
        st.info("Vibration data not available")

st.divider()

# ============================================================================
# SECTION 4: System Health
# ============================================================================
st.header("🔋 System Health & Status")

col1, col2, col3 = st.columns(3)

# Battery
with col1:
    st.subheader("Battery Status")
    batt_volt = [col for col in df.columns if 'batt' in col.lower() and 'volt' in col.lower() or 'Batt_Voltage' in col]
    batt_pct = [col for col in df.columns if 'batt' in col.lower() and ('remain' in col.lower() or 'pct' in col.lower() or 'Pct' in col)]
    
    fig_batt = make_subplots(specs=[[{"secondary_y": True}]])
    
    if batt_volt:
        fig_batt.add_trace(go.Scatter(
            x=df[time_col], y=df[batt_volt[0]],
            name='Voltage',
            line=dict(color='orange', width=2)
        ), secondary_y=False)
    
    if batt_pct:
        fig_batt.add_trace(go.Scatter(
            x=df[time_col], y=df[batt_pct[0]],
            name='Remaining %',
            line=dict(color='red', width=2, dash='dash')
        ), secondary_y=True)
    
    fig_batt.update_yaxes(title_text="Voltage (V)", secondary_y=False)
    fig_batt.update_yaxes(title_text="Remaining %", secondary_y=True)
    fig_batt.update_xaxes(title_text="Time (s)")
    fig_batt.update_layout(title="Battery Status", hovermode='x unified', height=400)
    st.plotly_chart(fig_batt, use_container_width=True)

# Current
with col2:
    st.subheader("Power Draw")
    curr_cols = [col for col in df.columns if 'current' in col.lower() or 'Batt_Current' in col]
    
    if curr_cols:
        fig_curr = go.Figure()
        fig_curr.add_trace(go.Scatter(
            x=df[time_col], y=df[curr_cols[0]],
            name='Current',
            line=dict(color='purple', width=2),
            fill='tozeroy'
        ))
        fig_curr.update_layout(
            title="Current Draw vs Time",
            xaxis_title="Time (s)",
            yaxis_title="Current (A)",
            hovermode='x unified',
            height=400
        )
        st.plotly_chart(fig_curr, use_container_width=True)
    else:
        st.info("Current data not available")

# CPU/Load
with col3:
    st.subheader("System Load")
    cpu_cols = [col for col in df.columns if 'cpu' in col.lower() or 'load' in col.lower()]
    
    if cpu_cols:
        fig_cpu = go.Figure()
        fig_cpu.add_trace(go.Scatter(
            x=df[time_col], y=df[cpu_cols[0]],
            name='CPU Load',
            line=dict(color='crimson', width=2)
        ))
        fig_cpu.update_layout(
            title="CPU Load vs Time",
            xaxis_title="Time (s)",
            yaxis_title="Load %",
            hovermode='x unified',
            height=400
        )
        st.plotly_chart(fig_cpu, use_container_width=True)
    else:
        st.info("CPU load data not available")

st.divider()

# ============================================================================
# SECTION 5: Control Inputs
# ============================================================================
rc_cols = [col for col in df.columns if 'RC' in col or 'rc' in col]
if rc_cols:
    st.header("🎮 Control Inputs (RC Channels)")
    
    fig_rc = go.Figure()
    for col in sorted(rc_cols)[:4]:  # Limit to first 4 channels
        fig_rc.add_trace(go.Scatter(
            x=df[time_col], y=df[col],
            name=col,
            mode='lines'
        ))
    
    fig_rc.update_layout(
        title="RC Channel Values (1000-2000 PWM)",
        xaxis_title="Time (s)",
        yaxis_title="PWM Value",
        hovermode='x unified',
        height=400
    )
    st.plotly_chart(fig_rc, use_container_width=True)
    st.divider()

# ============================================================================
# SECTION 6: Data Explorer
# ============================================================================
st.header("🔍 Raw Data Explorer")

if st.checkbox("Show raw data table", value=False):
    col1, col2 = st.columns([3, 1])
    with col1:
        search = st.text_input("Search columns:", "")
    with col2:
        n_rows = st.number_input("Rows to show:", 1, len(df), value=20)
    
    if search:
        cols = [col for col in df.columns if search.lower() in col.lower()]
        st.dataframe(df[[time_col] + cols].head(n_rows), use_container_width=True)
    else:
        st.dataframe(df.head(n_rows), use_container_width=True)

# ============================================================================
# SECTION 7: Statistics Summary
# ============================================================================
st.header("📈 Statistics Summary")

stats_cols = st.multiselect(
    "Select parameters to analyze:",
    df.select_dtypes(include=['number']).columns.tolist(),
    default=[col for col in df.select_dtypes(include=['number']).columns[:5]]
)

if stats_cols:
    stats_df = df[stats_cols].describe().T
    col1, col2 = st.columns([1, 2])
    with col1:
        st.dataframe(stats_df, use_container_width=True)
    with col2:
        st.markdown("### Interpretation")
        st.markdown(f"""
        - **count**: Number of valid measurements
        - **mean**: Average value across flight
        - **std**: Variability (higher = less stable)
        - **min/max**: Range of values
        """)
