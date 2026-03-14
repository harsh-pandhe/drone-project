# ASCEND v9.1 - 1 METER PERFECT FLIGHT TEST GUIDE
## Optimized for Rangefinder + Optical Flow (No LIDAR/SLAM)

---

## CURRENT HARDWARE
✓ **Rangefinder**: DISTANCE_SENSOR (sonar/ultrasonic)  
✓ **Optical Flow**: OPTICAL_FLOW camera  
✓ **Altitude Backup**: Barometer (GLOBAL_POSITION_INT)  
✓ **IMU**: Roll, Pitch, Yaw, Climb rate  
✓ **Battery**: Monitored, low-battery RTL triggered  

✗ **NO**: LIDAR, SLAM, GPS (not needed)  
✗ **NO**: Complex sensor fusion (using simple reliable methods)

---

## TEST FLIGHT PROFILE: 10 SECONDS TOTAL
```
Timeline:
  0:00-0:02  → Arm + Ground vibe check (2s)
  0:02-0:06  → Smooth climb to 1m (4s)
  0:06-0:11  → Hover at 1m (5s) ← Main test phase
  0:11-0:15  → Descent to ground (4s)
  TOTAL:     → 15 seconds flight
```

---

## CRITICAL TUNING PARAMETERS (ALREADY SET IN CODE)

### Altitude Control (PID)
```
ALT_PID_P = 8.0           ← Strong response for 1m
ALT_PID_I = 0.5           ← Gentle integral (prevents overshoot)
ALT_PID_D = 2.5           ← Smooth damping
ALT_DEADBAND = 0.08m      ← ±8cm band (very tight for 1m)
```

### Position Hold (Optical Flow Only)
```
POS_P_GAIN = 50.0         ← 50 PWM per m/s flow
POS_I_GAIN = 1.5          ← Gentle accumulation
POS_DEADBAND = 0.02m      ← ±2cm band (very tight)
POS_MAX_ANGLE = 3.0°      ← Max pitch/roll (gentle)
```

### Ground Vibration Check
```
SPOOL_THR = 1400 PWM      ← Motor idle throttle
MAX_GROUND_VIBE = 18.0    ← m/s² abort threshold
MAX_VIBRATION = 25.0      ← In-flight abort threshold
```

### Takeoff Tuning
```
LIFTOFF_THR = 1500 PWM    ← Just above hover
HOVER_THR = 1450 PWM      ← Neutral throttle (✓ CALIBRATE THIS FOR YOUR FRAME)
HOVER_SETTLE_TIME = 6.0s  ← Time to settle before main hover
HOVER_DURATION = 5.0s     ← Main hover test duration
CLIMB_RAMP_RATE = 2 PWM/cycle ← Smooth climb
```

---

## PRE-TEST CHECKLIST

### Hardware Check
- [ ] Rangefinder power ON (sonar sensor working)
- [ ] Optical flow camera lens CLEAN (no dust/condensation)
- [ ] Propellers balanced and tight
- [ ] Battery charged (>14.5V for 4S Li Po)
- [ ] All connectors seated firmly
- [ ] Pixhawk mounted level + vibration-damped
- [ ] RC transmitter paired + sticks centered

### Software Check
```bash
# 1. Connect to Pixhawk
python3 src/ascend_tui.py

# 2. Verify sensors in TUI:
#    - "LIDAR" value showing (rangefinder altitude)
#    - "OPTFLOW Q: XXX/255" showing quality > 50
#    - Battery voltage showing correctly
#    - Attitude (Roll/Pitch/Yaw) responding to tilt

# 3. If LIDAR shows 0.0m:
#    - Check sonar power cable
#    - Verify DISTANCE_SENSOR messages in MAVLink
#    - Point at floor ~1.5m away to test
```

### Flight Area Preparation
- [ ] Large open indoor space (≥3m x 3m minimum)
- [ ] No reflective surfaces (mirrors, water pools)
- [ ] No obstacles above 2m
- [ ] Smooth takeoff surface (carpet recommended)
- [ ] No moving fans or drafts
- [ ] Record flight with phone/camera from distance

---

## FLIGHT EXECUTION PROCEDURE

### 1. CONNECTION & PREFLIGHT
```
Terminal:
$ cd /home/iic/Desktop/GitHub/drone-project
$ source .venv/bin/activate
$ python3 src/ascend_tui.py

TUI Display Shows:
- Connected to Pixhawk ✓
- All sensors healthy ✓
- Battery >14V ✓
```

### 2. ARMING & GROUND TEST
```
Press: a    (ARM drone)
Watch: Vibration levels in dashboard
  - Should be <5 m/s² at rest
  - Max allowed: 18 m/s² during spool
  - If exceeded → press x and check props
```

### 3. TAKEOFF (AUTOMATED)
```
Press: k    (AUTO TAKEOFF)

Watch:
[AUTO: ✓ Armed! Switching to ALT_HOLD...]
[AUTO: Ground vibe OK. Starting climb...]
[AUTO: ✓ Home locked at (0.00, 0.00, 1.00)]
[AUTO: ✓ Holding at 1.00m for 5s...]
```

### 4. HOVER OBSERVATION (5 seconds)
```
Monitor During Hover:
- Altitude: Should stay within ±0.15m of 1.00m
- Drift: Should stay <0.05m (5cm) from home
- Vibration: Should stay <5 m/s²
- Throttle: Should be around 1450 PWM ±50
- Optical Flow Quality: Should be >100/255

Log Output:
[AUTO: Hover [0.1s] Alt=1.00m Thr=1450 Vib=3.21 Flow=0.002m/s]
[AUTO: Hover [0.2s] Alt=0.99m Thr=1452 Vib=3.18 Flow=0.001m/s]
...
[AUTO: Hover complete. Avg drift: 0.031m, Max: 0.048m]
```

### 5. LANDING (AUTOMATED)
```
Watch:
[AUTO: Descending to ground with position hold...]
[AUTO: Physical touchdown confirmed.]
[AUTO: Disarmed. Landing complete.]

TUI returns to IDLE

FLIGHT TEST COMPLETE! ✓
```

---

## EXPECTED PERFORMANCE METRICS

### Success Criteria for 1m Perfect Flight:
```
Altitude Stability:      ±0.10m (10cm band)
Position Hold Drift:     <0.05m (5cm)
Hover Duration:          5 seconds stable
Vibration Level:         <5 m/s² during flight
Landing:                 Soft touchdown, no bounce
```

### Typical Telemetry (Good Flight):
```
Phase        Time   Altitude   Drift    Throttle   Vibration
Ground Test  2s     0.00m      -        1400PWM    2-4 m/s²
Climb        4s     0.00→1.00m variable 1400→1500  3-6 m/s²
Hover        5s     1.00±0.08m <0.05m   1450±30    2-4 m/s²
Land         4s     1.00→0.00m ~0.02m   1450→1300  2-4 m/s²
TOTAL       15s     ✓          ✓        ✓          ✓
```

---

## TROUBLESHOOTING

### Problem: Drone Drifts Sideways During Hover
**Diagnosis:**
- Check optical flow quality: TUI shows "OPTFLOW Q: XXX/255"
- If Q < 50: Flow camera is poor - clean lens
- If Q > 100: Flow working fine, need tuning

**Solution:**
- Reduce `POS_MAX_ANGLE` to 2.0° (less aggressive)
- Check for air currents (close windows/doors)
- Clean up takeoff area (avoid reflective surfaces)

### Problem: Altitude Bouncy (±0.3m oscillation)
**Diagnosis:**
- Throttle varying rapidly (watch PWM in log)
- Barometer interference or rangefinder drift

**Solution:**
- Reduce `ALT_PID_P` from 8.0 → 6.0
- Increase `ALT_DEADBAND` from 0.08 → 0.12m
- Test rangefinder separately (point at floor)

### Problem: Refuses to Arm
**Diagnosis:**
- Safety switch engaged (physical switch on FC)
- Sensors not calibrated
- Battery too low

**Solution:**
- Physically flip safety switch
- Verify sensors in TUI before flying
- Charge battery to >14.5V

### Problem: Immediate Descends After Takeoff
**Diagnosis:**
- `HOVER_THR` too low for your specific frame
- Optical flow quality very poor
- Rangefinder giving wrong altitude

**Solution:**
- Manually increase neutral throttle (THR value in code)
- Check rangefinder readings in TUI (should show ~1.0m at rest 1m above ground)
- Reduce motor speed/prop size if overweight

---

## TUNING ADJUSTMENTS (If Needed After First Flight)

### If Hover Climbs Too High (>1.15m)
- Reduce `HOVER_THR` by 10 PWM
- Check: Is rangefinder measuring correctly?
- Reduce `ALT_PID_P` to 7.0

### If Hover Sinks Below Target (<0.85m)
- Increase `HOVER_THR` by 10 PWM
- Check: Frame weight vs motor power - OK?
- Increase `ALT_PID_I` to 0.7

### If Position Drifts >0.1m
- Increase `POS_P_GAIN` to 60
- Reduce `POS_DEADBAND` to 0.01
- Limit `POS_MAX_ANGLE` to 2.0°

### If Position Never Settles (oscillates)
- Reduce `POS_P_GAIN` to 40
- Increase `POS_DEADBAND` to 0.05
- Reduce `POS_I_GAIN` to 1.0

---

## DATA LOGGING & ANALYSIS

All flight data saved to:
```
/home/iic/Desktop/GitHub/drone-project/
├── telemetry_YYYYMMDD_HHMMSS.csv   ← Flight telemetry (35 columns)
└── events_YYYYMMDD_HHMMSS.txt      ← Mission log with diagnostics
```

### Export Flight Logs (During Flight)
```
Press: e
Creates:
diag_YYYYMMDD_HHMMSS.txt   ← Full diagnostic snapshot
```

### View Hover Statistics
```
Press: 'q' to quit after flight

Check terminal for:
[AUTO: Hover complete. Avg drift: 0.031m, Max: 0.048m]
```

---

## EMERGENCY PROCEDURES

### Immediate Stop (Any Time)
```
Press: x
Actions:
1. Throttle → 1000 PWM (kills motors)
2. System disarms
3. Flight ends immediately
```

### Safe Return-to-Launch
```
Press: n
Actions:
1. Switches to RTL mode
2. FC handles return automatically
3. Lands at starting position
```

### Low Battery Auto-RTL
```
If battery <10%:
1. System detects critical battery
2. Auto triggers RTL
3. Safe landing guaranteed
```

---

## SUCCESS INDICATORS ✓

After 10-second test flight completes:
1. ✓ Drone returned safely to ground
2. ✓ Altitude stayed within 1.00 ± 0.10m during hover
3. ✓ Position drift <0.05m
4. ✓ No excessive vibrations (< 5 m/s²)
5. ✓ Motor humming smooth (no stuttering)
6. ✓ Battery still >12V at landing
7. ✓ All systems healthy

---

## ADVANCED: SENSOR DIAGNOSTICS

### Check Rangefinder Raw Data
```
In TUI Dashboard:
"ALT: X.XXm [rangefinder] LIDAR:X.XXm BARO:X.XXm"
  ↓
If LIDAR: 0.00m → Sensor disconnected/broken
If LIDAR: >5.00m → Out of range (floor too far)
If LIDAR: matches altitude → ✓ Working perfectly
```

### Check Optical Flow Health
```
In TUI Dashboard:
"OPTFLOW Q: XXX/255"
  ↓
Q < 50     → Flow quality poor (clean camera)
Q 50-100   → Marginal (increases drift risk)
Q > 100    → Excellent (perfect conditions)

"OptFlow Vel: X.XXXm/s"
  ↓
During hover:
Vel ~0.0   → Excellent (stationary)
Vel <0.05  → Good (stable hold)
Vel 0.05-0.15 → Acceptable
Vel >0.15  → Problem (high position drift)
```

### Check Battery Health
```
In TUI Dashboard:
"V: XX.XXV I: ±X.XA Pct: XXX% Drain: X.XXXV/s"
  ↓
V should decrease smoothly (not suddenly drop)
I should be 2-5A during hover (normal)
Drain should be consistent (helps estimate flight time)
```

---

## REFERENCE: CONTROL LOOP TIMING

Flight control runs at 50Hz (0.02s per cycle):
- Altitude PID: Every 20ms
- Position Hold: Every 20ms
- Safety Checks: Every 20ms
- Sensor Fusion: Every 20ms

Total System Latency: <50ms (imperceptible)

---

## POST-FLIGHT NOTES

### What to Record
- Flight duration (should be ~15s total)
- Max altitude reached (should be ~1.00m)
- Max drift (should be <0.05m)
- Vibration peaks (should be <10 m/s²)
- Battery remaining (should be >12V)
- Any peculiar behavior in log

### Analysis Tips
```bash
# View mission log:
tail -100 events_YYYYMMDD_HHMMSS.txt

# Check telemetry (if not LFS):
head -50 telemetry_YYYYMMDD_HHMMSS.csv | column -t -s,
```

---

## SAFETY CERTIFICATION

This flight control system for 1m hover test has been designed with:
- ✓ Multi-layered sensor redundancy (rangefinder + baro + flow)
- ✓ Automatic emergency triggers (battery, vibration, tilt)
- ✓ Dual-mode safety (manual kill + RTL)
- ✓ Vibration monitoring (ground + flight)
- ✓ Smooth throttle profiles (no jerky movements)
- ✓ Timeout protection (15s max flight)

**ALWAYS** supervise flight and BE READY TO PRESS 'x' IF ANYTHING SEEMS WRONG.

---

**Ready for 1 Meter Perfect Test Flight!** 🚁
