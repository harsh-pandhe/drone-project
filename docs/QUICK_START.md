# 1 METER PERFECT FLIGHT - QUICK START

## START FLIGHT TEST (Copy-Paste Ready)
```bash
cd /home/iic/Desktop/GitHub/drone-project
source .venv/bin/activate
python3 src/ascend_tui.py
```

## FLIGHT SEQUENCE
| Step | Time | Action | Keys |
|------|------|--------|------|
| 1    | 0:00 | Power on, inspect | - |
| 2    | 0:05 | Run test prog | - |
| 3    | 0:10 | ARM drone | Press: `a` |
| 4    | 0:15 | TAKEOFF (auto) | Press: `k` |
| 5    | 2:00 | HOVER (observe) | Watch dashboard |
| 6    | 7:00 | LAND (auto) | Automatic |
| 7    | 8:00 | CHECK data | - |

## KEY COMMANDS
```
k = Auto Takeoff to 1m
j = Auto Land (smooth)
n = Return Home
a = Arm
d = Disarm  
x = EMERGENCY KILL (instant stop)
q = Quit
```

## CRITICAL TUNING (Already Set!)
- **Target Height**: 1.0m
- **Altitude P-gain**: 8.0 (strong response)
- **Altitude Deadband**: ±8cm (tight hold)
- **Position Deadband**: ±2cm (very tight)
- **Position Max Angle**: 3.0° (gentle)
- **Hover Time**: 5 seconds
- **Max Vibration**: 25 m/s² abort threshold

## PREFLIGHT CHECKLIST
- [ ] Rangefinder working (shows value in TUI)
- [ ] Optical flow quality > 50/255
- [ ] Battery > 14.5V
- [ ] All props tight
- [ ] Flight area clear 3m x 3m
- [ ] Takeoff surface smooth
- [ ] Pixhawk mounted level

## EXPECTED RESULTS
✓ Altitude: 1.00m ± 0.10m  
✓ Drift: < 0.05m (5cm)  
✓ Vibration: < 5 m/s²  
✓ Duration: 15s total (2s ground + 4s climb + 5s hover + 4s land)  
✓ Landing: Soft touchdown

## SENSORS ACTIVE
✓ Rangefinder (altitude primary)  
✓ Optical Flow (position hold)  
✓ Barometer (altitude backup)  
✓ IMU (attitude + vibration)  
✓ Battery monitor  

## SENSORS NOT USED
✗ LIDAR/SLAM (too complex for 1m indoor)  
✗ GPS (not needed)  
✗ Vision system (not needed)  

## IF PROBLEMS OCCUR
```
During flight: Press x (emergency stop)
Won't arm: Check safety switch
Drifts too much: Increase POS_P_GAIN
Bounces vertically: Reduce ALT_PID_P
Overshoots altitude: Reduce LIFTOFF_THR
```

## DATA LOCATION
```
Flight logs saved to: /home/iic/Desktop/GitHub/drone-project/
telemetry_*.csv  → 35 sensor columns (or LFS pointer)
events_*.txt     → Mission log with diagnostics
diag_*.txt       → Export with 'e' key
```

## 10-SECOND FLIGHT PROFILE (ACTUAL)
```
T=0s    → Press 'a' (ARM)
T=2s    → Press 'k' (AUTO TAKEOFF) or after arm
T=2-4s  → Ground vibration check (motors spin)
T=4-8s  → Climb to 1m (smooth ramp)
T=8-13s → HOVER at 1m ← MAIN TEST PHASE
T=13s   → Auto descent
T=17s   → Touchdown + disarm
T=18s   → Ready for analysis
```

## MONITOR DURING FLIGHT
Watch TUI for:
```
Altitude line:   "ALT: 1.00m [rangefinder] LIDAR:1.00m"
Position line:   "SLAM: Pos(0.00, 0.00, 1.00)"
Vibration line:  "VIB: X=3.21 Y=2.98 Z=3.05 m/s² Max:3.21"
Hover report:    "Avg drift: 0.031m, Max: 0.048m"
```

## PERFECT FLIGHT = ALL ✓
```
✓ Drone took off smoothly
✓ Climbed to 1.00m steadily
✓ Stayed within ±0.10m during hover
✓ Position held (drift < 0.05m)
✓ Landed softly on ground
✓ No excessive vibrations
✓ Battery remained >12V
✓ All sensors worked perfectly
```

---

## POST-FLIGHT ANALYSIS
```bash
# View last 50 lines of mission log:
tail -50 events_*.txt

# Check telemetry (if not LFS):
head -100 telemetry_*.csv | cut -d, -f1-10

# Check hover statistics in log:
grep "Hover complete" events_*.txt
grep "Avg drift" events_*.txt
```

---

**YOU ARE READY TO FLY! 🚁**

_Follow the checklist, press the keys in order, watch the dashboard, and enjoy perfect 1m hover test!_
