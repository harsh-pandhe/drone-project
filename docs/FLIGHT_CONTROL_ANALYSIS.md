# Flight Control System Issue Analysis & Fixes

## Overview
Three critical flight control failures identified in `src/ascend_tui.py`:
1. **Takeoff Failure** - Altitude stalled at 0.14-0.19m instead of 1.0m
2. **Yaw Oscillations** - Wild swings between -57° and +43°  
3. **Hard Landing** - Rapid uncontrolled descent causing crash

---

## Issue #1: Takeoff Failure - Root Cause Analysis

### Configuration Parameters (Lines 20-47)
```python
LIFTOFF_THR = 1500        # Only +50 PWM above hover
HOVER_THR = 1450          # Neutral throttle
CLIMB_DAMPING = 0.3       # Aggressive damping (70% reduction!)
CLIMB_DIST_THRESHOLD = 0.15  # Damping activates at 15cm altitude
CLIMB_RAMP_RATE = 2       # Very conservative ramp rate
```

### The Problem (Lines 780-800)
The takeoff sequence has a **catastrophic damping bug**:

```python
# Line 789-791 in auto_takeoff_sequence()
if alt_to_target < CLIMB_DIST_THRESHOLD:  # Triggers at just 15cm
    climb_throttle = HOVER_THR + int(
        (LIFTOFF_THR - HOVER_THR) * 
        alt_to_target / CLIMB_DIST_THRESHOLD * 
        CLIMB_DAMPING  # ← MULTIPLE BY 0.3 (KILLS CLIMB!)
    )
```

### What Happens
1. Throttle ramps from 1400 → 1500 during liftoff
2. Drone lifts off ~5-10cm (normal)
3. `alt_to_target` becomes < 0.15m → **damping activates**
4. Throttle formula reduces to: `1450 + (50 * 0.XX * 0.3)` = ~1450-1458 PWM
5. Loss of lift authority → **drone stalls and falls back to ground**

### Root Cause
- **LIFTOFF_THR (1500)** is too conservative - only +50 PWM above hover
- **CLIMB_DAMPING (0.3)** kicks in too early and too aggressively
- **CLIMB_DIST_THRESHOLD (0.15m)** activates damping during the climb phase instead of approach phase

### Impact
- Drone cannot sustain climb beyond ~0.2m altitude
- Takeoff sequence always aborts or stalls
- Prevents all autonomous flight testing

---

## Issue #2: Yaw Oscillations - Root Cause Analysis

### The Problem
Yaw angle swings between -57° and +43° with no apparent yaw command input.

### Investigation
- Line 1502: `state['rc_yaw'] = 1500` (neutral during auto_takeoff_sequence)
- Lines 78-88: `'rc_yaw': 1500` in global state (neutral)
- No active yaw controller implemented in takeoff sequence

### Root Cause
**Compass/IMU instability or firmware gyro mode active:**
1. ArduCopter default mode may have gyro rate damping enabled
2. Compass can oscillate if not calibrated properly
3. No explicit yaw rate controller in STABILIZE or ALT_HOLD modes
4. Possible feedback loop oscillation in firmware yaw stabilization

### Secondary Contributing Factor
- `STABILIZE` mode (line 714) enters with potentially uncalibrated compass
- No heading hold or rate damping implemented on the Python side

---

## Issue #3: Hard Landing - Root Cause Analysis

### The Problem  
Rapid uncontrolled descent at end of flight - drone crashes without gradual descent

### Possible Causes
1. **Battery failsafe** - Voltage drops → autopilot emergency landing
2. **Altitude sensor failure** - Lidar/Baro suddenly reads wrong → loss of altitude hold
3. **Position hold instability** - Drift correction causes erratic descent
4. **Timeout in landing sequence** - Line 827 has 6 second timeout that triggers abort

### Investigation Points
- Lines 827-831: 6-second timeout during settle phase → possible hard cutoff
- No gradual descent sequence implemented
- Battery voltage not monitored for graceful failsafe

---

## Recommended Fixes

### Fix #1: Increase Liftoff Authority & Fix Damping

**File:** `src/ascend_tui.py`

**Changes:**

#### A. Increase LIFTOFF_THR to provide adequate authority
```python
# Line 23 - BEFORE:
LIFTOFF_THR = 1500        # Only +50 above hover

# AFTER:
LIFTOFF_THR = 1600        # +150 PWM above hover for positive climb
```

#### B. Fix the damping formula to apply ONLY during approach
```python
# Lines 789-791 - BEFORE:
if alt_to_target < CLIMB_DIST_THRESHOLD:
    climb_throttle = HOVER_THR + int((LIFTOFF_THR - HOVER_THR) * alt_to_target / CLIMB_DIST_THRESHOLD * CLIMB_DAMPING)
else:
    climb_throttle += CLIMB_THROTTLE_RAMP_RATE

# AFTER:
if alt_to_target < CLIMB_DIST_THRESHOLD:
    # Damped approach: smoothly reduce throttle as we approach target
    # Formula: hover + (delta * ratio * damping) where ratio = 0 to 1
    damp_ratio = alt_to_target / CLIMB_DIST_THRESHOLD
    climb_throttle = HOVER_THR + int((LIFTOFF_THR - HOVER_THR) * damp_ratio * CLIMB_DAMPING)
    # Cap minimum to prevent stall
    climb_throttle = max(1500, climb_throttle)
elif climb_throttle < LIFTOFF_THR:
    climb_throttle = min(LIFTOFF_THR, climb_throttle + CLIMB_THROTTLE_RAMP_RATE)
```

#### C. Increase damping threshold to approach phase only
```python
# Line 36 - BEFORE:
CLIMB_DIST_THRESHOLD = 0.15 # Activates damping during climb!

# AFTER:
CLIMB_DIST_THRESHOLD = 0.30 # Only damp in final 30cm approach to target
```

#### D. Reduce damping aggressiveness
```python
# Line 35 - BEFORE:
CLIMB_DAMPING = 0.3         # 70% throttle reduction!

# AFTER:
CLIMB_DAMPING = 0.6         # 40% throttle reduction (less aggressive)
```

#### E. Increase ramp rate for faster climb
```python
# Line 47 - BEFORE:
CLIMB_RAMP_RATE = 2         # 2 PWM/cycle = 50 cycles to change 100 PWM

# AFTER:
CLIMB_RAMP_RATE = 5         # 5 PWM/cycle = 20 cycles to change 100 PWM (2x faster)
```

### Fix #2: Implement Yaw Rate Damping

**File:** `src/ascend_tui.py`

**Add new parameters (after line 42):**
```python
# --- YAW STABILIZATION ---
YAW_RATE_LIMIT = 45.0        # Degrees per second (prevent wild oscillations)
YAW_KP = 0.1                 # Yaw dampening gain
```

**Add yaw dampening to flying routine:**
Find where `send_rc_override()` is called and add:

```python
# Add this in the settling/hover loop around line 862
# YAW DAMPING to prevent oscillations
if abs(state['gyro_z']) > YAW_RATE_LIMIT:
    # Heavy damping if rate too high
    state['rc_yaw'] = 1500 - int(state['gyro_z'] * YAW_KP * 10)
else:
    # Light damping
    state['rc_yaw'] = 1500 - int(state['gyro_z'] * YAW_KP)
```

### Fix #3: Implement Graceful Landing Sequence

**File:** `src/ascend_tui.py`

**Replace aggressive descent with gradual landing (after line 831+):**

```python
# NEW: Graceful landing sequence instead of hard timeout
update_log("AUTO: Initiating graceful landing...")

landing_start = time.time()
landing_throttle = HOVER_THR
descent_rate = 50  # PWM/second reduction

while state['armed']:
    current_alt = state['lidar_alt'] if state['rangefinder_healthy'] else state['alt']
    
    if current_alt < 0.05:  # Ground detected
        update_log("AUTO: Landing complete")
        break
    
    # Gradual throttle reduction
    elapsed = time.time() - landing_start
    landing_throttle = max(1000, HOVER_THR - int(elapsed * descent_rate))
    
    state['rc_throttle'] = int(landing_throttle)
    send_rc_override(master)
    time.sleep(0.05)
    
    # Safety timeout (30 seconds)
    if elapsed > 30.0:
        update_log("AUTO: Landing timeout")
        break
    
    # Battery failsafe check
    if state['batt_v'] < MIN_SAFE_VOLTAGE:
        update_log(f"AUTO: Battery critical ({state['batt_v']}V)")
        state['rc_throttle'] = 1000
        send_rc_override(master)
        break

# Disarm
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
)
```

---

## Validation & Testing

### Test Plan After Fixes

1. **Takeoff Test**
   - Expected: Smooth climb from 0 → 1.0m with stable altitude
   - Monitor: Throttle PWM, altitude vs time
   - Success: Reaches 1.0m within 3-4 seconds

2. **Yaw Stability Test**
   - Expected: Yaw angle stays within ±5° of target
   - Monitor: Heading angle, gyro Z rate
   - Success: No oscillations > ±10°

3. **Landing Test**
   - Expected: Gradual descent from 1.0m → 0m over ~15-20 seconds
   - Monitor: Altitude descent rate, throttle reduction
   - Success: Lands gently without crash

---

## Parameter Summary - Before vs After

| Parameter | Before | After | Reason |
|-----------|--------|-------|--------|
| LIFTOFF_THR | 1500 | 1600 | +150 PWM provides positive climb authority |
| CLIMB_DAMPING | 0.3 | 0.6 | Less aggressive damping (40% vs 70% reduction) |
| CLIMB_DIST_THRESHOLD | 0.15m | 0.30m | Damping only in final approach, not during climb |
| CLIMB_RAMP_RATE | 2 PWM | 5 PWM | Faster climb acceleration |
| YAW_KP | - | 0.1 | New: Yaw rate damping to prevent oscillations |
| Landing | Hard timeout | Gradual descent | Prevents hard crashes |

---

## Implementation Priority

1. **Critical (Do First)** - Fix #1 (Takeoff)
2. **High (Do Second)** - Fix #3 (Landing) 
3. **Medium (Do Third)** - Fix #2 (Yaw)

Reason: Can't test yaw stability if takeoff fails. Can't evaluate landing quality if descent is uncontrolled.

