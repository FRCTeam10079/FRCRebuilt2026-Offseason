# Heading PID Tuning Guide

This guide explains how to use the updated tools to tune the `SwerveHeadingController` PID gains (SNAP and MAINTAIN) with minimal robot time.

---

## Quick Overview

| Tool | Purpose |
|------|---------|
| **Java: `SwerveHeadingController.logCSV()`** | Outputs CSV to stdout during robot operation |
| **Java: `HeadingStepTest` command** | Automates heading step responses (bound to test controller) |
| **Python: `tune_heading_pid.py`** | Analyzes CSV data, calculates optimal gains |

---

## Setup (One-Time)

### 1. Deploy Robot Code
```bash
./gradlew deploy
```
The CSV logging is now **enabled by default** in `SwerveHeadingController.logCSV()`.

### 2. Install Python Dependencies
```bash
pip install --user robotpy-wpilog  # optional, for .wpilog analysis
# No extra deps needed for CSV parsing
```

---

## Data Collection (Robot Time: ~15 minutes)

### Option A: Automated Step Test (Recommended)
**Controller: Test Xbox (Port 2)**

| Binding | Action |
|---------|--------|
| **Back + Y** | Full sequence: 0° → 90° → -90° → 0° (holds each ~2s) |
| **Back + POV Right** | Single step to 90° (holds 3s) |
| **Back + POV Left** | Single step to -90° (holds 3s) |
| **Back + POV Down** | Single step to 0° (holds 3s) |

**Procedure:**
1. Place robot on carpet (or practice field)
2. Point robot roughly at 0° (red alliance wall = 0° for Blue)
3. Hold **Back + Y** for full sequence, or individual POV bindings
4. Watch DS Log / RioLog for `HDG_CSV` lines streaming
5. Repeat 3-5 times for good data

### Option B: Manual Heading Lock Test
**Controller: Test Xbox (Port 2)**
- **Back + X** → Heading lock to 0° while driving
- Drive around, make sharp turns, strafe
- Tests MAINTAIN mode under realistic disturbance

### Option C: Driver Practice
Just drive normally with heading lock enabled (operator binds). The CSV captures all MAINTAIN mode data during real driving.

---

## Getting CSV Logs Off the Robot

### Method 1: RioLog (Easiest, Real-Time)
**On driver laptop:**
```bash
# SSH into roboRIO and tail the log
ssh lvuser@roboRIO-XXXX-frc.local "tail -f /home/lvuser/rioLog.log" | grep HDG_CSV > heading_log.csv
```
Or use the **VS Code WPILib extension** → "Open RioLog" → filter for `HDG_CSV` → copy/paste.

### Method 2: DS Log Viewer (If Using DS Log)
1. In Driver Station → "View Log" or use `~/wpilib/2026/tools/DSLogFileViewer`
2. Filter for `HDG_CSV`
3. Export to CSV

### Method 3: AdvantageKit .wpilog (Post-Match)
If you use AdvantageKit:
1. Download `.wpilog` from roboRIO: `scp lvuser@roboRIO-XXXX:/home/lvuser/logs/*.wpilog .`
2. Use AdvantageScope → Export CSV → filter for `HeadingController/*` keys
3. **Note:** Current Java doesn't log all fields to AdvantageKit. Use Method 1 or 2 for now.

### Method 4: USB Stick (No Network)
```bash
# On roboRIO (SSH in):
mkdir -p /media/sda1/logs
cp /home/lvuser/rioLog.log /media/sda1/logs/heading_$(date +%Y%m%d_%H%M%S).log
# Pull USB, copy file on laptop
```

---

## Analyzing the Data (Off-Robot, Unlimited Time)

### Basic Analysis
```bash
python tune_heading_pid.py --csv heading_log.csv
```

### With Interactive Tuning Calculator
```bash
python tune_heading_pid.py --csv heading_log.csv --interactive
```

### From RioLog/DS Log (Mixed Output)
```bash
python tune_heading_pid.py --stdout rioLog.log
```

---

## Understanding the Output

### Theoretical Analysis (Always Shown)
```
Steady-State Error While Driving (disturbance=22.9 deg/s):
   With SNAP_KP (0.0600):      2.12 deg
   With MAINTAIN_KP (0.0400):  3.18 deg  <- Target: < 3 deg for shooting

Required KP for Target Steady-State Errors:
   1.0 deg error -> KP = 0.1273
   2.0 deg error -> KP = 0.0637
   3.0 deg error -> KP = 0.0424  <- CURRENT
   4.0 deg error -> KP = 0.0318
   5.0 deg error -> KP = 0.0255

Dynamics Approximation (2nd order model):
   SNAP:      wn=3.29 rad/s, zeta=0.82, rise=0.547s, OS=1.1%
   MAINTAIN:  wn=2.68 rad/s, zeta=0.67, rise=0.671s, OS=5.7%

Recommendations:
   [ACTION] Increase MAINTAIN_KP from 0.0400 to >= 0.0424 for <3 deg error
   [OK] Resnap threshold gap = 6.5 deg (prevents chattering)
```

### Log Analysis (From Your Data)
```
MAINTAIN steady-state error (while driving):
   Average: 2.8 deg
   RMS:     3.1 deg
   Max:     5.4 deg
   Target:  < 3.0 deg (for shooting)
   Effective KP from data: 0.0452
   Configured MAINTAIN_KP:  0.0400

SNAP acquisition analysis:
   Step: 85.2 deg -> 42.1 deg in 0.050s
   Estimated 10-90% rise time: 0.420s

State transitions:
   12.345s: OFF -> SNAP (error=90.00 deg)
   12.890s: SNAP -> MAINTAIN (error=1.20 deg)
   15.200s: MAINTAIN -> SNAP (error=9.50 deg)

Threshold validation:
   HEADING_TOLERANCE_DEGREES = 1.5 deg
   RESNAP_THRESHOLD_DEGREES  = 8.0 deg
```

---

## Interpreting Results & Making Changes

### If MAINTAIN Error > 3° (Most Common Issue)
```
[ACTION] Increase MAINTAIN_KP from 0.0400 to >= 0.0424 for <3 deg error
```
**Edit:** `src/main/java/frc/robot/constants/HeadingControllerConstants.java`
```java
public static final double MAINTAIN_KP = 0.045;  // or higher
```

### If SNAP Overshoots / Oscillates
```
[TUNE] SNAP zeta=0.35 (underdamped). Consider KD >= 0.0042 for zeta=0.7
```
**Edit:** `SNAP_KD` in same file.

### If State Transitions Feel Wrong
- **SNAP exits too early** → Increase `HEADING_TOLERANCE_DEGREES`
- **Resnaps too aggressively** → Increase `RESNAP_THRESHOLD_DEGREES`
- **Chattering at boundary** → Ensure gap ≥ 4° (current 6.5° is good)

---

## Effective Tuning Workflow (Min Robot Time)

```
┌─────────────────────────────────────────────────────────────┐
│ 1. DEPLOY (5 min)                                           │
│    ./gradlew deploy                                         │
├─────────────────────────────────────────────────────────────┤
│ 2. CAPTURE (10 min robot)                                   │
│    - Back+Y × 3 runs (step responses)                       │
│    - Back+X driving × 2 min (MAINTAIN under load)           │
│    - Save rioLog via SSH or VS Code                         │
├─────────────────────────────────────────────────────────────┤
│ 3. ANALYZE (Unlimited off-robot)                            │
│    python tune_heading_pid.py --stdout rioLog.log           │
│    python tune_heading_pid.py --interactive  # calculate    │
├─────────────────────────────────────────────────────────────┤
│ 4. UPDATE (1 min)                                           │
│    Edit HeadingControllerConstants.java                     │
├─────────────────────────────────────────────────────────────┤
│ 5. VALIDATE (5 min robot)                                   │
│    Deploy, run Back+Y once, verify error < 3°               │
└─────────────────────────────────────────────────────────────┘
Total robot time: ~20 minutes
```

---

## Interactive Calculator Examples

### Calculate KP for 2° error:
```
Choice (1-4): 1
Target steady-state error (deg): 2
   Required KP = 0.0637
```

### Calculate KD for critical damping (zeta=0.7) with KP=0.06:
```
Choice (1-4): 2
KP value: 0.06
Target damping ratio (zeta, 0.7-1.0): 0.7
   Required KD = 0.0041
```

### Simulate step response:
```
Choice (1-4): 3
KP: 0.06
KD: 0.004
   wn: 3.29 rad/s, zeta: 0.82
   Rise time: 0.547s
   Overshoot: 1.1%
   SS error:  2.12 deg
```

---

## Advanced: Using SysId Rotation Data

The existing `m_sysIdRoutineRotation` characterizes the **rotation plant** (not the heading PID). If you run it:

1. Run SysId Rotation quasistatic + dynamic
2. Import to SysId tool → get `kV`, `kA`, `kS` (rad/s per volt)
3. The plant is roughly: `ω = kV * V + kA * dV/dt`
4. Your heading controller output (-1 to 1) × 12V = volts to rotation motors
5. This lets you **model the full loop** and compute gains analytically

But for most teams: **step response + theoretical formulas = 90% of the way there.**

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No `HDG_CSV` in rioLog | Check `logCSV()` is called (in `driveWithHeadingLock`) |
| CSV header repeats | Ensure `m_csvHeaderPrinted` resets in `reset()` |
| Timestamps look wrong | `Timer.getFPGATimestamp()` is seconds since boot |
| `omega_actual_radps` is 0 | Ensure `setLoggingContext()` called with measured omega |
| Python parsing fails | Check CSV has header line `HDG_CSV,timestamp_s,...` |

---

## Files Modified

| File | Change |
|------|--------|
| `SwerveHeadingController.java` | Enabled CSV logging (uncommented header) |
| `tune_heading_pid.py` | Added CSV/stdout parsing, enhanced analysis |
| `HeadingStepTest.java` | New command for automated step testing |
| `TestingBindings.java` | Bound step test to Back+Y, POV buttons |

---

## Next Steps for Your Team

1. **This weekend**: Deploy, run Back+Y 3x, capture logs
2. **Monday**: Run Python analysis, update constants
3. **Wednesday**: Validate, tune KD if overshoot
4. **Competition**: Lock gains, don't touch

The CSV logging is now **always on** during `driveWithHeadingLock()` — every match/practice automatically builds your tuning dataset!