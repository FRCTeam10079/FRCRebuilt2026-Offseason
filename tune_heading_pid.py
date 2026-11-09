#!/usr/bin/env python3
"""
Heading PID Tuning Tool for FRC Swerve Drivetrain

This tool:
1. Reads current PID constants from Java source files
2. Parses CSV logs from SwerveHeadingController (stdout capture)
3. Analyzes heading controller performance
4. Recommends PID values based on:
   - Steady-state error formula: error = disturbance_rate / (KP * max_angular_velocity)
   - Step response analysis (rise time, overshoot, settling time)
   - State transition validation

Usage:
    python tune_heading_pid.py                              # Show current constants and theory
    python tune_heading_pid.py --csv log.csv                # Analyze CSV data
    python tune_heading_pid.py --interactive                # Interactive tuning calculator
    python tune_heading_pid.py --csv log.csv --interactive  # Both
"""

import re
import sys
import argparse
import csv
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, List
import math


# ============================================================================
# CONSTANT PARSING FROM JAVA FILES
# ============================================================================

@dataclass
class HeadingConstants:
    # SNAP gains (acquisition)
    snap_kp: float = 0.06
    snap_ki: float = 0.0
    snap_kd: float = 0.003

    # MAINTAIN gains (holding while driving)
    maintain_kp: float = 0.04
    maintain_ki: float = 0.0
    maintain_kd: float = 0.002

    # Thresholds
    heading_tolerance_deg: float = 1.5
    resnap_threshold_deg: float = 8.0

    # Drivetrain
    max_angular_velocity_rad_s: float = math.pi * 1.0  # ~3.14 rad/s
    max_angular_velocity_deg_s: float = 180.0  # ~180 deg/s (pi rad/s)

    # Estimated disturbance rate from driver input while driving
    # From HeadingControllerConstants comments: 0.4 rad/s = ~22.9 deg/s
    disturbance_rate_rad_s: float = 0.4
    disturbance_rate_deg_s: float = 22.918  # 0.4 * 180/pi


def parse_java_constants(file_path: Path) -> HeadingConstants:
    """Parse constants from HeadingControllerConstants.java and DrivetrainConstants.java"""
    constants = HeadingConstants()

    # Parse HeadingControllerConstants.java
    hc_file = file_path.parent / "constants" / "HeadingControllerConstants.java"
    if hc_file.exists():
        content = hc_file.read_text()
        patterns = {
            'snap_kp': r'SNAP_KP\s*=\s*([\d.]+)',
            'snap_ki': r'SNAP_KI\s*=\s*([\d.]+)',
            'snap_kd': r'SNAP_KD\s*=\s*([\d.]+)',
            'maintain_kp': r'MAINTAIN_KP\s*=\s*([\d.]+)',
            'maintain_ki': r'MAINTAIN_KI\s*=\s*([\d.]+)',
            'maintain_kd': r'MAINTAIN_KD\s*=\s*([\d.]+)',
            'heading_tolerance_deg': r'HEADING_TOLERANCE_DEGREES\s*=\s*([\d.]+)',
            'resnap_threshold_deg': r'RESNAP_THRESHOLD_DEGREES\s*=\s*([\d.]+)',
        }
        for attr, pattern in patterns.items():
            match = re.search(pattern, content)
            if match:
                setattr(constants, attr, float(match.group(1)))

    # Parse DrivetrainConstants.java for max angular velocity
    dc_file = file_path.parent / "constants" / "DrivetrainConstants.java"
    if dc_file.exists():
        content = dc_file.read_text()
        # Match MAX_ANGULAR_RATE_RAD_PER_SEC = Math.PI * 1.0
        match = re.search(r'MAX_ANGULAR_RATE_RAD_PER_SEC\s*=\s*Math\.PI\s*\*\s*([\d.]+)', content)
        if match:
            constants.max_angular_velocity_rad_s = math.pi * float(match.group(1))

    return constants


# ============================================================================
# THEORETICAL CALCULATIONS
# ============================================================================

def steady_state_error(kp: float, max_omega_deg_s: float = 180.0, disturbance_deg_s: float = 22.918) -> float:
    """
    Calculate steady-state heading error in degrees.
    Formula from comments: error_deg = disturbance_rate_deg_s / (KP * max_angular_velocity_deg_s)
    """
    if kp <= 0:
        return float('inf')
    return disturbance_deg_s / (kp * max_omega_deg_s)


def required_kp_for_error(target_error_deg: float, max_omega_deg_s: float = 180.0, disturbance_deg_s: float = 22.918) -> float:
    """
    Calculate required KP to achieve target steady-state error.
    KP = disturbance_rate_deg_s / (target_error_deg * max_angular_velocity_deg_s)
    """
    if target_error_deg <= 0:
        return float('inf')
    return disturbance_deg_s / (target_error_deg * max_omega_deg_s)


def estimate_rise_time(kp: float, kd: float, max_omega: float) -> float:
    """
    Estimate 10-90% rise time for second-order system approximation.
    For PID with continuous input, approximate as: t_rise ≈ 1.8 / ω_n
    where ω_n ≈ sqrt(KP * max_omega) for position control
    """
    # Natural frequency approximation
    wn = math.sqrt(kp * max_omega)
    if wn > 0:
        return 1.8 / wn
    return float('inf')


def estimate_overshoot(kp: float, kd: float, max_omega: float) -> float:
    """
    Estimate percent overshoot for second-order system.
    ζ ≈ KD * max_omega / (2 * sqrt(KP * max_omega))
    %OS = 100 * exp(-π * ζ / sqrt(1 - ζ²))
    """
    wn = math.sqrt(kp * max_omega)
    if wn > 0:
        zeta = (kd * max_omega) / (2 * wn)
        if zeta < 1:
            return 100 * math.exp(-math.pi * zeta / math.sqrt(1 - zeta**2))
    return 0.0


# ============================================================================
# CSV PARSING (from SwerveHeadingController stdout)
# ============================================================================

@dataclass
class HeadingLogEntry:
    timestamp: float
    state: str  # OFF, SNAP, MAINTAIN
    goal_deg: float
    current_deg: float
    error_deg: float
    kp: float
    kd: float
    output: float
    omega_cmd_rad_s: float
    omega_actual_rad_s: float


def parse_csv_log(csv_path: Path) -> List[HeadingLogEntry]:
    """
    Parse CSV log from SwerveHeadingController.
    Expected format (from logCSV):
    HDG_CSV,timestamp_s,state,goal_deg,current_deg,error_deg,kP,kD,p_output,d_approx,total_output,omega_cmd_radps,omega_actual_radps
    """
    entries = []

    with open(csv_path, 'r') as f:
        # Find header line (might have non-CSV output before it)
        lines = f.readlines()

    # Find the header line
    header_idx = None
    for i, line in enumerate(lines):
        if line.startswith('HDG_CSV,'):
            header_idx = i
            break

    if header_idx is None:
        print("❌ No HDG_CSV header found in file")
        return []

    # Parse CSV from header onwards
    csv_lines = lines[header_idx:]
    reader = csv.DictReader(csv_lines)

    for row in reader:
        try:
            entry = HeadingLogEntry(
                timestamp=float(row['timestamp_s']),
                state=row['state'],
                goal_deg=float(row['goal_deg']),
                current_deg=float(row['current_deg']),
                error_deg=float(row['error_deg']),
                kp=float(row['kP']),
                kd=float(row['kD']),
                output=float(row['total_output']),
                omega_cmd_rad_s=float(row['omega_cmd_radps']),
                omega_actual_rad_s=float(row['omega_actual_radps']),
            )
            entries.append(entry)
        except (KeyError, ValueError) as e:
            print(f"⚠️  Skipping malformed row: {row} ({e})")
            continue

    print(f"📖 Parsed {len(entries)} heading controller samples from CSV")
    return entries


def parse_csv_from_stdout(stdout_path: Path) -> List[HeadingLogEntry]:
    """
    Parse CSV embedded in robot stdout/stderr log file.
    Extracts lines starting with "HDG_CSV," and parses them.
    """
    entries = []
    csv_lines = ['HDG_CSV,timestamp_s,state,goal_deg,current_deg,error_deg,kP,kD,p_output,d_approx,total_output,omega_cmd_radps,omega_actual_radps']

    with open(stdout_path, 'r') as f:
        for line in f:
            if line.startswith('HDG_CSV,'):
                csv_lines.append(line.strip())

    if len(csv_lines) == 1:
        print("❌ No HDG_CSV lines found in stdout log")
        return []

    reader = csv.DictReader(csv_lines)
    for row in reader:
        try:
            entry = HeadingLogEntry(
                timestamp=float(row['timestamp_s']),
                state=row['state'],
                goal_deg=float(row['goal_deg']),
                current_deg=float(row['current_deg']),
                error_deg=float(row['error_deg']),
                kp=float(row['kP']),
                kd=float(row['kD']),
                output=float(row['total_output']),
                omega_cmd_rad_s=float(row['omega_cmd_radps']),
                omega_actual_rad_s=float(row['omega_actual_radps']),
            )
            entries.append(entry)
        except (KeyError, ValueError) as e:
            continue

    print(f"📖 Extracted {len(entries)} heading controller samples from stdout")
    return entries


# ============================================================================
# LOG ANALYSIS
# ============================================================================

def analyze_log_data(entries: List[HeadingLogEntry], constants: HeadingConstants):
    """Analyze parsed log data and print recommendations"""
    if not entries:
        print("❌ No heading controller data found in log")
        return

    print("\n" + "="*70)
    print("LOG ANALYSIS RESULTS")
    print("="*70)

    # Separate by state
    snap_entries = [e for e in entries if e.state == 'SNAP']
    maintain_entries = [e for e in entries if e.state == 'MAINTAIN']
    off_entries = [e for e in entries if e.state == 'OFF']

    print(f"\nSample counts:")
    print(f"   SNAP:      {len(snap_entries)} samples")
    print(f"   MAINTAIN:  {len(maintain_entries)} samples")
    print(f"   OFF:       {len(off_entries)} samples")

    # Time range
    if entries:
        t_start = entries[0].timestamp
        t_end = entries[-1].timestamp
        print(f"   Time span: {t_end - t_start:.2f}s")

    # Analyze MAINTAIN steady-state error
    if maintain_entries:
        errors = [abs(e.error_deg) for e in maintain_entries]
        avg_error = sum(errors) / len(errors)
        max_error = max(errors)
        rms_error = math.sqrt(sum(e*e for e in errors) / len(errors))

        print(f"\nMAINTAIN steady-state error (while driving):")
        print(f"   Average: {avg_error:.2f} deg")
        print(f"   RMS:     {rms_error:.2f} deg")
        print(f"   Max:     {max_error:.2f} deg")
        print(f"   Target:  < 3.0 deg (for shooting)")

        # Calculate effective KP from observed error
        # error_deg = disturbance / (KP_eff * max_omega_deg_s)
        if avg_error > 0:
            kp_effective = constants.disturbance_rate_deg_s / (avg_error * constants.max_angular_velocity_deg_s)
            print(f"   Effective KP from data: {kp_effective:.4f}")
            print(f"   Configured MAINTAIN_KP:  {constants.maintain_kp:.4f}")

            if avg_error > 3.0:
                required_kp = required_kp_for_error(3.0, constants.max_angular_velocity_deg_s, constants.disturbance_rate_deg_s)
                print(f"   ⚠️  WARNING: ERROR TOO HIGH! Increase MAINTAIN_KP to at least {required_kp:.4f}")
            elif avg_error < 1.0:
                print(f"   ✅ OK: Error well within target")

    # Analyze SNAP performance (step responses)
    if snap_entries:
        print(f"\nSNAP acquisition analysis:")
        # Find step responses: large error -> decreasing error
        step_starts = []
        for i in range(1, len(snap_entries)):
            prev_err = abs(snap_entries[i-1].error_deg)
            curr_err = abs(snap_entries[i].error_deg)
            if prev_err > 10 and curr_err < prev_err * 0.8:
                dt = snap_entries[i].timestamp - snap_entries[i-1].timestamp
                step_starts.append((i, prev_err, curr_err, dt))

        for idx, prev_err, curr_err, dt in step_starts[:5]:  # Show first 5
            print(f"   Step: {prev_err:.1f} deg -> {curr_err:.1f} deg in {dt:.3f}s (idx {idx})")

        # Rise time estimation: time from 10% to 90% of initial error
        for idx, prev_err, curr_err, dt in step_starts[:3]:
            target_90 = prev_err * 0.1  # 10% remaining = 90% settled
            target_10 = prev_err * 0.9  # 90% remaining = 10% settled
            t_10 = None
            t_90 = None
            for j in range(idx, min(idx + 200, len(snap_entries))):  # Look ahead ~1 second at 200Hz
                e = abs(snap_entries[j].error_deg)
                if t_10 is None and e <= target_10:
                    t_10 = snap_entries[j].timestamp
                if t_90 is None and e <= target_90:
                    t_90 = snap_entries[j].timestamp
                    break
            if t_10 and t_90:
                rise_time = t_90 - t_10
                print(f"   Estimated 10-90% rise time: {rise_time:.3f}s")

    # Check state transitions
    print(f"\nState transitions:")
    prev_state = None
    for e in entries:
        if e.state != prev_state:
            print(f"   {e.timestamp:.3f}s: {prev_state} -> {e.state} (error={e.error_deg:.2f} deg)")
            prev_state = e.state

    # Validate thresholds
    print(f"\nThreshold validation:")
    print(f"   HEADING_TOLERANCE_DEGREES = {constants.heading_tolerance_deg} deg")
    print(f"   RESNAP_THRESHOLD_DEGREES  = {constants.resnap_threshold_deg} deg")

    # Check if transitions happen at correct thresholds
    for e in entries:
        if e.state == 'MAINTAIN' and abs(e.error_deg) > constants.heading_tolerance_deg * 1.5:
            print(f"   ⚠️  MAINTAIN with error {e.error_deg:.2f} deg > tolerance {constants.heading_tolerance_deg} deg")
        if e.state == 'SNAP' and abs(e.error_deg) < constants.heading_tolerance_deg:
            print(f"   ℹ️  SNAP with error {e.error_deg:.2f} deg < tolerance (normal during acquisition)")

    # Output analysis
    if maintain_entries:
        outputs = [abs(e.output) for e in maintain_entries]
        avg_output = sum(outputs) / len(outputs)
        max_output = max(outputs)
        print(f"\nMAINTAIN output (rotation demand):")
        print(f"   Average: {avg_output:.3f}")
        print(f"   Max:     {max_output:.3f}")
        if max_output > 0.8:
            print(f"   ⚠️  Output saturating! Consider higher max angular velocity or lower gains")

    # Tracking error vs command analysis
    if maintain_entries:
        tracking_errors = []
        for e in maintain_entries:
            if abs(e.omega_cmd_rad_s) > 0.01:  # Only when commanding movement
                tracking_error = abs(e.omega_cmd_rad_s - e.omega_actual_rad_s)
                tracking_errors.append(tracking_error)
        if tracking_errors:
            avg_track = sum(tracking_errors) / len(tracking_errors)
            print(f"\nRotation tracking (omega_cmd vs omega_actual):")
            print(f"   Avg tracking error: {avg_track:.3f} rad/s ({math.degrees(avg_track):.1f} deg/s)")


# ============================================================================
# INTERACTIVE TUNING CALCULATOR
# ============================================================================

def interactive_tuning(constants: HeadingConstants):
    """Interactive calculator for PID tuning"""
    print("\n" + "="*70)
    print("INTERACTIVE PID TUNING CALCULATOR")
    print("="*70)

    print(f"\nCurrent Constants:")
    print(f"   MAX_ANGULAR_VELOCITY = {constants.max_angular_velocity_rad_s:.3f} rad/s ({math.degrees(constants.max_angular_velocity_rad_s):.1f} deg/s)")
    print(f"   Disturbance rate     = {constants.disturbance_rate_rad_s:.2f} rad/s ({constants.disturbance_rate_deg_s:.1f} deg/s)")
    print(f"   SNAP_KP              = {constants.snap_kp:.4f}")
    print(f"   SNAP_KD              = {constants.snap_kd:.4f}")
    print(f"   MAINTAIN_KP          = {constants.maintain_kp:.4f}")
    print(f"   MAINTAIN_KD          = {constants.maintain_kd:.4f}")
    print(f"   HEADING_TOLERANCE    = {constants.heading_tolerance_deg:.1f} deg")
    print(f"   RESNAP_THRESHOLD     = {constants.resnap_threshold_deg:.1f} deg")

    print(f"\nTheoretical Steady-State Error (while driving):")
    snap_ss_error = steady_state_error(constants.snap_kp, constants.max_angular_velocity_deg_s, constants.disturbance_rate_deg_s)
    maintain_ss_error = steady_state_error(constants.maintain_kp, constants.max_angular_velocity_deg_s, constants.disturbance_rate_deg_s)
    print(f"   With SNAP_KP:      {snap_ss_error:.2f} deg")
    print(f"   With MAINTAIN_KP:  {maintain_ss_error:.2f} deg")

    print(f"\nRequired KP for Target Errors:")
    for target in [1.0, 2.0, 3.0, 4.0, 5.0]:
        req_kp = required_kp_for_error(target, constants.max_angular_velocity_deg_s, constants.disturbance_rate_deg_s)
        print(f"   {target} deg error -> KP = {req_kp:.4f}")

    print(f"\nEstimated Dynamics (approx, assuming 2nd order):")
    snap_wn = math.sqrt(constants.snap_kp * constants.max_angular_velocity_deg_s)
    snap_zeta = constants.snap_kd * constants.max_angular_velocity_deg_s / (2 * snap_wn) if snap_wn > 0 else 0
    maintain_wn = math.sqrt(constants.maintain_kp * constants.max_angular_velocity_deg_s)
    maintain_zeta = constants.maintain_kd * constants.max_angular_velocity_deg_s / (2 * maintain_wn) if maintain_wn > 0 else 0

    snap_rise = 1.8 / snap_wn if snap_wn > 0 else float('inf')
    snap_os = 100 * math.exp(-math.pi * snap_zeta / math.sqrt(1 - snap_zeta**2)) if snap_zeta < 1 else 0
    maintain_rise = 1.8 / maintain_wn if maintain_wn > 0 else float('inf')
    maintain_os = 100 * math.exp(-math.pi * maintain_zeta / math.sqrt(1 - maintain_zeta**2)) if maintain_zeta < 1 else 0

    print(f"   SNAP:      wn={snap_wn:.1f} rad/s, zeta={snap_zeta:.2f}, rise_time={snap_rise:.3f}s, overshoot={snap_os:.1f}%")
    print(f"   MAINTAIN:  wn={maintain_wn:.1f} rad/s, zeta={maintain_zeta:.2f}, rise_time={maintain_rise:.3f}s, overshoot={maintain_os:.1f}%")

    # Interactive prompts
    while True:
        print("\n" + "-"*50)
        print("Options:")
        print("  1. Calculate KP for target steady-state error")
        print("  2. Calculate KD for target damping (zeta)")
        print("  3. Simulate step response")
        print("  4. Exit")
        choice = input("Choice (1-4): ").strip()

        if choice == '1':
            target = float(input("Target steady-state error (deg): "))
            req_kp = required_kp_for_error(target, constants.max_angular_velocity_rad_s, constants.disturbance_rate_rad_s)
            print(f"   Required KP = {req_kp:.4f}")

        elif choice == '2':
            kp = float(input("KP value: "))
            zeta = float(input("Target damping ratio (zeta, 0.7-1.0): "))
            # zeta = KD * max_omega / (2 * sqrt(KP * max_omega))
            kd = 2 * zeta * math.sqrt(kp * constants.max_angular_velocity_deg_s) / constants.max_angular_velocity_deg_s
            print(f"   Required KD = {kd:.6f}")

        elif choice == '3':
            kp = float(input("KP: "))
            kd = float(input("KD: "))
            wn = math.sqrt(kp * constants.max_angular_velocity_deg_s)
            zeta = kd * constants.max_angular_velocity_deg_s / (2 * wn) if wn > 0 else 0
            rise = 1.8 / wn if wn > 0 else float('inf')
            os = 100 * math.exp(-math.pi * zeta / math.sqrt(1 - zeta**2)) if zeta < 1 else 0
            ss_err = steady_state_error(kp, constants.max_angular_velocity_deg_s, constants.disturbance_rate_deg_s)
            print(f"   wn: {wn:.1f} rad/s, zeta: {zeta:.2f}")
            print(f"   Rise time: {rise:.3f}s")
            print(f"   Overshoot: {os:.1f}%")
            print(f"   SS error:  {ss_err:.2f} deg")

        elif choice == '4':
            break


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='Heading PID Tuning Tool')
    parser.add_argument('--csv', type=Path, help='Path to CSV file (exported from robot stdout)')
    parser.add_argument('--stdout', type=Path, help='Path to robot stdout/stderr log file (extracts HDG_CSV lines)')
    parser.add_argument('--interactive', action='store_true', help='Run interactive tuning calculator')
    parser.add_argument('--no-interactive', action='store_true', help='Skip interactive mode, just show analysis')
    parser.add_argument('--constants-dir', type=Path, default=Path('src/main/java/frc/robot'),
                        help='Path to robot Java source directory')
    args = parser.parse_args()

    # Parse constants from Java files
    print("[INFO] Reading constants from Java source...")
    constants = parse_java_constants(args.constants_dir)

    print(f"\n[OK] Parsed Constants:")
    print(f"   SNAP:      KP={constants.snap_kp:.4f}, KI={constants.snap_ki:.4f}, KD={constants.snap_kd:.4f}")
    print(f"   MAINTAIN:  KP={constants.maintain_kp:.4f}, KI={constants.maintain_ki:.4f}, KD={constants.maintain_kd:.4f}")
    print(f"   Tolerance: {constants.heading_tolerance_deg:.1f} deg")
    print(f"   Resnap:    {constants.resnap_threshold_deg:.1f} deg")
    print(f"   Max omega: {constants.max_angular_velocity_rad_s:.3f} rad/s ({constants.max_angular_velocity_deg_s:.1f} deg/s)")
    print(f"   Disturbance: {constants.disturbance_rate_rad_s:.2f} rad/s ({constants.disturbance_rate_deg_s:.2f} deg/s)")

    # Show theoretical analysis always
    print_theoretical_analysis(constants)

    # Analyze log if provided
    entries = []
    if args.csv:
        entries = parse_csv_log(args.csv)
        analyze_log_data(entries, constants)
    elif args.stdout:
        entries = parse_csv_from_stdout(args.stdout)
        analyze_log_data(entries, constants)

    # Run interactive if requested
    if args.interactive and not args.no_interactive:
        interactive_tuning(constants)
    elif not args.csv and not args.stdout and not args.no_interactive:
        print("\n[INFO] Use --interactive for tuning calculator, or --csv <file.csv> / --stdout <file.log> to analyze data")
        print("       Use --no-interactive to skip interactive mode")


def print_theoretical_analysis(constants: HeadingConstants):
    """Print theoretical analysis of current PID values"""
    print("\n" + "="*70)
    print("THEORETICAL ANALYSIS")
    print("="*70)

    print(f"\nSteady-State Error While Driving (disturbance={constants.disturbance_rate_deg_s:.1f} deg/s):")
    snap_ss_error = steady_state_error(constants.snap_kp, constants.max_angular_velocity_deg_s, constants.disturbance_rate_deg_s)
    maintain_ss_error = steady_state_error(constants.maintain_kp, constants.max_angular_velocity_deg_s, constants.disturbance_rate_deg_s)
    print(f"   With SNAP_KP ({constants.snap_kp:.4f}):      {snap_ss_error:.2f} deg")
    print(f"   With MAINTAIN_KP ({constants.maintain_kp:.4f}):  {maintain_ss_error:.2f} deg  <- Target: < 3 deg for shooting")

    print(f"\nRequired KP for Target Steady-State Errors:")
    for target in [1.0, 2.0, 3.0, 4.0, 5.0]:
        req_kp = required_kp_for_error(target, constants.max_angular_velocity_deg_s, constants.disturbance_rate_deg_s)
        status = " <- CURRENT" if abs(req_kp - constants.maintain_kp) < 0.005 else ""
        print(f"   {target} deg error -> KP = {req_kp:.4f}{status}")

    print(f"\nDynamics Approximation (2nd order model):")
    snap_wn = math.sqrt(constants.snap_kp * constants.max_angular_velocity_deg_s)
    snap_zeta = constants.snap_kd * constants.max_angular_velocity_deg_s / (2 * snap_wn) if snap_wn > 0 else 0
    maintain_wn = math.sqrt(constants.maintain_kp * constants.max_angular_velocity_deg_s)
    maintain_zeta = constants.maintain_kd * constants.max_angular_velocity_deg_s / (2 * maintain_wn) if maintain_wn > 0 else 0

    snap_rise = 1.8 / snap_wn if snap_wn > 0 else float('inf')
    snap_os = 100 * math.exp(-math.pi * snap_zeta / math.sqrt(1 - snap_zeta**2)) if snap_zeta < 1 else 0
    maintain_rise = 1.8 / maintain_wn if maintain_wn > 0 else float('inf')
    maintain_os = 100 * math.exp(-math.pi * maintain_zeta / math.sqrt(1 - maintain_zeta**2)) if maintain_zeta < 1 else 0

    print(f"   SNAP:      wn={snap_wn:.1f} rad/s, zeta={snap_zeta:.2f}, rise={snap_rise:.3f}s, OS={snap_os:.1f}%")
    print(f"   MAINTAIN:  wn={maintain_wn:.1f} rad/s, zeta={maintain_zeta:.2f}, rise={maintain_rise:.3f}s, OS={maintain_os:.1f}%")

    # Recommendations
    print(f"\nRecommendations:")
    if maintain_ss_error > 3.0:
        req_kp = required_kp_for_error(3.0, constants.max_angular_velocity_deg_s, constants.disturbance_rate_deg_s)
        print(f"   [ACTION] Increase MAINTAIN_KP from {constants.maintain_kp:.4f} to >= {req_kp:.4f} for <3 deg error")
    else:
        print(f"   [OK] MAINTAIN_KP achieves <3 deg steady-state error")

    if snap_zeta < 0.5:
        req_kd = 2 * 0.7 * math.sqrt(constants.snap_kp * constants.max_angular_velocity_deg_s) / constants.max_angular_velocity_deg_s
        print(f"   [TUNE] SNAP zeta={snap_zeta:.2f} (underdamped). Consider KD >= {req_kd:.6f} for zeta=0.7")

    if maintain_zeta < 0.5:
        req_kd = 2 * 0.7 * math.sqrt(constants.maintain_kp * constants.max_angular_velocity_deg_s) / constants.max_angular_velocity_deg_s
        print(f"   [TUNE] MAINTAIN zeta={maintain_zeta:.2f} (underdamped). Consider KD >= {req_kd:.6f} for zeta=0.7")

    # Check threshold gap
    gap = constants.resnap_threshold_deg - constants.heading_tolerance_deg
    if gap < 4.0:
        print(f"   [WARNING] Resnap threshold ({constants.resnap_threshold_deg} deg) close to tolerance ({constants.heading_tolerance_deg} deg). Gap={gap:.1f} deg. Consider wider gap to prevent chattering.")
    else:
        print(f"   [OK] Resnap threshold gap = {gap:.1f} deg (prevents chattering)")


if __name__ == '__main__':
    main()