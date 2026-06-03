package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public class ShooterConstants {
  public static final int MASTER_MOTOR_ID = 7;
  public static final int SLAVE_MOTOR_ID = 20;

  // ==================== VELOCITY SETPOINTS ====================
  public static final double SHOOTER_IDLE_RPM = 0;
  /** Default fixed spin-up RPM (used when NOT in distance-based mode). */
  public static final AngularVelocity SHOOTER_SPINUP_SPEED = RPM.of(2200);

  public static final AngularVelocity SHOOTER_MAX_SPEED = RPM.of(5500);

  // ==================== FENDER / PRESET SHOTS ====================
  /** RPM for a close-range fender shot (pressed against the hub). */
  public static final AngularVelocity FENDER_SHOT_SPEED = RPM.of(1800.0);
  /** Pivot angle for a close-range fender shot (degrees). */
  public static final Angle FENDER_SHOT_PIVOT_ANGLE = Degrees.of(78.0);

  // ==================== TOLERANCES ====================
  public static final AngularVelocity SHOOTER_SPEED_TOLERANCE = RPM.of(150);
  public static final int STABILITY_CYCLES_REQUIRED = 5;
  /** Percentage tolerance for on-target check (inspired by 254's 4% tolerance). */
  public static final double ON_TARGET_RPM_PERCENT = 0.08;

  // ==================== PID GAINS (Slot 0) ====================
  public static final double SHOOTER_KS = 0.15;
  public static final double SHOOTER_KV = 0.12;
  public static final double SHOOTER_KP = 0.6;
  public static final double SHOOTER_KI = 0.0;
  public static final double SHOOTER_KD = 0.0;

  // ==================== INDEXER / FEEDER ====================
  public static final double FEEDER_SPEED = 1.0;
  public static final double SHOOT_FEED_TIMEOUT = 1.0;

  // ==================== AUTO SHOOTING ====================
  /** Timeout for auto shoot command before giving up (seconds). */
  public static final Time AUTO_SHOOT_TIMEOUT = Seconds.of(3.0);

  // ==================== HEADING ALIGNMENT ====================
  /** Heading tolerance for "aligned to hub" in degrees. */
  public static final Angle HEADING_TOLERANCE = Degrees.of(10.0);

  /**
   * Heading tolerance for shoot-on-the-move in degrees (yaw). MA (6328) uses 10.0deg for launch,
   * 15.0deg for passing.
   */
  public static final double LAUNCH_HEADING_TOLERANCE_DEGREES = 8.0;

  /**
   * Pitch tolerance for shoot-on-the-move in degrees. Rejects shots when the robot is tilted (e.g.,
   * driving over field elements). MA (6328) uses 5.0deg.
   */
  public static final double LAUNCH_PITCH_TOLERANCE_DEGREES = 5.0;

  /**
   * Roll tolerance for shoot-on-the-move in degrees. Rejects shots when the robot is tilted
   * laterally. MA (6328) uses 5.0deg.
   */
  public static final double LAUNCH_ROLL_TOLERANCE_DEGREES = 5.0;

  protected ShooterConstants() {}
}
