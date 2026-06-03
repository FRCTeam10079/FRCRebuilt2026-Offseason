package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class DrivetrainConstants {
  public static final double MAX_SPEED_MPS = 10.0;
  public static final double MAX_ANGULAR_RATE_RAD_PER_SEC = Math.PI * 2.0;

  public static final LinearVelocity MAX_ALIGNING_SPEED_MPS = MetersPerSecond.of(4.0);
  public static final AngularVelocity MAX_ALIGNING_ANGULAR_RATE_RAD_PER_SEC =
      RadiansPerSecond.of(Math.PI);

  public static final double NORMAL_SPEED_COEFFICIENT = 1.0;
  public static final double SLOW_MODE_COEFFICIENT = 0.7;
  public static final double SCORING_SPEED_COEFFICIENT = 0.5;

  public static final double DEADBAND_PERCENT = 0.1;
  public static final double SKEW_COMPENSATION_SCALAR = -0.03;

  public static final double CHOREO_TRANSLATION_KP = 7.0;
  public static final double CHOREO_TRANSLATION_KI = 0.0;
  public static final double CHOREO_TRANSLATION_KD = 0.0;
  public static final double CHOREO_HEADING_KP = 5.0;
  public static final double CHOREO_HEADING_KI = 0.0;
  public static final double CHOREO_HEADING_KD = 0.0;

  public static final double PP_TRANSLATION_KP = 7.0;
  public static final double PP_TRANSLATION_KI = 0.0;
  public static final double PP_TRANSLATION_KD = 0.0;
  public static final double PP_ROTATION_KP = 5.0;
  public static final double PP_ROTATION_KI = 0.0;
  public static final double PP_ROTATION_KD = 0.0;

  public static final double POSITION_TOLERANCE_METERS = 0.02;
  public static final double YAW_TOLERANCE_RADIANS = Math.PI / 32;

  public static final double ALIGN_PID_KP = 8.0;
  public static final double ALIGN_PID_KI = 0.0;
  public static final double ALIGN_PID_KD = 0.01;
  public static final double ALIGN_ROTATION_KP = 3.0;
  public static final double ALIGN_ROTATION_KD = 0.02;

  public static final double ALIGN_SPEED_MPS = 3.5;
  public static final double ALIGN_ROTATION_SPEED = 0.9;

  public static final double ALIGN_OFFSET_X_LEFT = -0.41;
  public static final double ALIGN_OFFSET_Y_LEFT = 0.13;
  public static final double ALIGN_OFFSET_X_RIGHT = -0.41;
  public static final double ALIGN_OFFSET_Y_RIGHT = -0.23;
  public static final double ALIGN_OFFSET_X_CENTER = -0.50;
  public static final double ALIGN_OFFSET_Y_CENTER = 0.0;

  // ==================== SHOOT-ON-THE-MOVE ====================
  /**
   * Maximum polar velocity of the ball at the hub (rad/s) during shoot-on-the-move. If the driver's
   * translation would cause the ball to sweep past the hub opening faster than this rate,
   * translation is clamped.
   *
   * <p>MA (6328) uses 0.5 rad/s. Start there and adjust: - Increase if driver feels too constrained
   * - Decrease if shots miss at speed
   */
  public static final double MAX_POLAR_VELOCITY_RAD_PER_SEC = 0.5;

  /**
   * Maximum translation speed (m/s) while in shoot-on-the-move mode. This is the
   * driver-controllable max speed; the velocity limiter may further reduce it.
   */
  public static final double MAX_SHOOTING_SPEED_MPS = 1.0;

  /**
   * Maximum angular velocity (rad/s) while in shoot-on-the-move mode. This is the cap on the
   * heading controller output.
   */
  public static final double MAX_SHOOTING_ANGULAR_RATE_RAD_PER_SEC = Math.PI * 0.75;

  /**
   * Heading error (degrees) below which center-of-rotation shifting is NOT applied. When the
   * heading is close enough to the target, pivot around robot center as normal.
   */
  public static final double COR_MIN_ERROR_DEG = 15.0;

  /**
   * Heading error (degrees) at which full center-of-rotation shifting is applied. The COR smoothly
   * interpolates between robot center and shooter position as heading error transitions from
   * COR_MIN to COR_MAX.
   */
  public static final double COR_MAX_ERROR_DEG = 30.0;

  // ==================== O-LOCK THRESHOLDS ====================
  /**
   * Linear speed threshold (m/s) below which O-lock engages during SOTM. When both linear speed AND
   * angular speed are below their respective thresholds, the drivetrain enters X-stop to prevent
   * jitter.
   */
  public static final double OLOCK_LINEAR_THRESHOLD_MPS = 0.1;

  /** Angular speed threshold (rad/s) below which O-lock engages during SOTM. */
  public static final double OLOCK_OMEGA_THRESHOLD_RAD_PER_SEC = 0.15;

  protected DrivetrainConstants() {}
}
