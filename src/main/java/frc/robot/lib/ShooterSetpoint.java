package frc.robot.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterPivotConstants;

/**
 * Represents a complete shooter setpoint: flywheel RPM and pivot angle. <br>
 * The setpoint is computed from the distance to the hub using interpolation tables.
 *
 * @see ShooterInterpolationTable
 */
public record ShooterSetpoint(AngularVelocity flywheelRPM, Angle pivotAngle, boolean isValid) {

  private static double rpmOffsetRpm = 0.0;
  private static double angleOffsetDeg = 0.0;
  private static long offsetRevision = 0;

  /**
   * Create a new ShooterSetpoint.
   *
   * @param flywheelRPM target flywheel RPM
   * @param pivotAngle target pivot angle in degrees
   * @param isValid whether this setpoint is achievable by the hardware
   */
  public ShooterSetpoint {}

  public ShooterSetpoint(AngularVelocity flywheelRPM, Angle pivotAngleDegrees) {
    this(flywheelRPM, pivotAngleDegrees, true);
  }

  /**
   * Compute a ShooterSetpoint from the distance to the hub.
   *
   * <p>Queries the interpolation tables for RPM and pivot angle, then validates that the resulting
   * angle is within the pivot's range of motion.
   *
   * @param distanceMeters 2D distance from robot to hub center (meters)
   * @return a ShooterSetpoint with the appropriate RPM and angle
   */
  public static ShooterSetpoint fromDistance(Distance distanceMeters) {
    AngularVelocity rpm = Constants.clamp(
        RPM.of(ShooterInterpolationTable.getRPM(distanceMeters).in(RPM) + rpmOffsetRpm),
        RPM.zero(),
        ShooterConstants.SHOOTER_MAX_SPEED);
    Angle angle =
        Degrees.of(ShooterInterpolationTable.getAngle(distanceMeters).in(Degrees) + angleOffsetDeg);

    // Validate that the angle is within the pivot's physical range
    boolean valid =
        angle.gte(ShooterPivotConstants.MIN_ANGLE) && angle.lte(ShooterPivotConstants.MAX_ANGLE);

    // Clamp to safe range even if invalid (so motor doesn't go crazy)
    angle =
        Constants.clamp(angle, ShooterPivotConstants.MIN_ANGLE, ShooterPivotConstants.MAX_ANGLE);

    return new ShooterSetpoint(rpm, angle, valid);
  }

  public static void adjustRPMOffset(AngularVelocity deltaRPM) {
    rpmOffsetRpm += deltaRPM.in(RPM);
    offsetRevision++;
  }

  public static void adjustAngleOffset(Angle deltaAngle) {
    angleOffsetDeg += deltaAngle.in(Degrees);
    offsetRevision++;
  }

  public static AngularVelocity getRPMOffset() {
    return RPM.of(rpmOffsetRpm);
  }

  public static Angle getAngleOffset() {
    return Degrees.of(angleOffsetDeg);
  }

  public static void resetOffsets() {
    rpmOffsetRpm = 0.0;
    angleOffsetDeg = 0.0;
    offsetRevision++;
  }

  public static long getOffsetRevision() {
    return offsetRevision;
  }

  /** @return the target flywheel RPM */
  @Override
  public AngularVelocity flywheelRPM() {
    return flywheelRPM;
  }

  /** @return the target pivot angle in degrees */
  @Override
  public Angle pivotAngle() {
    return pivotAngle;
  }

  /**
   * Whether this setpoint is valid (achievable by the hardware). If the setpoint exceeds hardware
   * limits, it's marked invalid and the robot should NOT fire.
   */
  @Override
  public boolean isValid() {
    return isValid;
  }

  /** A "stowed" setpoint - pivot at minimum angle, no flywheel spin. */
  public static final ShooterSetpoint STOWED =
      new ShooterSetpoint(RPM.zero(), ShooterPivotConstants.MIN_ANGLE, true);

  /**
   * A fixed "fender shot" setpoint for shooting while pressed up against the hub. These values
   * bypass distance-based interpolation for a known reliable shot.
   */
  public static final ShooterSetpoint FENDER_SHOT = new ShooterSetpoint(
      ShooterConstants.FENDER_SHOT_SPEED, ShooterConstants.FENDER_SHOT_PIVOT_ANGLE, true);
}
