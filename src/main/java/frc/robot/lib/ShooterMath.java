package frc.robot.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.GameConstants;
import java.util.function.Supplier;

/**
 * Math utilities for the shooter system.
 *
 * <p>Computes the distance from the robot to the hub and provides memoized setpoint suppliers.
 */
public final class ShooterMath {

  private ShooterMath() {} // Static utility class

  /**
   * Compute the 2D horizontal distance from the robot to its alliance hub.
   *
   * @param robotPose current field-relative robot pose
   * @return distance in meters
   */
  public static Distance getDistanceToHub(Pose2d robotPose) {
    Translation2d hubPosition = getHubPosition();
    return Meters.of(robotPose.getTranslation().getDistance(hubPosition));
  }

  /**
   * Get the field-relative position of the alliance hub.
   *
   * @return hub center as a Translation2d (meters)
   */
  public static Translation2d getHubPosition() {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    return isRed ? GameConstants.RED_HUB_CENTER : GameConstants.BLUE_HUB_CENTER;
  }

  /**
   * Compute the heading (degrees, field-relative) from the robot to the hub.
   *
   * <p>This is the angle the robot should face to be pointed at the hub. Uses atan2 to compute the
   * field-frame angle.
   *
   * @param robotPose current field-relative robot pose
   * @return heading in degrees (-180 to 180)
   */
  public static Angle getHeadingToHub(Pose2d robotPose) {
    Translation2d hubPosition = getHubPosition();
    double dx = hubPosition.getX() - robotPose.getX();
    double dy = hubPosition.getY() - robotPose.getY();

    double forwardHeading = Math.toDegrees(Math.atan2(dy, dx));
    return Degrees.of(MathUtil.inputModulus(forwardHeading, -180, 180));
  }

  /**
   * Convert robot-relative ChassisSpeeds to field-relative ChassisSpeeds.
   *
   * @param robotRelative robot-relative chassis speeds
   * @param heading current robot heading
   * @return field-relative chassis speeds
   */
  public static ChassisSpeeds toFieldRelative(ChassisSpeeds robotRelative, Rotation2d heading) {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, heading);
  }

  /**
   * Create a memoized setpoint supplier that recomputes only once per robot loop.
   *
   * <p>Multiple commands can read from the same supplier without triggering redundant calculations.
   *
   * @param poseSupplier supplier for the current robot pose
   * @return a supplier that yields the current ShooterSetpoint
   */
  public static Supplier<ShooterSetpoint> createSetpointSupplier(Supplier<Pose2d> poseSupplier) {
    return new MemoizedSetpointSupplier(poseSupplier);
  }

  /**
   * Memoized supplier that caches the setpoint and only recomputes when the pose timestamp changes
   * (i.e., once per robot loop iteration).
   */
  private static class MemoizedSetpointSupplier implements Supplier<ShooterSetpoint> {
    private final Supplier<Pose2d> poseSupplier;
    private ShooterSetpoint cached = ShooterSetpoint.STOWED;
    private double lastX = Double.NaN;
    private double lastY = Double.NaN;
    private double lastTheta = Double.NaN;
    private long lastOffsetRevision = Long.MIN_VALUE;

    MemoizedSetpointSupplier(Supplier<Pose2d> poseSupplier) {
      this.poseSupplier = poseSupplier;
    }

    @Override
    public ShooterSetpoint get() {
      Pose2d pose = poseSupplier.get();
      double x = pose.getX();
      double y = pose.getY();
      double theta = pose.getRotation().getRadians();
      long offsetRevision = ShooterSetpoint.getOffsetRevision();

      // Recompute when pose changes OR tuning offsets changed.
      if (x != lastX || y != lastY || theta != lastTheta || offsetRevision != lastOffsetRevision) {
        lastX = x;
        lastY = y;
        lastTheta = theta;
        lastOffsetRevision = offsetRevision;
        Distance distance = getDistanceToHub(pose);
        cached = ShooterSetpoint.fromDistance(distance);
      }
      return cached;
    }
  }
}
