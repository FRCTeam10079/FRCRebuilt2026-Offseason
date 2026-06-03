package frc.robot.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.GameConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Core shoot-on-the-move calculator, adapted from Mechanical Advantage (6328).
 *
 * <p>This class computes velocity-compensated launch parameters so the robot can shoot while
 * moving. Instead of aiming where the hub IS, it computes where to aim based on where the robot's
 * velocity would carry the ball during its flight time.
 *
 * <p>Algorithm overview: 1. Project the robot pose forward by a phase delay (latency compensation)
 * 2. Iteratively solve for a self-consistent (distance, TOF) pair using the robot's field-relative
 * velocity - the "lookahead" 3. Look up RPM, pivot angle, and heading from the lookahead distance
 * 4. Compute angular velocity feedforward for smooth heading and pivot tracking
 *
 * <p>Usage: Call {@link #update(Pose2d, ChassisSpeeds, Rotation2d)} once per robot loop, then read
 * the latest {@link LaunchParameters} via {@link #getParameters()}.
 */
public class LaunchCalculator {

  // ==================== SINGLETON ====================
  private static LaunchCalculator instance;

  public static LaunchCalculator getInstance() {
    if (instance == null) {
      instance = new LaunchCalculator();
    }
    return instance;
  }

  // ==================== TUNING CONSTANTS ====================

  /**
   * Phase delay in seconds - compensates for latency between sensing and actuation. Projection
   * forward along current velocity by this amount.
   *
   * <p>TODO: TUNE ON THE ROBOT - start at 0.03 (30ms) and adjust if shots consistently lead/lag.
   */
  private static final double PHASE_DELAY_SECONDS = 0.03;

  /**
   * Number of iterations for the lookahead convergence loop. 20 is more than enough for convergence
   * - MA uses 20.
   */
  private static final int LOOKAHEAD_ITERATIONS = 20;

  /**
   * Maximum allowed lookahead offset in meters. Prevents the iterative loop from diverging when TOF
   * values are imprecise or the robot is moving very fast. At 5 m/s with 0.5s TOF the natural
   * offset is 2.5m, so 3.0m is a safe cap.
   */
  private static final double MAX_LOOKAHEAD_OFFSET_METERS = 2.5;

  /** Robot loop period in seconds (50Hz = 0.02s). */
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  /**
   * Moving average filter window for pivot angle velocity feedforward. Smooths out the derivative
   * to prevent jitter.
   *
   * <p>0.4s window = 20 samples at 50Hz. Wider than MA default for smoother pivot tracking.
   */
  private static final double PIVOT_ANGLE_FILTER_WINDOW_SECONDS = 0.4;

  /**
   * Moving average filter window for drive heading velocity feedforward. Wider window = smoother
   * but more lag. Heavily smoothed to prevent heading oscillation.
   *
   * <p>0.4s window = 20 samples at 50Hz. Reduced to lower feedforward phase lag when changing
   * translation direction while retaining meaningful smoothing.
   */
  private static final double DRIVE_ANGLE_FILTER_WINDOW_SECONDS = 0.4;

  /**
   * Minimum distance (meters) at which shoot-on-the-move is valid. Below this the robot is too
   * close for the system to work well.
   *
   * <p>TODO: TUNE ON THE ROBOT - should match the closest distance in the interp tables.
   */
  private static final double MIN_VALID_DISTANCE = 0.4;

  /**
   * Maximum distance (meters) at which shoot-on-the-move is valid.
   *
   * <p>TODO: TUNE ON THE ROBOT - should match the farthest distance in the interp tables.
   */
  private static final double MAX_VALID_DISTANCE = 5.0;

  /**
   * Shooter offset from robot center as (x, y) in robot frame.
   *
   * <p>x = forward/back from center (positive = forward) y = left/right from center (positive =
   * left)
   *
   * <p>TODO: MEASURE ON THE ROBOT - set to (0, 0) if shooter is roughly centered. If lateral offset
   * is > ~5cm, measure and set it for accurate heading compensation.
   */
  /**
   * Shooter offset from robot center as (x, y) in robot frame.
   *
   * <p>x = forward/back from center (positive = forward) y = left/right from center (positive =
   * left)
   *
   * <p>Public so that the COR shifting in shootOnTheMoveDriveCommand can reference them.
   */
  public static final double SHOOTER_OFFSET_X = 10.5 * 0.0254; // 10.5 inches forward in meters

  public static final double SHOOTER_OFFSET_Y = 0.0; // meters, no lateral offset

  // ==================== PASSING TARGET ====================
  // When passing, aim at a point near the alliance wall where a teammate can
  // catch.
  // Adapted from MA (6328). X is near the wall, Y is offset from center.
  // TODO: TUNE THESE FOR OUR FIELD STRATEGY
  private static final LoggedTunableNumber passTargetXMeters =
      new LoggedTunableNumber("LaunchCalc/PassTargetXMeters", 54.0 * 0.0254);
  private static final LoggedTunableNumber passTargetYMeters =
      new LoggedTunableNumber("LaunchCalc/PassTargetYMeters", 65.0 * 0.0254);

  /**
   * Auto-pass target switching can cause heading to rotate away from the hub when crossing the hub
   * X line. Keep enabled so far-side SOTM uses the pass target while near-side SOTM still uses the
   * hub.
   */
  private static final boolean ENABLE_AUTO_PASSING = true;

  // ==================== FIELD GEOMETRY (for bad boxes) ====================

  /** Field length derived from symmetric hub placement. ~16.54 m. */
  private static final double FIELD_LENGTH =
      GameConstants.RED_HUB_CENTER.getX() + GameConstants.BLUE_HUB_CENTER.getX();

  /** Field width derived from hub Y-center (hubs are at field midline). ~8.07 m. */
  private static final double FIELD_WIDTH = GameConstants.BLUE_HUB_CENTER.getY() * 2.0;

  /** Hub half-width in meters (47 inches / 2). Used for bad box Y-span. */
  private static final double HUB_HALF_WIDTH_M = 47.0 * 0.0254 / 2.0; // ~0.597 m

  // ==================== BAD BOXES (exclusion zones) ====================
  // All defined from BLUE alliance perspective. Flipped for red via flipBounds().
  // Adapted from Mechanical Advantage (6328)

  /**
   * Under-tower zone near the (blue) alliance wall. Robot cannot reliably shoot from underneath the
   * tower structure.
   *
   * <p>MA values: Bounds(0, 46", 129", 168") = (0, 1.17m, 3.28m, 4.27m)
   */
  private static final Bounds TOWER_BOUND = new Bounds(0.0, 1.17, 3.28, 4.27);

  /**
   * Behind our (blue) hub, toward field center. The hub structure physically blocks shot
   * trajectories from this region. Runs from the neutral zone line to field midline, within the
   * hub's Y-span.
   *
   * <p>minX = field_center - 120" (~5.22 m), maxX = field_center (~8.27 m)
   */
  private static final Bounds NEAR_HUB_BOUND = new Bounds(
      FIELD_LENGTH / 2.0 - 3.048, // neutral zone near line
      FIELD_LENGTH / 2.0, // field center
      GameConstants.BLUE_HUB_CENTER.getY() - HUB_HALF_WIDTH_M,
      GameConstants.BLUE_HUB_CENTER.getY() + HUB_HALF_WIDTH_M);

  /**
   * Behind the opponent's (red) hub, toward the red alliance wall.
   *
   * <p>minX ~ red hub near face, maxX = field length.
   */
  private static final Bounds FAR_HUB_BOUND = new Bounds(
      GameConstants.RED_HUB_CENTER.getX() - HUB_HALF_WIDTH_M,
      FIELD_LENGTH,
      GameConstants.RED_HUB_CENTER.getY() - HUB_HALF_WIDTH_M,
      GameConstants.RED_HUB_CENTER.getY() + HUB_HALF_WIDTH_M);

  // ==================== FILTERS ====================

  private final LinearFilter pivotAngleFilter =
      LinearFilter.movingAverage((int) (PIVOT_ANGLE_FILTER_WINDOW_SECONDS / LOOP_PERIOD_SECONDS));

  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (DRIVE_ANGLE_FILTER_WINDOW_SECONDS / LOOP_PERIOD_SECONDS));

  // Low pass filters for velocities to prevent noise-induced oscillation loop.
  // Window of 10 samples (0.2s at 50Hz) for heavier smoothing to reduce jitter.
  private final LinearFilter vxFilter = LinearFilter.movingAverage(10);
  private final LinearFilter vyFilter = LinearFilter.movingAverage(10);
  private final LinearFilter omegaFilter = LinearFilter.movingAverage(10);

  // ==================== STATE ====================

  private double lastPivotAngleDegrees = Double.NaN;
  private Rotation2d lastDriveAngle = null;
  private LaunchParameters latestParameters = null;

  // ==================== DATA RECORD ====================

  /**
   * Complete set of launch parameters computed each cycle.
   *
   * @param isValid whether all conditions are met to shoot from this location/distance
   * @param driveAngle the field-relative heading the robot should face (Rotation2d)
   * @param driveVelocityRadPerSec angular velocity feedforward for heading tracking (rad/s)
   * @param pivotAngleDegrees target pivotangle for the shooter (degrees)
   * @param pivotVelocityDegPerSec angular velocity feedforward for pivot tracking (deg/s)
   * @param flywheelRPM target flywheel RPM from interpolation table
   * @param lookaheadDistance the velocity-compensated effective distance (meters)
   * @param rawDistance the static distance to hub with no velocity compensation (meters)
   * @param timeOfFlight estimated time for the ball to reach the hub (seconds)
   */
  public record LaunchParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocityRadPerSec,
      double pivotAngleDegrees,
      double pivotVelocityDegPerSec,
      double flywheelRPM,
      double lookaheadDistance,
      double rawDistance,
      double timeOfFlight,
      boolean passing) {}

  // ==================== CORE UPDATE ====================

  /**
   * Compute all launch parameters for the current robot state.
   *
   * <p>Call this ONCE per robot loop (typically from robotPeriodic or the drive command). The
   * result is cached and returned by {@link #getParameters()} until {@link #clearParameters()} is
   * called.
   *
   * @param robotPose current field-relative robot pose from drivetrain odometry
   * @param robotRelativeVelocity current robot-relative ChassisSpeeds from drivetrain
   * @param robotHeading current robot heading (for field-relative velocity conversion)
   */
  public void update(
      Pose2d robotPose, ChassisSpeeds rawRobotRelativeVelocity, Rotation2d robotHeading) {
    if (latestParameters != null) {
      // Already computed this cycle - skip
      return;
    }

    // new code: Filter the raw drivetrain velocity to prevent a massive noisy
    // positive feedback loop
    // Noisy odometry causes lookahead to shake, which causes target angle to shake,
    // which yanks modules, which adds more noise!
    double smoothedVx = vxFilter.calculate(rawRobotRelativeVelocity.vxMetersPerSecond);
    double smoothedVy = vyFilter.calculate(rawRobotRelativeVelocity.vyMetersPerSecond);
    double smoothedOmega = omegaFilter.calculate(rawRobotRelativeVelocity.omegaRadiansPerSecond);
    ChassisSpeeds robotRelativeVelocity = new ChassisSpeeds(smoothedVx, smoothedVy, smoothedOmega);

    // ---- SOTM filter debug logging ----
    Logger.recordOutput("LaunchCalc/RawVx", rawRobotRelativeVelocity.vxMetersPerSecond);
    Logger.recordOutput("LaunchCalc/RawVy", rawRobotRelativeVelocity.vyMetersPerSecond);
    Logger.recordOutput("LaunchCalc/RawOmega", rawRobotRelativeVelocity.omegaRadiansPerSecond);
    Logger.recordOutput("LaunchCalc/SmoothedVx", smoothedVx);
    Logger.recordOutput("LaunchCalc/SmoothedVy", smoothedVy);
    Logger.recordOutput("LaunchCalc/SmoothedOmega", smoothedOmega);

    // ---- Step 1: Phase delay compensation ----
    // Project the robot pose forward by the phase delay to compensate for system
    // latency
    Pose2d estimatedPose = robotPose.exp(new Twist2d(
        robotRelativeVelocity.vxMetersPerSecond * PHASE_DELAY_SECONDS,
        robotRelativeVelocity.vyMetersPerSecond * PHASE_DELAY_SECONDS,
        robotRelativeVelocity.omegaRadiansPerSecond * PHASE_DELAY_SECONDS));

    // ---- Step 2: Determine if passing ----
    // Passing mode activates when the robot is on the far side of the field from
    // its own hub (past the hub center X line). Adapted from MA (6328).
    Translation2d hubPos = ShooterMath.getHubPosition();
    double hubCenterX = hubPos.getX();
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    // On blue, passing = robot X > hub X (robot past hub toward red side)
    // On red, passing = robot X < hub X (robot past hub toward blue side)
    boolean sideBasedPassing =
        isRed ? estimatedPose.getX() < hubCenterX : estimatedPose.getX() > hubCenterX;
    boolean passing = ENABLE_AUTO_PASSING && sideBasedPassing;
    boolean mirrorY = estimatedPose.getY() > FIELD_WIDTH / 2.0;
    Translation2d passingTarget = getPassingTarget(estimatedPose);

    // ---- Step 3: Determine target ----
    Translation2d target = passing ? passingTarget : hubPos;

    Logger.recordOutput("LaunchCalc/Poses/RobotPose", robotPose);
    Logger.recordOutput("LaunchCalc/Poses/EstimatedPose", estimatedPose);
    Logger.recordOutput("LaunchCalc/Poses/HubPose", new Pose2d(hubPos, new Rotation2d()));
    Logger.recordOutput(
        "LaunchCalc/Poses/PassTargetPose", new Pose2d(passingTarget, new Rotation2d()));
    Logger.recordOutput("LaunchCalc/GoalPose", new Pose2d(target, new Rotation2d()));

    Logger.recordOutput("LaunchCalc/Passing/AutoEnabled", ENABLE_AUTO_PASSING);
    Logger.recordOutput("LaunchCalc/Passing/SideBasedPassing", sideBasedPassing);
    Logger.recordOutput("LaunchCalc/Passing/AllianceIsRed", isRed);
    Logger.recordOutput("LaunchCalc/Passing/HubCenterX", hubCenterX);
    Logger.recordOutput("LaunchCalc/Passing/MirrorY", mirrorY);
    Logger.recordOutput("LaunchCalc/Passing/PassTargetBaseX", passTargetXMeters.get());
    Logger.recordOutput("LaunchCalc/Passing/PassTargetBaseY", passTargetYMeters.get());
    Logger.recordOutput("LaunchCalc/Passing/TargetType", passing ? "PASS" : "HUB");

    // Compute launcher position on the field (applying robot-frame offset)
    // Adapted from MA (6328): use launcher position, not robot center, as the
    // base for distance and lookahead calculations.
    Translation2d launcherOffset =
        new Translation2d(SHOOTER_OFFSET_X, SHOOTER_OFFSET_Y).rotateBy(estimatedPose.getRotation());
    Translation2d launcherFieldPos = estimatedPose.getTranslation().plus(launcherOffset);
    Logger.recordOutput(
        "LaunchCalc/Poses/LauncherPose", new Pose2d(launcherFieldPos, estimatedPose.getRotation()));
    double rawDistance = target.getDistance(launcherFieldPos);

    // ---- Step 3: Field-relative launcher velocity ----
    // In auto, use the raw (setpoint) velocity for more predictable lookahead.
    // In teleop, transform robot-center velocity to launcher frame to account for
    // the offset between robot center and launcher during rotation.
    ChassisSpeeds fieldVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, robotHeading);
    ChassisSpeeds launcherVelocity;
    if (DriverStation.isAutonomous()) {
      // In auto the measured velocity is the setpoint velocity (no driver input),
      // so skip the launcher-frame transform - it can amplify noise.
      launcherVelocity = fieldVelocity;
    } else {
      // Transform velocity from robot center to launcher position.
      // This accounts for rotational velocity adding a tangential component at the
      // launcher offset.
      Translation2d robotToLauncher = new Translation2d(SHOOTER_OFFSET_X, SHOOTER_OFFSET_Y);
      launcherVelocity = transformVelocityForLauncher(fieldVelocity, robotToLauncher, robotHeading);
    }
    double fieldVelX = launcherVelocity.vxMetersPerSecond;
    double fieldVelY = launcherVelocity.vyMetersPerSecond;

    // ---- Step 5: Iterative lookahead convergence loop ----
    // The ball leaves the launcher with the robot's velocity superimposed.
    // We iterate to find a self-consistent (distance, TOF) pair where:
    // - distance determines TOF (from interpolation table)
    // - TOF determines how far the robot's velocity offsets the effective aim point
    // - the new aim point changes the distance, which changes TOF, etc.
    double timeOfFlight = passing
        ? ShooterInterpolationTable.getPassingTimeOfFlight(rawDistance)
        : ShooterInterpolationTable.getTimeOfFlight(rawDistance);
    Translation2d lookaheadPos = launcherFieldPos;
    double lookaheadDistance = rawDistance;

    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      timeOfFlight = passing
          ? ShooterInterpolationTable.getPassingTimeOfFlight(lookaheadDistance)
          : ShooterInterpolationTable.getTimeOfFlight(lookaheadDistance);
      double offsetX = fieldVelX * timeOfFlight;
      double offsetY = fieldVelY * timeOfFlight;

      // Clamp the total offset magnitude to prevent runaway when TOF is imprecise
      double offsetMag = Math.hypot(offsetX, offsetY);
      if (offsetMag > MAX_LOOKAHEAD_OFFSET_METERS) {
        double scale = MAX_LOOKAHEAD_OFFSET_METERS / offsetMag;
        offsetX *= scale;
        offsetY *= scale;
      }

      lookaheadPos = launcherFieldPos.plus(new Translation2d(offsetX, offsetY));
      lookaheadDistance = target.getDistance(lookaheadPos);
    }
    Logger.recordOutput(
        "LaunchCalc/Poses/LookaheadPose", new Pose2d(lookaheadPos, estimatedPose.getRotation()));

    // ---- Step 5: Compute drive heading angle ----
    // The robot should face from the lookahead position toward the target
    Rotation2d driveAngle = target.minus(lookaheadPos).getAngle();
    // If the shooter has a significant lateral offset, apply asin correction
    // (similar to MA's getDriveAngleWithLauncherOffset)
    if (Math.abs(SHOOTER_OFFSET_Y) > 0.01) {

      double offsetAngleRad =
          Math.asin(MathUtil.clamp(SHOOTER_OFFSET_Y / lookaheadDistance, -1.0, 1.0));
      driveAngle = driveAngle.plus(Rotation2d.fromRadians(offsetAngleRad));
      // double distForOffset = target.getDistance(estimatedPose.getTranslation());
      // double offsetAngleRad = Math.asin(MathUtil.clamp(SHOOTER_OFFSET_Y /
      // distForOffset, -1.0,
      // 1.0));
      // driveAngle = driveAngle.plus(Rotation2d.fromRadians(offsetAngleRad));
      // Rotate 180 deg if the shooter fires backwards (like MA's launcher)
      // Our shooter fires forward, so no rotation needed
    }

    // ---- Step 7: Compute setpoints from lookahead distance ----
    double pivotAngleDegrees;
    double flywheelRPM;
    if (passing) {
      pivotAngleDegrees = ShooterInterpolationTable.getPassingAngle(Meters.of(lookaheadDistance))
          .in(Degrees);
      flywheelRPM =
          ShooterInterpolationTable.getPassingRPM(Meters.of(lookaheadDistance)).in(RPM);
    } else {
      ShooterSetpoint setpoint = ShooterSetpoint.fromDistance(Meters.of(lookaheadDistance));
      pivotAngleDegrees = setpoint.pivotAngle().in(Degrees);
      flywheelRPM = setpoint.flywheelRPM().in(RPM);
    }

    // ---- Step 7: Compute angular velocity feedforwards via derivative + filter
    // ----
    double pivotVelocityDegPerSec = 0.0;
    double driveVelocityRadPerSec = 0.0;

    if (!Double.isNaN(lastPivotAngleDegrees) && lastDriveAngle != null) {
      pivotVelocityDegPerSec = pivotAngleFilter.calculate(
          (pivotAngleDegrees - lastPivotAngleDegrees) / LOOP_PERIOD_SECONDS);
      driveVelocityRadPerSec = driveAngleFilter.calculate(
          driveAngle.minus(lastDriveAngle).getRadians() / LOOP_PERIOD_SECONDS);
    } else {
      // First cycle - reset filters
      pivotAngleFilter.reset();
      driveAngleFilter.reset();
      vxFilter.reset(); // new code
      vyFilter.reset(); // new code
      omegaFilter.reset(); // new code
    }

    lastPivotAngleDegrees = pivotAngleDegrees;
    lastDriveAngle = driveAngle;

    // ---- Step 9: Validity check ----
    boolean outsideOfBadBoxes = !isInsideBadBox(estimatedPose.getTranslation());
    double minDist = passing ? ShooterInterpolationTable.PASSING_MIN_DISTANCE : MIN_VALID_DISTANCE;
    double maxDist = passing ? ShooterInterpolationTable.PASSING_MAX_DISTANCE : MAX_VALID_DISTANCE;
    boolean distanceValid = lookaheadDistance >= minDist && lookaheadDistance <= maxDist;
    boolean setpointValid = true;
    boolean isValid = outsideOfBadBoxes && distanceValid;
    // For non-passing shots, also check that the setpoint itself is valid
    if (!passing) {
      ShooterSetpoint setpoint = ShooterSetpoint.fromDistance(Meters.of(lookaheadDistance));
      setpointValid = setpoint.isValid();
      isValid = isValid && setpointValid;
    }
    Logger.recordOutput("LaunchCalc/OutsideBadBoxes", outsideOfBadBoxes);
    Logger.recordOutput("LaunchCalc/Passing", passing);
    Logger.recordOutput("LaunchCalc/Validity/DistanceValid", distanceValid);
    Logger.recordOutput("LaunchCalc/Validity/SetpointValid", setpointValid);
    Logger.recordOutput("LaunchCalc/Validity/MinDistance", minDist);
    Logger.recordOutput("LaunchCalc/Validity/MaxDistance", maxDist);

    // ---- Step 10: Build the result ----
    latestParameters = new LaunchParameters(
        isValid,
        driveAngle,
        driveVelocityRadPerSec,
        pivotAngleDegrees,
        pivotVelocityDegPerSec,
        flywheelRPM,
        lookaheadDistance,
        rawDistance,
        timeOfFlight,
        passing);

    // ---- Step 10: Telemetry ----
    Logger.recordOutput("LaunchCalc/RawDistance", rawDistance);
    Logger.recordOutput("LaunchCalc/LookaheadDistance", lookaheadDistance);
    Logger.recordOutput("LaunchCalc/TimeOfFlight", timeOfFlight);
    Logger.recordOutput("LaunchCalc/DriveAngleDeg", driveAngle.getDegrees());
    Logger.recordOutput("LaunchCalc/DriveVelocityRadPerSec", driveVelocityRadPerSec);
    Logger.recordOutput("LaunchCalc/PivotAngleDeg", pivotAngleDegrees);
    Logger.recordOutput("LaunchCalc/PivotVelocityDegPerSec", pivotVelocityDegPerSec);
    Logger.recordOutput("LaunchCalc/FlywheelRPM", flywheelRPM);
    Logger.recordOutput("LaunchCalc/IsValid", isValid);
    Logger.recordOutput("LaunchCalc/FieldVelX", fieldVelX);
    Logger.recordOutput("LaunchCalc/FieldVelY", fieldVelY);
  }

  /**
   * Get the latest computed launch parameters.
   *
   * <p>Returns null if {@link #update} has not been called this cycle (i.e., if
   * {@link #clearParameters()} was called and update hasn't been called yet).
   *
   * <p>If null, callers should fall back to static/stop-and-shoot behavior.
   *
   * @return the latest LaunchParameters, or null
   */
  public LaunchParameters getParameters() {
    return latestParameters;
  }

  /**
   * Clear the cached parameters. Call this at the START of each robot periodic loop so that
   * parameters are recomputed fresh each cycle.
   */
  public void clearParameters() {
    latestParameters = null;
  }

  /**
   * Get the raw (non-velocity-compensated) time of flight for a given distance. Useful for velocity
   * limiting calculations.
   *
   * @param distanceMeters distance to hub in meters
   * @return estimated time of flight in seconds
   */
  public double getNaiveTOF(double distanceMeters) {
    return ShooterInterpolationTable.getTimeOfFlight(distanceMeters);
  }

  /**
   * Create a ShooterSetpoint supplier backed by the LaunchCalculator. This returns setpoints
   * computed from the velocity-compensated lookahead distance.
   *
   * <p>When the calculator has no valid parameters (e.g., first cycle), falls back to the static
   * distance-based setpoint from the provided pose supplier.
   *
   * @param poseFallbackSupplier supplier for robot pose (used as fallback when LaunchCalc has no
   *     data)
   * @return a supplier producing velocity-compensated ShooterSetpoints
   */
  public Supplier<ShooterSetpoint> createLaunchSetpointSupplier(
      Supplier<Pose2d> poseFallbackSupplier) {
    return () -> {
      LaunchParameters params = getParameters();
      if (params != null) {
        return new ShooterSetpoint(
            RPM.of(params.flywheelRPM()), Degrees.of(params.pivotAngleDegrees()), params.isValid());
      }
      // Fallback: static distance-based setpoint
      Distance distance = ShooterMath.getDistanceToHub(poseFallbackSupplier.get());
      return ShooterSetpoint.fromDistance(distance);
    };
  }

  /**
   * Reset all internal state (filters, last angles). Call when the robot is disabled or the
   * shooting system is not active, to prevent stale filter data.
   */
  public void reset() {
    pivotAngleFilter.reset();
    driveAngleFilter.reset();
    vxFilter.reset(); // new code
    vyFilter.reset(); // new code
    omegaFilter.reset(); // new code
    lastPivotAngleDegrees = Double.NaN;
    lastDriveAngle = null;
    latestParameters = null;
  }

  // ==================== BAD BOX HELPERS ====================

  /**
   * Check whether the robot is inside any exclusion zone where shooting is unsafe.
   *
   * <p>Bad boxes are defined from the BLUE alliance perspective and flipped for red.
   *
   * @param robotPosition field-relative robot position
   * @return true if the robot is inside a bad box (should NOT shoot)
   */
  private boolean isInsideBadBox(Translation2d robotPosition) {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    Bounds tower = isRed ? flipBounds(TOWER_BOUND) : TOWER_BOUND;
    Bounds nearHub = isRed ? flipBounds(NEAR_HUB_BOUND) : NEAR_HUB_BOUND;
    Bounds farHub = isRed ? flipBounds(FAR_HUB_BOUND) : FAR_HUB_BOUND;
    return tower.contains(robotPosition)
        || nearHub.contains(robotPosition)
        || farHub.contains(robotPosition);
  }

  /**
   * Flip a Bounds from blue alliance coordinates to red alliance coordinates.
   *
   * <p>Both X and Y are mirrored (field is rotationally symmetric). Min/max swap because the axis
   * direction reverses, following the same pattern as MA's AllianceFlipUtil.
   */
  private static Bounds flipBounds(Bounds bounds) {
    return new Bounds(
        FIELD_LENGTH - bounds.maxX(),
        FIELD_LENGTH - bounds.minX(),
        FIELD_WIDTH - bounds.maxY(),
        FIELD_WIDTH - bounds.minY());
  }

  // ==================== PASSING TARGET ====================

  /**
   * Compute the passing target position on the field.
   *
   * <p>The target is near the alliance wall, mirrored based on which side of the field (Y axis) the
   * robot is on, so the pass goes to the near side. For blue alliance, the target is at
   * (PASS_TARGET_X, PASS_TARGET_Y or mirrored). For red alliance, it is flipped to the opposite end
   * of the field.
   *
   * @param robotPose current robot pose (for Y-side detection)
   * @return field-relative Translation2d of the passing target
   */
  private Translation2d getPassingTarget(Pose2d robotPose) {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    double passTargetX = MathUtil.clamp(passTargetXMeters.get(), 0.0, FIELD_LENGTH);
    double passTargetY = MathUtil.clamp(passTargetYMeters.get(), 0.0, FIELD_WIDTH);
    // Mirror Y if robot is on the far side of the field (past center Y)
    boolean mirrorY = robotPose.getY() > FIELD_WIDTH / 2.0;
    double targetY = mirrorY ? FIELD_WIDTH - passTargetY : passTargetY;
    double targetX = isRed ? FIELD_LENGTH - passTargetX : passTargetX;
    return new Translation2d(targetX, targetY);
  }

  // ==================== BOUNDS RECORD ====================

  /**
   * Axis-aligned bounding box for field exclusion zones.
   *
   * <p>Adapted from Mechanical Advantage (6328).
   */
  public record Bounds(double minX, double maxX, double minY, double maxY) {
    /** Whether the translation is contained within these bounds. */
    public boolean contains(Translation2d translation) {
      return translation.getX() >= minX()
          && translation.getX() <= maxX()
          && translation.getY() >= minY()
          && translation.getY() <= maxY();
    }
  }

  // ==================== VELOCITY TRANSFORM ====================

  /**
   * Transform field-relative velocity from robot center to a point offset in the robot frame.
   *
   * <p>Adapted from MA (6328) GeomUtil.transformVelocity. When the robot rotates, a point offset
   * from the center of rotation has additional tangential velocity. This method adds that component
   * so the lookahead uses the launcher's actual velocity, not the robot center's.
   *
   * @param velocity field-relative ChassisSpeeds at robot center
   * @param robotFrameOffset offset from robot center to the target point (robot frame)
   * @param currentRotation current robot heading (for frame conversion)
   * @return ChassisSpeeds representing the velocity at the offset point in field frame
   */
  private static ChassisSpeeds transformVelocityForLauncher(
      ChassisSpeeds velocity, Translation2d robotFrameOffset, Rotation2d currentRotation) {
    return new ChassisSpeeds(
        velocity.vxMetersPerSecond
            - velocity.omegaRadiansPerSecond
                * (robotFrameOffset.getX() * currentRotation.getSin()
                    + robotFrameOffset.getY() * currentRotation.getCos()),
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (robotFrameOffset.getX() * currentRotation.getCos()
                    - robotFrameOffset.getY() * currentRotation.getSin()),
        velocity.omegaRadiansPerSecond);
  }
}
