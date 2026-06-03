// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.lib.LaunchCalculator;
import frc.robot.lib.LaunchCalculator.LaunchParameters;
import frc.robot.lib.ShooterMath;
import frc.robot.subsystems.drive.SwerveHeadingController.HeadingControllerState;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  // Skew compensation and deadband sourced from Constants
  private static final double SKEW_COMPENSATION_SCALAR =
      Constants.DrivetrainConstants.SKEW_COMPENSATION_SCALAR;
  private static final double CONTROLLER_DEADBAND = Constants.DrivetrainConstants.DEADBAND_PERCENT;

  /*
   * Velocity coefficients for dynamic speed control These allow runtime
   * adjustment of speeds (e.g., slow mode, scoring mode) Range: 0.0 (stopped) to
   * 1.0 (full speed)
   */
  private double teleopVelocityCoefficient = 1.0;
  private double rotationVelocityCoefficient = 1.0;

  // ==================== HEADING CONTROLLER ====================
  private final SwerveHeadingController m_headingController = new SwerveHeadingController();
  private boolean m_headingLockEnabled = false;
  private double m_headingLockTarget = 0.0;

  // ==================== SOTM DEBUG LOGGING ====================
  private boolean lastHeadingOk = false;

  // ==================== ODOMETRY TILT CORRECTION ====================
  // Adapted from MA (6328): scale odometry twist by floor-projection factor
  // when the robot is tilted (e.g. driving over field elements / defense).
  // At 0 deg tilt, trust = 1.0; at 25 deg tilt, trust = 0.0.
  private static final double MAX_TILT_DEG = 25.0;
  private Pose2d m_lastRawPose = new Pose2d();
  private Pose2d m_tiltCorrectedPose = new Pose2d();

  // ==================== SOTM TUNABLE PARAMETERS ====================
  // All SOTM gains and thresholds are live-tunable via NetworkTables under
  // /Tuning/.
  // Adapted from MA (6328) LoggedTunableNumber pattern.
  private static final LoggedTunableNumber sotmLaunchKp =
      new LoggedTunableNumber("SOTM/LaunchKp", Constants.HeadingControllerConstants.LAUNCH_KP);
  private static final LoggedTunableNumber sotmLaunchKd =
      new LoggedTunableNumber("SOTM/LaunchKd", Constants.HeadingControllerConstants.LAUNCH_KD);
  private static final LoggedTunableNumber sotmYawToleranceDeg = new LoggedTunableNumber(
      "SOTM/YawToleranceDeg", Constants.ShooterConstants.LAUNCH_HEADING_TOLERANCE_DEGREES);
  private static final LoggedTunableNumber sotmPitchToleranceDeg = new LoggedTunableNumber(
      "SOTM/PitchToleranceDeg", Constants.ShooterConstants.LAUNCH_PITCH_TOLERANCE_DEGREES);
  private static final LoggedTunableNumber sotmRollToleranceDeg = new LoggedTunableNumber(
      "SOTM/RollToleranceDeg", Constants.ShooterConstants.LAUNCH_ROLL_TOLERANCE_DEGREES);
  private static final LoggedTunableNumber sotmMaxPolarVelocity = new LoggedTunableNumber(
      "SOTM/MaxPolarVelocityRadPerSec",
      Constants.DrivetrainConstants.MAX_POLAR_VELOCITY_RAD_PER_SEC);
  private static final LoggedTunableNumber sotmAdaptivePolarFullErrorDeg =
      new LoggedTunableNumber("SOTM/AdaptivePolarFullErrorDeg", 30.0);
  private static final LoggedTunableNumber sotmAdaptivePolarMaxScale =
      new LoggedTunableNumber("SOTM/AdaptivePolarMaxScale", 1.8);
  private static final LoggedTunableNumber sotmMaxShootingSpeedMps = new LoggedTunableNumber(
      "SOTM/MaxShootingSpeedMps", Constants.DrivetrainConstants.MAX_SHOOTING_SPEED_MPS);
  private static final LoggedTunableNumber sotmMaxShootingAngularRate = new LoggedTunableNumber(
      "SOTM/MaxShootingAngularRateRadPerSec",
      Constants.DrivetrainConstants.MAX_SHOOTING_ANGULAR_RATE_RAD_PER_SEC);
  private static final LoggedTunableNumber sotmCorMinErrorDeg = new LoggedTunableNumber(
      "SOTM/CORMinErrorDeg", Constants.DrivetrainConstants.COR_MIN_ERROR_DEG);
  private static final LoggedTunableNumber sotmCorMaxErrorDeg = new LoggedTunableNumber(
      "SOTM/CORMaxErrorDeg", Constants.DrivetrainConstants.COR_MAX_ERROR_DEG);
  private static final LoggedTunableNumber sotmOlockLinearThreshold = new LoggedTunableNumber(
      "SOTM/OLockLinearThresholdMps", Constants.DrivetrainConstants.OLOCK_LINEAR_THRESHOLD_MPS);
  private static final LoggedTunableNumber sotmOlockOmegaThreshold = new LoggedTunableNumber(
      "SOTM/OLockOmegaThresholdRadPerSec",
      Constants.DrivetrainConstants.OLOCK_OMEGA_THRESHOLD_RAD_PER_SEC);
  private static final LoggedTunableNumber sotmPassingYawToleranceDeg =
      new LoggedTunableNumber("SOTM/PassingYawToleranceDeg", 15.0);

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  // ==================== CHOREO TRAJECTORY FOLLOWER ====================
  private final PIDController m_choreoXController = new PIDController(
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KP,
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KI,
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KD);
  private final PIDController m_choreoYController = new PIDController(
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KP,
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KI,
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KD);
  private final PIDController m_choreoHeadingController = new PIDController(
      Constants.DrivetrainConstants.CHOREO_HEADING_KP,
      Constants.DrivetrainConstants.CHOREO_HEADING_KI,
      Constants.DrivetrainConstants.CHOREO_HEADING_KD);

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default
          // ramp rate
          // (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /**
   * Resets the robot's field-relative heading to the alliance-correct forward direction while
   * preserving the current XY position from the pose estimator.
   *
   * <p>This is the correct "reset heading" for drivers: it tells the pose estimator "I am facing
   * the field" (0 deg for Blue alliance, 180 deg for Red alliance) without destroying vision
   * localization. MegaTag2 immediately adapts because it reads the fused heading from
   * getState().Pose each frame via SetRobotOrientation().
   *
   * <p>Inspired by Team 6328 (Mechanical Advantage) approach: adjust gyro offset to map old
   * rotation to new rotation, preserving XY.
   */
  public void resetFieldHeading() {
    Pose2d currentPose = getState().Pose;

    // Determine the correct "forward" heading based on alliance
    // Blue alliance: 0 deg means facing the red alliance wall
    // Red alliance: 180 deg means facing the blue alliance wall
    Rotation2d allianceForwardHeading =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Rotation2d.k180deg
            : Rotation2d.kZero;

    // Build a new pose that keeps the current XY but sets the heading
    // to the alliance-correct forward direction
    Pose2d correctedPose = new Pose2d(currentPose.getTranslation(), allianceForwardHeading);

    // resetPose() recalibrates the Pigeon2 fused heading offset so the
    // pose estimator's heading matches our desired heading. XY is preserved.
    resetPose(correctedPose);

    Logger.recordOutput(
        "Events/Drive/HeadingReset",
        "[Heading] Reset field heading: "
            + currentPose.getRotation().getDegrees() + "° -> "
            + allianceForwardHeading.getDegrees() + "° (XY preserved: "
            + String.format("%.2f, %.2f", currentPose.getX(), currentPose.getY()) + ")");
    Logger.recordOutput("Vision/HeadingResetOldDeg", currentPose.getRotation().getDegrees());
    Logger.recordOutput("Vision/HeadingResetNewDeg", allianceForwardHeading.getDegrees());
  }

  /*
   * SysId routine for characterizing rotation. This is used to find PID gains for
   * the FieldCentricFacingAngle HeadingController. See the documentation of
   * SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> {
            /* output is actually radians per second, but SysId only supports "volts" */
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) -> setControl(m_pathApplyRobotSpeeds
              .withSpeeds(speeds)
              .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
              .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(
                  Constants.DrivetrainConstants.PP_TRANSLATION_KP,
                  Constants.DrivetrainConstants.PP_TRANSLATION_KI,
                  Constants.DrivetrainConstants.PP_TRANSLATION_KD),
              // PID constants for rotation
              new PIDConstants(
                  Constants.DrivetrainConstants.PP_ROTATION_KP,
                  Constants.DrivetrainConstants.PP_ROTATION_KI,
                  Constants.DrivetrainConstants.PP_ROTATION_KD)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the
          // case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for
          // requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Follow a Choreo swerve sample. This method is passed into Choreo's AutoFactory.
   *
   * @param sample Choreo sample at the current autonomous timestamp
   */
  public void followTrajectory(SwerveSample sample) {
    Pose2d currentPose = getState().Pose;

    ChassisSpeeds targetFieldSpeeds = new ChassisSpeeds(
        sample.vx + m_choreoXController.calculate(currentPose.getX(), sample.x),
        sample.vy + m_choreoYController.calculate(currentPose.getY(), sample.y),
        sample.omega
            + m_choreoHeadingController.calculate(
                currentPose.getRotation().getRadians(), sample.heading));

    setControl(m_fieldCentricRequest.withSpeeds(targetFieldSpeeds));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by
   * {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by
   * {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective. If we haven't applied the
     * operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match. Otherwise, only check and apply the operator perspective if the DS
     * is disabled. This ensures driving behavior doesn't change until an explicit
     * disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(allianceColor -> {
        setOperatorPerspectiveForward(
            allianceColor == Alliance.Red
                ? kRedAlliancePerspectiveRotation
                : kBlueAlliancePerspectiveRotation);
        m_hasAppliedOperatorPerspective = true;
      });
    }

    var state = getState();
    SwerveModuleState[] measuredStates = copyModuleStates(state.ModuleStates);

    Logger.recordOutput("Odometry/Robot", state.Pose);
    Logger.recordOutput("SwerveStates/Measured", measuredStates);
    Logger.recordOutput("SwerveChassisSpeeds/Measured", state.Speeds);

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    } else {
      SwerveModuleState[] targetStates = copyModuleStates(state.ModuleTargets);
      Logger.recordOutput("SwerveStates/Setpoints", targetStates);
      // No separate optimizer pipeline exists here yet, so optimized == commanded
      // targets.
      Logger.recordOutput("SwerveStates/SetpointsOptimized", targetStates);
    }

    // Keep legacy key for existing layouts and tools.
    Logger.recordOutput("Drive Pos", state.Pose);

    // ---- Odometry tilt correction (adapted from MA 6328) ----
    // Compute tilt from Pigeon2 pitch/roll, scale odometry delta 100%→0% over
    // 0 deg→25 deg.
    Pose2d rawPose = getState().Pose;
    double pitchRad = Math.toRadians(getPigeon2().getPitch().getValueAsDouble());
    double rollRad = Math.toRadians(getPigeon2().getRoll().getValueAsDouble());
    double tiltDeg = Math.abs(Math.toDegrees(Math.acos(Math.cos(pitchRad) * Math.cos(rollRad))));
    double tiltScalar =
        MathUtil.clamp(1.0 - MathUtil.inverseInterpolate(0, MAX_TILT_DEG, tiltDeg), 0.0, 1.0);

    // Compute the raw delta since last cycle and scale it by tilt trust
    var rawTwist = m_lastRawPose.log(rawPose);
    var correctedTwist = new edu.wpi.first.math.geometry.Twist2d(
        rawTwist.dx * tiltScalar, rawTwist.dy * tiltScalar, rawTwist.dtheta * tiltScalar);
    m_tiltCorrectedPose = m_tiltCorrectedPose.exp(correctedTwist);
    m_lastRawPose = rawPose;

    Logger.recordOutput("Drive/TiltDeg", tiltDeg);
    Logger.recordOutput("Drive/TiltScalar", tiltScalar);
    Logger.recordOutput("Drive/TiltCorrectedPose", m_tiltCorrectedPose);
  }

  private static SwerveModuleState[] copyModuleStates(SwerveModuleState[] source) {
    SwerveModuleState[] copy = new SwerveModuleState[source.length];
    for (int i = 0; i < source.length; i++) {
      copy[i] = new SwerveModuleState(source[i].speedMetersPerSecond, source[i].angle);
    }
    return copy;
  }

  /**
   * Returns the tilt-corrected pose estimate.
   *
   * <p>Unlike {@code getState().Pose}, this pose has odometry deltas scaled down when the robot is
   * tilted (pitch/roll). At 0° tilt the poses are identical; at 25°+ tilt, odometry deltas are
   * zeroed to prevent drift from wheel slip on ramps/defense.
   *
   * @return tilt-corrected Pose2d
   */
  public Pose2d getTiltCorrectedPose() {
    return m_tiltCorrectedPose;
  }

  {
    m_choreoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ==================== VISION MEASUREMENT OVERRIDES ====================
  // These overrides convert FPGA timestamps to current time for the Kalman filter
  // Following the official CTRE Phoenix6 2026 examples pattern

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to
   * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  /**
   * Return the pose at a given timestamp, if the buffer is not empty.
   *
   * @param timestampSeconds The timestamp of the pose in seconds.
   * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
   */
  @Override
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  // ==================== SMOOTH DRIVING METHODS ====================

  /**
   * Creates a SwerveRequest for smooth teleop field-centric driving.
   *
   * <p>Uses OpenLoopVoltage for more responsive driving feel during teleop (as opposed to Velocity
   * mode which can feel sluggish).
   */
  private final SwerveRequest.ApplyFieldSpeeds m_fieldCentricRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);

  /**
   * Calculate chassis speeds with skew compensation for smooth driving.
   *
   * <p>Skew compensation corrects for the drift that occurs when both translating and rotating
   * simultaneously. The robot's actual heading is predicted slightly ahead based on current
   * rotational velocity.
   *
   * @param xVelocity Field-relative X velocity (m/s, positive = toward opposing alliance)
   * @param yVelocity Field-relative Y velocity (m/s, positive = left)
   * @param angularVelocity Angular velocity (rad/s, positive = counter-clockwise)
   * @return ChassisSpeeds with skew compensation applied
   */
  public ChassisSpeeds calculateSpeedsWithSkewCompensation(
      double xVelocity, double yVelocity, double angularVelocity) {

    var currentPose = getState().Pose;
    var currentSpeeds = getState().Speeds;

    // Calculate skew compensation factor based on current angular velocity
    Rotation2d skewCompensationFactor =
        Rotation2d.fromRadians(currentSpeeds.omegaRadiansPerSecond * SKEW_COMPENSATION_SCALAR);

    // Convert field-relative speeds to robot-relative, then back to field-relative
    // with the skew compensation applied
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xVelocity, yVelocity, angularVelocity), currentPose.getRotation()),
        currentPose.getRotation().plus(skewCompensationFactor));
  }

  /**
   * Transform field-relative velocity to account for a shifted center of rotation.
   *
   * <p>Adapted from Mechanical Advantage (6328) GeomUtil.transformVelocity. When the swerve modules
   * pivot around a point offset from the robot center, the linear velocity at the robot center
   * changes due to the rotational component. This method computes the adjusted velocity.
   *
   * @param velocity original field-relative ChassisSpeeds
   * @param transform robot-frame translation from robot center to the desired pivot point
   * @param currentRotation current robot heading (for frame conversion)
   * @return ChassisSpeeds with linear velocity adjusted for the offset pivot point
   */
  private static ChassisSpeeds transformVelocityForCOR(
      ChassisSpeeds velocity, Translation2d transform, Rotation2d currentRotation) {
    return new ChassisSpeeds(
        velocity.vxMetersPerSecond
            - velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getSin()
                    + transform.getY() * currentRotation.getCos()),
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getCos()
                    - transform.getY() * currentRotation.getSin()),
        velocity.omegaRadiansPerSecond);
  }

  /**
   * Apply field-centric driving with smooth driving tuning.
   *
   * <p>Features: - Deadband application to eliminate joystick drift - Squared angular input for
   * finer low-speed rotation control - Skew compensation for smooth combined translation/rotation -
   * OpenLoopVoltage mode for responsive feel - Runtime velocity coefficients for slow mode /
   * scoring mode
   *
   * @param xInput Raw X joystick input (-1 to 1)
   * @param yInput Raw Y joystick input (-1 to 1)
   * @param rotationInput Raw rotation joystick input (-1 to 1)
   * @param maxVelocity Maximum translation velocity (m/s)
   * @param maxAngularVelocity Maximum angular velocity (rad/s)
   */
  public void driveFieldCentricSmooth(
      double xInput,
      double yInput,
      double rotationInput,
      double maxVelocity,
      double maxAngularVelocity) {

    // Apply deadband to eliminate joystick drift
    double xMagnitude = MathUtil.applyDeadband(xInput, CONTROLLER_DEADBAND);
    double yMagnitude = MathUtil.applyDeadband(yInput, CONTROLLER_DEADBAND);
    double angularMagnitude = MathUtil.applyDeadband(rotationInput, CONTROLLER_DEADBAND);

    // Square the angular magnitude for finer low-speed control
    // while maintaining direction (sign)
    angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

    // Convert joystick axes to field-relative velocities.
    // Joystick Y-axis is inverted (push forward = negative), X-axis is inverted
    // (push left = negative).
    // Alliance rotation is handled by setOperatorPerspectiveForward() in
    // periodic(), so no
    // per-alliance flip is needed here.
    double xVelocity = -xMagnitude * maxVelocity * teleopVelocityCoefficient;
    double yVelocity = -yMagnitude * maxVelocity * teleopVelocityCoefficient;
    double angularVelocity = -angularMagnitude * maxAngularVelocity * rotationVelocityCoefficient;

    // Apply skew compensation for smooth combined translation/rotation
    ChassisSpeeds compensatedSpeeds =
        calculateSpeedsWithSkewCompensation(xVelocity, yVelocity, angularVelocity);

    // Apply to drivetrain using OpenLoopVoltage for responsive feel
    setControl(m_fieldCentricRequest.withSpeeds(compensatedSpeeds));
  }

  /**
   * Set the translation velocity coefficient for teleop driving. Use this for slow mode, scoring
   * mode, etc.
   *
   * @param coefficient Speed multiplier (0.0 = stopped, 1.0 = full speed)
   */
  public void setTeleopVelocityCoefficient(double coefficient) {
    this.teleopVelocityCoefficient = MathUtil.clamp(coefficient, 0.0, 1.0);
  }

  /**
   * Set the rotation velocity coefficient for teleop driving.
   *
   * @param coefficient Rotation speed multiplier (0.0 = no rotation, 1.0 = full rotation)
   */
  public void setRotationVelocityCoefficient(double coefficient) {
    this.rotationVelocityCoefficient = MathUtil.clamp(coefficient, 0.0, 1.0);
  }

  /** Get the current translation velocity coefficient. */
  public double getTeleopVelocityCoefficient() {
    return teleopVelocityCoefficient;
  }

  /**
   * Creates a command for smooth teleop driving
   *
   * @param xInputSupplier Supplier for X joystick input (typically leftY, inverted)
   * @param yInputSupplier Supplier for Y joystick input (typically leftX, inverted)
   * @param rotationInputSupplier Supplier for rotation joystick input (typically rightX)
   * @param maxVelocity Maximum translation velocity (m/s)
   * @param maxAngularVelocity Maximum angular velocity (rad/s)
   * @return Command that continuously applies smooth driving
   */
  public Command smoothTeleopDriveCommand(
      Supplier<Double> xInputSupplier,
      Supplier<Double> yInputSupplier,
      Supplier<Double> rotationInputSupplier,
      double maxVelocity,
      double maxAngularVelocity) {

    return run(() -> driveFieldCentricSmooth(
        xInputSupplier.get(),
        yInputSupplier.get(),
        rotationInputSupplier.get(),
        maxVelocity,
        maxAngularVelocity));
  }

  // ==================== HEADING LOCK CONTROL ====================

  /** Get the heading controller for external configuration */
  public SwerveHeadingController getHeadingController() {
    return m_headingController;
  }

  /**
   * Enable heading lock mode with a specific target heading. In this mode, the robot will
   * automatically rotate to face the target heading while still allowing the driver to control
   * translation.
   *
   * @param targetDegrees The target heading in degrees (field-relative)
   */
  public void enableHeadingLock(double targetDegrees) {
    m_headingLockEnabled = true;
    m_headingLockTarget = targetDegrees;
    m_headingController.setGoal(targetDegrees);
    m_headingController.setHeadingControllerState(HeadingControllerState.SNAP);
  }

  /** Update the target heading without resetting the controller state */
  public void updateHeadingLockTarget(double targetDegrees) {
    m_headingLockTarget = targetDegrees;
    m_headingController.setGoal(targetDegrees);
  }

  /** Disable heading lock mode */
  public void disableHeadingLock() {
    m_headingLockEnabled = false;
    m_headingController.setHeadingControllerState(HeadingControllerState.OFF);
  }

  /**
   * Check if the robot is currently locked to the target heading within tolerance
   *
   * @return true if heading lock is enabled and robot is at target
   */
  public boolean isHeadingLocked() {
    return m_headingLockEnabled && m_headingController.isAtGoal();
  }

  /**
   * Get the current heading lock target
   *
   * @return Target heading in degrees
   */
  public double getHeadingLockTarget() {
    return m_headingLockTarget;
  }

  /**
   * Apply field-centric driving WITH heading lock.
   *
   * <p>The driver controls translation (X/Y), but rotation is automatically controlled by the
   * heading controller to maintain the locked heading. This creates a "turret mode" where the robot
   * faces a specific direction regardless of how the driver is strafing.
   *
   * @param xInput Raw X joystick input (-1 to 1)
   * @param yInput Raw Y joystick input (-1 to 1)
   * @param maxVelocity Maximum translation velocity (m/s)
   * @param maxAngularVelocity Maximum angular velocity (rad/s)
   */
  public void driveWithHeadingLock(
      double xInput, double yInput, double maxVelocity, double maxAngularVelocity) {

    // Apply deadband to translation inputs
    double xMagnitude = MathUtil.applyDeadband(xInput, CONTROLLER_DEADBAND);
    double yMagnitude = MathUtil.applyDeadband(yInput, CONTROLLER_DEADBAND);

    // Convert joystick to field-relative velocities.
    // Alliance rotation handled by setOperatorPerspectiveForward() — no
    // per-alliance flip.
    double xVelocity = -xMagnitude * maxVelocity * teleopVelocityCoefficient;
    double yVelocity = -yMagnitude * maxVelocity * teleopVelocityCoefficient;

    // Get current heading
    double currentHeadingDegrees = getState().Pose.getRotation().getDegrees();

    // Pass logging context (measured omega) before update
    double measuredOmega = getState().Speeds.omegaRadiansPerSecond;
    m_headingController.setLoggingContext(maxAngularVelocity, measuredOmega);

    // Calculate rotation output from heading controller (-1 to 1)
    double rotationOutput = m_headingController.update(currentHeadingDegrees);

    // Convert to angular velocity
    double angularVelocity = rotationOutput * maxAngularVelocity;

    // Apply skew compensation
    ChassisSpeeds compensatedSpeeds =
        calculateSpeedsWithSkewCompensation(xVelocity, yVelocity, angularVelocity);

    // Apply to drivetrain
    setControl(m_fieldCentricRequest.withSpeeds(compensatedSpeeds));

    // Log heading controller telemetry
    m_headingController.logTelemetry(currentHeadingDegrees);
  }

  /**
   * Creates a command for heading-locked driving.
   *
   * <p>The driver controls translation with the left stick, but the robot automatically rotates to
   * face the specified target heading.
   *
   * @param xInputSupplier Supplier for X joystick input
   * @param yInputSupplier Supplier for Y joystick input
   * @param targetHeadingSupplier Supplier for target heading in degrees
   * @param maxVelocity Maximum translation velocity (m/s)
   * @param maxAngularVelocity Maximum angular velocity (rad/s)
   * @return Command that applies heading-locked driving
   */
  public Command headingLockedDriveCommand(
      DoubleSupplier xInputSupplier,
      DoubleSupplier yInputSupplier,
      DoubleSupplier targetHeadingSupplier,
      double maxVelocity,
      double maxAngularVelocity) {

    return run(() -> {
          // Update target heading each loop (allows dynamic targeting like AprilTag
          // tracking)
          double targetHeading = targetHeadingSupplier.getAsDouble();
          if (!m_headingLockEnabled) {
            enableHeadingLock(targetHeading);
          } else {
            updateHeadingLockTarget(targetHeading);
          }

          driveWithHeadingLock(
              xInputSupplier.getAsDouble(),
              yInputSupplier.getAsDouble(),
              maxVelocity,
              maxAngularVelocity);
        })
        .finallyDo(this::disableHeadingLock);
  }

  /**
   * Creates a command for heading-locked driving to a FIXED heading.
   *
   * @param xInputSupplier Supplier for X joystick input
   * @param yInputSupplier Supplier for Y joystick input
   * @param fixedTargetHeading Fixed target heading in degrees
   * @param maxVelocity Maximum translation velocity (m/s)
   * @param maxAngularVelocity Maximum angular velocity (rad/s)
   * @return Command that applies heading-locked driving to the fixed heading
   */
  public Command headingLockedDriveCommand(
      DoubleSupplier xInputSupplier,
      DoubleSupplier yInputSupplier,
      double fixedTargetHeading,
      double maxVelocity,
      double maxAngularVelocity) {

    return headingLockedDriveCommand(
        xInputSupplier, yInputSupplier, () -> fixedTargetHeading, maxVelocity, maxAngularVelocity);
  }

  // ==================== SHOOT-ON-THE-MOVE DRIVING ====================

  /**
   * Creates a command for driving while shooting on the move.
   *
   * <p>This is the core shoot-on-the-move driving command, adapted from Mechanical Advantage
   * (6328). The driver retains full translational control while the robot's heading is
   * automatically locked to the launch angle computed by {@link LaunchCalculator}.
   *
   * <p>Heading control uses a PD + feedforward controller (NOT the SNAP/MAINTAIN PID): omega =
   * driveVelocityFF + kP * headingError + kD * (driveVelocityFF - measuredOmega)
   *
   * <p>Velocity limiting (law of sines) prevents the driver from moving so fast that the ball's
   * polar velocity at the hub exceeds a configurable threshold.
   *
   * @param xInputSupplier Supplier for X joystick input (forward/back)
   * @param yInputSupplier Supplier for Y joystick input (left/right)
   * @return Command that applies shoot-on-the-move driving
   */
  public Command shootOnTheMoveDriveCommand(
      DoubleSupplier xInputSupplier, DoubleSupplier yInputSupplier) {

    return runOnce(() -> {})
        .andThen(run(() -> {
          // LaunchCalculator is updated in robotPeriodic() before triggers evaluate
          LaunchCalculator calc = LaunchCalculator.getInstance();
          Pose2d currentPose = getState().Pose;
          ChassisSpeeds currentSpeeds = getState().Speeds;
          Rotation2d currentHeading = currentPose.getRotation();

          LaunchParameters params = calc.getParameters();

          // If no parameters available, fall back to normal heading lock
          if (params == null) {
            driveFieldCentricSmooth(
                xInputSupplier.getAsDouble(),
                yInputSupplier.getAsDouble(),
                0.0, // no rotation
                sotmMaxShootingSpeedMps.get(),
                sotmMaxShootingAngularRate.get());
            return;
          }

          // ---- PD + Feedforward heading controller ----
          // Output = FF + P * error + D * (FF - measured omega)
          double headingErrorRad = params.driveAngle().minus(currentHeading).getRadians();
          double measuredOmega = currentSpeeds.omegaRadiansPerSecond;

          double pTerm = sotmLaunchKp.get() * headingErrorRad;
          double dTerm = sotmLaunchKd.get() * (params.driveVelocityRadPerSec() - measuredOmega);

          // Guard against derivative overpowering in the opposite direction during
          // fast translation-direction changes. Keep this inactive near zero error.
          if (Math.abs(Math.toDegrees(headingErrorRad)) > 3.0
              && Math.signum(dTerm) != Math.signum(pTerm)
              && Math.abs(dTerm) > Math.abs(pTerm) * 0.75) {
            dTerm = Math.copySign(Math.abs(pTerm) * 0.75, dTerm);
          }

          double omegaOutput = params.driveVelocityRadPerSec() + pTerm + dTerm;
          omegaOutput = MathUtil.clamp(
              omegaOutput, -sotmMaxShootingAngularRate.get(), sotmMaxShootingAngularRate.get());

          // ---- Translation from joystick ----
          double xMagnitude = MathUtil.applyDeadband(
              xInputSupplier.getAsDouble(), Constants.DrivetrainConstants.DEADBAND_PERCENT);
          double yMagnitude = MathUtil.applyDeadband(
              yInputSupplier.getAsDouble(), Constants.DrivetrainConstants.DEADBAND_PERCENT);

          // Convert joystick to field-relative velocities
          double xVelocity =
              -xMagnitude * sotmMaxShootingSpeedMps.get() * teleopVelocityCoefficient;
          double yVelocity =
              -yMagnitude * sotmMaxShootingSpeedMps.get() * teleopVelocityCoefficient;

          // ---- Velocity limiting (law of sines) ----
          // Prevents the ball from sweeping past the hub too fast.
          // Computes the maximum linear speed that keeps the ball's angular velocity
          // at the hub below MAX_POLAR_VELOCITY_RAD_PER_SEC.
          // Skipped for passing shots (MA 6328): passing doesn't need precision aiming.
          Translation2d velocityVector = new Translation2d(xVelocity, yVelocity);
          double linearSpeed = velocityVector.getNorm();
          double headingErrorDegAbs = Math.abs(Math.toDegrees(headingErrorRad));
          double adaptivePolarScale = 1.0;
          double effectiveMaxPolarVelocity = sotmMaxPolarVelocity.get();
          double maxLinearSpeedFromLimiter = sotmMaxShootingSpeedMps.get();
          boolean velocityLimited = false;

          if (!params.passing() && linearSpeed > 0.01) { // avoid division by zero
            double errorNorm = MathUtil.clamp(
                headingErrorDegAbs / Math.max(1.0, sotmAdaptivePolarFullErrorDeg.get()), 0.0, 1.0);
            double adaptiveMaxScale = Math.max(1.0, sotmAdaptivePolarMaxScale.get());
            adaptivePolarScale = 1.0 + (adaptiveMaxScale - 1.0) * errorNorm;
            effectiveMaxPolarVelocity = sotmMaxPolarVelocity.get() * adaptivePolarScale;

            Translation2d hubPos = ShooterMath.getHubPosition();
            Rotation2d hubDirection = hubPos.minus(currentPose.getTranslation()).getAngle();
            Rotation2d velocityDirection = velocityVector.getAngle();

            double robotAngle = Math.abs(hubDirection.minus(velocityDirection).getRadians());
            double rawDistance = params.rawDistance();
            double naiveTOF = calc.getNaiveTOF(rawDistance);

            if (naiveTOF > 1e-3) {
              double hubAngle = effectiveMaxPolarVelocity * naiveTOF;
              double lookaheadAngle = Math.PI - robotAngle - hubAngle;

              if (lookaheadAngle > 0) {
                double robotLookaheadDist =
                    rawDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle);
                double maxLinearSpeed = robotLookaheadDist / naiveTOF;
                maxLinearSpeedFromLimiter = maxLinearSpeed;

                if (linearSpeed > maxLinearSpeed) {
                  double scale = maxLinearSpeed / linearSpeed;
                  xVelocity *= scale;
                  yVelocity *= scale;
                  velocityLimited = true;
                }
              }
            }
          }

          // ---- Center-of-rotation shifting ----
          // When heading error is large, shift the swerve pivot point toward the
          // shooter so the robot rotates around the launch point instead of its
          // geometric center. This improves aiming responsiveness.
          // Linear interpolation: no shift below COR_MIN, full shift at COR_MAX.
          double corScalar = MathUtil.clamp(
              (headingErrorDegAbs - sotmCorMinErrorDeg.get())
                  / (sotmCorMaxErrorDeg.get() - sotmCorMinErrorDeg.get()),
              0.0,
              1.0);

          // The shooter offset from robot center (robot frame).
          // When SHOOTER_OFFSET is (0,0), this has no effect - corScalar * (0,0) = (0,0).
          Translation2d shooterToRobot = new Translation2d(
              -LaunchCalculator.SHOOTER_OFFSET_X, -LaunchCalculator.SHOOTER_OFFSET_Y);

          // Transform field-relative velocity with the COR offset.
          // corScalar=0 -> pivot at robot center (normal); corScalar=1 -> pivot at
          // shooter.
          Translation2d corOffset = shooterToRobot.times(1.0 - corScalar);
          ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xVelocity, yVelocity, omegaOutput);
          ChassisSpeeds corAdjustedSpeeds =
              transformVelocityForCOR(fieldSpeeds, corOffset, currentHeading);

          // ---- O-Lock: brake when nearly stationary ----
          // Adapted from MA (6328). When the robot is barely moving, sending
          // near-zero velocities causes jitter. Instead, lock wheels in X-pattern.
          double corLinearSpeed =
              Math.hypot(corAdjustedSpeeds.vxMetersPerSecond, corAdjustedSpeeds.vyMetersPerSecond);
          boolean oLock = corLinearSpeed < sotmOlockLinearThreshold.get()
              && Math.abs(corAdjustedSpeeds.omegaRadiansPerSecond) < sotmOlockOmegaThreshold.get();

          if (oLock) {
            // X-stop: point all wheels inward to prevent drift
            setControl(new SwerveRequest.SwerveDriveBrake());
          } else {
            // ---- Apply skew compensation and drive ----
            ChassisSpeeds compensatedSpeeds = calculateSpeedsWithSkewCompensation(
                corAdjustedSpeeds.vxMetersPerSecond,
                corAdjustedSpeeds.vyMetersPerSecond,
                corAdjustedSpeeds.omegaRadiansPerSecond);

            setControl(m_fieldCentricRequest.withSpeeds(compensatedSpeeds));
          }

          // ---- Telemetry ----
          Logger.recordOutput("SOTM/HeadingErrorDeg", Math.toDegrees(headingErrorRad));
          Logger.recordOutput("SOTM/OmegaOutput", omegaOutput);
          Logger.recordOutput("SOTM/IsValid", params.isValid());
          Logger.recordOutput("SOTM/Passing", params.passing());
          Logger.recordOutput(
              "SOTM/YawToleranceDeg",
              params.passing() ? sotmPassingYawToleranceDeg.get() : sotmYawToleranceDeg.get());
          Logger.recordOutput("SOTM/AdaptivePolarScale", adaptivePolarScale);
          Logger.recordOutput("SOTM/EffectiveMaxPolarVelocityRadPerSec", effectiveMaxPolarVelocity);
          Logger.recordOutput("SOTM/MaxLinearSpeedLimiterMps", maxLinearSpeedFromLimiter);
          Logger.recordOutput("SOTM/VelocityLimited", velocityLimited);
          Logger.recordOutput("SOTM/CORScalar", corScalar);
          Logger.recordOutput("SOTM/OLock", oLock);

          // ---- Detailed SOTM telemetry (every cycle, logged to .wpilog) ----
          ChassisSpeeds fieldVel =
              ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentHeading);
          double robotSpeedMps = Math.hypot(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond);
          Logger.recordOutput("SOTM/HeadingTargetDeg", params.driveAngle().getDegrees());
          Logger.recordOutput("SOTM/HeadingActualDeg", currentHeading.getDegrees());
          Logger.recordOutput("SOTM/OmegaFF", params.driveVelocityRadPerSec());
          Logger.recordOutput("SOTM/OmegaMeasured", measuredOmega);
          Logger.recordOutput("SOTM/RobotSpeedMps", robotSpeedMps);
          Logger.recordOutput("SOTM/FieldVelX", fieldVel.vxMetersPerSecond);
          Logger.recordOutput("SOTM/FieldVelY", fieldVel.vyMetersPerSecond);
        }))
        .finallyDo(() -> {
          LaunchCalculator.getInstance().reset();
        })
        .withName("ShootOnTheMove Drive");
  }

  /**
   * Check if the robot heading is within launch tolerance of the target AND the robot is level
   * enough to shoot. Uses the wider launch-mode tolerance (not the static 3 deg tolerance).
   *
   * <p>Combines heading (yaw) check with pitch/roll tolerance to prevent shooting while the robot
   * is tilted (e.g., driving over field elements). Adapted from MA (6328) atLaunchGoal().
   *
   * @return true if heading is close enough AND robot is level enough to fire
   */
  public boolean isAtLaunchHeadingGoal() {
    LaunchParameters params = LaunchCalculator.getInstance().getParameters();
    if (params == null) return false;

    double errorDeg =
        Math.abs(params.driveAngle().minus(getState().Pose.getRotation()).getDegrees());
    double yawTolerance =
        params.passing() ? sotmPassingYawToleranceDeg.get() : sotmYawToleranceDeg.get();
    boolean headingOk = errorDeg <= yawTolerance;
    boolean levelOk = isLevelForLaunch();

    Logger.recordOutput("SOTM/HeadingOk", headingOk);
    Logger.recordOutput("SOTM/LevelOk", levelOk);
    Logger.recordOutput("SOTM/Passing", params.passing());
    Logger.recordOutput("SOTM/YawToleranceDeg", yawTolerance);
    Logger.recordOutput("SOTM/HeadingErrorDegGate", errorDeg);

    // ---- Heading state transition logging (event-based, not periodic) ----
    boolean currentHeadingOk = headingOk && levelOk;
    if (currentHeadingOk != lastHeadingOk) {
      ChassisSpeeds spd = getState().Speeds;
      double speed = Math.hypot(spd.vxMetersPerSecond, spd.vyMetersPerSecond);
      Logger.recordOutput("SOTM/HeadingTransition", currentHeadingOk ? "OK" : "LOST");
      Logger.recordOutput("SOTM/HeadingTransitionErrorDeg", errorDeg);
      Logger.recordOutput("SOTM/HeadingTransitionOmega", spd.omegaRadiansPerSecond);
      Logger.recordOutput("SOTM/HeadingTransitionSpeed", speed);
    }
    lastHeadingOk = currentHeadingOk;

    return headingOk && levelOk;
  }

  /**
   * Check if the robot is level enough to shoot on the move.
   *
   * <p>Reads pitch and roll from the Pigeon2 IMU. If either exceeds the tolerance (default 5deg,
   * matching MA), the robot is considered too tilted for an accurate shot.
   *
   * @return true if pitch and roll are within tolerance
   */
  public boolean isLevelForLaunch() {
    double pitchDeg = Math.abs(getPigeon2().getPitch().getValueAsDouble());
    double rollDeg = Math.abs(getPigeon2().getRoll().getValueAsDouble());
    // Normalize for inverted Pigeon2 mounting: roll reads ~180° when level
    if (rollDeg > 90) {
      rollDeg = 180 - rollDeg;
    }
    return pitchDeg <= sotmPitchToleranceDeg.get() && rollDeg <= sotmRollToleranceDeg.get();
  }

  // ==================== PATHFINDING COMMANDS ====================

  /**
   * Create a command to pathfind to AprilTag 10 (Red Alliance Hub Face).
   *
   * <p>Uses the AD* pathfinding algorithm to find a safe path around obstacles, then follows the
   * path using PID control.
   *
   * @return Command that pathfinds and drives to the scoring position in front of AprilTag 10
   */
  public Command pathfindToAprilTag10() {
    return frc.robot.pathfinding.PathfindToTagCommand.toAprilTag10(this);
  }

  /**
   * Create a command to pathfind to a specific AprilTag.
   *
   * @param tagId The AprilTag ID to target
   * @return Command that pathfinds and drives to the scoring position
   */
  public Command pathfindToAprilTag(int tagId) {
    return frc.robot.pathfinding.PathfindToTagCommand.toAprilTag(this, tagId);
  }

  /**
   * Create a command to pathfind to a specific pose.
   *
   * @param targetPose The target pose to pathfind to
   * @return Command that pathfinds and drives to the target
   */
  public Command pathfindToPose(edu.wpi.first.math.geometry.Pose2d targetPose) {
    return new frc.robot.pathfinding.PathfindToTagCommand(
        this, () -> targetPose, frc.robot.pathfinding.PathConstraints.DEFAULT);
  }

  /**
   * Create a command to pathfind to a dynamically-supplied pose.
   *
   * @param targetPoseSupplier Supplier for the target pose
   * @return Command that pathfinds and drives to the target
   */
  public Command pathfindToPose(Supplier<edu.wpi.first.math.geometry.Pose2d> targetPoseSupplier) {
    return new frc.robot.pathfinding.PathfindToTagCommand(
        this, targetPoseSupplier, frc.robot.pathfinding.PathConstraints.DEFAULT);
  }
}
