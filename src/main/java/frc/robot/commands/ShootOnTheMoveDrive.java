// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagMaps;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.lib.ShooterInterpolationTable;
import frc.robot.lib.ShooterMath;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.statemachine.DrivetrainMode;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Command to align the robot to an AprilTag using vision
 *
 * <p>This command: 1. Finds the closest AprilTag (or uses Limelight-detected tag) 2. Calculates
 * target position with optional offset (LEFT/RIGHT/CENTER) 3. Uses PID control to drive the robot
 * to the target pose 4. Rotates to face opposite the tag (facing the tag)
 *
 * <p>For REBUILT 2026 - Generic AprilTag alignment for any field element
 */
public class ShootOnTheMoveDrive extends Command {

  static Supplier<ShooterSetpoint> sotmSetpointSupplier;

  // Subsystems
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final RobotStateMachine stateMachine;

  // Timer for logging/timeout
  private final Timer timer = new Timer();

  // PID Controllers for position control
  private final PIDController pidX;
  private final PIDController pidY;
  private final PIDController pidRotate;

  // Input suppliers

  DoubleSupplier xInputSupplier;
  DoubleSupplier yInputSupplier;

  // Swerve drive request - field centric with velocity control
  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Stop request
  private final SwerveRequest stop;

  // Target pose to align to
  private Pose2d targetPose;

  // Configuration from Constants
  private final double speed;
  private final double positionTolerance;
  private final double yawTolerance;

  // Is simulated
  private boolean isSimulation = RobotBase.isSimulation();

  // Offset configuration
  private double offsetX = 0;
  private double offsetY = 0;

  // Target AprilTag info
  private int targetTagID;
  private boolean tagDetected = false;

  // Timeout duration (seconds) - prevents indefinite alignment attempts
  private static final Time ALIGN_TIMEOUT = Seconds.of(3.0);

  /**
   * Creates a new AlignToAprilTag command
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param vision The vision subsystem
   * @param alignPosition LEFT, RIGHT, or CENTER offset from the AprilTag
   */
  public ShootOnTheMoveDrive(
      CommandSwerveDrivetrain drivetrain,
      Vision vision,
      DoubleSupplier xInputSupplier,
      DoubleSupplier yInputSupplier) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.stateMachine = RobotStateMachine.getInstance();

    // Get constants
    this.speed = DrivetrainConstants.ALIGN_SPEED_MPS;
    this.positionTolerance = DrivetrainConstants.POSITION_TOLERANCE_METERS;
    this.yawTolerance = DrivetrainConstants.YAW_TOLERANCE_RADIANS;

    this.yInputSupplier = yInputSupplier;
    this.xInputSupplier = xInputSupplier;

    // Create the memoized \[]
    // setpoint supplier (caches by pose X/Y/theta)
    sotmSetpointSupplier = ShooterMath.createSetpointSupplier(() -> this.getCorrectedRobotPose());

    // Initialize PID controllers with values from Constants
    pidX = new PIDController(0, 0, 0);
    pidY = new PIDController(0, 0, 0);
    pidRotate = new PIDController(4, 0, 0); // search this

    // Enable continuous input for rotation (-PI to PI are same point)
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);

    // Create stop request
    stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

    // This command requires the drivetrain
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Start timer
    timer.restart();

    // Set state machine to vision tracking mode
    stateMachine.setDrivetrainMode(DrivetrainMode.VISION_TRACKING);
    stateMachine.setAlignedToTarget(false);

    // Get current robot pose
    Pose2d currentPose = drivetrain.getState().Pose;
    if (currentPose == null) {
      return;
    }

    // Convert tag position to Pose2d
    Pose2d aprilTagPose = calculateTagPose();

    // If no tag detected, don't execute
    if (!tagDetected) {
      return;
    }

    // Calculate velocities using PID
    double[] velocities = calculatePIDVelocities(currentPose, aprilTagPose);

    // Log data
    SmartDashboard.putNumber("AlignToAprilTag/CurrentX", currentPose.getX());
    SmartDashboard.putNumber("AlignToAprilTag/CurrentY", currentPose.getY());
    SmartDashboard.putNumber(
        "AlignToAprilTag/CurrentYaw", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorX", pidX.getError());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorY", pidY.getError());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorYaw", Math.toDegrees(pidRotate.getError()));

    // Get currentPose
    Pose2d robotPose = drivetrain.getState().Pose;
    if (robotPose == null) {
      return;
    }
  }

  @Override
  public void execute() {

    // Get current robot pose
    Pose2d currentPose = drivetrain.getState().Pose;
    if (currentPose == null) {
      return;
    }

    Pose2d correctedHubPose = getCorrectedHubPose();

    // Calculate velocities using PID
    double[] velocities = calculatePIDVelocities(currentPose, correctedHubPose);

    SmartDashboard.putNumber(
        "AlignToAprilTag/VelocityX", drivetrain.getState().Speeds.vyMetersPerSecond);

    SmartDashboard.putNumberArray("CorrectedPose", new double[] {
      correctedHubPose.getX(),
      correctedHubPose.getY(),
      correctedHubPose.getRotation().getDegrees()
    });

    // Apply velocities to drivetrain
    drivetrain.setControl(driveRequest
        .withVelocityX(MetersPerSecond.of(-xInputSupplier.getAsDouble() * 1.0))
        .withVelocityY(MetersPerSecond.of(-yInputSupplier.getAsDouble() * 1.0))
        .withRotationalRate(velocities[2]));
  }

  private Pose2d calculateTagPose() {

    // Assume there is a tag detected until proven otherwise
    tagDetected = true;

    // Get current robot pose
    Pose2d robotPose = drivetrain.getState().Pose;
    if (robotPose == null) {
      // DataLogManager.log("[AlignToAprilTag] ERROR: Robot pose is null!");
      tagDetected = false;
      return new Pose2d();
    }
    // Determine which tags to search based on alliance
    // Prevents aligning to opponent's tags across the field
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    int[] allianceTags = isRed ? AprilTagMaps.RED_SIDE_TAGS : AprilTagMaps.BLUE_SIDE_TAGS;

    // Find the closest AprilTag within our alliance's tags
    double minDistance = Double.MAX_VALUE;
    targetTagID = -1;

    for (int id : allianceTags) {
      double[] tagData = AprilTagMaps.aprilTagMap.get(id);
      if (tagData == null) continue;
      Pose2d tagPose = new Pose2d(
          tagData[0] * Constants.INCHES_TO_METERS,
          tagData[1] * Constants.INCHES_TO_METERS,
          new Rotation2d(Math.toRadians(tagData[3])));

      double distance = calculateDistance(robotPose, tagPose);
      if (distance < minDistance) {
        minDistance = distance;
        targetTagID = id;
      }
    }

    // Log to SmartDashboard
    SmartDashboard.putNumber("AlignToAprilTag/TargetTagID", targetTagID);

    // Check if a tag was found
    if (targetTagID == -1) {
      // DataLogManager.log("[AlignToAprilTag] ERROR: No AprilTag found in map!");
      tagDetected = false;
      return robotPose;
    }

    // DataLogManager.log("[AlignToAprilTag] Closest tag from odometry: " +
    // targetTagID);

    // Check if Limelight sees a valid tag - prefer it over odometry,
    // but only if it's on our alliance side
    int limelightTagID = 0;
    for (String name : VisionConstants.LIMELIGHT_NAMES) {
      int fid = (int) LimelightHelpers.getFiducialID(name);
      if (fid != 0) {
        limelightTagID = fid;
        break;
      }
    }
    if (limelightTagID != 0
        && AprilTagMaps.aprilTagMap.containsKey(limelightTagID)
        && Constants.contains(allianceTags, limelightTagID)) {
      targetTagID = limelightTagID;
      // DataLogManager.log("[AlignToAprilTag] Using Limelight tag: " + targetTagID);
    } else if (limelightTagID == 0) {
      // DataLogManager.log(
      // "[AlignToAprilTag] Limelight has no target, using odometry closest tag: " +
      // targetTagID);
    } else {
      // DataLogManager.log("[AlignToAprilTag] Limelight tag "
      // + limelightTagID
      // + " not valid for alliance, using odometry: "
      // + targetTagID);
    }

    if (isSimulation) {
      targetTagID = 10;
    }

    // Get tag data
    double[] tagData = AprilTagMaps.aprilTagMap.get(targetTagID);
    if (tagData == null) {
      // DataLogManager.log("[AlignToAprilTag] ERROR: Tag data is null for ID: " +
      // targetTagID);
      tagDetected = false;
      return robotPose;
    }

    return new Pose2d(
        tagData[0] * Constants.INCHES_TO_METERS,
        tagData[1] * Constants.INCHES_TO_METERS,
        new Rotation2d(Math.toRadians(tagData[3])));
  }

  public Pose2d getCorrectedRobotPose() {
    Pose2d currentPose = drivetrain.getState().Pose;

    Pose2d hubPose;

    ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
        drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());

    double xVelFieldCentric = fieldVelocity.vxMetersPerSecond;

    double yVelFieldCentric = fieldVelocity.vyMetersPerSecond;

    hubPose = getCorrectedHubPose();

    // Pose2d aprilTagPose = calculateTagPose();

    // If no tag detected, don't execute
    if (!tagDetected) {
      return currentPose;
    }

    double distanceToAprilTag = currentPose.getTranslation().getDistance(hubPose.getTranslation());

    Pose2d correctedPose = new Pose2d(
        currentPose.getX()
            + (xVelFieldCentric * ShooterInterpolationTable.getTimeOfFlight(distanceToAprilTag)),
        currentPose.getY()
            + (yVelFieldCentric * ShooterInterpolationTable.getTimeOfFlight(distanceToAprilTag)),
        drivetrain.getState().Pose.getRotation());
    return correctedPose;
  }

  private Pose2d getCorrectedHubPose() {

    // Get current robot pose
    Pose2d currentPose = drivetrain.getState().Pose;
    if (currentPose == null) {
      return new Pose2d();
    }

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    // Convert tag position to Pose2d
    Pose2d aprilTagPose = calculateTagPose();

    Pose2d hubPose;

    Pose2d correctedHubPose = new Pose2d();

    double distanceToHub;

    ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
        drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());

    double xVelFieldCentric = fieldVelocity.vxMetersPerSecond;

    double yVelFieldCentric = fieldVelocity.vyMetersPerSecond;

    hubPose = (isRed)
        ? new Pose2d(GameConstants.RED_HUB_CENTER, new Rotation2d())
        : new Pose2d(GameConstants.BLUE_HUB_CENTER, new Rotation2d());

    double xVelRobotCentric = drivetrain.getState().Speeds.vxMetersPerSecond;

    double movingBackwardsFactor = (Math.min(0, xVelRobotCentric) * .5);
    SmartDashboard.putNumber("Moving Backwards Factor", movingBackwardsFactor);

    distanceToHub =
        currentPose.getTranslation().getDistance(hubPose.getTranslation()) * movingBackwardsFactor;

    correctedHubPose = new Pose2d(
        hubPose.getX()
            - (ShooterInterpolationTable.getTimeOfFlight(distanceToHub) * xVelFieldCentric),
        hubPose.getY()
            - (ShooterInterpolationTable.getTimeOfFlight(distanceToHub) * yVelFieldCentric),
        new Rotation2d());
    // Change the number of loops for more precision in estimating the pose
    for (int i = 0; i < 11; i++) {
      distanceToHub = currentPose.getTranslation().getDistance(correctedHubPose.getTranslation());
      correctedHubPose = new Pose2d(
          hubPose.getX()
              - (ShooterInterpolationTable.getTimeOfFlight(distanceToHub) * xVelFieldCentric),
          hubPose.getY()
              - (ShooterInterpolationTable.getTimeOfFlight(distanceToHub) * yVelFieldCentric),
          new Rotation2d());
      SmartDashboard.putNumber(
          "AlignToAprilTag/ToF " + (i + 1),
          ShooterInterpolationTable.getTimeOfFlight(distanceToHub));
    }
    SmartDashboard.putNumber("AlignToAprilTag/Final Target Dist", distanceToHub);

    return correctedHubPose;
  }

  public static Supplier<ShooterSetpoint> getShooterSetpointSupplier() {
    return sotmSetpointSupplier;
  }

  /** Calculate velocities using PID control */
  private double[] calculatePIDVelocities(Pose2d currentPose, Pose2d aprilTagPose) {
    // Target rotation is opposite the tag (facing the tag)
    double targetRotation = Math.atan2(
        aprilTagPose.getY() - drivetrain.getState().Pose.getY(),
        aprilTagPose.getX() - drivetrain.getState().Pose.getX());
    // targetRotation = MathUtil.angleModulus(targetRotation);

    // // Rotate the offset from tag-relative to field-relative
    // double rotatedOffsetX =
    // (offsetX * Math.cos(targetRotation)) - (offsetY * Math.sin(targetRotation));
    // double rotatedOffsetY =
    // (offsetX * Math.sin(targetRotation)) + (offsetY * Math.cos(targetRotation));

    // Calculate final target pose
    targetPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(targetRotation));

    // Set PID setpoints
    pidX.setSetpoint(targetPose.getX());
    pidY.setSetpoint(targetPose.getY());
    pidRotate.setSetpoint(targetPose.getRotation().getRadians());
    // Calculate X velocity
    double velocityX = pidX.calculate(currentPose.getX());
    velocityX = MathUtil.clamp(velocityX, -speed, speed);

    // Calculate Y velocity
    double velocityY = pidY.calculate(currentPose.getY());
    velocityY = MathUtil.clamp(velocityY, -speed, speed);

    // Calculate rotation velocity
    double velocityYaw = pidRotate.calculate(currentPose.getRotation().getRadians());
    velocityYaw = MathUtil.clamp(velocityYaw, -2.0, 2.0);

    // DataLogManager.log("[AlignToAprilTag] Target Pose: X="
    // + targetPose.getX()
    // + ", Y="
    // + targetPose.getY()
    // + ", Yaw="
    // + Math.toDegrees(targetPose.getRotation().getRadians())
    // + "°");

    SmartDashboard.putNumber("AlignToAprilTag/TargetX", targetPose.getX());
    SmartDashboard.putNumber("AlignToAprilTag/TargetY", targetPose.getY());

    return new double[] {velocityX, velocityY, velocityYaw};
  }

  /** Calculate distance between two poses */
  private double calculateDistance(Pose2d pose1, Pose2d pose2) {
    double dx = pose1.getX() - pose2.getX();
    double dy = pose1.getY() - pose2.getY();
    return Math.sqrt(dx * dx + dy * dy);
  }

  @Override
  public boolean isFinished() {
    // End if no tag detected
    if (!tagDetected) {
      return false;
    }

    // Get current pose
    Pose2d currentPose = drivetrain.getState().Pose;
    if (currentPose == null || targetPose == null) {
      return false;
    }

    // Calculate position and yaw error
    double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
    double yawError = Math.abs(MathUtil.angleModulus(
        targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));

    // Check if within tolerance
    boolean positionReached = distance <= positionTolerance;
    boolean yawReached = true;

    // Timeout after ALIGN_TIMEOUT_SECONDS to prevent indefinite alignment attempts
    boolean timedOut = timer.hasElapsed(ALIGN_TIMEOUT);
    if (timedOut) {
      // DataLogManager.log("[AlignToAprilTag] Timed out after " + ALIGN_TIMEOUT + "s"
      // + " (distance=" + String.format("%.3f", distance)
      // + "m, yawError=" + String.format("%.1f", Math.toDegrees(yawError)) + "°)");
    }

    SmartDashboard.putNumber(
        "AlignToAprilTag/TargetYaw", targetPose.getRotation().getDegrees());

    SmartDashboard.putNumber("AlignToAprilTag/Distance", distance);
    SmartDashboard.putBoolean("AlignToAprilTag/PositionReached", positionReached);
    SmartDashboard.putBoolean("AlignToAprilTag/YawReached", yawReached);

    return false; // (positionReached && yawReached) || timedOut;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    drivetrain.setControl(stop);

    // Return to field-centric drive mode
    stateMachine.setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);

    // Set alignment status based on completion
    if (!interrupted) {
      stateMachine.setAlignedToTarget(true);
      // DataLogManager.log(
      // "[AlignToAprilTag] Completed successfully - aligned to tag " + targetTagID);
    } else {
      stateMachine.setAlignedToTarget(false);
      // DataLogManager.log("[AlignToAprilTag] "
      // + (interrupted ? "Interrupted" : "Failed")
      // + " - alignment not confirmed");
    }

    // Log completion
    SmartDashboard.putBoolean("AlignToAprilTag/Completed", !interrupted);
    SmartDashboard.putNumber("AlignToAprilTag/Duration", timer.get());
  }
}
