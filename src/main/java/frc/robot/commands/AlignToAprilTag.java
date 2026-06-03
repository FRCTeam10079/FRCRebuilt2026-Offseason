// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.LimelightHelpers;
import frc.robot.statemachine.DrivetrainMode;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Command to align the robot to an AprilTag using vision
 *
 * <p>This command: 1. Finds the closest AprilTag (or uses Limelight-detected tag) 2. Calculates
 * target position with optional offset (LEFT/RIGHT/CENTER) 3. Uses PID control to drive the robot
 * to the target pose 4. Rotates to face opposite the tag (facing the tag)
 *
 * <p>For REBUILT 2026 - Generic AprilTag alignment for any field element
 */
public class AlignToAprilTag extends Command {

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

  // Swerve drive request - field centric with velocity control
  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  // Stop request
  private final SwerveRequest stop;

  // Target pose to align to
  private Pose2d targetPose;

  // Configuration from Constants
  private final double speed;
  private final double positionTolerance;
  private final double yawTolerance;

  // Offset configuration
  private final AlignPosition alignPosition;
  private double offsetX = 0;
  private double offsetY = 0;

  // Target AprilTag info
  private int targetTagID;
  private boolean tagDetected = false;
  private int eventSequence = 0;

  // Timeout duration (seconds) - prevents indefinite alignment attempts
  private static final double ALIGN_TIMEOUT_SECONDS = 3.0;

  /**
   * Creates a new AlignToAprilTag command
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param vision The vision subsystem
   * @param alignPosition LEFT, RIGHT, or CENTER offset from the AprilTag
   */
  public AlignToAprilTag(
      CommandSwerveDrivetrain drivetrain, Vision vision, AlignPosition alignPosition) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.stateMachine = RobotStateMachine.getInstance();
    this.alignPosition = alignPosition;

    // Get constants
    this.speed = DrivetrainConstants.ALIGN_SPEED_MPS;
    this.positionTolerance = DrivetrainConstants.POSITION_TOLERANCE_METERS;
    this.yawTolerance = DrivetrainConstants.YAW_TOLERANCE_RADIANS;

    // Initialize PID controllers with values from Constants
    pidX = new PIDController(
        DrivetrainConstants.ALIGN_PID_KP,
        DrivetrainConstants.ALIGN_PID_KI,
        DrivetrainConstants.ALIGN_PID_KD);
    pidY = new PIDController(
        DrivetrainConstants.ALIGN_PID_KP,
        DrivetrainConstants.ALIGN_PID_KI,
        DrivetrainConstants.ALIGN_PID_KD);
    pidRotate = new PIDController(
        DrivetrainConstants.ALIGN_ROTATION_KP, 0, DrivetrainConstants.ALIGN_ROTATION_KD);

    // Enable continuous input for rotation (-PI to PI are same point)
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);

    // Create stop request
    stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

    // This command requires the drivetrain
    addRequirements(drivetrain);

    recordEvent("[AlignToAprilTag] Created for " + alignPosition + " position");
  }

  /** Convenience constructor for CENTER alignment */
  public AlignToAprilTag(CommandSwerveDrivetrain drivetrain, Vision vision) {
    this(drivetrain, vision, AlignPosition.CENTER);
  }

  @Override
  public void initialize() {
    // Start timer
    timer.restart();

    // Set state machine to vision tracking mode
    stateMachine.setDrivetrainMode(DrivetrainMode.VISION_TRACKING);
    stateMachine.setAlignedToTarget(false);

    // Get current robot pose
    Pose2d robotPose = drivetrain.getState().Pose;
    if (robotPose == null) {
      recordEvent("[AlignToAprilTag] ERROR: Robot pose is null!");
      tagDetected = false;
      return;
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

    // Check if a tag was found
    if (targetTagID == -1) {
      recordEvent("[AlignToAprilTag] ERROR: No AprilTag found in map!");
      tagDetected = false;
      return;
    }

    recordEvent("[AlignToAprilTag] Closest tag from odometry: " + targetTagID);

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
      recordEvent("[AlignToAprilTag] Using Limelight tag: " + targetTagID);
    } else if (limelightTagID == 0) {
      recordEvent(
          "[AlignToAprilTag] Limelight has no target, using odometry closest tag: " + targetTagID);
    } else {
      recordEvent("[AlignToAprilTag] Limelight tag "
          + limelightTagID
          + " not valid for alliance, using odometry: "
          + targetTagID);
    }

    // Get tag data
    double[] tagData = AprilTagMaps.aprilTagMap.get(targetTagID);
    if (tagData == null) {
      recordEvent("[AlignToAprilTag] ERROR: Tag data is null for ID: " + targetTagID);
      tagDetected = false;
      return;
    }

    // Convert tag position to Pose2d
    Pose2d aprilTagPose = new Pose2d(
        tagData[0] * Constants.INCHES_TO_METERS,
        tagData[1] * Constants.INCHES_TO_METERS,
        new Rotation2d(Math.toRadians(tagData[3])));

    tagDetected = true;

    // Calculate offset based on alignment position
    // Offsets are relative to the tag's coordinate frame
    switch (alignPosition) {
      case LEFT:
        offsetX = DrivetrainConstants.ALIGN_OFFSET_X_LEFT;
        offsetY = DrivetrainConstants.ALIGN_OFFSET_Y_LEFT;
        break;
      case RIGHT:
        offsetX = DrivetrainConstants.ALIGN_OFFSET_X_RIGHT;
        offsetY = DrivetrainConstants.ALIGN_OFFSET_Y_RIGHT;
        break;
      case CENTER:
      default:
        offsetX = DrivetrainConstants.ALIGN_OFFSET_X_CENTER;
        offsetY = DrivetrainConstants.ALIGN_OFFSET_Y_CENTER;
        break;
    }

    // Target rotation is opposite the tag (facing the tag)
    double targetRotation = aprilTagPose.getRotation().getRadians() - Math.PI;
    targetRotation = MathUtil.angleModulus(targetRotation);

    // Rotate the offset from tag-relative to field-relative
    double rotatedOffsetX =
        (offsetX * Math.cos(targetRotation)) - (offsetY * Math.sin(targetRotation));
    double rotatedOffsetY =
        (offsetX * Math.sin(targetRotation)) + (offsetY * Math.cos(targetRotation));

    // Calculate final target pose
    targetPose = new Pose2d(
        aprilTagPose.getX() + rotatedOffsetX,
        aprilTagPose.getY() + rotatedOffsetY,
        new Rotation2d(targetRotation));

    // Set PID setpoints
    pidX.setSetpoint(targetPose.getX());
    pidY.setSetpoint(targetPose.getY());
    pidRotate.setSetpoint(targetPose.getRotation().getRadians());

    recordEvent("[AlignToAprilTag] Target Pose: X="
        + targetPose.getX()
        + ", Y="
        + targetPose.getY()
        + ", Yaw="
        + Math.toDegrees(targetPose.getRotation().getRadians())
        + "°");

    // Log to AdvantageKit
    Logger.recordOutput("AlignToAprilTag/TargetTagID", targetTagID);
    Logger.recordOutput("AlignToAprilTag/TargetX", targetPose.getX());
    Logger.recordOutput("AlignToAprilTag/TargetY", targetPose.getY());
    Logger.recordOutput("AlignToAprilTag/TargetYaw", targetPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    // If no tag detected, don't execute
    if (!tagDetected) {
      return;
    }

    // Get current robot pose
    Pose2d currentPose = drivetrain.getState().Pose;
    if (currentPose == null) {
      return;
    }

    // Calculate velocities using PID
    double[] velocities = calculatePIDVelocities(currentPose);

    // Log data
    Logger.recordOutput("AlignToAprilTag/CurrentX", currentPose.getX());
    Logger.recordOutput("AlignToAprilTag/CurrentY", currentPose.getY());
    Logger.recordOutput("AlignToAprilTag/CurrentYaw", currentPose.getRotation().getDegrees());
    Logger.recordOutput("AlignToAprilTag/ErrorX", pidX.getError());
    Logger.recordOutput("AlignToAprilTag/ErrorY", pidY.getError());
    Logger.recordOutput("AlignToAprilTag/ErrorYaw", Math.toDegrees(pidRotate.getError()));

    // Apply velocities to drivetrain
    drivetrain.setControl(driveRequest
        .withVelocityX(velocities[0])
        .withVelocityY(velocities[1])
        .withRotationalRate(velocities[2]));
  }

  /** Calculate velocities using PID control */
  private double[] calculatePIDVelocities(Pose2d currentPose) {
    // Calculate X velocity
    double velocityX = pidX.calculate(currentPose.getX());
    velocityX = MathUtil.clamp(velocityX, -speed, speed);

    // Calculate Y velocity
    double velocityY = pidY.calculate(currentPose.getY());
    velocityY = MathUtil.clamp(velocityY, -speed, speed);

    // Calculate rotation velocity
    double velocityYaw = pidRotate.calculate(currentPose.getRotation().getRadians());
    velocityYaw = MathUtil.clamp(velocityYaw, -2.0, 2.0);

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
      return true;
    }

    // Get current pose
    Pose2d currentPose = drivetrain.getState().Pose;
    if (currentPose == null || targetPose == null) {
      return true;
    }

    // Calculate position and yaw error
    double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
    double yawError = Math.abs(MathUtil.angleModulus(
        targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));

    // Check if within tolerance
    boolean positionReached = distance <= positionTolerance;
    boolean yawReached = yawError <= yawTolerance;

    // Timeout after ALIGN_TIMEOUT_SECONDS to prevent indefinite alignment attempts
    boolean timedOut = timer.hasElapsed(ALIGN_TIMEOUT_SECONDS);
    if (timedOut) {
      recordEvent("[AlignToAprilTag] Timed out after " + ALIGN_TIMEOUT_SECONDS + "s"
          + " (distance=" + String.format("%.3f", distance)
          + "m, yawError=" + String.format("%.1f", Math.toDegrees(yawError)) + "°)");
    }

    Logger.recordOutput("AlignToAprilTag/Distance", distance);
    Logger.recordOutput("AlignToAprilTag/PositionReached", positionReached);
    Logger.recordOutput("AlignToAprilTag/YawReached", yawReached);

    return (positionReached && yawReached) || timedOut;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    drivetrain.setControl(stop);

    // Return to field-centric drive mode
    stateMachine.setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);

    // Set alignment status based on completion
    if (!interrupted && tagDetected) {
      stateMachine.setAlignedToTarget(true);
      recordEvent("[AlignToAprilTag] Completed successfully - aligned to tag " + targetTagID);
    } else {
      stateMachine.setAlignedToTarget(false);
      recordEvent("[AlignToAprilTag] "
          + (interrupted ? "Interrupted" : "Failed")
          + " - alignment not confirmed");
    }

    // Log completion
    Logger.recordOutput("AlignToAprilTag/Completed", !interrupted);
    Logger.recordOutput("AlignToAprilTag/Duration", timer.get());
  }

  private void recordEvent(String message) {
    Logger.recordOutput("Events/AlignToAprilTag/Last", message);
    Logger.recordOutput("Events/AlignToAprilTag/Sequence", ++eventSequence);
  }
}
