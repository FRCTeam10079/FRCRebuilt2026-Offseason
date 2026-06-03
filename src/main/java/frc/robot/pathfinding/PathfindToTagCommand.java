// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathfinding;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command that pathfinds to a target pose using the AD* algorithm and follows the path using PID
 * control.
 *
 * <p>This command combines pathfinding logic (AD* search on a navgrid) with motion control pattern
 * (PID-based trajectory following).
 *
 * <p>The command: Sets the pathfinding goal (e.g., AprilTag scoring pose) Waits for the AD*
 * background thread to calculate a path Follows the path using pure pursuit / PID control Ends when
 * the robot reaches the goal within tolerance
 */
public class PathfindToTagCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final PathConstraints constraints;

  // PID controllers for path following
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  // State
  private Pose2d targetPose;
  private List<Translation2d> currentPath;
  private int currentWaypointIndex;
  private final Timer pathWaitTimer = new Timer();
  private boolean hasPath = false;
  private boolean waitWarningLogged = false;
  private int eventSequence = 0;

  // NetworkTables publishers for AdvantageScope visualization
  private final NetworkTable pathfindingTable;
  private final StructPublisher<Pose2d> currentPosePublisher;
  private final StructPublisher<Pose2d> goalPosePublisher;
  private final StructPublisher<Pose2d> targetWaypointPublisher;
  private final StructArrayPublisher<Pose2d> pathPublisher;

  // Field2d for visualization in AdvantageScope / Shuffleboard
  private static final Field2d pathfindingField = new Field2d();

  static {
    // Publish Field2d once so AdvantageScope can see it
    SmartDashboard.putData("Pathfinding/Field", pathfindingField);
  }

  // Pure pursuit parameters
  private static final double LOOKAHEAD_DISTANCE =
      Constants.PathfindingPIDConstants.LOOKAHEAD_DISTANCE_METERS;
  private static final double WAYPOINT_TOLERANCE =
      Constants.PathfindingPIDConstants.WAYPOINT_TOLERANCE_METERS;

  /**
   * Create a command to pathfind to a specific AprilTag.
   *
   * @param drivetrain The swerve drivetrain
   * @param tagId The AprilTag ID to target
   */
  public static PathfindToTagCommand toAprilTag(CommandSwerveDrivetrain drivetrain, int tagId) {
    return new PathfindToTagCommand(
        drivetrain,
        () -> PathfindingConstants.getScoringPoseForTag(tagId),
        PathConstraints.DEFAULT);
  }

  /**
   * Create a command to pathfind to AprilTag 10 (Red Alliance Hub Face).
   *
   * @param drivetrain The swerve drivetrain
   */
  public static PathfindToTagCommand toAprilTag10(CommandSwerveDrivetrain drivetrain) {
    return toAprilTag(drivetrain, 10);
  }

  /**
   * Create a pathfinding command.
   *
   * @param drivetrain The swerve drivetrain
   * @param targetPoseSupplier Supplier for the target pose
   * @param constraints Path following constraints
   */
  public PathfindToTagCommand(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> targetPoseSupplier,
      PathConstraints constraints) {

    this.drivetrain = drivetrain;
    this.targetPoseSupplier = targetPoseSupplier;
    this.constraints = constraints;

    // Translation PID (field-relative)
    this.xController = new PIDController(
        Constants.PathfindingPIDConstants.FOLLOW_KP,
        Constants.PathfindingPIDConstants.FOLLOW_KI,
        Constants.PathfindingPIDConstants.FOLLOW_KD);
    this.yController = new PIDController(
        Constants.PathfindingPIDConstants.FOLLOW_KP,
        Constants.PathfindingPIDConstants.FOLLOW_KI,
        Constants.PathfindingPIDConstants.FOLLOW_KD);

    // Rotation PID with continuous input
    this.rotationController = new PIDController(
        Constants.PathfindingPIDConstants.FOLLOW_KP,
        Constants.PathfindingPIDConstants.FOLLOW_KI,
        Constants.PathfindingPIDConstants.FOLLOW_KD);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    // Initialize NetworkTables publishers for AdvantageScope
    pathfindingTable = NetworkTableInstance.getDefault().getTable("Pathfinding");
    currentPosePublisher =
        pathfindingTable.getStructTopic("CurrentPose", Pose2d.struct).publish();
    goalPosePublisher =
        pathfindingTable.getStructTopic("GoalPose", Pose2d.struct).publish();
    targetWaypointPublisher =
        pathfindingTable.getStructTopic("TargetWaypoint", Pose2d.struct).publish();
    pathPublisher = pathfindingTable.getStructArrayTopic("Path", Pose2d.struct).publish();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Get target pose
    targetPose = targetPoseSupplier.get();

    // Get current robot state
    Pose2d currentPose = drivetrain.getState().Pose;
    ChassisSpeeds currentSpeeds = drivetrain.getState().Speeds;

    // Convert to field-relative velocity
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
    Translation2d velocity =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    // Ensure pathfinder is initialized
    Pathfinding.ensureInitialized();

    // Set the pathfinding problem
    Pathfinding.setProblem(currentPose, targetPose, velocity);

    // Reset state
    currentPath = null;
    currentWaypointIndex = 0;
    hasPath = false;
    waitWarningLogged = false;
    pathWaitTimer.restart();

    // Reset PID controllers
    xController.reset();
    yController.reset();
    rotationController.reset();

    recordEvent("[PathfindToTag] Starting pathfind from "
        + formatPose(currentPose)
        + " to "
        + formatPose(targetPose));
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    // Check for new path from background thread
    if (Pathfinding.isNewPathAvailable()) {
      currentPath = Pathfinding.getCurrentPathWaypoints();
      currentWaypointIndex = 0;
      hasPath = currentPath != null && currentPath.size() >= 2;

      if (hasPath) {
        recordEvent("[PathfindToTag] Received path with " + currentPath.size() + " waypoints");
      } else {
        recordEvent("[PathfindToTag] WARNING: Received invalid path (null or < 2 waypoints)");
      }
    }

    // If no path yet, wait (but NEVER drive directly - that would ignore
    // obstacles!)
    if (!hasPath) {
      Logger.recordOutput("Pathfinding/Status", "Waiting for path...");
      Logger.recordOutput("Pathfinding/WaitTime", pathWaitTimer.get());

      // Keep publishing current pose even while waiting - VERY IMPORTANT for
      // visibility!
      currentPosePublisher.set(currentPose);
      goalPosePublisher.set(targetPose);

      // Also publish to Field2d while waiting
      pathfindingField.setRobotPose(currentPose);
      pathfindingField.getObject("Goal").setPose(targetPose);
      pathfindingField.getObject("TargetWaypoint").setPose(targetPose); // No waypoint yet
      pathfindingField.getObject("Path").setPoses(); // Empty path

      // If waiting too long, log one warning but DON'T drive directly
      // (that ignores obstacles).
      if (pathWaitTimer.hasElapsed(2.0) && !waitWarningLogged) {
        waitWarningLogged = true;
        recordEvent("[PathfindToTag] WARNING: Still waiting for path after "
            + String.format("%.1f", pathWaitTimer.get()) + "s");
      }
      return;
    }

    // Find current target waypoint using lookahead
    Translation2d targetWaypoint = findLookaheadPoint(currentPose);

    // Log state to AdvantageKit
    Logger.recordOutput("Pathfinding/Status", "Following path");
    Logger.recordOutput("Pathfinding/CurrentWaypoint", currentWaypointIndex);
    Logger.recordOutput("Pathfinding/TotalWaypoints", currentPath.size());

    // Calculate velocities using PID
    double xVelocity = clampVelocity(
        xController.calculate(currentPose.getX(), targetWaypoint.getX()),
        constraints.maxVelocityMps());

    double yVelocity = clampVelocity(
        yController.calculate(currentPose.getY(), targetWaypoint.getY()),
        constraints.maxVelocityMps());

    // Rotation towards goal pose
    double rotationVelocity = clampVelocity(
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()),
        constraints.maxAngularVelocityRadPerSec());

    // Apply to drivetrain as field-relative speeds
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
    ChassisSpeeds robotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentPose.getRotation());

    // === NETWORKTABLES LOGGING FOR ADVANTAGESCOPE ===
    // Publish poses for 2D field visualization
    currentPosePublisher.set(currentPose);
    goalPosePublisher.set(targetPose);
    targetWaypointPublisher.set(new Pose2d(targetWaypoint, targetPose.getRotation()));

    // === FIELD2D VISUALIZATION (works with AdvantageScope/Shuffleboard) ===
    // Update robot pose on field
    pathfindingField.setRobotPose(currentPose);
    // Goal pose shown as a separate object
    pathfindingField.getObject("Goal").setPose(targetPose);
    // Target waypoint shown as another object
    pathfindingField
        .getObject("TargetWaypoint")
        .setPose(new Pose2d(targetWaypoint, targetPose.getRotation()));
    // Full path trajectory
    if (currentPath != null && !currentPath.isEmpty()) {
      Pose2d[] pathPoses = new Pose2d[currentPath.size()];
      for (int i = 0; i < currentPath.size(); i++) {
        pathPoses[i] = new Pose2d(currentPath.get(i), targetPose.getRotation());
      }
      pathfindingField.getObject("Path").setPoses(pathPoses);
    } else {
      pathfindingField.getObject("Path").setPoses();
    }

    // Publish velocity data via Logger
    Logger.recordOutput("Pathfinding/VelX", xVelocity);
    Logger.recordOutput("Pathfinding/VelY", yVelocity);
    Logger.recordOutput("Pathfinding/VelOmega", rotationVelocity);
    Logger.recordOutput("Pathfinding/ErrorX", targetWaypoint.getX() - currentPose.getX());
    Logger.recordOutput("Pathfinding/ErrorY", targetWaypoint.getY() - currentPose.getY());
    Logger.recordOutput("Pathfinding/CurrentWaypointIdx", currentWaypointIndex);
    Logger.recordOutput("Pathfinding/PathLength", currentPath != null ? currentPath.size() : 0);

    // Publish path for raw struct visualization too
    if (currentPath != null && !currentPath.isEmpty()) {
      Pose2d[] pathPoses = new Pose2d[currentPath.size()];
      for (int i = 0; i < currentPath.size(); i++) {
        pathPoses[i] = new Pose2d(currentPath.get(i), targetPose.getRotation());
      }
      pathPublisher.set(pathPoses);
    } else {
      // Publish empty array when no path to clear stale data
      pathPublisher.set(new Pose2d[0]);
    }

    drivetrain.setControl(
        new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotSpeeds));

    // Advance waypoint if close enough
    advanceWaypointIfNeeded(currentPose);

    // Update pathfinder with current pose for dynamic replanning
    Pathfinding.setStartPose(currentPose);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds()
        .withSpeeds(new ChassisSpeeds()));

    if (interrupted) {
      recordEvent("[PathfindToTag] Command interrupted");
    } else {
      recordEvent("[PathfindToTag] Reached goal: " + formatPose(targetPose));
    }
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = drivetrain.getState().Pose;

    // Check position tolerance
    double positionError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    boolean positionOk = positionError < PathfindingConstants.POSITION_TOLERANCE_METERS;

    // Check rotation tolerance
    double rotationError =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    boolean rotationOk = rotationError < PathfindingConstants.ROTATION_TOLERANCE_RADIANS;

    return positionOk && rotationOk;
  }

  // ==================== Path Following Helpers ====================

  private Translation2d findLookaheadPoint(Pose2d currentPose) {
    if (currentPath == null || currentPath.isEmpty()) {
      return targetPose.getTranslation();
    }

    Translation2d robotPos = currentPose.getTranslation();

    // Start from current waypoint index
    for (int i = currentWaypointIndex; i < currentPath.size(); i++) {
      Translation2d waypoint = currentPath.get(i);
      double distance = robotPos.getDistance(waypoint);

      if (distance >= LOOKAHEAD_DISTANCE) {
        // Interpolate to exact lookahead distance
        if (i > 0) {
          Translation2d prev = currentPath.get(i - 1);
          Translation2d direction = waypoint.minus(prev);
          double segmentLength = direction.getNorm();
          if (segmentLength > 0.01) {
            // Find point on segment at lookahead distance from robot
            Translation2d toRobot = robotPos.minus(prev);
            double projection =
                toRobot.getX() * direction.getX() + toRobot.getY() * direction.getY();
            projection /= segmentLength * segmentLength;
            projection = Math.max(0, Math.min(1, projection));
            Translation2d closestPoint = prev.plus(direction.times(projection));

            // Move lookahead distance along segment from closest point
            Translation2d segmentDir = direction.div(segmentLength);
            return closestPoint.plus(segmentDir.times(LOOKAHEAD_DISTANCE));
          }
        }
        return waypoint;
      }
    }

    // Return final goal
    return targetPose.getTranslation();
  }

  private void advanceWaypointIfNeeded(Pose2d currentPose) {
    if (currentPath == null || currentWaypointIndex >= currentPath.size() - 1) {
      return;
    }

    Translation2d waypoint = currentPath.get(currentWaypointIndex);
    double distance = currentPose.getTranslation().getDistance(waypoint);

    if (distance < WAYPOINT_TOLERANCE) {
      currentWaypointIndex++;
    }
  }

  private double clampVelocity(double velocity, double max) {
    return Math.max(-max, Math.min(max, velocity));
  }

  private String formatPose(Pose2d pose) {
    return String.format(
        "(%.2f, %.2f, %.1f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  private void recordEvent(String message) {
    Logger.recordOutput("Events/PathfindToTag/Last", message);
    Logger.recordOutput("Events/PathfindToTag/Sequence", ++eventSequence);
  }
}
