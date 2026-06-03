// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Static class for accessing the active pathfinder from anywhere in the codebase.
 *
 * <p>This allows pathfinding commands to interact with the pathfinder without needing direct
 * references.
 */
public final class Pathfinding {

  private static Pathfinder pathfinder = null;

  private Pathfinding() {}

  /**
   * Set the pathfinder implementation to use.
   *
   * @param pathfinder The pathfinder instance
   */
  public static void setPathfinder(Pathfinder pathfinder) {
    Pathfinding.pathfinder = pathfinder;
  }

  /**
   * Get the current pathfinder instance.
   *
   * @return The active pathfinder, or null if not initialized
   */
  public static Pathfinder getPathfinder() {
    return pathfinder;
  }

  /** Ensure the pathfinding system is initialized. Creates a LocalADStar if not set. */
  public static void ensureInitialized() {
    if (pathfinder == null) {
      Logger.recordOutput("Events/Pathfinding/Last", "Initializing default LocalADStar pathfinder");
      pathfinder = new LocalADStar();
    }
  }

  /**
   * Check if a new path has been calculated.
   *
   * @return True if a new path is available
   */
  public static boolean isNewPathAvailable() {
    ensureInitialized();
    return pathfinder.isNewPathAvailable();
  }

  /**
   * Get the most recently calculated path.
   *
   * @return List of waypoints forming the path
   */
  public static List<Translation2d> getCurrentPathWaypoints() {
    ensureInitialized();
    return pathfinder.getCurrentPathWaypoints();
  }

  /**
   * Get the current goal pose.
   *
   * @return The goal pose
   */
  public static Pose2d getGoalPose() {
    ensureInitialized();
    return pathfinder.getGoalPose();
  }

  /**
   * Set the start pose for pathfinding.
   *
   * @param startPose The robot's current pose
   */
  public static void setStartPose(Pose2d startPose) {
    ensureInitialized();
    pathfinder.setStartPose(startPose);
  }

  /**
   * Set the goal pose for pathfinding.
   *
   * @param goalPose The target pose
   */
  public static void setGoalPose(Pose2d goalPose) {
    ensureInitialized();
    pathfinder.setGoalPose(goalPose);
  }

  /**
   * Set the robot's current velocity for smoother path starts.
   *
   * @param velocity Field-relative velocity
   */
  public static void setStartVelocity(Translation2d velocity) {
    ensureInitialized();
    pathfinder.setStartVelocity(velocity);
  }

  /**
   * Set the complete pathfinding problem.
   *
   * @param startPose Robot's current pose
   * @param goalPose Target pose
   * @param startVelocity Robot's current velocity
   */
  public static void setProblem(Pose2d startPose, Pose2d goalPose, Translation2d startVelocity) {
    ensureInitialized();
    pathfinder.setProblem(startPose, goalPose, startVelocity);
  }

  /** Use teleop obstacle set. */
  public static void setTeleopObstacles() {
    ensureInitialized();
    pathfinder.setTeleopObstacles();
  }

  /** Use autonomous obstacle set. */
  public static void setAutoObstacles() {
    ensureInitialized();
    pathfinder.setAutoObstacles();
  }

  /**
   * Check if the pathfinder is actively computing.
   *
   * @return True if computing a path
   */
  public static boolean isComputing() {
    ensureInitialized();
    return pathfinder.isComputing();
  }

  /** Shutdown the pathfinder. */
  public static void shutdown() {
    if (pathfinder != null) {
      pathfinder.shutdown();
    }
  }
}
