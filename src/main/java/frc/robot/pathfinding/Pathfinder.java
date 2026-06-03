// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Interface for a pathfinder implementation.
 *
 * <p>The pathfinder runs in a background thread and produces paths asynchronously.
 */
public interface Pathfinder {

  /**
   * Get if a new path has been calculated since the last time a path was retrieved.
   *
   * @return True if a new path is available
   */
  boolean isNewPathAvailable();

  /**
   * Get the most recently calculated path as a list of waypoints.
   *
   * @return List of Translation2d waypoints forming the path, or empty list if no path available
   */
  List<Translation2d> getCurrentPathWaypoints();

  /**
   * Get the goal pose for the current path.
   *
   * @return The goal pose, or null if not set
   */
  Pose2d getGoalPose();

  /**
   * Set the start position to pathfind from.
   *
   * @param startPose Start position on the field. If this is within an obstacle, it will be moved
   *     to the nearest non-obstacle node.
   */
  void setStartPose(Pose2d startPose);

  /**
   * Set the goal position to pathfind to.
   *
   * @param goalPose Goal position on the field. If this is within an obstacle, it will be moved to
   *     the nearest non-obstacle node.
   */
  void setGoalPose(Pose2d goalPose);

  /**
   * Set the current robot velocity for smoother path starts.
   *
   * @param startVelocity The robot's current field-relative velocity
   */
  void setStartVelocity(Translation2d startVelocity);

  /**
   * Set the full pathfinding problem atomically.
   *
   * @param startPose Start position on the field
   * @param goalPose Goal position on the field
   * @param startVelocity Current robot velocity
   */
  void setProblem(Pose2d startPose, Pose2d goalPose, Translation2d startVelocity);

  /** Use the standard obstacle set (teleop mode). */
  void setTeleopObstacles();

  /** Use the autonomous obstacle set (tighter margins). */
  void setAutoObstacles();

  /** Check if the pathfinder is actively computing a path. */
  boolean isComputing();

  /** Shutdown the pathfinder thread. */
  void shutdown();
}
