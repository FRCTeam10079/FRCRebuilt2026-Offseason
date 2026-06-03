// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathfinding;

/**
 * Kinematic constraints for path following.
 *
 * @param maxVelocityMps Maximum linear velocity (m/s)
 * @param maxAccelerationMpsSq Maximum linear acceleration (m/s²)
 * @param maxAngularVelocityRadPerSec Maximum angular velocity (rad/s)
 * @param maxAngularAccelerationRadPerSecSq Maximum angular acceleration (rad/s²)
 */
public record PathConstraints(
    double maxVelocityMps,
    double maxAccelerationMpsSq,
    double maxAngularVelocityRadPerSec,
    double maxAngularAccelerationRadPerSecSq) {

  /** Default pathfinding constraints based on PathfindingConstants. */
  public static final PathConstraints DEFAULT = new PathConstraints(
      PathfindingConstants.MAX_VELOCITY_MPS,
      PathfindingConstants.MAX_ACCELERATION_MPSS,
      PathfindingConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
      PathfindingConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ);

  /** Slower constraints for precision scoring. */
  public static final PathConstraints SCORING = new PathConstraints(
      PathfindingConstants.MAX_VELOCITY_MPS * 0.5,
      PathfindingConstants.MAX_ACCELERATION_MPSS * 0.5,
      PathfindingConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC * 0.5,
      PathfindingConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ * 0.5);

  /** Faster constraints for autonomous. */
  public static final PathConstraints AUTO = new PathConstraints(
      PathfindingConstants.MAX_VELOCITY_MPS * 1.2,
      PathfindingConstants.MAX_ACCELERATION_MPSS * 1.2,
      PathfindingConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC * 1.0,
      PathfindingConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ * 1.0);

  /**
   * Create constraints scaled by a factor.
   *
   * @param factor Scale factor (e.g., 0.5 for half speed)
   * @return New scaled constraints
   */
  public PathConstraints scaled(double factor) {
    return new PathConstraints(
        maxVelocityMps * factor,
        maxAccelerationMpsSq * factor,
        maxAngularVelocityRadPerSec * factor,
        maxAngularAccelerationRadPerSecSq * factor);
  }
}
