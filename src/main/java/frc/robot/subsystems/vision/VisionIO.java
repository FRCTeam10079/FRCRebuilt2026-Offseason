// Stolen from Mercer Island
// Original: Copyright (c) 2021-2026 Littleton Robotics

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default void setRobotOffset(Transform3d offset) {}
}
