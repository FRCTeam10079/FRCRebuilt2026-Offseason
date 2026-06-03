// Stolen from Mercer Island
// Original: Copyright (c) 2021-2026 Littleton Robotics
// Adapted: removed turret camera gating, removed PhotonVision, adapted imports

package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private record VisionEstimate(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {}

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // Pre-allocated collections, reused each periodic() call to avoid GC pressure
  private final List<Pose3d> allTagPoses = new ArrayList<>();
  private final List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
  private final List<Pose3d> allRobotPosesRejected = new ArrayList<>();
  private final List<VisionEstimate> pendingVisionEstimates = new ArrayList<>();
  private final List<Pose3d>[] tagPosesPerCamera;
  private final List<Pose3d>[] robotPosesAcceptedPerCamera;
  private final List<Pose3d>[] robotPosesRejectedPerCamera;
  private final double[] tagStdevMultipliersArray;
  private boolean hasMinTagMultiplierRaw = false;
  private final Trigger haveMinTagMultiplier =
      new Trigger(() -> hasMinTagMultiplierRaw).debounce(0.75, DebounceType.kFalling);

  // Pre-computed logger keys
  private final String[] logKeyProcessInputs;
  private final String[] logKeyTagPoses;
  private final String[] logKeyRobotPosesAccepted;
  private final String[] logKeyRobotPosesRejected;

  @SuppressWarnings("unchecked")
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] = new Alert(
          "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // Initialize per-camera lists, stdev array, and pre-compute logger keys
    tagPosesPerCamera = new List[io.length];
    robotPosesAcceptedPerCamera = new List[io.length];
    robotPosesRejectedPerCamera = new List[io.length];
    tagStdevMultipliersArray = new double[io.length];
    logKeyProcessInputs = new String[io.length];
    logKeyTagPoses = new String[io.length];
    logKeyRobotPosesAccepted = new String[io.length];
    logKeyRobotPosesRejected = new String[io.length];
    for (int i = 0; i < io.length; i++) {
      tagPosesPerCamera[i] = new ArrayList<>();
      robotPosesAcceptedPerCamera[i] = new ArrayList<>();
      robotPosesRejectedPerCamera[i] = new ArrayList<>();
      logKeyProcessInputs[i] = "Vision/Camera" + i;
      logKeyTagPoses[i] = "Vision/Camera" + i + "/TagPoses";
      logKeyRobotPosesAccepted[i] = "Vision/Camera" + i + "/RobotPosesAccepted";
      logKeyRobotPosesRejected[i] = "Vision/Camera" + i + "/RobotPosesRejected";
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  private boolean isPoseObservationRejected(VisionIO.PoseObservation observation) {
    return observation.tagCount() == 0 // Must have at least one tag
        || (observation.ambiguity() > maxAmbiguity.getAsDouble()) // Cannot be high ambiguity
        || Math.abs(observation.pose().getZ())
            > maxZError.getAsDouble() // Must have realistic Z coordinate
        // Must be within the field boundaries
        || observation.pose().getX() < 0.0
        || observation.pose().getX() > aprilTagLayout.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > aprilTagLayout.getFieldWidth();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(logKeyProcessInputs[i], inputs[i]);
    }

    // Clear reusable collections
    allTagPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();
    pendingVisionEstimates.clear();

    // Compute global minimum-multiplier state across all cameras
    double lowestTagStdevMultiplierOverall = Double.POSITIVE_INFINITY;
    boolean hasMinTagMultiplierRawOverall = false;
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      boolean cameraHasAcceptedPoseObservation = false;
      for (var observation : inputs[cameraIndex].poseObservations) {
        if (!isPoseObservationRejected(observation)) {
          cameraHasAcceptedPoseObservation = true;
          break;
        }
      }

      double lowestTagStdevMultiplierForCamera = Double.POSITIVE_INFINITY;
      if (cameraHasAcceptedPoseObservation) {
        for (int tagId : inputs[cameraIndex].tagIds) {
          double tagStdevMultiplierCandidate = getTagStdevMultiplier(tagId);
          if (tagStdevMultiplierCandidate < lowestTagStdevMultiplierForCamera) {
            lowestTagStdevMultiplierForCamera = tagStdevMultiplierCandidate;
          }
          if (tagStdevMultiplierCandidate < lowestTagStdevMultiplierOverall) {
            lowestTagStdevMultiplierOverall = tagStdevMultiplierCandidate;
          }
          if (tagStdevMultiplierCandidate == 1.0) {
            hasMinTagMultiplierRawOverall = true;
          }
        }
      }
      tagStdevMultipliersArray[cameraIndex] = lowestTagStdevMultiplierForCamera;
    }
    hasMinTagMultiplierRaw = hasMinTagMultiplierRawOverall;
    boolean haveMinTagMultiplierOverall = haveMinTagMultiplier.getAsBoolean();
    Logger.recordOutput("Vision/HaveMinTagMultiplier", haveMinTagMultiplierOverall);

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Reuse per-camera lists
      List<Pose3d> tagPoses = tagPosesPerCamera[cameraIndex];
      List<Pose3d> robotPosesAccepted = robotPosesAcceptedPerCamera[cameraIndex];
      List<Pose3d> robotPosesRejected = robotPosesRejectedPerCamera[cameraIndex];
      tagPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Calculate stdev multiplier from precomputed camera minimum
      double tagStdevMultiplier = tagStdevMultipliersArray[cameraIndex];
      if (!haveMinTagMultiplierOverall
          && Double.isFinite(lowestTagStdevMultiplierOverall)
          && Double.isFinite(tagStdevMultiplier)) {
        tagStdevMultiplier /= lowestTagStdevMultiplierOverall;
      }
      tagStdevMultipliersArray[cameraIndex] = tagStdevMultiplier;

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose = isPoseObservationRejected(observation);

        // Add pose to log
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 1.8)
            / observation.tagCount()
            * tagStdevMultiplier;
        double linearStdDev = linearStdDevBaseline.getAsDouble() * stdDevFactor;
        double angularStdDev = angularStdDevBaseline.getAsDouble() * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor.getAsDouble();
          angularStdDev *= angularStdDevMegatag2Factor;
        }

        // Queue vision observation
        pendingVisionEstimates.add(new VisionEstimate(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
      }

      // Log camera metadata
      Logger.recordOutput(
          logKeyTagPoses[cameraIndex], tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          logKeyRobotPosesAccepted[cameraIndex],
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          logKeyRobotPosesRejected[cameraIndex],
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

      allTagPoses.addAll(tagPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Send newest observations first, capped at 5 per cycle
    pendingVisionEstimates.sort(
        Comparator.comparingDouble(VisionEstimate::timestampSeconds).reversed());
    pendingVisionEstimates.stream()
        .limit(5)
        .forEach(estimate -> consumer.accept(
            estimate.visionRobotPoseMeters(),
            estimate.timestampSeconds(),
            estimate.visionMeasurementStdDevs()));

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    Logger.recordOutput("Vision/Summary/TagStdevMultipliers", tagStdevMultipliersArray);
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
