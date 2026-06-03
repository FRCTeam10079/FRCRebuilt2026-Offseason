// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.GameConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Smart shoot controller that gates indexer feeding based on hub shift timing.
 *
 * <p>When the driver holds shoot in the scoring zone and the hub is inactive, this controller:
 *
 * <p>1. Keeps the flywheel spinning and pivot tracking (QUEUED state) 2. Does NOT allow the indexer
 * to feed 3. Predicts when the hub will become active using `HubShiftTracker` 4. Accounts for ball
 * time-of-flight from `ShooterInterpolationTable` 5. Releases the indexer at the exact moment so
 * the ball arrives when the hub activates
 *
 * <p>Outside the scoring zone, the controller allows normal shooting behavior.
 */
public class SmartShootController {

  public enum SmartShootState {
    /** No shoot request active. */
    IDLE,
    /** In scoring zone, hub inactive, not yet time to fire. Flywheel spinning, indexer stopped. */
    QUEUED,
    /** Time to fire - indexer should release NOW. Ball will arrive as hub activates. */
    PRE_FIRING,
    /** Hub is active - immediate fire allowed. */
    FIRING,
    /** Outside scoring zone - bypass smart shoot, use normal behavior. */
    BYPASSED
  }

  /** Time for ball to travel from indexer through shooter barrel (seconds). */
  private static final double INDEXER_TO_EXIT_DELAY = 0.12;

  /** Maximum distance from hub to be considered in scoring zone (meters). */
  private static final double MAX_SCORING_DISTANCE = 5.0;

  private final HubShiftTracker hubShiftTracker;
  private final Supplier<Pose2d> poseSupplier;

  private SmartShootState state = SmartShootState.IDLE;
  private boolean driverWantsToShoot = false;

  public SmartShootController(HubShiftTracker hubShiftTracker, Supplier<Pose2d> poseSupplier) {
    this.hubShiftTracker = hubShiftTracker;
    this.poseSupplier = poseSupplier;
  }

  /**
   * Call every cycle with the current driver shoot intent.
   *
   * @param wantsToShoot true if the driver is holding the shoot trigger
   */
  public void update(boolean wantsToShoot) {
    this.driverWantsToShoot = wantsToShoot;

    if (!wantsToShoot) {
      state = SmartShootState.IDLE;
      publishTelemetry();
      return;
    }

    Pose2d pose = poseSupplier.get();
    double distanceToHub = getDistanceToHub(pose);
    boolean inScoringZone = isInScoringZone(pose, distanceToHub);

    if (!inScoringZone) {
      // Outside scoring zone - bypass smart shoot, allow normal behavior
      state = SmartShootState.BYPASSED;
      publishTelemetry();
      return;
    }

    // In scoring zone - check hub state
    if (hubShiftTracker.isMyHubActive()) {
      // Hub is active right now - fire immediately
      state = SmartShootState.FIRING;
      publishTelemetry();
      return;
    }

    // Hub is INACTIVE - check if we should pre-fire based on TOF
    double timeUntilActive = hubShiftTracker.getTimeUntilHubActive();
    if (timeUntilActive < 0) {
      // Can't determine timing - stay queued
      state = SmartShootState.QUEUED;
      publishTelemetry();
      return;
    }

    double tof = ShooterInterpolationTable.getTimeOfFlight(distanceToHub);
    double totalFlightTime = tof + INDEXER_TO_EXIT_DELAY;

    if (timeUntilActive <= totalFlightTime) {
      // Fire NOW - ball will arrive exactly when hub activates
      state = SmartShootState.PRE_FIRING;
    } else {
      // Not yet time - stay queued (flywheel spinning, no feed)
      state = SmartShootState.QUEUED;
    }

    publishTelemetry();
  }

  /**
   * Whether the indexer should feed the ball.
   *
   * <p>Returns true in FIRING (hub active), PRE_FIRING (pre-fire timing), and BYPASSED (outside
   * scoring zone, normal behavior). Returns false in QUEUED (waiting) and IDLE.
   */
  public boolean shouldFeed() {
    return state == SmartShootState.FIRING
        || state == SmartShootState.PRE_FIRING
        || state == SmartShootState.BYPASSED;
  }

  /**
   * Whether the shooter should spin up (regardless of feed state).
   *
   * <p>Returns true whenever the driver wants to shoot, so the flywheel is ready.
   */
  public boolean shouldSpinUp() {
    return driverWantsToShoot;
  }

  public SmartShootState getState() {
    return state;
  }

  /** Get time until the auto-fire will happen. -1 if not queued. */
  public double getTimeUntilFire() {
    if (state != SmartShootState.QUEUED) return -1.0;

    double timeUntilActive = hubShiftTracker.getTimeUntilHubActive();
    if (timeUntilActive < 0) return -1.0;

    Pose2d pose = poseSupplier.get();
    double distanceToHub = getDistanceToHub(pose);
    double tof = ShooterInterpolationTable.getTimeOfFlight(distanceToHub);
    double totalFlightTime = tof + INDEXER_TO_EXIT_DELAY;

    return Math.max(0, timeUntilActive - totalFlightTime);
  }

  // ==================== SCORING ZONE LOGIC ====================

  /**
   * Determine if the robot is in the scoring zone.
   *
   * <p>Scoring zone: between the alliance wall and the hub X-coordinate, within range of hub. Blue
   * alliance: X < BLUE_HUB_CENTER.getX() (4.625m), Red: X > RED_HUB_CENTER.getX() (11.915m)
   */
  private boolean isInScoringZone(Pose2d pose, double distanceToHub) {
    if (distanceToHub > MAX_SCORING_DISTANCE) return false;

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return false;

    double robotX = pose.getX();
    if (alliance.get() == Alliance.Blue) {
      return robotX < GameConstants.BLUE_HUB_CENTER.getX();
    } else {
      return robotX > GameConstants.RED_HUB_CENTER.getX();
    }
  }

  private double getDistanceToHub(Pose2d pose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return Double.MAX_VALUE;

    Translation2d hubCenter = (alliance.get() == Alliance.Blue)
        ? GameConstants.BLUE_HUB_CENTER
        : GameConstants.RED_HUB_CENTER;

    return pose.getTranslation().getDistance(hubCenter);
  }

  private void publishTelemetry() {
    Logger.recordOutput("SmartShoot/State", state.name());
    Logger.recordOutput("SmartShoot/ShouldFeed", shouldFeed());
    Logger.recordOutput("SmartShoot/DriverWantsShoot", driverWantsToShoot);
    Logger.recordOutput("SmartShoot/TimeUntilFire", getTimeUntilFire());

    Pose2d pose = poseSupplier.get();
    double dist = getDistanceToHub(pose);
    Logger.recordOutput("SmartShoot/DistanceToHub", dist);
    Logger.recordOutput("SmartShoot/InScoringZone", isInScoringZone(pose, dist));
  }
}
