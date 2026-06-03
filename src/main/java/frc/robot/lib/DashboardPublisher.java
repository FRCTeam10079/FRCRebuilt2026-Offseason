// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.statemachine.RobotStateMachine;

/**
 * Centralized NT4 publisher for Elastic Dashboard widgets.
 *
 * <p>Replaces scattered SmartDashboard.put*() calls with organized, typed NT4 publishers. All
 * topics live under /Robot/ for clean AdvantageScope-style organization.
 *
 * <p>Call {@link #periodic()} from Robot.robotPeriodic() every cycle.
 */
public class DashboardPublisher {

  private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();

  // ==================== STATE PUBLISHERS ====================

  private final StringPublisher matchStatePub =
      nt.getStringTopic("/Robot/State/MatchState").publish();
  private final BooleanPublisher robotEnabledPub =
      nt.getBooleanTopic("/Robot/State/RobotEnabled").publish();
  private final BooleanPublisher isAutonomousPub =
      nt.getBooleanTopic("/Robot/State/IsAutonomous").publish();
  private final StringPublisher gameStatePub =
      nt.getStringTopic("/Robot/State/GameState").publish();
  private final StringPublisher driveModePub =
      nt.getStringTopic("/Robot/State/DriveMode").publish();
  private final StringPublisher hubShiftStatePub =
      nt.getStringTopic("/Robot/State/HubShiftState").publish();
  private final BooleanPublisher myHubActivePub =
      nt.getBooleanTopic("/Robot/State/MyHubActive").publish();
  private final StringPublisher alliancePub =
      nt.getStringTopic("/Robot/State/Alliance").publish();
  private final BooleanPublisher isEndgamePub =
      nt.getBooleanTopic("/Robot/State/IsEndgame").publish();
  private final BooleanPublisher isTransitionPub =
      nt.getBooleanTopic("/Robot/State/IsTransition").publish();
  private final BooleanPublisher isCollectingPub =
      nt.getBooleanTopic("/Robot/State/IsCollecting").publish();
  private final BooleanPublisher isScoringPub =
      nt.getBooleanTopic("/Robot/State/IsScoring").publish();
  private final StringPublisher fuelStatePub =
      nt.getStringTopic("/Robot/State/FuelState").publish();

  // ==================== SHOOTER PUBLISHERS ====================

  private final BooleanPublisher hasFuelPub =
      nt.getBooleanTopic("/Robot/State/HasFuel").publish();
  private final BooleanPublisher readyToFirePub =
      nt.getBooleanTopic("/Robot/State/ReadyToFire").publish();
  private final BooleanPublisher shooterAtRPMPub =
      nt.getBooleanTopic("/Robot/Shooter/AtRPM").publish();
  private final BooleanPublisher alignedToTargetPub =
      nt.getBooleanTopic("/Robot/Shooter/Aligned").publish();

  // ==================== SMART SHOOT PUBLISHERS ====================

  private final StringPublisher smartShootStatusPub =
      nt.getStringTopic("/Robot/SmartShoot/Status").publish();
  private final DoublePublisher timeUntilActivePub =
      nt.getDoubleTopic("/Robot/SmartShoot/TimeUntilActive").publish();
  private final DoublePublisher timeUntilFirePub =
      nt.getDoubleTopic("/Robot/SmartShoot/TimeUntilFire").publish();
  private final StringPublisher shiftPhasePub =
      nt.getStringTopic("/Robot/SmartShoot/ShiftPhase").publish();

  // ==================== STATS PUBLISHERS ====================

  private final DoublePublisher matchTimePub =
      nt.getDoubleTopic("/Robot/MatchTime").publish();
  private final DoublePublisher batteryVoltagePub =
      nt.getDoubleTopic("/Robot/BatteryVoltage").publish();
  private final DoublePublisher totalFuelScoredPub =
      nt.getDoubleTopic("/Robot/Stats/TotalFuelScored").publish();
  private final DoublePublisher fuelScoredAutoPub =
      nt.getDoubleTopic("/Robot/Stats/FuelScoredAuto").publish();
  private final DoublePublisher fuelScoredTeleopPub =
      nt.getDoubleTopic("/Robot/Stats/FuelScoredTeleop").publish();
  private final DoublePublisher fastestCycleTimePub =
      nt.getDoubleTopic("/Robot/Stats/FastestCycleTime").publish();

  // ==================== DEPENDENCIES ====================

  private final RobotStateMachine stateMachine;
  private final HubShiftTracker hubShiftTracker;
  private SmartShootController smartShootController;

  // Throttle to ~10 Hz
  private static final int PUBLISH_DIVISOR = 5;
  private int cycleCount = 0;

  public DashboardPublisher(
      RobotStateMachine stateMachine,
      HubShiftTracker hubShiftTracker,
      SmartShootController smartShootController) {
    this.stateMachine = stateMachine;
    this.hubShiftTracker = hubShiftTracker;
    this.smartShootController = smartShootController;
  }

  /** Call from Robot.robotPeriodic(). Publishes all dashboard data at ~10 Hz. */
  public void periodic() {
    if (++cycleCount < PUBLISH_DIVISOR) return;
    cycleCount = 0;

    publishStateData();
    publishShooterData();
    publishSmartShootData();
    publishStatsData();
  }

  private void publishStateData() {
    matchStatePub.set(stateMachine.getMatchState().name());
    robotEnabledPub.set(stateMachine.isEnabled());
    isAutonomousPub.set(stateMachine.isAutonomous());
    gameStatePub.set(stateMachine.getGameState().name());
    driveModePub.set(stateMachine.getDrivetrainMode().name());

    // Hub shift - use HubShiftTracker for accurate state
    hubShiftStatePub.set(hubShiftTracker.getCurrentPhase().name());
    myHubActivePub.set(hubShiftTracker.isMyHubActive());

    alliancePub.set(DriverStation.getAlliance().map(Enum::name).orElse("UNKNOWN"));
    isEndgamePub.set(stateMachine.getMatchState() == frc.robot.statemachine.MatchState.ENDGAME);
    isTransitionPub.set(
        stateMachine.getMatchState() == frc.robot.statemachine.MatchState.TRANSITION_SHIFT);
    isCollectingPub.set(stateMachine.getGameState() == frc.robot.statemachine.GameState.COLLECTING);
    isScoringPub.set(stateMachine.getGameState() == frc.robot.statemachine.GameState.SCORING);
    fuelStatePub.set(stateMachine.getFuelState().name());
  }

  private void publishShooterData() {
    hasFuelPub.set(stateMachine.hasFuel());
    readyToFirePub.set(stateMachine.isReadyToFire());
    shooterAtRPMPub.set(stateMachine.isShooterAtRPM());
    alignedToTargetPub.set(stateMachine.isAlignedToTarget());
  }

  private void publishSmartShootData() {
    if (smartShootController != null) {
      smartShootStatusPub.set(smartShootController.getState().name());
      timeUntilFirePub.set(smartShootController.getTimeUntilFire());
    } else {
      smartShootStatusPub.set("N/A");
      timeUntilFirePub.set(-1.0);
    }

    timeUntilActivePub.set(hubShiftTracker.getTimeUntilHubActive());
    shiftPhasePub.set(hubShiftTracker.getCurrentPhase().name());
  }

  private void publishStatsData() {
    matchTimePub.set(DriverStation.getMatchTime());
    batteryVoltagePub.set(RobotController.getBatteryVoltage());
    totalFuelScoredPub.set(stateMachine.getTotalFuelScored());
    fuelScoredAutoPub.set(stateMachine.getFuelScoredAuto());
    fuelScoredTeleopPub.set(stateMachine.getFuelScoredTeleop());
    fastestCycleTimePub.set(stateMachine.getFastestCycleTime());
  }
}
