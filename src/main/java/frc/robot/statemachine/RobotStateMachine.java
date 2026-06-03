// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.lib.Elastic;
import frc.robot.lib.Elastic.NotificationLevel;
import frc.robot.lib.HubShiftTracker;
import frc.robot.lib.ShooterInterpolationTable;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * MASTER ROBOT STATE MACHINE - FRC 2026 REBUILT
 *
 * <p>Tracks the robot's lifecycle phase (MatchState), high-level strategy (GameState), drivetrain
 * control mode, hub-shift timing, climb progress, and fuel inventory.
 *
 * <p>Robot.java drives MatchState transitions; game-level states are set by commands/subsystems as
 * mechanisms come online. Telemetry is published to SmartDashboard every cycle.
 */
public class RobotStateMachine extends SubsystemBase {

  // ===== SINGLETON =====
  private static RobotStateMachine instance;

  public static RobotStateMachine getInstance() {
    if (instance == null) {
      instance = new RobotStateMachine();
    }
    return instance;
  }

  // ===== CURRENT STATE =====
  private MatchState matchState = MatchState.DISABLED;
  private DrivetrainMode driveMode = DrivetrainMode.FIELD_CENTRIC;
  private GameState gameState = GameState.IDLE;
  private FuelState fuelState = FuelState.EMPTY;
  private HubShiftState hubShiftState = HubShiftState.UNKNOWN;
  private ClimbState climbState = ClimbState.NOT_CLIMBING;

  // ===== TRACKING =====
  private boolean isAlignedToTarget = false;
  private boolean isShooterAtRPM = false;
  private double stateStartTime = 0;
  private double matchStartTime = 0;
  private MatchState previousMatchState = MatchState.DISABLED;
  private GameState previousGameState = GameState.IDLE;
  private int fuelCount = 0;

  // ===== CYCLE STATISTICS =====
  private int fuelScoredAuto = 0;
  private int fuelScoredTeleop = 0;
  private int intakeCyclesCompleted = 0;
  private int scoringCyclesCompleted = 0;
  private double lastCycleTime = 0;
  private double fastestCycleTime = Double.MAX_VALUE;

  // ===== STATE HISTORY =====
  private static final int STATE_HISTORY_SIZE = Constants.StateMachineConstants.STATE_HISTORY_SIZE;
  private final List<StateHistoryEntry> stateHistory = new ArrayList<>();

  // ===== DRIVER FEEDBACK =====
  private CommandXboxController driverController;
  private CommandXboxController operatorController;
  private double rumbleEndTime = 0;
  private boolean hasFiredCriticalTimeWarning = false;

  // ===== SUBSYSTEM POLLING =====
  private BooleanSupplier shooterReadySupplier = () -> false;
  private BooleanSupplier headingAlignedSupplier = () -> false;

  // Telemetry throttle — publish at ~5 Hz (every 10th cycle at 50 Hz)
  private static final int TELEMETRY_DIVISOR = 10;
  private int telemetryCycleCount = 0;
  private int eventSequence = 0;

  /** State history entry for debugging. */
  private record StateHistoryEntry(double timestamp, String type, String from, String to) {
    @Override
    public String toString() {
      return String.format("[%.2f] %s: %s -> %s", timestamp, type, from, to);
    }
  }

  // ==================== CONSTRUCTOR ====================

  private RobotStateMachine() {}

  // ==================== STATE TRANSITIONS ====================

  /** Transition match state (called by Robot.java). */
  public void setMatchState(MatchState newState) {
    if (matchState != newState) {
      previousMatchState = matchState;
      matchState = newState;
      stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      onMatchStateChange();
      logStateChange("Match", previousMatchState.name(), newState.name());
    }
  }

  /** Transition game state. */
  public void setGameState(GameState newState) {
    if (gameState != newState) {
      previousGameState = gameState;
      gameState = newState;
      stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      onGameStateChange();
      logStateChange("Game", previousGameState.name(), newState.name());
    }
  }

  /** Safe game state transition with validation. */
  public void requestGameState(GameState newState) {
    if (newState == GameState.EMERGENCY_STOP || canTransitionTo(newState)) {
      setGameState(newState);
    } else {
      System.err.println("INVALID STATE TRANSITION: " + gameState + " -> " + newState);
    }
  }

  /** Transition drivetrain mode. */
  public void setDrivetrainMode(DrivetrainMode newMode) {
    if (driveMode != newMode) {
      DrivetrainMode prev = driveMode;
      driveMode = newMode;
      logStateChange("Drivetrain", prev.name(), newMode.name());
    }
  }

  /** Set hub shift state. */
  public void setHubShiftState(HubShiftState newState) {
    if (hubShiftState != newState) {
      HubShiftState prev = hubShiftState;
      hubShiftState = newState;
      logStateChange("HubShift", prev.name(), newState.name());
      onHubShiftChange();
    }
  }

  /** Set climb state. */
  public void setClimbState(ClimbState newState) {
    if (climbState != newState) {
      ClimbState prev = climbState;
      climbState = newState;
      logStateChange("Climb", prev.name(), newState.name());

      if (newState == ClimbState.ENGAGED) {
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_MAX,
            Constants.StateMachineConstants.RUMBLE_EXTRA_LONG);
        recordEvent("=== CLIMB COMPLETE - BRAKE ENGAGED ===");
      } else if (newState == ClimbState.FAILED) {
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_MEDIUM_INTENSITY,
            Constants.StateMachineConstants.RUMBLE_LONG);
        recordEvent("!!! CLIMB FAILED - RECOVERY NEEDED !!!");
      }
    }
  }

  // ==================== INTERNAL TRANSITION LOGIC ====================

  private void onMatchStateChange() {
    switch (matchState) {
      case DISABLED -> {
        setGameState(GameState.IDLE);
        setDrivetrainMode(DrivetrainMode.DISABLED);
        isAlignedToTarget = false;
        isShooterAtRPM = false;
        climbState = ClimbState.NOT_CLIMBING;
      }
      case AUTO_INIT -> {
        setGameState(GameState.AUTO_RUNNING);
        setDrivetrainMode(DrivetrainMode.PATH_FOLLOWING);
        isAlignedToTarget = false;
        resetCycleCounters();
      }
      case TELEOP_INIT -> {
        setGameState(GameState.IDLE);
        setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);
        isAlignedToTarget = false;
        hubShiftState = HubShiftState.UNKNOWN;
        hasFiredCriticalTimeWarning = false;
      }
      case TRANSITION_SHIFT -> {
        setHubShiftState(HubShiftState.TRANSITION);
        setGameState(GameState.TRANSITION);
        recordEvent("=== TRANSITION SHIFT - BOTH HUBS ACTIVE! ===");
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_MAX,
            Constants.StateMachineConstants.RUMBLE_LONG);
      }
      case ENDGAME -> {
        recordEvent("=== ENDGAME PERIOD STARTED! ===");
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_STRONG,
            Constants.StateMachineConstants.RUMBLE_LONG);
      }
      default -> {}
    }
  }

  private void onGameStateChange() {
    switch (gameState) {
      case EMERGENCY_STOP -> setDrivetrainMode(DrivetrainMode.DISABLED);
      case CLIMBING, CLIMBED -> setDrivetrainMode(DrivetrainMode.LOCKED);
      case DEFENDING -> setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);
      case SCORING -> setDrivetrainMode(DrivetrainMode.VISION_TRACKING);
      default -> {} // Other states do not force a drivetrain mode
    }
  }

  private void onHubShiftChange() {
    switch (hubShiftState) {
      case MY_HUB_ACTIVE -> {
        if (!gameState.isEndgame() && !gameState.isClimbing()) {
          setGameState(GameState.HUB_ACTIVE);
          rumbleDriver(
              Constants.StateMachineConstants.RUMBLE_MEDIUM_INTENSITY,
              Constants.StateMachineConstants.RUMBLE_MEDIUM);
        }
      }
      case MY_HUB_INACTIVE -> {
        if (!gameState.isEndgame() && !gameState.isClimbing()) {
          setGameState(GameState.HUB_INACTIVE);
          rumbleDriver(
              Constants.StateMachineConstants.RUMBLE_LIGHT,
              Constants.StateMachineConstants.RUMBLE_SHORT);
        }
      }
      case TRANSITION ->
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_MAX,
            Constants.StateMachineConstants.RUMBLE_LONG);
      default -> {}
    }
  }

  /** Validate whether a game state transition is legal. */
  private boolean canTransitionTo(GameState newState) {
    if (matchState == MatchState.DISABLED && newState != GameState.IDLE) {
      return false;
    }
    if (newState.isAuto() && !matchState.autonomous) {
      return false;
    }
    return true;
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    checkPeriodTransitions();
    pollSubsystemState();
    updateRumble();

    // Throttle telemetry to ~5 Hz (every TELEMETRY_DIVISOR cycles)
    if (++telemetryCycleCount >= TELEMETRY_DIVISOR) {
      telemetryCycleCount = 0;
      updateTelemetry();
    }
  }

  /**
   * Poll registered subsystem suppliers to keep isShooterAtRPM and isAlignedToTarget in sync every
   * cycle. This fixes the original bug where these were never set from actual hardware state.
   */
  private void pollSubsystemState() {
    isShooterAtRPM = shooterReadySupplier.getAsBoolean();
    isAlignedToTarget = headingAlignedSupplier.getAsBoolean();
  }

  /** Detect endgame / transition periods and auto-update hub shift state from FMS data. */
  private void checkPeriodTransitions() {
    if (matchState == MatchState.TELEOP_RUNNING || matchState == MatchState.TRANSITION_SHIFT) {
      if (isEndgamePeriod()) {
        setMatchState(MatchState.ENDGAME);
      } else if (matchState == MatchState.TELEOP_RUNNING && isTransitionPeriod()) {
        setMatchState(MatchState.TRANSITION_SHIFT);
      }
    }

    // === Auto-detect hub shift state from HubShiftTracker (FMS data) ===
    if (isTeleop()) {
      HubShiftTracker tracker = HubShiftTracker.getInstance();
      if (!tracker.isOperatorOverrideActive()) {
        boolean hubActive = tracker.isMyHubActive();
        HubShiftState desired;
        HubShiftTracker.ShiftPhase phase = tracker.getCurrentPhase();

        if (phase == HubShiftTracker.ShiftPhase.TRANSITION
            || phase == HubShiftTracker.ShiftPhase.ENDGAME
            || phase == HubShiftTracker.ShiftPhase.AUTO) {
          desired = HubShiftState.TRANSITION;
        } else if (hubActive) {
          desired = HubShiftState.MY_HUB_ACTIVE;
        } else {
          desired = HubShiftState.MY_HUB_INACTIVE;
        }

        if (hubShiftState != desired) {
          setHubShiftState(desired);
          // Send Elastic notification on hub shift change
          if (desired == HubShiftState.MY_HUB_ACTIVE) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle("Hub ACTIVE")
                .withDescription("Your hub is now active - GO SCORE!")
                .withDisplaySeconds(3.0));
          } else if (desired == HubShiftState.MY_HUB_INACTIVE) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle("Hub INACTIVE")
                .withDescription("Your hub is inactive - collect fuel or defend")
                .withDisplaySeconds(3.0));
          }
        }
      }
    }

    // === Critical time warning (10 seconds remaining) ===
    if (isTeleop() && !hasFiredCriticalTimeWarning) {
      double t = DriverStation.getMatchTime();
      if (t > 0 && t <= Constants.GameConstants.CRITICAL_TIME_THRESHOLD) {
        hasFiredCriticalTimeWarning = true;
        recordEvent("!!! CRITICAL: 10 SECONDS REMAINING !!!");
        rumbleBoth(
            Constants.StateMachineConstants.RUMBLE_MAX,
            Constants.StateMachineConstants.RUMBLE_EXTRA_LONG);
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.ERROR)
            .withTitle("10 SECONDS!")
            .withDescription("Match ending soon - climb NOW!")
            .withDisplaySeconds(5.0));
      }
    }
  }

  // ==================== TELEMETRY ====================

  private void updateTelemetry() {
    // Match lifecycle
    Logger.recordOutput("Match State", matchState.name());
    Logger.recordOutput("Robot Enabled", matchState.enabled);
    Logger.recordOutput("Is Autonomous", matchState.autonomous);

    // Game strategy
    Logger.recordOutput("Game State", gameState.name());
    Logger.recordOutput("Game Description", gameState.description);

    // Drivetrain
    Logger.recordOutput("Drivetrain Mode", driveMode.description);

    // Hub shift
    Logger.recordOutput("Hub Shift State", hubShiftState.name());
    Logger.recordOutput("My Hub Active", hubShiftState == HubShiftState.MY_HUB_ACTIVE);

    // Climb
    Logger.recordOutput("Climb State", climbState.name());
    Logger.recordOutput("Climb Complete", climbState.isCompleted());

    // Fuel
    Logger.recordOutput("Fuel Count", fuelCount);
    Logger.recordOutput("Has Fuel", hasFuel());

    // Shooter / Alignment
    Logger.recordOutput("Aligned to Target", isAlignedToTarget);
    Logger.recordOutput("Shooter at RPM", isShooterAtRPM);
    Logger.recordOutput("Ready to Fire", isReadyToFire());

    // Alliance & Timing
    Logger.recordOutput("Alliance", DriverStation.getAlliance().map(Enum::name).orElse("UNKNOWN"));
    Logger.recordOutput("Time in State", getTimeInState());

    // Cycle stats
    Logger.recordOutput("Total Fuel Scored", getTotalFuelScored());
    Logger.recordOutput("Fastest Cycle Time", getFastestCycleTime());

    Logger.recordOutput("tofTable", ShooterInterpolationTable.getTimeOfFlight(1.0));
  }

  // ==================== GETTERS ====================

  public MatchState getMatchState() {
    return matchState;
  }

  public GameState getGameState() {
    return gameState;
  }

  public DrivetrainMode getDrivetrainMode() {
    return driveMode;
  }

  /**
   * Get the current alliance from DriverStation.
   *
   * @return the current alliance, or empty if unknown
   */
  public java.util.Optional<Alliance> getAlliance() {
    return DriverStation.getAlliance();
  }

  public FuelState getFuelState() {
    return fuelState;
  }

  public HubShiftState getHubShiftState() {
    return hubShiftState;
  }

  public ClimbState getClimbState() {
    return climbState;
  }

  public int getFuelCount() {
    return fuelCount;
  }

  public boolean isEnabled() {
    return matchState.enabled;
  }

  public boolean isAutonomous() {
    return matchState.autonomous;
  }

  public boolean isTeleop() {
    return matchState == MatchState.TELEOP_RUNNING
        || matchState == MatchState.TRANSITION_SHIFT
        || matchState == MatchState.ENDGAME;
  }

  public double getTimeInState() {
    return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - stateStartTime;
  }

  public boolean isEndgamePeriod() {
    return DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() > 0
        && DriverStation.getMatchTime() <= Constants.GameConstants.ENDGAME_THRESHOLD;
  }

  public boolean isTransitionPeriod() {
    if (!DriverStation.isTeleopEnabled()) return false;
    double t = DriverStation.getMatchTime();
    return t >= Constants.GameConstants.TRANSITION_END_TIME
        && t <= Constants.GameConstants.TRANSITION_START_TIME;
  }

  // ==================== ALIGNMENT / SHOOTER TRACKING ====================

  public void setAlignedToTarget(boolean aligned) {
    if (isAlignedToTarget != aligned) {
      isAlignedToTarget = aligned;
      if (aligned) {
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_LIGHT,
            Constants.StateMachineConstants.RUMBLE_SHORT);
      }
    }
  }

  public boolean isAlignedToTarget() {
    return isAlignedToTarget;
  }

  public void setShooterAtRPM(boolean atRPM) {
    isShooterAtRPM = atRPM;
  }

  public boolean isShooterAtRPM() {
    return isShooterAtRPM;
  }

  public boolean isReadyToFire() {
    return hasFuel()
        && isAlignedToTarget
        && isShooterAtRPM
        && (hubShiftState == HubShiftState.MY_HUB_ACTIVE
            || hubShiftState == HubShiftState.TRANSITION);
  }

  // ==================== FUEL MANAGEMENT ====================

  public boolean hasFuel() {
    return fuelCount > 0;
  }

  public boolean isEmpty() {
    return fuelCount == 0;
  }

  public void setFuelState(FuelState newState) {
    if (fuelState != newState) {
      FuelState prev = fuelState;
      fuelState = newState;
      addToStateHistory("Fuel", prev.name(), newState.name());

      if (newState == FuelState.LOADED) {
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_LIGHT,
            Constants.StateMachineConstants.RUMBLE_SHORT);
        intakeCyclesCompleted++;
      } else if (newState == FuelState.EMPTY && prev == FuelState.FIRING) {
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_MAX,
            Constants.StateMachineConstants.RUMBLE_LONG);
        if (isAutonomous()) {
          fuelScoredAuto += fuelCount;
        } else {
          fuelScoredTeleop += fuelCount;
        }
        scoringCyclesCompleted++;
        updateCycleTime();
      }
    }
  }

  public void setFuelCount(int count) {
    fuelCount = Math.max(0, count);
    if (fuelCount == 0) {
      setFuelState(FuelState.EMPTY);
    } else {
      setFuelState(FuelState.LOADED);
    }
  }

  public void addFuel(int amount) {
    setFuelCount(fuelCount + amount);
  }

  public void removeFuel(int amount) {
    setFuelCount(fuelCount - amount);
  }

  // ==================== DRIVER FEEDBACK ====================

  public void registerControllers(CommandXboxController driver, CommandXboxController operator) {
    this.driverController = driver;
    this.operatorController = operator;
  }

  /**
   * Register subsystem suppliers so the state machine can poll shooter/alignment status each cycle.
   * This replaces the old approach where isShooterAtRPM was never set from anywhere.
   *
   * @param shooterReady supplier from ShooterSubsystem.isReady()
   * @param headingAligned supplier that returns true when drivetrain heading is on target
   */
  public void registerShooterSuppliers(
      BooleanSupplier shooterReady, BooleanSupplier headingAligned) {
    this.shooterReadySupplier = shooterReady;
    this.headingAlignedSupplier = headingAligned;
  }

  public void rumbleDriver(double intensity, double durationSeconds) {
    if (driverController != null) {
      driverController.getHID().setRumble(RumbleType.kBothRumble, intensity);
      rumbleEndTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() + durationSeconds;
    }
  }

  public void rumbleOperator(double intensity, double durationSeconds) {
    if (operatorController != null) {
      operatorController.getHID().setRumble(RumbleType.kBothRumble, intensity);
      rumbleEndTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() + durationSeconds;
    }
  }

  /** Rumble both controllers simultaneously. */
  public void rumbleBoth(double intensity, double durationSeconds) {
    rumbleDriver(intensity, durationSeconds);
    rumbleOperator(intensity, durationSeconds);
  }

  private void updateRumble() {
    if (rumbleEndTime > 0 && edu.wpi.first.wpilibj.Timer.getFPGATimestamp() >= rumbleEndTime) {
      if (driverController != null) {
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
      if (operatorController != null) {
        operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
      rumbleEndTime = 0;
    }
  }

  // ==================== STATE HISTORY ====================

  private void recordEvent(String message) {
    Logger.recordOutput("Events/StateMachine/Last", message);
    Logger.recordOutput("Events/StateMachine/Sequence", ++eventSequence);
  }

  private void logStateChange(String type, String from, String to) {
    String message = String.format("%s State: %s -> %s", type, from, to);
    recordEvent(message);
    Logger.recordOutput("LastStateChange", message);
    addToStateHistory(type, from, to);
  }

  private void addToStateHistory(String type, String from, String to) {
    stateHistory.add(
        new StateHistoryEntry(edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), type, from, to));
    if (stateHistory.size() > STATE_HISTORY_SIZE) {
      stateHistory.remove(0);
    }
  }

  public List<StateHistoryEntry> getStateHistory() {
    return new ArrayList<>(stateHistory);
  }

  public void printStateHistory() {
    recordEvent("=== STATE HISTORY ===");
    stateHistory.forEach(entry -> recordEvent(entry.toString()));
    recordEvent("====================");
  }

  // ==================== CYCLE STATISTICS ====================

  private void updateCycleTime() {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (lastCycleTime > 0) {
      double cycleTime = now - lastCycleTime;
      if (cycleTime < fastestCycleTime
          && cycleTime > Constants.StateMachineConstants.MIN_VALID_CYCLE_TIME) {
        fastestCycleTime = cycleTime;
      }
    }
    lastCycleTime = now;
  }

  public int getFuelScoredAuto() {
    return fuelScoredAuto;
  }

  public int getFuelScoredTeleop() {
    return fuelScoredTeleop;
  }

  public int getTotalFuelScored() {
    return fuelScoredAuto + fuelScoredTeleop;
  }

  public int getIntakeCyclesCompleted() {
    return intakeCyclesCompleted;
  }

  public int getScoringCyclesCompleted() {
    return scoringCyclesCompleted;
  }

  public double getFastestCycleTime() {
    return fastestCycleTime == Double.MAX_VALUE ? 0 : fastestCycleTime;
  }

  public double getMatchElapsedTime() {
    if (matchStartTime == 0) return 0;
    return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - matchStartTime;
  }

  public void resetCycleCounters() {
    fuelScoredAuto = 0;
    fuelScoredTeleop = 0;
    intakeCyclesCompleted = 0;
    scoringCyclesCompleted = 0;
    lastCycleTime = 0;
    fastestCycleTime = Double.MAX_VALUE;
    fuelCount = 0;
    stateHistory.clear();
    matchStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }
}
