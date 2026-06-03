// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootOnTheMoveDrive;
import frc.robot.lib.LaunchCalculator.LaunchParameters;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.lib.SmartShootController;
import frc.robot.statemachine.ClimbState;
import frc.robot.statemachine.GameState;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure - the "brain" that coordinates all mechanism subsystems.
 *
 * <p>Buttons and auto routines set a {@link WantedSuperState}. Every cycle, {@link #periodic()}
 * translates that into the appropriate {@link CurrentSuperState} and drives each child subsystem's
 * wanted state accordingly.
 *
 * <p>This centralizes all mechanism coordination so DriverControls and OperatorControls only need
 * to express intent - not manage individual subsystems directly.
 */
public class Superstructure extends SubsystemBase {

  // ==================== STATE ENUMS ====================

  /** What the operator/auto wants the robot to do right now. */
  public enum WantedSuperState {
    /** Default - mechanisms off, pivot stays where it is. */
    IDLE,
    /** Deploy intake pivot + run wheels + index fuel. */
    COLLECT,
    /** Stow intake pivot + stop wheels. */
    STOW,
    /** Pre-spin flywheel + track shooter pivot angle (prepare to fire). */
    AIM,
    /** Full shoot: aim + feed indexer when flywheel, pivot, and heading are on-target. */
    SHOOT,
    /** Force-shoot: aim + feed as soon as flywheel is at speed (bypass heading/pivot gates). */
    FORCE_SHOOT,
    /** Reverse feeder/indexer without touching intake. */
    UNJAM,
    /** Shoot-on-the-move: velocity-compensated aiming via LaunchCalculator. */
    SOTM,
    /** Extend climber for endgame. */
    CLIMB,
    /** E-stop all mechanisms. */
    STOPPED
  }

  /** What the robot is actually doing this cycle (derived from wanted + sensor feedback). */
  public enum CurrentSuperState {
    IDLE,
    COLLECTING,
    STOWING,
    AIMING,
    /** Shooter spinning up, waiting for on-target conditions. */
    WAITING_FOR_TARGET,
    /** Actively feeding - all conditions met. */
    SHOOTING,
    /** Actively feeding - force-shoot bypass. */
    FORCE_SHOOTING,
    /** SOTM: flywheel spinning up / pivot tracking, waiting for on-target conditions. */
    SOTM_AIMING,
    /** SOTM: all conditions met, indexer feeding. */
    SOTM_SHOOTING,
    UNJAMMING,
    CLIMBING,
    STOPPED
  }

  // ==================== SUBSYSTEM REFERENCES ====================

  private final ShooterSubsystem shooter;
  private final ShooterPivotSubsystem shooterPivot;
  private final IndexerSubsystem indexer;
  private final IntakeWheelsSubsystem intake;
  private final PivotSubsystem pivot;
  private final ClimberSubsystem climber;

  // ==================== EXTERNAL DEPENDENCIES ====================

  private final RobotStateMachine stateMachine;
  private final Supplier<ShooterSetpoint> setpointSupplier;
  private final Supplier<Boolean> headingAlignedSupplier;
  private final SmartShootController smartShootController;
  private final Supplier<LaunchParameters> launchParametersSupplier;
  private final Supplier<Boolean> sotmHeadingAlignedSupplier;

  // ==================== STATE TRACKING ====================

  private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
  private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
  private CurrentSuperState previousSuperState = CurrentSuperState.IDLE;

  /**
   * When true, the Superstructure will NOT set the shooter pivot's wanted state, allowing a direct
   * command (manual control, homing) to take exclusive control.
   */
  private boolean shooterPivotOverride = false;

  /**
   * When true, intake wheels run and intake pivot deploys regardless of the current super-state.
   * This allows intaking to happen simultaneously with aiming, shooting, or SOTM actions.
   */
  private boolean intakeActive = false;

  /**
   * Hold-on timer for SOTM feeding. Once all on-target conditions are met the indexer feeds; if
   * conditions briefly flicker false the state machine holds SOTM_SHOOTING for up to this duration
   * before falling back to SOTM_AIMING. Replicates the 0.25 s kFalling debounce that was previously
   * in the DriverControls compound trigger.
   */
  private static final double SOTM_FEED_HOLD_DURATION = 0.25;

  private final Timer sotmFeedHoldTimer = new Timer();
  private boolean sotmFeedHoldActive = false;

  // ==================== CONSTRUCTOR ====================

  public Superstructure(
      ShooterSubsystem shooter,
      ShooterPivotSubsystem shooterPivot,
      IndexerSubsystem indexer,
      IntakeWheelsSubsystem intake,
      PivotSubsystem pivot,
      ClimberSubsystem climber,
      RobotStateMachine stateMachine,
      Supplier<ShooterSetpoint> setpointSupplier,
      Supplier<Boolean> headingAlignedSupplier,
      SmartShootController smartShootController,
      Supplier<LaunchParameters> launchParametersSupplier,
      Supplier<Boolean> sotmHeadingAlignedSupplier) {
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.indexer = indexer;
    this.intake = intake;
    this.pivot = pivot;
    this.climber = climber;
    this.stateMachine = stateMachine;
    this.setpointSupplier = setpointSupplier;
    this.headingAlignedSupplier = headingAlignedSupplier;
    this.smartShootController = smartShootController;
    this.launchParametersSupplier = launchParametersSupplier;
    this.sotmHeadingAlignedSupplier = sotmHeadingAlignedSupplier;
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    // Update smart shoot controller with current driver intent
    smartShootController.update(wantedSuperState == WantedSuperState.SHOOT);

    handleStateTransitions();
    applyStates();
    syncGameState();

    // Telemetry via AdvantageKit
    Logger.recordOutput("Superstructure/WantedState", wantedSuperState.name());
    Logger.recordOutput("Superstructure/CurrentState", currentSuperState.name());
    Logger.recordOutput("Superstructure/PreviousState", previousSuperState.name());
    Logger.recordOutput("Superstructure/ShooterPivotOverride", shooterPivotOverride);
    Logger.recordOutput(
        "Superstructure/SmartShoot/State", smartShootController.getState().name());
    Logger.recordOutput("Superstructure/SmartShoot/ShouldFeed", smartShootController.shouldFeed());
    Logger.recordOutput("Superstructure/IntakeActive", intakeActive);
  }

  // ==================== PUBLIC API ====================

  public void setWantedSuperState(WantedSuperState state) {
    this.wantedSuperState = state;
  }

  public WantedSuperState getWantedSuperState() {
    return wantedSuperState;
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public ShooterPivotSubsystem getShooterPivot() {
    return shooterPivot;
  }

  /**
   * Enable/disable shooter-pivot override. When enabled, the Superstructure will skip setting the
   * shooter pivot's wanted state so a direct command (manual control, homing) can take exclusive
   * control.
   */
  public void setShooterPivotOverride(boolean override) {
    this.shooterPivotOverride = override;
  }

  /** Enable or disable the independent intake overlay (intake runs alongside any main state). */
  public void setIntakeActive(boolean active) {
    this.intakeActive = active;
  }

  public boolean isIntakeActive() {
    return intakeActive;
  }

  /** Convenience: stop intake and stow the intake pivot. */
  public void stowIntake() {
    this.intakeActive = false;
    pivot.setWantedState(PivotSubsystem.WantedState.STOW);
  }

  /** Returns true when the Superstructure is in CLIMBING mode (endgame active). */
  public boolean isClimbing() {
    return currentSuperState == CurrentSuperState.CLIMBING;
  }

  // ==================== STATE TRANSITIONS ====================

  /**
   * Map the wanted super-state to the actual current super-state. Some wanted states branch based
   * on sensor feedback (e.g., SHOOT checks on-target conditions).
   */
  private void handleStateTransitions() {
    previousSuperState = currentSuperState;

    switch (wantedSuperState) {
      case IDLE -> currentSuperState = CurrentSuperState.IDLE;
      case COLLECT -> currentSuperState = CurrentSuperState.COLLECTING;
      case STOW -> currentSuperState = CurrentSuperState.STOWING;
      case AIM -> currentSuperState = CurrentSuperState.AIMING;
      case SHOOT -> {
        if (isOnTarget() && smartShootController.shouldFeed()) {
          currentSuperState = CurrentSuperState.SHOOTING;
        } else {
          currentSuperState = CurrentSuperState.WAITING_FOR_TARGET;
        }
      }
      case FORCE_SHOOT -> {
        if (isFlywheelReady() && !shooterPivot.isInTrenchZone()) {
          currentSuperState = CurrentSuperState.FORCE_SHOOTING;
        } else {
          // Still aiming - waiting for flywheel
          currentSuperState = CurrentSuperState.AIMING;
        }
      }
      case SOTM -> {
        if (isOnTargetSotm()) {
          // All conditions met - feed. Reset the hold-on timer.
          currentSuperState = CurrentSuperState.SOTM_SHOOTING;
          sotmFeedHoldTimer.restart();
          sotmFeedHoldActive = true;
        } else if (sotmFeedHoldActive
            && previousSuperState == CurrentSuperState.SOTM_SHOOTING
            && !sotmFeedHoldTimer.hasElapsed(SOTM_FEED_HOLD_DURATION)) {
          // Brief off-target flicker - hold feeding for up to SOTM_FEED_HOLD_DURATION.
          currentSuperState = CurrentSuperState.SOTM_SHOOTING;
        } else {
          // Not on-target and hold-on expired (or never started).
          currentSuperState = CurrentSuperState.SOTM_AIMING;
          sotmFeedHoldActive = false;
        }
      }
      case UNJAM -> currentSuperState = CurrentSuperState.UNJAMMING;
      case CLIMB -> currentSuperState = CurrentSuperState.CLIMBING;
      case STOPPED -> currentSuperState = CurrentSuperState.STOPPED;
    }
  }

  // ==================== APPLY STATES ====================

  /** Dispatch to per-state handler methods that set each child subsystem's wanted state. */
  private void applyStates() {
    // Detect state-change side effects
    // When leaving CLIMBING super state, only reset climber if it hasn't retracted
    if (previousSuperState == CurrentSuperState.CLIMBING
        && currentSuperState != CurrentSuperState.CLIMBING
        && !climber.isRetracted()) {
      climber.setWantedState(ClimberSubsystem.WantedState.IDLE);
    }

    switch (currentSuperState) {
      case IDLE -> applyIdle();
      case COLLECTING -> applyCollect();
      case STOWING -> applyStow();
      case AIMING -> applyAim();
      case WAITING_FOR_TARGET -> applyAim(); // Same outputs - still spinning up
      case SHOOTING -> applyShoot();
      case FORCE_SHOOTING -> applyForceShoot();
      case SOTM_AIMING -> applySotmAim();
      case SOTM_SHOOTING -> applySotmShoot();
      case UNJAMMING -> applyUnjam();
      case CLIMBING -> applyClimb();
      case STOPPED -> applyStopped();
    }

    // Independent intake overlay - runs alongside any main state
    applyIntakeOverlay();
  }

  // ==================== STATE HANDLERS ====================

  private void applyIdle() {
    shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
    trackPivotContinuously();
    indexer.setWantedState(IndexerSubsystem.WantedState.OFF);
    intake.setWantedState(IntakeWheelsSubsystem.WantedState.OFF);
    // Pivot: don't force stow in IDLE - use STOW state explicitly
  }

  private void applyCollect() {
    pivot.setWantedState(PivotSubsystem.WantedState.DEPLOY);
    intake.setWantedState(IntakeWheelsSubsystem.WantedState.INTAKE);
    indexer.setWantedState(
        IndexerSubsystem.WantedState.OFF); // Indexer only runs during shooting/feed, never intake.
    // Don't force shooter OFF here so collect can coexist with active spin-up
    // states (AIM/SOTM).
    trackPivotContinuously();
  }

  private void applyStow() {
    // STOW is pivot-only so it never interrupts active shooting/indexing behavior.
    pivot.setWantedState(PivotSubsystem.WantedState.STOW);
  }

  private void applyAim() {
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp != null && sp.isValid()) {
      shooter.setWantedState(ShooterSubsystem.WantedState.HOLD_RPM, sp.flywheelRPM());
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.TRACK_ANGLE, sp.pivotAngle());
      }
    } else {
      shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.IDLE);
      }
    }
    indexer.setWantedState(IndexerSubsystem.WantedState.OFF);
  }

  private void applyShoot() {
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp != null && sp.isValid()) {
      shooter.setWantedState(ShooterSubsystem.WantedState.HOLD_RPM, sp.flywheelRPM());
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.TRACK_ANGLE, sp.pivotAngle());
      }
    }
    // All conditions met - feed!
    indexer.setWantedState(IndexerSubsystem.WantedState.FEED);
  }

  private void applyForceShoot() {
    ShooterSetpoint sp = ShootOnTheMoveDrive.getShooterSetpointSupplier().get();
    if (sp != null && sp.isValid()) {
      shooter.setWantedState(ShooterSubsystem.WantedState.HOLD_RPM, sp.flywheelRPM());
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.TRACK_ANGLE, sp.pivotAngle());
      }
    }
    // Force feed - flywheel at speed is the only gate
    indexer.setWantedState(IndexerSubsystem.WantedState.FEED);
  }

  private void applyUnjam() {
    shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
    indexer.setWantedState(IndexerSubsystem.WantedState.REVERSE);
  }

  private void applyClimb() {
    // Climber state is driven by OperatorControls commands, not the Superstructure.
    // CLIMB is intentionally non-blocking: do not force other mechanisms off here.
  }

  private void applyStopped() {
    shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
    if (!shooterPivotOverride) {
      shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.IDLE);
    }
    indexer.setWantedState(IndexerSubsystem.WantedState.OFF);
    intake.setWantedState(IntakeWheelsSubsystem.WantedState.OFF);
    climber.setWantedState(ClimberSubsystem.WantedState.IDLE);
  }

  // ==================== INDEPENDENT INTAKE OVERLAY ====================

  /**
   * Independent intake overlay. When {@link #intakeActive} is true, intake wheels spin and the
   * intake pivot deploys regardless of the current super-state. Skipped for states that have their
   * own intake behavior (unjam, stopped, and the legacy COLLECT/STOW states used by auto).
   */
  private void applyIntakeOverlay() {
    if (currentSuperState == CurrentSuperState.UNJAMMING
        || currentSuperState == CurrentSuperState.STOPPED
        || currentSuperState == CurrentSuperState.COLLECTING
        || currentSuperState == CurrentSuperState.STOWING) {
      return;
    }

    if (intakeActive) {
      pivot.setWantedState(PivotSubsystem.WantedState.DEPLOY);
      intake.setWantedState(IntakeWheelsSubsystem.WantedState.INTAKE);
    } else {
      intake.setWantedState(IntakeWheelsSubsystem.WantedState.OFF);
    }
  }

  // ==================== CONTINUOUS PIVOT TRACKING ====================

  /**
   * Keep the shooter pivot tracking the distance-based angle at all times. Falls back to IDLE only
   * when there is no valid setpoint.
   */
  private void trackPivotContinuously() {
    if (shooterPivotOverride) return;
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp != null && sp.isValid()) {
      shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.TRACK_ANGLE, sp.pivotAngle());
    } else {
      shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.IDLE);
    }
  }

  // ==================== GAME STATE SYNC ====================

  /**
   * Keep the RobotStateMachine's GameState in sync with the Superstructure's current state. This
   * replaces the scattered stateMachine.setGameState() calls that were in DriverControls /
   * OperatorControls.
   */
  private void syncGameState() {
    if (!stateMachine.isEnabled()) return;

    // Sync ClimbState from the climber subsystem's internal state machine
    ClimbState climbDesired =
        switch (climber.getSystemState()) {
          case EXTENDING, EXTENDED, MANUAL -> ClimbState.APPROACHING;
          case RETRACTING -> ClimbState.CLIMBING_L1;
          case RETRACTED -> ClimbState.ENGAGED;
          default -> ClimbState.NOT_CLIMBING;
        };
    if (stateMachine.getClimbState() != climbDesired) {
      stateMachine.setClimbState(climbDesired);
    }

    // Sync GameState
    GameState desired =
        switch (currentSuperState) {
          case COLLECTING -> GameState.COLLECTING;
          case AIMING, SHOOTING, FORCE_SHOOTING, SOTM_AIMING, SOTM_SHOOTING -> GameState.SCORING;
          case WAITING_FOR_TARGET -> {
            // If SmartShoot is queued (hub inactive), don't override to SCORING
            // let the RobotStateMachine's HUB_INACTIVE state stand.
            if (smartShootController.getState() == SmartShootController.SmartShootState.QUEUED) {
              yield null;
            }
            yield GameState.SCORING;
          }
          case CLIMBING -> {
            if (climber.isRetracted()) {
              yield GameState.CLIMBED;
            }
            yield GameState.CLIMBING;
          }
          case UNJAMMING -> GameState.MANUAL_OVERRIDE;
          default -> intakeActive ? GameState.COLLECTING : null;
        };

    if (desired != null && stateMachine.getGameState() != desired) {
      stateMachine.setGameState(desired);
    }
  }

  // ==================== ON-TARGET CHECKS ====================

  /** Full on-target check: flywheel RPM + pivot angle + heading alignment. */
  private boolean isOnTarget() {
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp == null || !sp.isValid()) return false;

    boolean flywheelReady = shooter.isAt(sp.flywheelRPM());
    boolean pivotReady = shooterPivot.isAtAngle(sp.pivotAngle());
    boolean headingReady = headingAlignedSupplier.get();

    Logger.recordOutput("Superstructure/OnTarget/Flywheel", flywheelReady);
    Logger.recordOutput("Superstructure/OnTarget/Pivot", pivotReady);
    Logger.recordOutput("Superstructure/OnTarget/Heading", headingReady);
    Logger.recordOutput("Superstructure/OnTarget/All", flywheelReady && pivotReady && headingReady);

    return flywheelReady && pivotReady && headingReady;
  }

  /** Flywheel-only ready check (for force-shoot gate). */
  private boolean isFlywheelReady() {
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp == null || !sp.isValid()) return false;
    AngularVelocity targetRPM = sp.flywheelRPM();
    return targetRPM.gt(edu.wpi.first.units.Units.RPM.zero()) && shooter.isAt(targetRPM);
  }

  // ==================== SOTM STATE HANDLERS ====================

  /**
   * SOTM aiming: spin flywheel and track pivot from LaunchCalculator predictions. Indexer stays off
   * - waiting for all on-target conditions.
   */
  private void applySotmAim() {
    LaunchParameters params = launchParametersSupplier.get();
    if (params != null && params.isValid()) {
      shooter.setWantedState(ShooterSubsystem.WantedState.HOLD_RPM, RPM.of(params.flywheelRPM()));
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(
            ShooterPivotSubsystem.WantedState.TRACK_ANGLE, Degrees.of(params.pivotAngleDegrees()));
      }
    } else {
      shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.IDLE);
      }
    }
    indexer.setWantedState(IndexerSubsystem.WantedState.OFF);
  }

  /** SOTM shooting: same flywheel/pivot tracking as {@link #applySotmAim()}, plus indexer feeds. */
  private void applySotmShoot() {
    LaunchParameters params = launchParametersSupplier.get();
    if (params != null && params.isValid()) {
      shooter.setWantedState(ShooterSubsystem.WantedState.HOLD_RPM, RPM.of(params.flywheelRPM()));
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(
            ShooterPivotSubsystem.WantedState.TRACK_ANGLE, Degrees.of(params.pivotAngleDegrees()));
      }
    }
    indexer.setWantedState(IndexerSubsystem.WantedState.FEED);
  }

  // ==================== SOTM ON-TARGET CHECK ====================

  /**
   * SOTM on-target check: flywheel RPM + pivot angle + heading alignment using LaunchCalculator
   * predictions and the wider SOTM heading tolerance.
   *
   * <p>Each condition is evaluated into a separate variable (no short-circuit) so all gate
   * diagnostics are always logged.
   */
  private boolean isOnTargetSotm() {
    LaunchParameters params = launchParametersSupplier.get();
    if (params == null || !params.isValid()) {
      Logger.recordOutput("Superstructure/SOTM/OnTarget/Valid", false);
      return false;
    }

    boolean flywheelReady = params.flywheelRPM() > 0 && shooter.isReady();
    boolean pivotReady = shooterPivot.isAtAngle(Degrees.of(params.pivotAngleDegrees()));
    boolean headingReady = sotmHeadingAlignedSupplier.get();

    Logger.recordOutput("Superstructure/SOTM/OnTarget/Valid", true);
    Logger.recordOutput("Superstructure/SOTM/OnTarget/Flywheel", flywheelReady);
    Logger.recordOutput("Superstructure/SOTM/OnTarget/Pivot", pivotReady);
    Logger.recordOutput("Superstructure/SOTM/OnTarget/Heading", headingReady);
    Logger.recordOutput(
        "Superstructure/SOTM/OnTarget/All", flywheelReady && pivotReady && headingReady);

    return flywheelReady && pivotReady && headingReady;
  }
}
