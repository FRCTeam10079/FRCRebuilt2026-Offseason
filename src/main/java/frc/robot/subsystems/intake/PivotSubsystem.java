// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  // ==================== STATE MACHINE ====================

  public enum WantedState {
    DEPLOY,
    STOW,
    IDLE
  }

  private enum SystemState {
    DEPLOYING,
    DEPLOYED,
    STOWING,
    STOWED,
    STALLED,
    IDLE
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  private Angle pivotSetpoint;

  // Stall detection
  private final Timer stallTimer = new Timer();

  // Idle detection — go to neutral after holding setpoint for a while
  private final Timer atSetpointTimer = new Timer();

  public PivotSubsystem(PivotIO io) {
    this.io = io;
    pivotSetpoint = getPivotPosition();
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    systemState = handleStateTransitions();
    applyStates();

    Logger.recordOutput("IntakePivot/WantedState", wantedState);
    Logger.recordOutput("IntakePivot/SystemState", systemState);
    Logger.recordOutput("IntakePivot/setpoint", pivotSetpoint.in(Rotations));
    Logger.recordOutput("IntakePivot/position", getPivotPosition().in(Rotations));
    Logger.recordOutput("IntakePivot/velocityRPS", inputs.velocityRPS);
    Logger.recordOutput(
        "IntakePivot/closedLoopReferenceRotations", inputs.closedLoopReferenceRotations);
    Logger.recordOutput("IntakePivot/closedLoopErrorRotations", inputs.closedLoopErrorRotations);
    Logger.recordOutput("IntakePivot/motionMagicAtTarget", inputs.motionMagicAtTarget);
    Logger.recordOutput("IntakePivot/motionMagicIsRunning", inputs.motionMagicIsRunning);
    Logger.recordOutput("IntakePivot/reachedSetpoint", reachedSetpoint());
    Logger.recordOutput("IntakePivot/statorCurrent", inputs.statorCurrentAmps);
    Logger.recordOutput(
        "IntakePivot/stallCurrentExceeded",
        Amps.of(inputs.statorCurrentAmps).gt(IntakeConstants.Pivot.STALL_CURRENT_THRESHOLD));
    Logger.recordOutput("IntakePivot/stallTimerSeconds", stallTimer.get());
    Logger.recordOutput("IntakePivot/atSetpointTimerSeconds", atSetpointTimer.get());
    Logger.recordOutput("IntakePivot/faultField", inputs.faultField);
  }

  // ==================== STATE TRANSITIONS ====================

  private SystemState handleStateTransitions() {
    // When robot is disabled, force IDLE to prevent motors from trying to hold
    // position
    if (DriverStation.isDisabled()) {
      stallTimer.stop();
      stallTimer.reset();
      atSetpointTimer.stop();
      atSetpointTimer.reset();
      return SystemState.IDLE;
    }

    switch (wantedState) {
      case DEPLOY:
        pivotSetpoint = IntakeConstants.Pivot.INTAKE_POSITION;
        stallTimer.stop();
        stallTimer.reset();
        if (reachedSetpoint()) {
          if (!atSetpointTimer.isRunning()) {
            atSetpointTimer.start();
          }
          if (atSetpointTimer.hasElapsed(
              IntakeConstants.Pivot.IDLE_DEBOUNCE_TIME.in(edu.wpi.first.units.Units.Seconds))) {
            return SystemState.DEPLOYED;
          }
          return SystemState.DEPLOYING;
        }
        atSetpointTimer.stop();
        atSetpointTimer.reset();
        return SystemState.DEPLOYING;

      case STOW:
        pivotSetpoint = IntakeConstants.Pivot.STOWED_POSITION;
        atSetpointTimer.stop();
        atSetpointTimer.reset();

        if (reachedSetpoint()) {
          stallTimer.stop();
          stallTimer.reset();
          return SystemState.STOWED;
        }

        if (hasStowStalled()) {
          pivotSetpoint = getPivotPosition();
          return SystemState.STALLED;
        }
        return SystemState.STOWING;

      case IDLE:
      default:
        stallTimer.stop();
        stallTimer.reset();
        atSetpointTimer.stop();
        atSetpointTimer.reset();
        return SystemState.IDLE;
    }
  }

  private boolean hasStowStalled() {
    if (Amps.of(inputs.statorCurrentAmps).gt(IntakeConstants.Pivot.STALL_CURRENT_THRESHOLD)) {
      if (!stallTimer.isRunning()) {
        stallTimer.start();
      }
      return stallTimer.hasElapsed(
          IntakeConstants.Pivot.STALL_TIME_THRESHOLD.in(edu.wpi.first.units.Units.Seconds));
    }

    stallTimer.stop();
    stallTimer.reset();
    return false;
  }

  private void applyStates() {
    switch (systemState) {
      case DEPLOYING:
      case STOWING:
      // Intentionally keep holding STOWED with Motion Magic so the pivot does not
      // sag/flop out during acceleration; NeutralOut is only safe when deployed; I
      // think...
      case DEPLOYED: // INTENTIONAL KEEP FORCE DOWNWARDS WHEN DEPLOYED (its okay)
      case STOWED:
        io.setMotionMagicPosition(pivotSetpoint);
        break;
      case IDLE:
      case STALLED:
        io.setNeutral();
        break;
    }
  }

  // ==================== PUBLIC API ====================

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public boolean isStalled() {
    return systemState == SystemState.STALLED;
  }

  public boolean reachedSetpoint() {
    return getPivotPosition().isNear(pivotSetpoint, IntakeConstants.Pivot.DEPLOY_TOLERANCE);
  }

  public boolean isDeployed() {
    return systemState == SystemState.DEPLOYED
        || systemState == SystemState.DEPLOYING && reachedSetpoint();
  }

  public boolean isStowed() {
    return systemState == SystemState.STOWED;
  }

  public Angle getPivotPosition() {
    return Rotations.of(inputs.positionRotations);
  }

  // ==================== COMMAND FACTORIES ====================

  public Command deployCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> setWantedState(WantedState.DEPLOY)),
            Commands.waitUntil(this::reachedSetpoint))
        .withName("Pivot Deploy");
  }

  public Command stowCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> setWantedState(WantedState.STOW)),
            Commands.waitUntil(() -> reachedSetpoint() || isStalled()))
        .withName("Pivot Stow");
  }

  // ==================== TELEMETRY ====================

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getStatorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  public double getMotorVoltageVolts() {
    return inputs.voltageVolts;
  }
}
