// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ClimberSubsystem extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert climberMotorDisconnectedAlert =
      new Alert("Climber motor disconnected, climb may fail", AlertType.kError);

  // ==================== TUNABLE POSITIONS (NetworkTables) ====================
  private static final LoggedNetworkNumber extendPosition = new LoggedNetworkNumber(
      "/Tuning/Climb/ExtendPositionRotations", ClimberConstants.EXTEND_POSITION_ROTATIONS);
  private static final LoggedNetworkNumber retractPosition = new LoggedNetworkNumber(
      "/Tuning/Climb/RetractPositionRotations", ClimberConstants.RETRACT_POSITION_ROTATIONS);
  private static final LoggedNetworkNumber positionTolerance = new LoggedNetworkNumber(
      "/Tuning/Climb/PositionToleranceRotations", ClimberConstants.POSITION_TOLERANCE_ROTATIONS);

  // ==================== STATE MACHINE ====================

  public enum WantedState {
    IDLE,
    CLIMB,
    RETRACT,
    MANUAL,
    ABORT
  }

  public enum SystemState {
    IDLE,
    EXTENDING,
    EXTENDED,
    RETRACTING,
    RETRACTED,
    MANUAL
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  // ==================== MANUAL CONTROL ====================
  private double manualVoltage = 0.0;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (RobotBase.isReal()) {
      climberMotorDisconnectedAlert.set(!inputs.motorConnected);
    }

    SystemState previousState = systemState;
    systemState = handleStateTransitions(previousState);
    applyStates();

    Logger.recordOutput("Climber/WantedState", wantedState.name());
    Logger.recordOutput("Climber/SystemState", systemState.name());
    Logger.recordOutput("Climber/PositionRotations", inputs.positionRotations);
    Logger.recordOutput("Climber/MotorConnected", inputs.motorConnected);
    Logger.recordOutput("Climber/AppliedVoltage", inputs.appliedVoltage);
    Logger.recordOutput("Climber/StatorCurrentAmps", inputs.statorCurrentAmps);
    Logger.recordOutput("Climber/VelocityRPS", inputs.velocityRPS);
    Logger.recordOutput("Climber/DutyCycle", inputs.dutyCycle);
    Logger.recordOutput("Climber/SupplyVoltage", inputs.supplyVoltage);
    Logger.recordOutput("Climber/SupplyCurrentAmps", inputs.supplyCurrentAmps);
    Logger.recordOutput("Climber/TempCelsius", inputs.tempCelsius);

    Logger.recordOutput("Climber/ExtendTargetRotations", extendPosition.get());
    Logger.recordOutput("Climber/RetractTargetRotations", retractPosition.get());
    Logger.recordOutput("Climber/ManualVoltage", manualVoltage);
  }

  // ==================== STATE TRANSITIONS ====================

  private SystemState handleStateTransitions(SystemState previous) {
    if (wantedState == WantedState.ABORT) {
      wantedState = WantedState.IDLE;
      return SystemState.IDLE;
    }

    if (wantedState == WantedState.IDLE
        && previous != SystemState.EXTENDING
        && previous != SystemState.EXTENDED
        && previous != SystemState.RETRACTING) {
      return SystemState.IDLE;
    }

    if (wantedState == WantedState.MANUAL) {
      return SystemState.MANUAL;
    }

    if (wantedState == WantedState.CLIMB) {
      switch (previous) {
        case IDLE:
        case RETRACTED:
          return SystemState.EXTENDING;

        case EXTENDING:
          if (inputs.positionRotations >= extendPosition.get() - positionTolerance.get()) {
            return SystemState.EXTENDED;
          }
          return SystemState.EXTENDING;

        case EXTENDED:
          return SystemState.EXTENDED;

        case RETRACTING:
          if (inputs.positionRotations <= retractPosition.get() + positionTolerance.get()) {
            return SystemState.RETRACTED;
          }
          return SystemState.RETRACTING;

        case MANUAL:
          return SystemState.EXTENDING;

        default:
          return previous;
      }
    }

    if (wantedState == WantedState.RETRACT) {
      if (previous == SystemState.EXTENDED || previous == SystemState.EXTENDING) {
        return SystemState.RETRACTING;
      }
      if (previous == SystemState.RETRACTING) {
        if (inputs.positionRotations <= retractPosition.get() + positionTolerance.get()) {
          wantedState = WantedState.IDLE;
          return SystemState.RETRACTED;
        }
        return SystemState.RETRACTING;
      }
      return previous;
    }

    if (previous == SystemState.RETRACTED) {
      if (wantedState == WantedState.IDLE) {
        return SystemState.IDLE;
      }
      return SystemState.RETRACTED;
    }

    return previous;
  }

  private void applyStates() {
    switch (systemState) {
      case EXTENDING:
        io.setVoltage(ClimberConstants.EXTEND_VOLTAGE);
        break;
      case RETRACTING:
        io.setVoltage(ClimberConstants.RETRACT_VOLTAGE);
        break;
      case MANUAL:
        io.setVoltage(manualVoltage);
        break;
      case EXTENDED:
      case RETRACTED:
      case IDLE:
      default:
        io.stop();
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

  public SystemState getSystemState() {
    return systemState;
  }

  public void setManualVoltage(double voltage) {
    this.manualVoltage = MathUtil.clamp(
        voltage, ClimberConstants.PEAK_REVERSE_VOLTAGE, ClimberConstants.PEAK_FORWARD_VOLTAGE);
  }

  public boolean isExtended() {
    return systemState == SystemState.EXTENDED;
  }

  public boolean isRetracted() {
    return systemState == SystemState.RETRACTED;
  }

  public boolean isIdle() {
    return systemState == SystemState.IDLE;
  }

  public double getPositionRotations() {
    return inputs.positionRotations;
  }

  // ==================== COMMAND FACTORIES ====================

  public Command extendCommand() {
    return run(() -> setWantedState(WantedState.CLIMB))
        .until(this::isExtended)
        .withName("Climber Extend");
  }

  public Command retractCommand() {
    return run(() -> setWantedState(WantedState.RETRACT))
        .until(this::isRetracted)
        .finallyDo(interrupted -> {
          if (interrupted) {
            setWantedState(WantedState.IDLE);
          }
        })
        .withName("Climber Retract");
  }

  public Command retractToZeroCommand() {
    // Drive toward encoder position 0 using retract voltage, stop when close.
    return run(() -> {
          setWantedState(WantedState.MANUAL);
          setManualVoltage(ClimberConstants.RETRACT_VOLTAGE);
        })
        .until(() -> inputs.positionRotations <= positionTolerance.get())
        .finallyDo(interrupted -> {
          setManualVoltage(0.0);
          setWantedState(WantedState.IDLE);
        })
        .withName("Climber Retract To Zero");
  }

  public Command climbCommand() {
    return Commands.sequence(extendCommand(), retractCommand()).withName("Climber Full Climb");
  }

  public Command manualControlCommand(DoubleSupplier stickInput) {
    return run(() -> {
          setWantedState(WantedState.MANUAL);
          setManualVoltage(stickInput.getAsDouble() * ClimberConstants.EXTEND_VOLTAGE);
        })
        .finallyDo(interrupted -> {
          setManualVoltage(0.0);
          setWantedState(WantedState.IDLE);
        })
        .withName("Climber Manual Control");
  }

  public Command abortCommand() {
    return Commands.runOnce(() -> setWantedState(WantedState.ABORT), this)
        .withName("Climber Abort");
  }

  public Command stopCommand() {
    return abortCommand();
  }
}
