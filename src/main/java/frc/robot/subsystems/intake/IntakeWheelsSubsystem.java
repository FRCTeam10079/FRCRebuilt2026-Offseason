// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeWheelsSubsystem extends SubsystemBase {

  private final IntakeWheelsIO io;
  private final IntakeWheelsIOInputsAutoLogged inputs = new IntakeWheelsIOInputsAutoLogged();

  // ==================== STATE MACHINE ====================

  public enum WantedState {
    OFF,
    INTAKE,
    EJECT,
    REVERSE
  }

  private enum SystemState {
    IDLE,
    INTAKING,
    EJECTING,
    REVERSING
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.IDLE;
  private double targetVelocityRPS = 0.0;

  public IntakeWheelsSubsystem(IntakeWheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeWheels", inputs);

    systemState = handleStateTransitions();
    applyStates();

    Logger.recordOutput("IntakeWheels/WantedState", wantedState);
    Logger.recordOutput("IntakeWheels/SystemState", systemState);
    Logger.recordOutput("IntakeWheels/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("IntakeWheels/MeasuredVelocityRPS", inputs.velocityRPS);
    Logger.recordOutput("IntakeWheels/VelocityErrorRPS", targetVelocityRPS - inputs.velocityRPS);
    Logger.recordOutput("IntakeWheels/SlaveVelocityRPS", inputs.slaveVelocityRPS);
  }

  // ==================== STATE TRANSITIONS ====================

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case INTAKE -> SystemState.INTAKING;
      case EJECT, REVERSE -> SystemState.REVERSING;
      case OFF -> SystemState.IDLE;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case INTAKING:
        targetVelocityRPS = IntakeConstants.Wheels.INTAKE_IN_RPM / 60.0;
        io.setVelocity(targetVelocityRPS);
        break;
      case REVERSING:
        targetVelocityRPS = IntakeConstants.Wheels.INTAKE_OUT_RPM / 60.0;
        io.setVelocity(targetVelocityRPS);
        break;
      case IDLE:
      default:
        targetVelocityRPS = 0.0;
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

  // ==================== COMMAND FACTORIES ====================

  public Command intakeInCommand() {
    return startEnd(() -> setWantedState(WantedState.INTAKE), () -> setWantedState(WantedState.OFF))
        .withName("Intake In");
  }

  public Command intakeOutCommand() {
    return startEnd(() -> setWantedState(WantedState.EJECT), () -> setWantedState(WantedState.OFF))
        .withName("Intake Out");
  }

  public Command stopCommand() {
    return runOnce(() -> setWantedState(WantedState.OFF)).withName("Intake Stop");
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
