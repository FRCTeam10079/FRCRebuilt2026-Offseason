// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // ==================== STATE MACHINE ====================

  public enum WantedState {
    OFF,
    SPIN_UP,
    HOLD_RPM,
    SHOOT
  }

  private enum SystemState {
    IDLE,
    SPINNING_UP,
    AT_SPEED,
    SHOOTING
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.IDLE;

  private AngularVelocity targetRPM = RPM.zero();

  // Slew rate limiter: prevents abrupt flywheel speed changes.
  // Rate is in RPM/second. 3000 RPM/s means the flywheel can ramp from 0 to 3000
  // RPM in 1s.
  // Adapted from MA (6328). Adjust to match our motor/gearing response.
  private static final double FLYWHEEL_SLEW_RATE_RPM_PER_SEC = 3000.0;
  private final SlewRateLimiter flywheelSlew = new SlewRateLimiter(FLYWHEEL_SLEW_RATE_RPM_PER_SEC);
  private double slewedTargetRPM = 0.0;

  // Stability tracking with debouncing
  private int stabilityCounter = 0;

  private final SysIdRoutine sysIdRoutine;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    systemState = handleStateTransitions();
    applyStates();

    Logger.recordOutput("Shooter/WantedState", wantedState);
    Logger.recordOutput("Shooter/SystemState", systemState);
    Logger.recordOutput("Shooter/TargetRPM", targetRPM.in(RPM));
    Logger.recordOutput("Shooter/SlewedTargetRPM", slewedTargetRPM);
    Logger.recordOutput("Shooter/CurrentRPM", getCurrentRPM().in(RPM));
    Logger.recordOutput(
        "Shooter/ClosedLoopReferenceRPM",
        RotationsPerSecond.of(inputs.masterClosedLoopReferenceRPS).in(RPM));
    Logger.recordOutput(
        "Shooter/ClosedLoopErrorRPM",
        RotationsPerSecond.of(inputs.masterClosedLoopErrorRPS).in(RPM));
    Logger.recordOutput("Shooter/IsReady", isReady());
    Logger.recordOutput("Shooter/StabilityCounter", stabilityCounter);
    Logger.recordOutput("Shooter/MasterFaultField", inputs.masterFaultField);
    Logger.recordOutput("Shooter/SlaveFaultField", inputs.slaveFaultField);
  }

  // ==================== STATE TRANSITIONS ====================

  private SystemState handleStateTransitions() {
    AngularVelocity currentRPM = getCurrentRPM();

    switch (wantedState) {
      case SPIN_UP:
      case HOLD_RPM:
        if (targetRPM.gt(RPM.zero())) {
          if (currentRPM.isNear(targetRPM, ShooterConstants.SHOOTER_SPEED_TOLERANCE)) {
            stabilityCounter =
                Math.min(stabilityCounter + 1, ShooterConstants.STABILITY_CYCLES_REQUIRED);
          } else {
            stabilityCounter = 0;
          }
          if (stabilityCounter >= ShooterConstants.STABILITY_CYCLES_REQUIRED) {
            return SystemState.AT_SPEED;
          }
          return SystemState.SPINNING_UP;
        }
        return SystemState.IDLE;

      case SHOOT:
        if (targetRPM.gt(RPM.zero())) {
          if (currentRPM.isNear(targetRPM, ShooterConstants.SHOOTER_SPEED_TOLERANCE)) {
            stabilityCounter =
                Math.min(stabilityCounter + 1, ShooterConstants.STABILITY_CYCLES_REQUIRED);
          } else {
            stabilityCounter = 0;
          }
          return SystemState.SHOOTING;
        }
        return SystemState.IDLE;

      case OFF:
      default:
        stabilityCounter = 0;
        return SystemState.IDLE;
    }
  }

  private void applyStates() {
    switch (systemState) {
      case SPINNING_UP:
      case AT_SPEED:
      case SHOOTING:
        slewedTargetRPM = flywheelSlew.calculate(targetRPM.in(RPM));
        io.setVelocity(RPM.of(slewedTargetRPM).in(RotationsPerSecond));
        break;
      case IDLE:
      default:
        flywheelSlew.reset(0.0);
        slewedTargetRPM = 0.0;
        io.stop();
        break;
    }
  }

  // ==================== PUBLIC API ====================

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public void setWantedState(WantedState state, AngularVelocity rpm) {
    this.wantedState = state;
    this.targetRPM = Constants.clamp(rpm, RPM.zero(), ShooterConstants.SHOOTER_MAX_SPEED);
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public SystemState getSystemState() {
    return systemState;
  }

  public boolean isReady() {
    return systemState == SystemState.AT_SPEED || systemState == SystemState.SHOOTING;
  }

  public boolean isAtSetpoint() {
    return getCurrentRPM().isNear(targetRPM, ShooterConstants.SHOOTER_SPEED_TOLERANCE)
        && targetRPM.gt(RPM.zero());
  }

  public AngularVelocity getCurrentRPM() {
    return RotationsPerSecond.of(inputs.masterVelocityRPS);
  }

  public AngularVelocity getTargetRPM() {
    return targetRPM;
  }

  public boolean isAt(AngularVelocity target) {
    if (target.lt(RPM.zero())) return false;
    return getCurrentRPM().isNear(target, ShooterConstants.ON_TARGET_RPM_PERCENT);
  }

  // ==================== COMMAND FACTORIES ====================

  public Command spinUpCommand() {
    return runOnce(() -> setWantedState(WantedState.SPIN_UP, ShooterConstants.SHOOTER_SPINUP_SPEED))
        .withName("Shooter Spin Up");
  }

  public Command spinUpAndWaitCommand() {
    return run(() -> setWantedState(WantedState.SPIN_UP, ShooterConstants.SHOOTER_SPINUP_SPEED))
        .until(this::isReady)
        .withName("Shooter Spin Up & Wait");
  }

  public Command stopCommand() {
    return runOnce(() -> setWantedState(WantedState.OFF)).withName("Shooter Stop");
  }

  public Command holdRPMCommand(AngularVelocity rpm) {
    return startEnd(
            () -> setWantedState(WantedState.HOLD_RPM, rpm), () -> setWantedState(WantedState.OFF))
        .withName("Shooter Hold " + rpm + " RPM");
  }

  public Command holdRPMCommand(java.util.function.Supplier<AngularVelocity> rpmSupplier) {
    return run(() -> setWantedState(WantedState.HOLD_RPM, rpmSupplier.get()))
        .finallyDo(interrupted -> setWantedState(WantedState.OFF))
        .withName("Shooter Dynamic RPM");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  // ==================== TELEMETRY ====================

  public double getMasterSupplyCurrentAmps() {
    return inputs.masterSupplyCurrentAmps;
  }

  public double getSlaveSupplyCurrentAmps() {
    return inputs.slaveSupplyCurrentAmps;
  }

  public double getMasterStatorCurrentAmps() {
    return inputs.masterStatorCurrentAmps;
  }

  public double getSlaveStatorCurrentAmps() {
    return inputs.slaveStatorCurrentAmps;
  }

  public double getMasterVoltageVolts() {
    return inputs.masterVoltageVolts;
  }

  public double getSlaveVoltageVolts() {
    return inputs.slaveVoltageVolts;
  }
}
