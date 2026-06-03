package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double masterVelocityRPS = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
    public double masterStatorCurrentAmps = 0.0;
    public double masterVoltageVolts = 0.0;
    public double masterDutyCycle = 0.0;
    public double masterClosedLoopReferenceRPS = 0.0;
    public double masterClosedLoopErrorRPS = 0.0;
    public double masterDeviceTempCelsius = 0.0;
    public int masterFaultField = 0;
    public int masterStickyFaultField = 0;
    public double slaveVelocityRPS = 0.0;
    public double slaveSupplyCurrentAmps = 0.0;
    public double slaveStatorCurrentAmps = 0.0;
    public double slaveVoltageVolts = 0.0;
    public double slaveDutyCycle = 0.0;
    public double slaveDeviceTempCelsius = 0.0;
    public int slaveFaultField = 0;
    public int slaveStickyFaultField = 0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  /** Set velocity target for the flywheel (master); slave follows automatically. */
  default void setVelocity(double rps) {}

  /** Set raw voltage output (for SysId characterization). */
  default void setVoltage(double volts) {}

  /** Coast both motors to stop. */
  default void stop() {}
}
