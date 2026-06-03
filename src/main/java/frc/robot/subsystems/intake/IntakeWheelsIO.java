package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWheelsIO {
  @AutoLog
  class IntakeWheelsIOInputs {
    public double velocityRPS = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double voltageVolts = 0.0;
    public double dutyCycle = 0.0;
    public double closedLoopReferenceRPS = 0.0;
    public double closedLoopErrorRPS = 0.0;
    public double deviceTempCelsius = 0.0;
    public int faultField = 0;
    public int stickyFaultField = 0;
    public double slaveVelocityRPS = 0.0;
    public double slaveSupplyCurrentAmps = 0.0;
    public double slaveStatorCurrentAmps = 0.0;
    public double slaveVoltageVolts = 0.0;
    public double slaveDutyCycle = 0.0;
    public double slaveDeviceTempCelsius = 0.0;
    public int slaveFaultField = 0;
    public int slaveStickyFaultField = 0;
  }

  default void updateInputs(IntakeWheelsIOInputs inputs) {}

  default void setVelocity(double rps) {}

  default void stop() {}
}
