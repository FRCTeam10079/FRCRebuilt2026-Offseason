package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public double feederVelocityRPS = 0.0;
    public double spindexerVelocityRPS = 0.0;
    public double feederSupplyCurrentAmps = 0.0;
    public double feederStatorCurrentAmps = 0.0;
    public double feederVoltageVolts = 0.0;
    public double feederDutyCycle = 0.0;
    public double feederClosedLoopReferenceRPS = 0.0;
    public double feederClosedLoopErrorRPS = 0.0;
    public double feederDeviceTempCelsius = 0.0;
    public int feederFaultField = 0;
    public int feederStickyFaultField = 0;
    public double spindexerSupplyCurrentAmps = 0.0;
    public double spindexerStatorCurrentAmps = 0.0;
    public double spindexerVoltageVolts = 0.0;
    public double spindexerDutyCycle = 0.0;
    public double spindexerClosedLoopReferenceRPS = 0.0;
    public double spindexerClosedLoopErrorRPS = 0.0;
    public double spindexerDeviceTempCelsius = 0.0;
    public int spindexerFaultField = 0;
    public int spindexerStickyFaultField = 0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setFeederVelocity(double rps) {}

  default void setSpindexerVelocity(double rps) {}

  default void stop() {}
}
