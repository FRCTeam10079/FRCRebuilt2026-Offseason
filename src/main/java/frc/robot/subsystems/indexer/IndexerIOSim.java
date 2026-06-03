package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;

/** Simulated indexer IO. Models feeder and spindexer as simple first-order velocity systems. */
public class IndexerIOSim implements IndexerIO {
  private static final double LOOP_PERIOD_SEC = 0.02;
  private static final double TAU = 0.1; // fast response for small rollers
  private static final double NOMINAL_VOLTAGE = 12.0;

  private double feederVelocityRPS = 0.0;
  private double feederTargetRPS = 0.0;
  private double spindexerVelocityRPS = 0.0;
  private double spindexerTargetRPS = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    double alpha = 1.0 - Math.exp(-LOOP_PERIOD_SEC / TAU);
    feederVelocityRPS += alpha * (feederTargetRPS - feederVelocityRPS);
    spindexerVelocityRPS += alpha * (spindexerTargetRPS - spindexerVelocityRPS);

    inputs.feederVelocityRPS = feederVelocityRPS;
    inputs.spindexerVelocityRPS = spindexerVelocityRPS;

    double feederVolts = MathUtil.clamp(feederTargetRPS * 0.12, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
    inputs.feederVoltageVolts = feederVolts;
    inputs.feederSupplyCurrentAmps = Math.abs(feederVolts) * 1.5;
    inputs.feederStatorCurrentAmps = Math.abs(feederVolts) * 2.0;

    double spindexerVolts =
        MathUtil.clamp(spindexerTargetRPS * 0.12, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
    inputs.spindexerVoltageVolts = spindexerVolts;
    inputs.spindexerSupplyCurrentAmps = Math.abs(spindexerVolts) * 1.5;
    inputs.spindexerStatorCurrentAmps = Math.abs(spindexerVolts) * 2.0;
  }

  @Override
  public void setFeederVelocity(double rps) {
    feederTargetRPS = rps;
  }

  @Override
  public void setSpindexerVelocity(double rps) {
    spindexerTargetRPS = rps;
  }

  @Override
  public void stop() {
    feederTargetRPS = 0.0;
    spindexerTargetRPS = 0.0;
  }
}
