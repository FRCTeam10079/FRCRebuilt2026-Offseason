package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;

/** Simulated intake-wheels IO. First-order velocity approach toward the setpoint. */
public class IntakeWheelsIOSim implements IntakeWheelsIO {
  private static final double LOOP_PERIOD_SEC = 0.02;
  private static final double TAU = 0.1;
  private static final double NOMINAL_VOLTAGE = 12.0;

  private double velocityRPS = 0.0;
  private double targetRPS = 0.0;

  @Override
  public void updateInputs(IntakeWheelsIOInputs inputs) {
    double alpha = 1.0 - Math.exp(-LOOP_PERIOD_SEC / TAU);
    velocityRPS += alpha * (targetRPS - velocityRPS);

    inputs.velocityRPS = velocityRPS;
    double volts = MathUtil.clamp(targetRPS * 0.12, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
    inputs.voltageVolts = volts;
    inputs.supplyCurrentAmps = Math.abs(volts) * 1.5;
    inputs.statorCurrentAmps = Math.abs(volts) * 2.5;
  }

  @Override
  public void setVelocity(double rps) {
    targetRPS = rps;
  }

  @Override
  public void stop() {
    targetRPS = 0.0;
  }
}
