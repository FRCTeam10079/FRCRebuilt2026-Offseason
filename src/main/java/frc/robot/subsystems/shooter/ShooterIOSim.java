package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;

/**
 * Simulated shooter IO that models a first-order flywheel spin-up/spin-down. Velocity ramps toward
 * the setpoint each cycle using a simple time-constant model.
 */
public class ShooterIOSim implements ShooterIO {
  private static final double LOOP_PERIOD_SEC = 0.02;
  /** Time constant (seconds) for the flywheel to reach ~63% of target velocity. */
  private static final double SPIN_UP_TAU = 0.5;
  /** Nominal battery voltage. */
  private static final double NOMINAL_VOLTAGE = 12.0;
  /** kV approximation: RPS per volt. */
  private static final double KV_RPS_PER_VOLT = 91.67 / NOMINAL_VOLTAGE; // ~5500 RPM / 60 / 12V

  private double velocityRPS = 0.0;
  private double targetVelocityRPS = 0.0;
  private double appliedVolts = 0.0;
  private boolean velocityMode = false;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double target;
    if (velocityMode) {
      target = targetVelocityRPS;
    } else {
      target = appliedVolts * KV_RPS_PER_VOLT;
    }

    // First-order exponential approach
    double alpha = 1.0 - Math.exp(-LOOP_PERIOD_SEC / SPIN_UP_TAU);
    velocityRPS += alpha * (target - velocityRPS);

    double simVolts = velocityMode ? (targetVelocityRPS / KV_RPS_PER_VOLT) : appliedVolts;
    simVolts = MathUtil.clamp(simVolts, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);

    inputs.masterVelocityRPS = velocityRPS;
    inputs.masterVoltageVolts = simVolts;
    inputs.masterSupplyCurrentAmps = Math.abs(simVolts) * 2.0; // rough estimate
    inputs.masterStatorCurrentAmps = Math.abs(simVolts) * 3.0;
    inputs.slaveVoltageVolts = simVolts;
    inputs.slaveSupplyCurrentAmps = inputs.masterSupplyCurrentAmps;
    inputs.slaveStatorCurrentAmps = inputs.masterStatorCurrentAmps;
  }

  @Override
  public void setVelocity(double rps) {
    velocityMode = true;
    targetVelocityRPS = rps;
  }

  @Override
  public void setVoltage(double volts) {
    velocityMode = false;
    appliedVolts = MathUtil.clamp(volts, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
  }

  @Override
  public void stop() {
    velocityMode = false;
    targetVelocityRPS = 0.0;
    appliedVolts = 0.0;
  }
}
