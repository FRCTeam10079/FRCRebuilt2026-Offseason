package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ClimberConstants;

/**
 * Simulated climber IO. Models a first-order winch driven by voltage with stall simulation at
 * physical limits.
 */
public class ClimberIOSim implements ClimberIO {
  private static final double LOOP_PERIOD_SEC = 0.02;
  private static final double NOMINAL_VOLTAGE = 12.0;

  /** Approximate free speed of the Kraken X60 in RPS (~6000 RPM / 60). */
  private static final double KV_RPS_PER_VOLT = (6000.0 / 60.0) / NOMINAL_VOLTAGE;

  /** Time constant for velocity ramping (seconds). Models rotor + spool inertia. */
  private static final double TAU = 0.3;

  /** Sim physical limits (rotations) ??? stall is simulated at these bounds. */
  private static final double SIM_MIN_POSITION = 0.0;

  private static final double SIM_MAX_POSITION = 200.0;

  /** Stator current when stalled at a physical limit (amps). */
  private static final double STALL_CURRENT_AMPS = 60.0;

  private boolean isVoltageMode = false;
  private double positionRotations = 0.0;
  private double velocityRPS = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorConnected = true;

    if (!isVoltageMode) {
      appliedVolts = 0.0;
    }

    // First-order model: velocity approaches target exponentially
    double targetVelocityRPS = isVoltageMode ? appliedVolts * KV_RPS_PER_VOLT : 0.0;
    double alpha = 1.0 - Math.exp(-LOOP_PERIOD_SEC / TAU);
    velocityRPS += alpha * (targetVelocityRPS - velocityRPS);

    // Integrate position
    positionRotations += velocityRPS * LOOP_PERIOD_SEC;

    // Simulate stall at physical limits
    boolean atLimit = false;
    if (positionRotations <= SIM_MIN_POSITION && velocityRPS < 0) {
      positionRotations = SIM_MIN_POSITION;
      velocityRPS = 0.0;
      atLimit = true;
    }
    if (positionRotations >= SIM_MAX_POSITION && velocityRPS > 0) {
      positionRotations = SIM_MAX_POSITION;
      velocityRPS = 0.0;
      atLimit = true;
    }

    inputs.positionRotations = positionRotations;
    inputs.velocityRPS = velocityRPS;
    inputs.appliedVoltage = appliedVolts;

    if (atLimit && isVoltageMode && Math.abs(appliedVolts) > 0.1) {
      // Simulated stall: high current, zero velocity
      inputs.statorCurrentAmps = STALL_CURRENT_AMPS;
      inputs.supplyCurrentAmps = STALL_CURRENT_AMPS * 0.5;
    } else {
      inputs.statorCurrentAmps = Math.abs(appliedVolts) * 5.0;
      inputs.supplyCurrentAmps = Math.abs(appliedVolts) * 3.0;
    }

    inputs.tempCelsius = 30.0;
    inputs.dutyCycle = appliedVolts / NOMINAL_VOLTAGE;
    inputs.supplyVoltage = NOMINAL_VOLTAGE;
  }

  @Override
  public void setVoltage(double volts) {
    isVoltageMode = true;
    // Clamp to TalonFX peak voltage config to match real hardware behavior
    // otherwise testing in sim is gonna be hella annoying bruh
    appliedVolts = MathUtil.clamp(
        volts, ClimberConstants.PEAK_REVERSE_VOLTAGE, ClimberConstants.PEAK_FORWARD_VOLTAGE);
  }

  @Override
  public void stop() {
    isVoltageMode = false;
    appliedVolts = 0.0;
  }

  @Override
  public void setEncoderPosition(double rotations) {
    positionRotations = rotations;
  }
}
