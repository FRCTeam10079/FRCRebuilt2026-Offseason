package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ShooterPivotConstants;

/**
 * Simulated shooter-pivot IO. Models a MotionMagic-style position controller with a first-order
 * approach toward the target. Positions are in motor rotations (multiply by GEAR_RATIO from
 * degrees).
 */
public class ShooterPivotIOSim implements ShooterPivotIO {
  private static final double LOOP_PERIOD_SEC = 0.02;
  /** Time constant for MotionMagic position approach (seconds). */
  private static final double POSITION_TAU = 0.15;

  private double positionRotations = 0.0;
  private double targetPositionRotations = 0.0;
  private double dutyCycleOutput = 0.0;
  private boolean positionMode = false;
  private boolean neutral = true;

  // Software limits (in motor rotations)
  private boolean forwardLimitEnabled = false;
  private boolean reverseLimitEnabled = false;
  private double forwardLimitRotations = Double.MAX_VALUE;
  private double reverseLimitRotations = -Double.MAX_VALUE;

  public ShooterPivotIOSim() {
    // Start at MIN_ANGLE (home position) in motor rotations
    positionRotations = ShooterPivotConstants.MIN_ANGLE
        .times(ShooterPivotConstants.GEAR_RATIO)
        .in(edu.wpi.first.units.Units.Rotations);
  }

  @Override
  public void updateInputs(ShooterPivotIOInputs inputs) {
    if (neutral) {
      // Hold position
    } else if (positionMode) {
      double alpha = 1.0 - Math.exp(-LOOP_PERIOD_SEC / POSITION_TAU);
      positionRotations += alpha * (targetPositionRotations - positionRotations);
    } else {
      // Duty cycle mode: move proportional to output (rough: ±full range in ~2s at
      // full output)
      double maxSpeedRPS = 40.0; // cruise velocity from constants
      positionRotations += dutyCycleOutput * maxSpeedRPS * LOOP_PERIOD_SEC;
    }

    // Apply software limits
    if (forwardLimitEnabled) {
      positionRotations = Math.min(positionRotations, forwardLimitRotations);
    }
    if (reverseLimitEnabled) {
      positionRotations = Math.max(positionRotations, reverseLimitRotations);
    }

    double velocityRPS =
        positionMode ? (targetPositionRotations - positionRotations) / LOOP_PERIOD_SEC : 0.0;

    inputs.positionRotations = positionRotations;
    inputs.velocityRPS = velocityRPS;
    inputs.dutyCycle = positionMode ? 0.0 : dutyCycleOutput;

    double volts = positionMode
        ? MathUtil.clamp((targetPositionRotations - positionRotations) * 2.0, -12.0, 12.0)
        : dutyCycleOutput * 12.0;
    inputs.voltageVolts = volts;
    inputs.supplyCurrentAmps = Math.abs(volts) * 1.5;
    inputs.statorCurrentAmps = Math.abs(volts) * 2.5;
  }

  @Override
  public void setMotionMagicPosition(double positionRotations) {
    this.targetPositionRotations = positionRotations;
    positionMode = true;
    neutral = false;
  }

  @Override
  public void setDutyCycle(double output) {
    dutyCycleOutput = MathUtil.clamp(output, -1.0, 1.0);
    positionMode = false;
    neutral = false;
  }

  @Override
  public void setNeutral() {
    neutral = true;
    positionMode = false;
    dutyCycleOutput = 0.0;
  }

  @Override
  public void setEncoderPosition(double positionRotations) {
    this.positionRotations = positionRotations;
  }

  @Override
  public void applySoftwareLimits(SoftwareLimitSwitchConfigs config) {
    forwardLimitEnabled = config.ForwardSoftLimitEnable;
    reverseLimitEnabled = config.ReverseSoftLimitEnable;
    forwardLimitRotations = config.ForwardSoftLimitThreshold;
    reverseLimitRotations = config.ReverseSoftLimitThreshold;
  }
}
