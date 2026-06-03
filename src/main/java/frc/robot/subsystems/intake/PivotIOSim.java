package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

/**
 * Simulated intake-pivot IO. Models MotionMagic position control with a first-order approach. The
 * position unit is motor rotations (same as the real PivotIOTalonFX).
 */
public class PivotIOSim implements PivotIO {
  private static final double LOOP_PERIOD_SEC = 0.02;
  private static final double POSITION_TAU = 0.12;

  private double positionRotations = 0.0;
  private double targetPositionRotations = 0.0;
  private boolean positionMode = false;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (positionMode) {
      double alpha = 1.0 - Math.exp(-LOOP_PERIOD_SEC / POSITION_TAU);
      positionRotations += alpha * (targetPositionRotations - positionRotations);
    }

    inputs.positionRotations = positionRotations;

    double volts = positionMode
        ? MathUtil.clamp((targetPositionRotations - positionRotations) * 2.0, -12.0, 12.0)
        : 0.0;
    inputs.voltageVolts = volts;
    inputs.supplyCurrentAmps = Math.abs(volts) * 1.5;
    inputs.statorCurrentAmps = Math.abs(volts) * 2.5;
  }

  @Override
  public void setMotionMagicPosition(Angle position) {
    targetPositionRotations = position.in(edu.wpi.first.units.Units.Rotations);
    positionMode = true;
  }

  @Override
  public void setNeutral() {
    positionMode = false;
  }

  @Override
  public void periodic() {
    // No-op in sim — NetworkedTalonFX periodic not needed
  }
}
