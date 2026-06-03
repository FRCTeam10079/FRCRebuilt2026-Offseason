package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {
  @AutoLog
  class ShooterPivotIOInputs {
    public double positionRotations = 0.0;
    public double velocityRPS = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double voltageVolts = 0.0;
    public double dutyCycle = 0.0;
    public double closedLoopReferenceRotations = 0.0;
    public double closedLoopErrorRotations = 0.0;
    public double deviceTempCelsius = 0.0;
    public int faultField = 0;
    public int stickyFaultField = 0;
    public boolean motionMagicAtTarget = false;
    public boolean motionMagicIsRunning = false;
  }

  default void updateInputs(ShooterPivotIOInputs inputs) {}

  /** Set MotionMagic position target (in motor rotations). */
  default void setMotionMagicPosition(double positionRotations) {}

  /** Set raw duty cycle output (for manual/homing). */
  default void setDutyCycle(double output) {}

  /** Set motor to NeutralOut. */
  default void setNeutral() {}

  /** Zero the integrated encoder position. */
  default void setEncoderPosition(double positionRotations) {}

  /** Apply software limit switch configuration. */
  default void applySoftwareLimits(SoftwareLimitSwitchConfigs config) {}
}
