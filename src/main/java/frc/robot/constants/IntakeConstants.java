package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;

public class IntakeConstants {
  public static class Pivot {
    public static final int MOTOR_ID = 24;

    public static final Angle INTAKE_POSITION = Rotations.of(0);
    public static final Angle STOWED_POSITION = Rotations.of(-6.25);

    public static final int SUPPLY_CURRENT_LIMIT = 60;
    public static final int STATOR_CURRENT_LIMIT = 90;

    /** Stator current (amps) above which the pivot is considered stalling. */
    public static final Current STALL_CURRENT_THRESHOLD = Amps.of(75);
    /** How long (seconds) current must exceed the threshold before declaring a stall. */
    public static final Time STALL_TIME_THRESHOLD = Seconds.of(2.0);

    public static final Angle DEPLOY_TOLERANCE = Rotations.of(0.05);

    /**
     * How long (seconds) the pivot must be at setpoint before switching to idle (NeutralOut). Brake
     * mode holds position mechanically once the motor is off.
     */
    public static final Time IDLE_DEBOUNCE_TIME = Seconds.of(0.5);

    public static final double KA = 0;
    public static final double KS = 0.4;
    /**
     * Gravity compensation feedforward. Tune this by commanding the pivot to 90deg and measuring
     * the duty cycle needed to hold it still - that's approximately kG. With Arm_Cosine gravity
     * type, the controller applies kG * cos(angle) automatically. TODO: Tune this value on the
     * robot. Start at ~0.15 and adjust.
     */
    public static final double KG = 0.15;

    public static final double KP = 2.0;
    public static final double KI = 0;
    public static final double KD = 0.2;
    public static final double KV = 0.12;

    // ==================== MOTION MAGIC ====================
    // Profiled motion for smooth pivot movement instead of instant PositionVoltage
    // snaps.
    // 6.25 rotor rotations total travel (stow -> deploy).
    // With cruise=80, accel=240, jerk=1200:
    // time to cruise = 80/240 = 0.33s, distance = 0.5*240*0.11 = 13.3 rot
    // (triangular)
    // triangular profile: t = 2*sqrt(6.25/240) = ~0.32s total traverse
    // This is FAST but the Kraken X60 can handle it easily on a light intake.
    // ShooterPivot (heavier) uses cruise=40, accel=80, jerk=400 for comparison.

    /** Cruise velocity for MotionMagic (rotations per second). */
    public static final AngularVelocity MM_CRUISE_VELOCITY = RotationsPerSecond.of(80.0);
    /** Acceleration for MotionMagic (rotations per second^2). */
    public static final AngularAcceleration MM_ACCELERATION = RotationsPerSecondPerSecond.of(30.0);

    /** Jerk for MotionMagic (rotations per second^3). Limits snap in acceleration. */
    public static final Velocity<AngularAccelerationUnit> MM_JERK =
        RotationsPerSecondPerSecond.per(Second).of(800.0);

    protected Pivot() {}
  }

  public static class Wheels {
    public static final int MOTOR_ID = 19;
    public static final int SLAVE_MOTOR_ID = 25; // Needs to be changed to the actual motor id

    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(90);

    public static final double INTAKE_IN_RPM = 5500;
    public static final double INTAKE_OUT_RPM = -3000;

    public static final double KA = 0;
    public static final double KS = 0.2;
    public static final double KP = 1.5;

    public static final double KI = 0;
    public static final double KD = 0.1;
    public static final double KV = 1;

    protected Wheels() {}
  }

  protected IntakeConstants() {}
}
