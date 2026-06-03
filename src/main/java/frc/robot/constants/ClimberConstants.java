package frc.robot.constants;

public class ClimberConstants {

  // ==================== HARDWARE IDs ====================
  public static final int CLIMBER_MOTOR_ID = 26;
  public static final String CLIMBER_CANBUS = "rio";

  // ==================== MOTOR CONFIGURATION ====================
  /** Set true if motor positive direction needs to be inverted for extend. */
  public static final boolean MOTOR_INVERTED = true;

  /** Enable TalonFX FOC for smoother control under heavy climb load. */
  public static final boolean ENABLE_FOC = true;

  /**
   * Peak command voltages allowed by TalonFX voltage requests. Reduced to 2V to prevent the
   * string/belay from breaking under excessive commanded voltage.
   */
  public static final double PEAK_FORWARD_VOLTAGE = 6.0;

  public static final double PEAK_REVERSE_VOLTAGE = -6.0;

  public static final int SUPPLY_CURRENT_LIMIT = 30;
  public static final int STATOR_CURRENT_LIMIT = 60;

  /**
   * Supply current limit applied after the stall protection timeout window elapses.
   *
   * <p>Set to 0 A to effectively cut motor power if sustained high-load current persists.
   */
  public static final double STALL_PROTECTION_LOWER_SUPPLY_CURRENT_LIMIT = 0.0;

  /**
   * Time in seconds before reducing supply current to
   * {@link #STALL_PROTECTION_LOWER_SUPPLY_CURRENT_LIMIT}.
   */
  public static final double STALL_PROTECTION_TIMEOUT_SECONDS = 3.0;

  // ==================== VOLTAGE CONTROL ====================
  /** Voltage applied when extending the climber (positive = extend direction). */
  public static final double EXTEND_VOLTAGE = 6.0;

  /** Voltage applied when retracting the climber (negative = retract direction). */
  public static final double RETRACT_VOLTAGE = -6.0;

  // ==================== POSITION TARGETS ====================
  /** Motor-side position (rotations) for fully extended climber. */
  public static final double EXTEND_POSITION_ROTATIONS = 90.0;

  /** Motor-side position (rotations) for fully retracted climber. */
  public static final double RETRACT_POSITION_ROTATIONS = 20.0;

  /** Allowable error (rotations) when checking if target position is reached. */
  public static final double POSITION_TOLERANCE_ROTATIONS = 1.0;

  // ==================== STATUS SIGNALS ====================
  /** Frequency (Hz) for climber Talon status signals. */
  public static final double STATUS_SIGNAL_UPDATE_HZ = 50.0;

  /** Debounce time before declaring motor connected/disconnected. */
  public static final double MOTOR_CONNECTED_DEBOUNCE_SECONDS = 0.25;

  // ==================== MECHANISM GEOMETRY ====================
  /** Installed climber reduction (motor turns : winch turns). */
  public static final double GEAR_RATIO = 25.0;

  protected ClimberConstants() {}
}
