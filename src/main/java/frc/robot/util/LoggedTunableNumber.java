package frc.robot.util;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A tunable number that can be adjusted via NetworkTables at runtime.
 *
 * <p>Adapted from Mechanical Advantage (6328). In simulation or on the robot, the value is
 * published to NetworkTables under "/Tuning/{key}" and can be changed live. This enables rapid
 * iteration of PID gains, tolerances, and thresholds without redeploying code.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * private static final LoggedTunableNumber kP = new LoggedTunableNumber("DriveCommands/Launching/kP", 8.0);
 *
 * // In periodic():
 * double proportionalGain = kP.get();
 * }</pre>
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber.
   *
   * @param dashboardKey Key on dashboard (e.g. "DriveCommands/Launching/kP")
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableNumber with a default value.
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
  }

  /**
   * Get the current value, from dashboard if available.
   *
   * @return The current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    }
    return dashboardNumber != null ? dashboardNumber.get() : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check.
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed.
   *
   * @param id Unique identifier for the caller
   * @param action Callback to run when any of the tunable numbers have changed
   * @param tunableNumbers All tunable numbers to check
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(
          Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  /** Runs action if any of the tunableNumbers have changed. */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
