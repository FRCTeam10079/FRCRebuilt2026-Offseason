package frc.robot.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.NavigableSet;
import java.util.TreeSet;

/**
 * Distance-based lookup tables for shooter RPM and pivot angle.
 *
 * <p>Uses WPILib's InterpolatingTreeMap for smooth linear interpolation between
 * empirically-measured data points.
 *
 * <p>Tuning guide: Drive to each distance, manually adjust RPM and angle until fuel consistently
 * scores, then record the values here. The interpolation handles all intermediate distances
 * automatically.
 *
 * <p>Hub opening front edge is 72in (1.8288m) off the carpet. Shooter release height is 19.5in
 * (0.4953m) off the carpet. The differential height is ~52.5in (1.3335m). Pivot range: 60deg to
 * 80deg from horizontal.
 */
public final class ShooterInterpolationTable {

  private ShooterInterpolationTable() {} // Static utility class

  // ==================== RPM TABLE ====================
  // Maps distance (meters) -> flywheel RPM
  private static InterpolatingTreeMap<Double, Double> rpmTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final NavigableSet<Double> rpmKeys = new TreeSet<>();

  static {
    // -------------------------------------------------------
    // PLACEHOLDER VALUES - must be tuned on the robot!
    // Distance (m) -> RPM
    // -------------------------------------------------------
    // Close range: low RPM, steep angle (fuel doesn't need much speed)
    putRpm(1.0, 2000.0);
    putRpm(2.2, 2100.0);
    putRpm(2.878, 2396.512);
    putRpm(2.925, 2467.086);
    // rpmTable.put(2.5, 2100.0);
    rpmTable.put(3.0, 2450.0);
    // rpmTable.put(3.5, 2500.0);
    rpmTable.put(3.7, 2475.0);
    // rpmTable.put(4.0, 2680.0);
    // rpmTable.put(4.5, 3300.0);
    rpmTable.put(4.086, 2637.968);
    rpmTable.put(4.1, 2670.0);
    // rpmTable.put(5.5, 3900.0);
    rpmTable.put(6.0, 3200.0);

    rpmTable.put(7.0, 3200.0);
  }

  // ==================== ANGLE TABLE ====================
  // Maps distance (meters) -> pivot angle (degrees from horizontal)
  private static InterpolatingTreeMap<Double, Double> angleTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final NavigableSet<Double> angleKeys = new TreeSet<>();

  static {
    // -------------------------------------------------------
    // PLACEHOLDER VALUES - must be tuned on the robot!
    // Distance (m) -> Pivot angle (degrees)
    // At close range, need steeper angle (closer to 80deg) to lob upward
    // At far range, need shallower angle (closer to 60deg) for flatter trajectory
    // -------------------------------------------------------
    putAngle(1.0, 60.0);
    putAngle(2.0, 60.0);
    putAngle(2.8, 62.0);
    putAngle(2.878, 63.389);
    putAngle(2.878, 63.389);

    // angleTable.put(2.5, 60.0);
    putAngle(3.2, 64.0);
    // angleTable.put(3.5, 64.0);

    putAngle(4.5, 72.5);

    putAngle(5.2, 74.5);

    putAngle(6.0, 75.5);

    putAngle(7.0, 76.5);
  }

  /**
   * Get the interpolated flywheel RPM for a given distance.
   *
   * @param distance horizontal distance to hub in meters
   * @return target flywheel RPM
   */
  public static AngularVelocity getRPM(Distance distance) {
    return RPM.of(rpmTable.get(distance.in(Meters)));
  }

  // ==================== TIME OF FLIGHT TABLE ====================
  // Maps distance (meters) -> time of flight (seconds)
  // Time from ball leaving the shooter to arriving at the hub.
  // CRITICAL for shoot-on-the-move - determines lookahead offset.
  private static InterpolatingTreeMap<Double, Double> tofTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  static {
    // -------------------------------------------------------
    // Estimated from projectile physics given our RPM/angle tables.
    // At 60deg launch with ~8-10 m/s exit velocity, horizontal
    // component is ~4-5 m/s -> TOF ~ distance / horizontal_velocity.
    // Steeper angles (72.5deg at 4.5m) have smaller horizontal
    // component, so TOF rises faster at long range.
    // TUNE NEEDED!
    // -------------------------------------------------------

    tofTable.put(1.0, 1.45); // very close - short flight
    // tofTable.put(1.5, 1.1); // TODO: TUNE - placeholder estimate
    tofTable.put(2.7, 1.7); // TODO: TUNE - placeholder estimate
    // tofTable.put(2.5, 1.55); // TODO: TUNE - placeholder estimate
    tofTable.put(3.2, 1.9); // TODO: TUNE - placeholder estimate
    // tofTable.put(3.5, 0.50); // TODO: TUNE - placeholder estimate
    tofTable.put(4.56, 2.5); // TODO: TUNE - placeholder estimate
    // tofTable.put(4.5, 0.60); // TODO: TUNE - placeholder estimate
    tofTable.put(5.0, 2.7); // TODO: TUNE - placeholder estimate

    tofTable.put(7.0, 2.9);
  }

  /**
   * Get the interpolated pivot angle for a given distance.
   *
   * @param distance horizontal distance to hub in meters
   * @return target pivot angle in degrees from horizontal
   */
  public static Angle getAngle(Distance distance) {
    return Degrees.of(angleTable.get(distance.in(Meters)));
  }

  /**
   * Get the interpolated time-of-flight for a given distance.
   *
   * <p>This is used by the LaunchCalculator for velocity-compensated lookahead. The ball's flight
   * time determines how much the robot's velocity offsets the effective aim point.
   *
   * @param distanceMeters horizontal distance to hub in meters
   * @return estimated time of flight in seconds
   */
  public static double getTimeOfFlight(double distanceMeters) {
    return tofTable.get(distanceMeters);
  }

  // ==================== PASSING TABLES ====================
  // Separate interpolation tables for passing shots (long lobs over/around the
  // hub).
  // Passing uses a flatter pivot angle, higher RPM, and longer TOF.
  // Adapted from MA (6328) passing infrastructure.

  private static InterpolatingTreeMap<Double, Double> passingRpmTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  private static InterpolatingTreeMap<Double, Double> passingAngleTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  private static InterpolatingTreeMap<Double, Double> passingTofTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  /** Minimum valid distance for passing shots (meters). */
  public static final double PASSING_MIN_DISTANCE = 6.27;

  /** Maximum valid distance for passing shots (meters). */
  public static final double PASSING_MAX_DISTANCE = 17.0;

  static {
    // -------------------------------------------------------
    // TODO: MUST BE TUNED ON REAL ROBOT!
    // Passing shots are long-range lobs aimed at a teammate.
    // These placeholders are adapted from MA's data as starting points.
    // Distance (m) -> RPM (higher than normal for long range)
    // -------------------------------------------------------
    passingRpmTable.put(5.5, 2300.0);
    passingRpmTable.put(7.0, 2700.0);
    passingRpmTable.put(8.0, 3100.0);
    passingRpmTable.put(17.0, 4300.0);
  }

  static {
    // -------------------------------------------------------
    // TODO: MUST BE TUNED ON REAL ROBOT!
    // Passing shots use a fixed steep angle for consistent lob trajectory.
    // Distance (m) -> Pivot angle (degrees)
    // -------------------------------------------------------
    passingAngleTable.put(5.5, 79.0);
    passingAngleTable.put(7.0, 79.0);
    passingAngleTable.put(8.0, 79.0);
    passingAngleTable.put(17.0, 79.0);
  }

  static {
    // -------------------------------------------------------
    // TODO: MUST BE TUNED ON REAL ROBOT!
    // Passing shots have longer flight times due to high arc.
    // Distance (m) -> TOF (seconds)
    // -------------------------------------------------------
    passingTofTable.put(5.5, 1.3);
    passingTofTable.put(7.0, 1.4);
    passingTofTable.put(8.0, 1.5);
    passingTofTable.put(11.0, 1.75);
    passingTofTable.put(13.0, 1.8);
    passingTofTable.put(17.0, 2.2);
  }

  public static AngularVelocity getPassingRPM(Distance distance) {
    return RPM.of(passingRpmTable.get(distance.in(Meters)));
  }

  public static Angle getPassingAngle(Distance distance) {
    return Degrees.of(passingAngleTable.get(distance.in(Meters)));
  }

  public static double getPassingTimeOfFlight(double distanceMeters) {
    return passingTofTable.get(distanceMeters);
  }

  public static void hotSwapTofValues(Double key, Double newValue) {
    tofTable.put(key, newValue);

    System.out.println(tofTable.get(key));
  }

  public static void hotSwapRPMValues(Double key, Double newValue) {
    putRpm(key, newValue);

    System.out.println(rpmTable.get(key));
  }

  public static void hotSwapAngleValues(Double key, Double newValue) {
    putAngle(key, newValue);

    System.out.println(angleTable.get(key));
  }

  public static double getClosestRPMKey(double distanceMeters) {
    return getClosestKey(rpmKeys, distanceMeters);
  }

  public static double getClosestAngleKey(double distanceMeters) {
    return getClosestKey(angleKeys, distanceMeters);
  }

  private static double getClosestKey(NavigableSet<Double> keys, double queryMeters) {
    if (keys.isEmpty()) {
      return queryMeters;
    }

    Double lower = keys.floor(queryMeters);
    Double upper = keys.ceiling(queryMeters);

    if (lower == null) return upper;
    if (upper == null) return lower;
    return (queryMeters - lower) <= (upper - queryMeters) ? lower : upper;
  }

  private static void putRpm(Double key, Double value) {
    rpmTable.put(key, value);
    rpmKeys.add(key);
  }

  private static void putAngle(Double key, Double value) {
    angleTable.put(key, value);
    angleKeys.add(key);
  }

  /** TOF table keys in ascending order for nearest-key lookup. */
  private static final double[] TOF_KEYS = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0};

  /** Step size for each D-pad press when tuning TOF. */
  private static final double TOF_STEP = 0.01;

  /**
   * Find the nearest TOF table key to the given distance.
   *
   * @param distanceMeters current distance to hub
   * @return the closest key in the TOF table
   */
  public static double nearestTofKey(double distanceMeters) {
    double bestKey = TOF_KEYS[0];
    double bestDiff = Math.abs(distanceMeters - bestKey);
    for (int i = 1; i < TOF_KEYS.length; i++) {
      double diff = Math.abs(distanceMeters - TOF_KEYS[i]);
      if (diff < bestDiff) {
        bestDiff = diff;
        bestKey = TOF_KEYS[i];
      }
    }
    return bestKey;
  }

  /**
   * Adjust the TOF value at the nearest key by the given direction.
   *
   * @param distanceMeters current distance to hub
   * @param up true to increase TOF, false to decrease
   */
  public static void adjustTof(double distanceMeters, boolean up) {
    double key = nearestTofKey(distanceMeters);
    double current = tofTable.get(key);
    double adjusted = Math.max(0.01, current + (up ? TOF_STEP : -TOF_STEP));
    tofTable.put(key, adjusted);
    SmartDashboard.putNumber("TofTune/Key (m)", key);
    SmartDashboard.putNumber("TofTune/Old TOF (s)", current);
    SmartDashboard.putNumber("TofTune/New TOF (s)", adjusted);
    SmartDashboard.putNumber("TofTune/Actual Dist (m)", distanceMeters);
    SmartDashboard.putString(
        "TofTune/Last Action",
        (up ? "UP" : "DOWN") + String.format(" @ %.1fm -> %.3fs", key, adjusted));
  }

  /**
   * Print the current TOF value at the nearest key for the given distance.
   *
   * @param distanceMeters current distance to hub
   */
  public static void printCurrentTof(double distanceMeters) {
    double key = nearestTofKey(distanceMeters);
    SmartDashboard.putNumber("TofTune/Key (m)", key);
    SmartDashboard.putNumber("TofTune/Current TOF (s)", tofTable.get(key));
    SmartDashboard.putNumber("TofTune/Actual Dist (m)", distanceMeters);
    SmartDashboard.putString(
        "TofTune/Last Action", String.format("READ @ %.1fm = %.3fs", key, tofTable.get(key)));
  }
}
