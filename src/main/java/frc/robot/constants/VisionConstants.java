package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.LoggedTunableNumber;

/**
 * Vision constants stolen from Mercer Island's approach. Uses LoggedTunableNumber for live tuning
 * of filtering thresholds and standard deviation baselines.
 */
public class VisionConstants {

  // ==================== FIELD LAYOUT ====================
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // ==================== CAMERA NAMES ====================
  // Must match names configured on coprocessor
  public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
  public static final String LIMELIGHT_RIGHT_NAME = "limelight-right";
  public static final String[] LIMELIGHT_NAMES = {LIMELIGHT_LEFT_NAME, LIMELIGHT_RIGHT_NAME};

  // ==================== ROBOT-TO-CAMERA TRANSFORMS ====================
  // Convention: forward (x), left (y), up (z) in meters, rotation in radians.

  // Limelight Left: Forward: 0.304, Right: -0.156 (Left: 0.156), Up: 0.214,
  // Pitch: 28, Yaw: 0
  public static Transform3d robotToCameraLeft = new Transform3d(
      0.304, 0.156, 0.214, new Rotation3d(0.0, Math.toRadians(-28.0), Math.toRadians(0.0)));

  // Limelight Right: Forward: 0.298, Right: 0.252 (Left: -0.252), Up: 0.218559,
  // Pitch: 28, Yaw: -90
  public static Transform3d robotToCameraRight = new Transform3d(
      0.298, -0.252, 0.218559, new Rotation3d(0.0, Math.toRadians(-28.0), Math.toRadians(-90.0)));

  // ==================== FILTERING THRESHOLDS ====================
  // Tunable via NetworkTables at /Tuning/Vision/*
  public static LoggedTunableNumber maxAmbiguity =
      new LoggedTunableNumber("Vision/MaxAmbiguity", 0.3);
  public static LoggedTunableNumber maxZError = new LoggedTunableNumber("Vision/MaxZError", 0.75);

  // ==================== STANDARD DEVIATION MODEL ====================
  /** Baseline linear standard deviation for 1 meter distance and 1 tag (meters). */
  public static LoggedTunableNumber linearStdDevBaseline =
      new LoggedTunableNumber("Vision/LinearStdDevBaseline", 0.7); // 0.35
  // *
  // 2

  /** Baseline angular standard deviation for 1 meter distance and 1 tag (radians). */
  public static LoggedTunableNumber angularStdDevBaseline =
      new LoggedTunableNumber("Vision/AngularStdDevBaseline", 0.72); // 0.36 * 2

  // ==================== MEGATAG 2 MULTIPLIERS ====================
  public static LoggedTunableNumber linearStdDevMegatag2Factor =
      new LoggedTunableNumber("Vision/LinearStdDevMegatag2Factor", 0.25);

  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

  // ==================== PER-TAG QUALITY MULTIPLIERS ====================
  public static double getTagStdevMultiplier(int tag) {
    switch (tag) {
      case 9, 10, 11, 2, 8, 5, 4, 3, 19, 20, 21, 24, 18, 27, 26, 25: // HUB TAGS
        return 1.0;
      case 14, 13, 15, 16, 29, 30, 31, 32: // OUTPOST, TOWER TAGS
        return 3.5;
      case 1, 6, 22, 17: // TRENCH TAGS SEEN FROM NEUTRAL ZONE
        return 1.0;
      case 12, 7, 28, 23: // TRENCH TAGS SEEN FROM ALLIANCE ZONE
        return 9.0;
      default:
        return Double.POSITIVE_INFINITY; // Unknown tag, reject
    }
  }

  protected VisionConstants() {}
}
