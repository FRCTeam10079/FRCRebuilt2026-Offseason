package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Climb approach poses for pathfinding to the 2026 REBUILT climb structure.
 *
 * <p>Six poses total: three lanes (Left / Center / Right) x two alliances (Blue / Red). Each pose
 * is stored as three {@link LoggedNetworkNumber} values (X, Y, HeadingDeg) under the
 * {@code /Tuning/Climb/} table so they are live-editable in AdvantageScope tuning mode without
 * redeploying.
 *
 * <p>2026 REBUILT field reference: the Blue climb structure area is near x ~= 4.66 m and the Red
 * climb structure area is near x ~= 11.88 m (WPILib 2026-rebuilt layout).
 *
 * <p>Defaults come from on-field tuning values and mirrored field transforms.
 */
public class ClimbConstants {

  protected ClimbConstants() {}

  /** Which side of the climb structure the robot should approach. */
  public enum ClimbLane {
    LEFT,
    CENTER,
    RIGHT
  }

  // ======================== BLUE ALLIANCE POSES ========================
  // Mirrored from red defaults using 2026 REBUILT field dimensions:
  // field length = 16.541m, field width = 8.069m
  // xBlue = 16.541 - xRed, yBlue = 8.069 - yRed, headingBlue = headingRed + 180

  // --- Blue Left ---
  private static final LoggedNetworkNumber BLUE_LEFT_X =
      new LoggedNetworkNumber("/Tuning/Climb/Blue/Left/X", 0.991);
  private static final LoggedNetworkNumber BLUE_LEFT_Y =
      new LoggedNetworkNumber("/Tuning/Climb/Blue/Left/Y", 2.869);
  private static final LoggedNetworkNumber BLUE_LEFT_HEADING_DEG =
      new LoggedNetworkNumber("/Tuning/Climb/Blue/Left/HeadingDeg", 0.0);

  // --- Blue Center ---
  private static final LoggedNetworkNumber BLUE_CENTER_X =
      new LoggedNetworkNumber("/Tuning/Climb/Blue/Center/X", 0.991);
  private static final LoggedNetworkNumber BLUE_CENTER_Y =
      new LoggedNetworkNumber("/Tuning/Climb/Blue/Center/Y", 3.749);
  private static final LoggedNetworkNumber BLUE_CENTER_HEADING_DEG =
      new LoggedNetworkNumber("/Tuning/Climb/Blue/Center/HeadingDeg", 180.0);

  // --- Blue Right ---
  private static final LoggedNetworkNumber BLUE_RIGHT_X =
      new LoggedNetworkNumber("/Tuning/Climb/Blue/Right/X", 0.991);
  private static final LoggedNetworkNumber BLUE_RIGHT_Y =
      new LoggedNetworkNumber("/Tuning/Climb/Blue/Right/Y", 4.719);
  private static final LoggedNetworkNumber BLUE_RIGHT_HEADING_DEG =
      new LoggedNetworkNumber("/Tuning/Climb/Blue/Right/HeadingDeg", 180.0);

  // ======================== RED ALLIANCE POSES ========================

  // --- Red Left ---
  private static final LoggedNetworkNumber RED_LEFT_X =
      new LoggedNetworkNumber("/Tuning/Climb/Red/Left/X", 15.550);
  private static final LoggedNetworkNumber RED_LEFT_Y =
      new LoggedNetworkNumber("/Tuning/Climb/Red/Left/Y", 5.2);
  private static final LoggedNetworkNumber RED_LEFT_HEADING_DEG =
      new LoggedNetworkNumber("/Tuning/Climb/Red/Left/HeadingDeg", 180.0);

  // --- Red Center ---
  private static final LoggedNetworkNumber RED_CENTER_X =
      new LoggedNetworkNumber("/Tuning/Climb/Red/Center/X", 15.550);
  private static final LoggedNetworkNumber RED_CENTER_Y =
      new LoggedNetworkNumber("/Tuning/Climb/Red/Center/Y", 4.320);
  private static final LoggedNetworkNumber RED_CENTER_HEADING_DEG =
      new LoggedNetworkNumber("/Tuning/Climb/Red/Center/HeadingDeg", 0.0);

  // --- Red Right ---
  private static final LoggedNetworkNumber RED_RIGHT_X =
      new LoggedNetworkNumber("/Tuning/Climb/Red/Right/X", 15.550);
  private static final LoggedNetworkNumber RED_RIGHT_Y =
      new LoggedNetworkNumber("/Tuning/Climb/Red/Right/Y", 3.35);
  private static final LoggedNetworkNumber RED_RIGHT_HEADING_DEG =
      new LoggedNetworkNumber("/Tuning/Climb/Red/Right/HeadingDeg", 0.0);

  // Shared additive offsets applied only when lane == RIGHT (both alliances).
  // Keep defaults at 0.0 to preserve existing behavior until field tuning begins.
  private static final LoggedNetworkNumber RIGHT_SIDE_OFFSET_X =
      new LoggedNetworkNumber("/Tuning/Climb/RightSideOffsetX", 0.0);
  private static final LoggedNetworkNumber RIGHT_SIDE_OFFSET_Y =
      new LoggedNetworkNumber("/Tuning/Climb/RightSideOffsetY", 0.0);

  // ======================== APPROACH STANDOFF ========================
  /**
   * How far in front of the climb structure (toward the open field) the approach waypoint sits. The
   * robot pathfinds to this safe point first, then drives straight into the climb structure for the
   * final approach. This prevents the AD* path from routing through the climb structure itself.
   */
  private static final LoggedNetworkNumber APPROACH_STANDOFF_METERS =
      new LoggedNetworkNumber("/Tuning/Climb/ApproachStandoffMeters", 1.5);

  // ======================== AUTO CLIMB SEQUENCE ========================
  /** Final forward nudge distance after climb pull-up. Default is 1 inch. */
  private static final LoggedNetworkNumber AUTO_CLIMB_FINAL_FORWARD_DISTANCE_METERS =
      new LoggedNetworkNumber("/Tuning/Climb/AutoSequence/FinalForwardDistanceMeters", 0.01);

  /** Final forward nudge speed (m/s). */
  private static final LoggedNetworkNumber AUTO_CLIMB_FINAL_FORWARD_SPEED_MPS =
      new LoggedNetworkNumber("/Tuning/Climb/AutoSequence/FinalForwardSpeedMps", 1.0);

  /** Small settle delay after the final nudge. */
  private static final LoggedNetworkNumber AUTO_CLIMB_POST_MOVE_SETTLE_SECONDS =
      new LoggedNetworkNumber("/Tuning/Climb/AutoSequence/PostMoveSettleSeconds", 0.150);

  /**
   * RIGHT-lane-only pre-entry strafe distance (meters, robot-left direction) inserted after
   * reaching approach pose and before final entry.
   */
  private static final LoggedNetworkNumber AUTO_CLIMB_RIGHT_PRE_ENTRY_LEFT_DISTANCE_METERS =
      new LoggedNetworkNumber(
          "/Tuning/Climb/AutoSequence/RightPreEntryLeftDistanceMeters", 0.00015);

  /** RIGHT-lane-only pre-entry strafe speed (m/s, robot-left direction). */
  private static final LoggedNetworkNumber AUTO_CLIMB_RIGHT_PRE_ENTRY_LEFT_SPEED_MPS =
      new LoggedNetworkNumber("/Tuning/Climb/AutoSequence/RightPreEntryLeftSpeedMps", 0.12);

  /** RIGHT-lane-only post-entry forward distance (meters) before retract. */
  private static final LoggedNetworkNumber AUTO_CLIMB_RIGHT_POST_ENTRY_FORWARD_DISTANCE_METERS =
      new LoggedNetworkNumber(
          "/Tuning/Climb/AutoSequence/RightPostEntryForwardDistanceMeters", 0.0);

  /** RIGHT-lane-only post-entry forward speed (m/s) before retract. */
  private static final LoggedNetworkNumber AUTO_CLIMB_RIGHT_POST_ENTRY_FORWARD_SPEED_MPS =
      new LoggedNetworkNumber("/Tuning/Climb/AutoSequence/RightPostEntryForwardSpeedMps", 0.20);

  /**
   * Resolve the climb approach pose for the given lane and alliance.
   *
   * <p>Reads live-tunable values from NetworkTables and logs the selection and resolved pose via
   * AdvantageKit for verification in AdvantageScope.
   *
   * @param lane which climb lane to approach (Left / Center / Right)
   * @param isRedAlliance true when on the Red alliance
   * @return the target {@link Pose2d} for pathfinding
   */
  public static Pose2d getClimbPose(ClimbLane lane, boolean isRedAlliance) {
    double x;
    double y;
    double headingDeg;

    if (isRedAlliance) {
      switch (lane) {
        case LEFT:
          x = RED_LEFT_X.get();
          y = RED_LEFT_Y.get();
          headingDeg = RED_LEFT_HEADING_DEG.get();
          break;
        case RIGHT:
          x = RED_RIGHT_X.get();
          y = RED_RIGHT_Y.get();
          headingDeg = RED_RIGHT_HEADING_DEG.get();
          break;
        default: // CENTER
          x = RED_CENTER_X.get();
          y = RED_CENTER_Y.get();
          headingDeg = RED_CENTER_HEADING_DEG.get();
          break;
      }
    } else {
      switch (lane) {
        case LEFT:
          x = BLUE_LEFT_X.get();
          y = BLUE_LEFT_Y.get();
          headingDeg = BLUE_LEFT_HEADING_DEG.get();
          break;
        case RIGHT:
          x = BLUE_RIGHT_X.get();
          y = BLUE_RIGHT_Y.get();
          headingDeg = BLUE_RIGHT_HEADING_DEG.get();
          break;
        default: // CENTER
          x = BLUE_CENTER_X.get();
          y = BLUE_CENTER_Y.get();
          headingDeg = BLUE_CENTER_HEADING_DEG.get();
          break;
      }
    }

    if (lane == ClimbLane.RIGHT) {
      x += RIGHT_SIDE_OFFSET_X.get();
      y += RIGHT_SIDE_OFFSET_Y.get();
    }

    Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg));

    // Log selection and resolved pose for AdvantageScope verification
    Logger.recordOutput("Climb/SelectedLane", lane.name());
    Logger.recordOutput("Climb/Alliance", isRedAlliance ? "Red" : "Blue");
    Logger.recordOutput("Climb/RightOffsetX", RIGHT_SIDE_OFFSET_X.get());
    Logger.recordOutput("Climb/RightOffsetY", RIGHT_SIDE_OFFSET_Y.get());
    Logger.recordOutput("Climb/TargetPose", pose);

    return pose;
  }

  /**
   * Resolve the approach waypoint for the given lane and alliance.
   *
   * <p>The approach pose has the same Y and heading as the final climb pose, but its X is pulled
   * further into the open field by {@link #APPROACH_STANDOFF_METERS}. This ensures the AD*
   * pathfinder routes around the climb structure rather than through it.
   *
   * @param lane which climb lane to approach
   * @param isRedAlliance true when on the Red alliance
   * @return the approach {@link Pose2d} (safe waypoint in front of the climb structure)
   */
  public static Pose2d getClimbApproachPose(ClimbLane lane, boolean isRedAlliance) {
    Pose2d finalPose = getClimbPose(lane, isRedAlliance);
    double standoff = APPROACH_STANDOFF_METERS.get();

    // Blue climb structure is near Blue wall (low X), so approach from higher X
    // (+standoff).
    // Red climb structure is near Red wall (high X), so approach from lower X
    // (-standoff).
    double approachX = isRedAlliance ? finalPose.getX() - standoff : finalPose.getX() + standoff;

    Pose2d approachPose = new Pose2d(approachX, finalPose.getY(), finalPose.getRotation());

    Logger.recordOutput("Climb/ApproachPose", approachPose);

    return approachPose;
  }

  /** Live-tunable final forward nudge distance (meters) for auto-climb sequence. */
  public static double getAutoClimbFinalForwardDistanceMeters() {
    return AUTO_CLIMB_FINAL_FORWARD_DISTANCE_METERS.get();
  }

  /** Live-tunable final forward nudge speed (m/s) for auto-climb sequence. */
  public static double getAutoClimbFinalForwardSpeedMps() {
    return AUTO_CLIMB_FINAL_FORWARD_SPEED_MPS.get();
  }

  /** Live-tunable settle delay (seconds) after final auto-climb nudge. */
  public static double getAutoClimbPostMoveSettleSeconds() {
    return AUTO_CLIMB_POST_MOVE_SETTLE_SECONDS.get();
  }

  /**
   * Live-tunable RIGHT-lane pre-entry strafe-left distance (meters) used before final climb entry.
   */
  public static double getAutoClimbRightPreEntryLeftDistanceMeters() {
    return AUTO_CLIMB_RIGHT_PRE_ENTRY_LEFT_DISTANCE_METERS.get();
  }

  /** Live-tunable RIGHT-lane pre-entry strafe-left speed (m/s). */
  public static double getAutoClimbRightPreEntryLeftSpeedMps() {
    return AUTO_CLIMB_RIGHT_PRE_ENTRY_LEFT_SPEED_MPS.get();
  }

  /** Live-tunable RIGHT-lane post-entry forward distance (meters). */
  public static double getAutoClimbRightPostEntryForwardDistanceMeters() {
    return AUTO_CLIMB_RIGHT_POST_ENTRY_FORWARD_DISTANCE_METERS.get();
  }

  /** Live-tunable RIGHT-lane post-entry forward speed (m/s). */
  public static double getAutoClimbRightPostEntryForwardSpeedMps() {
    return AUTO_CLIMB_RIGHT_POST_ENTRY_FORWARD_SPEED_MPS.get();
  }
}
