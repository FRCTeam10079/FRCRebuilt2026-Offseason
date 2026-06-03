package frc.robot.constants;

public class PathfindingPIDConstants {
  public static final double FOLLOW_KP = 3.0;
  public static final double FOLLOW_KI = 0.0;
  public static final double FOLLOW_KD = 0.1;

  public static final double LOOKAHEAD_DISTANCE_METERS = 0.5;
  public static final double WAYPOINT_TOLERANCE_METERS = 0.3;

  protected PathfindingPIDConstants() {}
}
