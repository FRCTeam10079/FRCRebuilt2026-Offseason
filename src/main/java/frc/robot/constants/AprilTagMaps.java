package frc.robot.constants;

import java.util.HashMap;

public class AprilTagMaps {
  public static final HashMap<Integer, double[]> aprilTagMap = new HashMap<>();

  static {
    aprilTagMap.put(1, new double[] {467.64, 292.31, 35.00, 180.0, 0.0});
    aprilTagMap.put(2, new double[] {469.11, 182.60, 44.25, 90.0, 0.0});
    aprilTagMap.put(3, new double[] {445.35, 172.84, 44.25, 180.0, 0.0});
    aprilTagMap.put(4, new double[] {445.35, 158.84, 44.25, 180.0, 0.0});
    aprilTagMap.put(5, new double[] {469.11, 135.09, 44.25, 270.0, 0.0});
    aprilTagMap.put(6, new double[] {467.64, 25.37, 35.00, 180.0, 0.0});
    aprilTagMap.put(7, new double[] {470.59, 25.37, 35.00, 0.0, 0.0});
    aprilTagMap.put(8, new double[] {483.11, 135.09, 44.25, 270.0, 0.0});
    aprilTagMap.put(9, new double[] {492.88, 144.84, 44.25, 0.0, 0.0});
    aprilTagMap.put(10, new double[] {492.88, 158.84, 44.25, 0.0, 0.0});
    aprilTagMap.put(11, new double[] {483.11, 182.60, 44.25, 90.0, 0.0});
    aprilTagMap.put(12, new double[] {470.59, 292.31, 35.00, 0.0, 0.0});
    aprilTagMap.put(13, new double[] {650.92, 291.47, 21.75, 180.0, 0.0});
    aprilTagMap.put(14, new double[] {650.92, 274.47, 21.75, 180.0, 0.0});
    aprilTagMap.put(15, new double[] {650.90, 170.22, 21.75, 180.0, 0.0});
    aprilTagMap.put(16, new double[] {650.90, 153.22, 21.75, 180.0, 0.0});
    aprilTagMap.put(17, new double[] {183.59, 25.37, 35.00, 0.0, 0.0});
    aprilTagMap.put(18, new double[] {182.11, 135.09, 44.25, 270.0, 0.0});
    aprilTagMap.put(19, new double[] {205.87, 144.84, 44.25, 0.0, 0.0});
    aprilTagMap.put(20, new double[] {205.87, 158.84, 44.25, 0.0, 0.0});
    aprilTagMap.put(21, new double[] {182.11, 182.60, 44.25, 90.0, 0.0});
    aprilTagMap.put(22, new double[] {183.59, 292.31, 35.00, 0.0, 0.0});
    aprilTagMap.put(23, new double[] {180.64, 292.31, 35.00, 180.0, 0.0});
    aprilTagMap.put(24, new double[] {168.11, 182.60, 44.25, 90.0, 0.0});
    aprilTagMap.put(25, new double[] {158.34, 172.84, 44.25, 180.0, 0.0});
    aprilTagMap.put(26, new double[] {158.34, 158.84, 44.25, 180.0, 0.0});
    aprilTagMap.put(27, new double[] {168.11, 135.09, 44.25, 270.0, 0.0});
    aprilTagMap.put(28, new double[] {180.64, 25.37, 35.00, 180.0, 0.0});
    aprilTagMap.put(29, new double[] {0.30, 26.22, 21.75, 0.0, 0.0});
    aprilTagMap.put(30, new double[] {0.30, 43.22, 21.75, 0.0, 0.0});
    aprilTagMap.put(31, new double[] {0.32, 147.47, 21.75, 0.0, 0.0});
    aprilTagMap.put(32, new double[] {0.32, 164.47, 21.75, 0.0, 0.0});
  }

  public static final int[] RED_SIDE_TAGS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

  public static final int[] BLUE_SIDE_TAGS = {
    17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
  };

  protected AprilTagMaps() {}
}
