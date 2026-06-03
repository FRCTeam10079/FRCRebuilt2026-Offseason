// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Constants for FRC 2026 REBUILT season Contains game-specific values, timing, and robot
 * configuration
 */
public final class Constants {

  public static final CANBus kCANBus = new CANBus("rio");

  /**
   * AdvantageKit logging mode. Auto-detects REAL vs SIM. Set to REPLAY manually when replaying a
   * log file.
   */
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static final class GameConstants extends frc.robot.constants.GameConstants {}

  public static final class ControllerConstants extends frc.robot.constants.ControllerConstants {}

  public static final class DrivetrainConstants extends frc.robot.constants.DrivetrainConstants {}

  public static final class IntakeConstants extends frc.robot.constants.IntakeConstants {}

  public static final class ShooterConstants extends frc.robot.constants.ShooterConstants {}

  public static final class ShooterPivotConstants
      extends frc.robot.constants.ShooterPivotConstants {}

  public static final class ClimberConstants extends frc.robot.constants.ClimberConstants {}

  public static final class ClimbConstants extends frc.robot.constants.ClimbConstants {}

  public static final class HeadingControllerConstants
      extends frc.robot.constants.HeadingControllerConstants {}

  public static final class StateMachineConstants
      extends frc.robot.constants.StateMachineConstants {}

  public static final class VisionConstants extends frc.robot.constants.VisionConstants {}

  public static final class IndexerConstants extends frc.robot.constants.IndexerConstants {}

  public static final class PathfindingPIDConstants
      extends frc.robot.constants.PathfindingPIDConstants {}

  // ==================== UTILITY METHODS ====================

  /** Inches to Meters conversion factor */
  public static final double INCHES_TO_METERS = 0.0254;

  /**
   * Check if a value exists in an array
   *
   * @param array The array to search
   * @param value The value to find
   * @return True if value is in array
   */
  public static boolean contains(double[] array, double value) {
    for (double element : array) {
      if (element == value) {
        return true;
      }
    }
    return false;
  }

  /**
   * Check if an int value exists in an int array
   *
   * @param array The array to search
   * @param value The value to find
   * @return True if value is in array
   */
  public static boolean contains(int[] array, int value) {
    for (int element : array) {
      if (element == value) {
        return true;
      }
    }
    return false;
  }

  public static <U extends Unit, T extends Measure<U>> T clamp(T x, T min, T max) {
    if (x.lte(min)) return min;
    if (x.gte(max)) return max;
    return x;
  }

  public static Angle angleDistance(Angle a, Angle b) {
    return Degrees.of(
        Math.abs(MathUtil.inputModulus(b.in(Degrees) - a.in(Degrees), -180.0, +180.0)));
  }

  // ==================== APRIL TAG FIELD LAYOUT ====================

  /**
   * AprilTag positions on the field
   *
   * <p>Format: HashMap<TagID, double[]{X_inches, Y_inches, Z_inches, Yaw_degrees, Pitch_degrees}>
   */
  public static final class AprilTagMaps extends frc.robot.constants.AprilTagMaps {}

  /** Alignment position enum - left or right side offset from AprilTag */
  public enum AlignPosition {
    LEFT,
    RIGHT,
    CENTER
  }

  /** Starting position enum for autonomous */
  public enum StartingPosition {
    LEFT,
    CENTER,
    RIGHT
  }
}
