// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachine;

/**
 * How the drivetrain is being controlled. Only modes that are actively set by code or needed for
 * upcoming mechanisms.
 */
public enum DrivetrainMode {
  FIELD_CENTRIC("Field-Centric Drive"),
  ROBOT_CENTRIC("Robot-Centric Drive"),
  PATH_FOLLOWING("Path Following"),
  VISION_TRACKING("Vision Tracking"),
  LOCKED("Wheels Locked"),
  DISABLED("Drivetrain Disabled");

  public final String description;

  DrivetrainMode(String description) {
    this.description = description;
  }
}
