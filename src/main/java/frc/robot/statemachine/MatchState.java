// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachine;

/**
 * Robot lifecycle phase. Driven by Robot.java in disabledInit, autonomousInit, teleopInit,
 * testInit. Endgame and transition shifts are detected automatically.
 */
public enum MatchState {
  DISABLED("Robot Disabled", false, false),
  AUTO_INIT("Autonomous Initializing", true, true),
  AUTO_RUNNING("Autonomous Running", true, true),
  TELEOP_INIT("Teleop Initializing", true, false),
  TELEOP_RUNNING("Teleop Running", true, false),
  TRANSITION_SHIFT("Transition Shift Period", true, false),
  TEST_INIT("Test Mode Initializing", true, false),
  TEST_RUNNING("Test Mode Running", true, false),
  ENDGAME("Endgame Period", true, false),
  ESTOP("Emergency Stop", false, false);

  public final String description;
  public final boolean enabled;
  public final boolean autonomous;

  MatchState(String description, boolean enabled, boolean autonomous) {
    this.description = description;
    this.enabled = enabled;
    this.autonomous = autonomous;
  }
}
