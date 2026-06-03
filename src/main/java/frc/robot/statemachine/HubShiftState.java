// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachine;

/** Which hub is currently active for scoring. */
public enum HubShiftState {
  UNKNOWN,
  MY_HUB_ACTIVE,
  MY_HUB_INACTIVE,
  TRANSITION
}
