// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachine;

/** Tower climbing progress. Wire up when climber hardware is built. */
public enum ClimbState {
  NOT_CLIMBING,
  APPROACHING,
  CLIMBING_L1,
  CLIMBING_L2,
  CLIMBING_L3,
  ENGAGED,
  FAILED;

  public boolean isClimbing() {
    return this == CLIMBING_L1 || this == CLIMBING_L2 || this == CLIMBING_L3;
  }

  public boolean isCompleted() {
    return this == ENGAGED;
  }
}
