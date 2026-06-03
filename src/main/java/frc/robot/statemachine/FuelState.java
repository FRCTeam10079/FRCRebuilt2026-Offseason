// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachine;

/** Ball inventory tracking. Wire up when indexer sensors are installed. */
public enum FuelState {
  EMPTY,
  ACQUIRING,
  LOADED,
  FIRING
}
