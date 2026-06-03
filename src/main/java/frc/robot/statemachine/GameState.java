// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachine;

/**
 * High-level robot strategy. Slimmed to states that code actually transitions to plus meaningful
 * placeholders for mechanisms being built.
 */
public enum GameState {
  // Core states (actively used by state machine)
  IDLE("Idle/Stowed"),
  AUTO_RUNNING("Executing Autonomous"),
  TRANSITION("Both Hubs ACTIVE"),
  HUB_ACTIVE("Offensive - Our Hub Active"),
  HUB_INACTIVE("Defensive - Our Hub Inactive"),

  // Mechanism states (wire up as hardware comes online)
  COLLECTING("Actively Intaking"),
  SCORING("Aiming & Shooting"),
  DEFENDING("Blocking Opponent"),
  CLIMBING("Actively Climbing"),
  CLIMBED("Climb Complete"),

  // Override / Safety
  MANUAL_OVERRIDE("Driver Direct Control"),
  EMERGENCY_STOP("All Systems Halted");

  public final String description;

  GameState(String description) {
    this.description = description;
  }

  public boolean isOffensive() {
    return this == HUB_ACTIVE || this == SCORING || this == TRANSITION;
  }

  public boolean isDefensive() {
    return this == HUB_INACTIVE || this == DEFENDING;
  }

  public boolean isEndgame() {
    return this == CLIMBING || this == CLIMBED;
  }

  public boolean isClimbing() {
    return this == CLIMBING;
  }

  public boolean isAuto() {
    return this == AUTO_RUNNING;
  }
}
