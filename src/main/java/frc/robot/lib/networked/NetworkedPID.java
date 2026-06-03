// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.networked;

import edu.wpi.first.math.controller.PIDController;

/**
 * Exactly like the original PIDController Except it allows for quick tuning via network modified
 * values
 */
public class NetworkedPID extends PIDController {
  private static int instanceCount;

  private String name;

  private NetworkedDouble networkedKp;
  private NetworkedDouble networkedKi;
  private NetworkedDouble networkedKd;

  /**
   * Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
   * 0.02 seconds.
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   * @throws IllegalArgumentException if kp &lt; 0
   * @throws IllegalArgumentException if ki &lt; 0
   * @throws IllegalArgumentException if kd &lt; 0
   */
  public NetworkedPID(double kp, double ki, double kd) {
    super(kp, ki, kd);
    instanceCount++;

    this.name = "NetworkedPIDController" + instanceCount;

    setupNT(kp, ki, kd);
  }

  /**
   * Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
   * 0.02 seconds. Takes in a name for the NetworkTable
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   * @param name The name to use in the NetworkTable
   * @throws IllegalArgumentException if kp &lt; 0
   * @throws IllegalArgumentException if ki &lt; 0
   * @throws IllegalArgumentException if kd &lt; 0
   */
  public NetworkedPID(double kp, double ki, double kd, String name) {
    super(kp, ki, kd);
    instanceCount++;

    this.name = name;

    setupNT(kp, ki, kd);
  }

  /**
   * Allocates a PIDController with the given constants for kp, ki, and kd. Takes in a name for the
   * NetworkTable
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   * @param period The period between controller updates in seconds.
   * @param name The name to use in the NetworkTable
   * @throws IllegalArgumentException if kp &lt; 0
   * @throws IllegalArgumentException if ki &lt; 0
   * @throws IllegalArgumentException if kd &lt; 0
   * @throws IllegalArgumentException if period &lt;= 0
   */
  public NetworkedPID(double kp, double ki, double kd, double period, String name) {
    super(kp, ki, kd, period);
    instanceCount++;

    this.name = name;

    setupNT(kp, ki, kd);
  }

  private void setupNT(double kp, double ki, double kd) {
    this.networkedKp = new NetworkedDouble("/NetworkedLib/" + name + "/kp", kp);
    this.networkedKi = new NetworkedDouble("/NetworkedLib/" + name + "/ki", ki);
    this.networkedKd = new NetworkedDouble("/NetworkedLib/" + name + "/kd", kd);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param setpoint The new setpoint of the controller.
   * @return The next controller output.
   */
  public double calculate(double measurement, double setpoint) {
    updatePID();
    return super.calculate(measurement, setpoint);
  }

  /** Updates the current PID with new values */
  public void updatePID() {
    this.setPID(networkedKp.get(), networkedKi.get(), networkedKd.get());
  }
}
