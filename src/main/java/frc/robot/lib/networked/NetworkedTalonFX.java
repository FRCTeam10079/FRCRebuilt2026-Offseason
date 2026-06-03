// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.networked;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/*
 * TalonFX motor but slot0 is dynamic
 *
 * WARNING: THIS ONLY WORKS WITH SLOT 0 DO NOT ATTEMPT TO USE OTHER SLOTS
 * (this is not designed to be production code, this is code meant for fast prototyping)
 */
public class NetworkedTalonFX extends TalonFX {
  private static int instanceCount;

  private String name;

  // networked prefix to just to avoid ambigiouity between actual values and their
  // networked
  // partners
  private NetworkedDouble networkedKA; // acceleration gain
  private NetworkedDouble networkedkD; // derivative gain
  private NetworkedDouble networkedkG; // gravity gain
  private NetworkedDouble networkedkI; // integral gain
  private NetworkedDouble networkedkP; // proprotional gain
  private NetworkedDouble networkedKS; // static constant (static friction)
  private NetworkedDouble networkedKV; // velo feed forward

  private TalonFXConfiguration activeConfig;

  /**
   * Drop-in constructor for default TalonFX motor (using CANBus object)
   *
   * @param ID motor ID
   * @param canbus CANBus object
   */
  public NetworkedTalonFX(int ID, CANBus canbus) {
    super(ID, canbus);
    instanceCount++;

    this.name = "NetworkedTalonFX_" + instanceCount;
  }

  /**
   * Constructor with name setter and CANBus object support
   *
   * @param ID motor ID
   * @param canbus CANBus object
   * @param name motor name for Network Tables
   */
  public NetworkedTalonFX(int ID, CANBus canbus, String name) {
    super(ID, canbus);
    instanceCount++;

    this.name = name;
  }

  private void setupNT(
      double kA, double kD, double kG, double kI, double kP, double kS, double kV) {
    this.networkedKA = new NetworkedDouble("/NetworkedLib/" + this.name + "/kA", kA);
    this.networkedkD = new NetworkedDouble("/NetworkedLib/" + this.name + "/kD", kD);
    this.networkedkG = new NetworkedDouble("/NetworkedLib/" + this.name + "/kG", kG);
    this.networkedkI = new NetworkedDouble("/NetworkedLib/" + this.name + "/kI", kI);
    this.networkedkP = new NetworkedDouble("/NetworkedLib/" + this.name + "/kP", kP);
    this.networkedKS = new NetworkedDouble("/NetworkedLib/" + this.name + "/kS", kS);
    this.networkedKV = new NetworkedDouble("/NetworkedLib/" + this.name + "/kV", kV);
  }

  /**
   * Use this function instead of Motor.getConfigurator().Apply(_)
   *
   * <p>Designed to apply the config and save the whole config for dynamic updates
   *
   * @param config The TalonFXConfiguration to apply
   */
  public void applyConfiguration(TalonFXConfiguration config) {
    this.activeConfig = config;

    this.getConfigurator().apply(config);
    setupNT(
        activeConfig.Slot0.kA,
        activeConfig.Slot0.kD,
        activeConfig.Slot0.kG,
        activeConfig.Slot0.kI,
        activeConfig.Slot0.kP,
        activeConfig.Slot0.kS,
        activeConfig.Slot0.kV);
  }

  /**
   * Call this in the periodic function to update the motor configs
   *
   * <p>THIS IS REQUIRED FOR THE DYNAMIC SLOT0 TO WORK!
   */
  public void periodic() {
    // checks if any networked doubles have new information
    if (networkedKA.available()
        || networkedkD.available()
        || networkedkG.available()
        || networkedkI.available()
        || networkedkP.available()
        || networkedKS.available()
        || networkedKV.available()) {

      // ensures values like gravity FF type and other are perserved
      this.activeConfig.Slot0 = this.activeConfig
          .Slot0
          .withKA(networkedKA.get())
          .withKD(networkedkD.get())
          .withKG(networkedkG.get())
          .withKI(networkedkI.get())
          .withKP(networkedkP.get())
          .withKS(networkedKS.get())
          .withKV(networkedKV.get());

      // the reason we arent directly applying slot0Configs is because it leaves open
      // more stuff to
      // add to the networked possiblities later.
      // in the future we could network other parts of the config (like current
      // limits!)

      this.getConfigurator().apply(this.activeConfig);
    }
  }
}
