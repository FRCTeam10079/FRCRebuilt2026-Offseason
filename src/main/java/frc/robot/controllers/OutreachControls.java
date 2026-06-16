// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.ShooterFactory;
import frc.robot.lib.ShooterMath;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CurrentSuperState;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

/**
 * Outreach / demo mode controller bindings (Port 0 only).
 *
 * <p>All drive speeds are capped at 50 % of competition values via the drivetrain velocity
 * coefficients. An extra-slow mode (Left Bumper hold) further reduces to 25 % for nervous
 * first-time drivers. Complex competition-only features (SOTM, climb, shooter tuning, invert
 * toggle) are intentionally omitted.
 *
 * <p><b>Driver controller layout:</b>
 *
 * <ul>
 *   <li><b>Left Stick</b> - translate (50 % max speed)
 *   <li><b>Right Stick X</b> - rotate (50 % max angular rate)
 *   <li><b>Left Trigger</b> - intake (hold to collect)
 *   <li><b>Right Trigger</b> - force shoot (hold; feeds as soon as flywheel is at speed)
 *   <li><b>Right Bumper</b> - aim at hub (heading lock + pre-spin flywheel; hold)
 *   <li><b>Left Bumper</b> - extra-slow mode while held (25 % of full speed)
 *   <li><b>Start + Back</b> - extend climber (two-button safety combo, same as competition)
 *   <li><b>Y</b> - retract climber (only active while climbing and extended)
 *   <li><b>B</b> - stow intake
 *   <li><b>X</b> - unjam (reverse intake + indexer; hold)
 *   <li><b>D-Pad Down</b> - stow intake pivot
 *   <li><b>Start</b> - reset field-centric heading (when held alone, not with Back)
 * </ul>
 *
 * <p>Operator controller (Port 1) is not configured in outreach mode; all actions are accessible
 * from the single driver controller above.
 */
public final class OutreachControls {

  private OutreachControls() {} // Static utility class

  /**
   * Configure outreach mode controls on the driver controller and set drivetrain speed caps for the
   * session.
   *
   * @param controller the driver's Xbox controller (Port 0)
   * @param drivetrain swerve drivetrain subsystem
   * @param vision vision subsystem (for heading to hub)
   * @param superstructure the Superstructure coordinator
   * @param climber climber subsystem (for end-game demo)
   */
  public static void configure(
      CommandXboxController controller,
      CommandSwerveDrivetrain drivetrain,
      Vision vision,
      Superstructure superstructure,
      ClimberSubsystem climber) {

    // ==================== SPEED CAP ====================
    // Lock both translation and rotation to 50 % for the entire outreach session.
    // Every drive path (default, heading lock, AprilTag align) reads these
    // coefficients internally, so this single call caps all driving.
    drivetrain.setTeleopVelocityCoefficient(
        Constants.DrivetrainConstants.OUTREACH_SPEED_COEFFICIENT);
    drivetrain.setRotationVelocityCoefficient(
        Constants.DrivetrainConstants.OUTREACH_SPEED_COEFFICIENT);

    // ==================== DEFAULT DRIVE ====================
    drivetrain.setDefaultCommand(drivetrain.smoothTeleopDriveCommand(
        controller::getLeftY,
        controller::getLeftX,
        controller::getRightX,
        Constants.DrivetrainConstants.MAX_SPEED_MPS,
        Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // ==================== EXTRA-SLOW MODE ====================
    // Left Bumper (hold) - halves the outreach coefficient while held,
    // bringing effective speed to 25 % of full competition speed.
    controller
        .leftBumper()
        .whileTrue(Commands.startEnd(
            () -> {
              drivetrain.setTeleopVelocityCoefficient(
                  Constants.DrivetrainConstants.OUTREACH_SPEED_COEFFICIENT * 0.5);
              drivetrain.setRotationVelocityCoefficient(
                  Constants.DrivetrainConstants.OUTREACH_SPEED_COEFFICIENT * 0.5);
            },
            () -> {
              drivetrain.setTeleopVelocityCoefficient(
                  Constants.DrivetrainConstants.OUTREACH_SPEED_COEFFICIENT);
              drivetrain.setRotationVelocityCoefficient(
                  Constants.DrivetrainConstants.OUTREACH_SPEED_COEFFICIENT);
            }));

    // ==================== INTAKE ====================
    // Left Trigger (hold) - deploy intake pivot + run intake wheels.
    controller
        .leftTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
        .onTrue(Commands.runOnce(() -> superstructure.setIntakeActive(true)))
        .onFalse(Commands.runOnce(() -> superstructure.setIntakeActive(false)));

    // ==================== AIM AT HUB ====================
    // Right Bumper (hold) - pre-spin flywheel + heading lock toward hub.
    // Cool demo feature: robot auto-rotates to face the target.
    controller
        .rightBumper()
        .onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.AIM)))
        .onFalse(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.IDLE)));

    controller
        .rightBumper()
        .whileTrue(ShooterFactory.aimAtHub(
            drivetrain,
            controller::getLeftY,
            controller::getLeftX,
            () -> ShooterMath.getHeadingToHub(drivetrain.getState().Pose),
            Constants.DrivetrainConstants.MAX_ALIGNING_SPEED_MPS,
            Constants.DrivetrainConstants.MAX_ALIGNING_ANGULAR_RATE_RAD_PER_SEC));

    // ==================== FORCE SHOOT ====================
    // Right Trigger (hold) - feed the indexer once the flywheel reaches speed.
    // Skips heading/pivot gate checks so the shot fires without needing
    // precise aim - forgiving for first-time drivers.
    controller
        .rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
        .onTrue(Commands.runOnce(
            () -> superstructure.setWantedSuperState(WantedSuperState.FORCE_SHOOT)))
        .onFalse(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.IDLE)));

    // Light rumble while the robot is actively shooting (gentle feedback).
    new Trigger(() -> superstructure.getCurrentSuperState() == CurrentSuperState.FORCE_SHOOTING)
        .and(controller.rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD))
        .onTrue(Commands.runOnce(() -> controller
            .getHID()
            .setRumble(RumbleType.kBothRumble, Constants.StateMachineConstants.RUMBLE_LIGHT)))
        .onFalse(
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // ==================== STOW ====================
    // B - stow intake
    controller.b().onTrue(Commands.runOnce(() -> superstructure.stowIntake()));

    // D-Pad Down - also stows the intake pivot (redundant safety stow).
    controller.povDown().onTrue(Commands.runOnce(() -> superstructure.stowIntake()));

    // ==================== UNJAM ====================
    // X (hold) - reverse indexer + intake to clear a jam.
    controller
        .x()
        .onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.UNJAM)))
        .onFalse(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.IDLE)));

    // ==================== CLIMB ====================
    // Start + Back (held simultaneously) - extend the climber.
    // Two-button combo prevents accidental activation by visitors.
    // Mirrors the exact guard logic from OperatorControls for consistency.
    final boolean[] wasComboActive = {false};
    Trigger comboBoth =
        new Trigger(() -> controller.start().getAsBoolean() && controller.back().getAsBoolean());

    comboBoth.onTrue(Commands.runOnce(() -> wasComboActive[0] = true));
    new Trigger(() -> !controller.start().getAsBoolean() && !controller.back().getAsBoolean())
        .onTrue(Commands.runOnce(() -> wasComboActive[0] = false));

    comboBoth.onTrue(Commands.sequence(
        Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.CLIMB)),
        climber.extendCommand()));

    // Y - retract climber (only fires when climbing and extended).
    controller
        .y()
        .onTrue(Commands.either(
            climber.retractCommand(),
            Commands.none(),
            () -> superstructure.isClimbing() && climber.isExtended()));

    // Back alone (not with Start, not just released from combo) - abort climb.
    controller
        .back()
        .and(() -> !controller.start().getAsBoolean())
        .and(() -> !wasComboActive[0])
        .and(() -> superstructure.isClimbing())
        .onTrue(Commands.sequence(
            climber.abortCommand(),
            Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.IDLE))));

    // ==================== HEADING RESET ====================
    // Start alone (not with Back, not just released from combo) - reset heading.
    controller
        .start()
        .and(() -> !controller.back().getAsBoolean())
        .and(() -> !wasComboActive[0])
        .onTrue(Commands.runOnce(drivetrain::resetFieldHeading));
  }
}
