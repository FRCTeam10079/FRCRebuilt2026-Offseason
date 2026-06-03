// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AlignPosition;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.ShootOnTheMoveDrive;
import frc.robot.commands.ShooterFactory;
import frc.robot.lib.LaunchCalculator;
import frc.robot.lib.ShooterInterpolationTable;
import frc.robot.lib.ShooterMath;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.statemachine.FuelState;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CurrentSuperState;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;

/**
 * Driver controller bindings (Port 0). All mechanism actions route through the
 * {@link Superstructure} so it can coordinate subsystems. Drivetrain commands (heading lock, brake,
 * vision align) remain direct since the Superstructure does not manage the drivetrain.
 */
public final class DriverControls {

  private DriverControls() {} // Static utility class

  /**
   * Bind all driver controls.
   *
   * @param controller the driver's Xbox controller
   * @param drivetrain swerve drivetrain subsystem
   * @param vision vision subsystem (for alignment commands)
   * @param superstructure the Superstructure coordinator
   * @param stateMachine global robot state machine (for fuel feedback triggers)
   * @param setpointSupplier memoized distance-based setpoint supplier
   */
  public static void configure(
      CommandXboxController controller,
      CommandSwerveDrivetrain drivetrain,
      Vision vision,
      Superstructure superstructure,
      RobotStateMachine stateMachine,
      Supplier<ShooterSetpoint> setpointSupplier) {

    // Toggle to invert driver translation controls (left stick X/Y).
    final boolean[] invertTranslation = {false};
    Supplier<Double> translationY =
        () -> invertTranslation[0] ? -controller.getLeftY() : controller.getLeftY();
    Supplier<Double> translationX =
        () -> invertTranslation[0] ? -controller.getLeftX() : controller.getLeftX();

    // ==================== DEFAULT DRIVE ====================
    drivetrain.setDefaultCommand(drivetrain.smoothTeleopDriveCommand(
        translationY::get,
        translationX::get,
        // controller::getLeftY,
        // controller::getLeftX,
        () -> controller.getRightX(),
        Constants.DrivetrainConstants.MAX_SPEED_MPS,
        Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // ==================== INTAKE (independent of main state) ====================
    // Left Trigger - Hold to collect (deploy pivot + run intake wheels).
    // Runs alongside any main state (AIM, SHOOT, SOTM, etc.).
    controller
        .leftTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
        .onTrue(Commands.runOnce(() -> superstructure.setIntakeActive(true)))
        .onFalse(Commands.runOnce(() -> superstructure.setIntakeActive(false)));

    // ==================== SHOOTING (through Superstructure) ====================
    // Right Bumper - Hold to aim at hub (heading lock) + pre-spin via
    // Superstructure
    // Drivetrain heading lock is direct (Superstructure doesn't manage drivetrain).
    // Superstructure handles shooter + pivot coordination via AIM state.
    controller
        .rightBumper()
        .onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.AIM)))
        .onFalse(updateWantedStateFromDriverInputs(controller, superstructure));

    // Heading lock while RB is held (drivetrain-only, parallel to Superstructure
    // AIM)
    controller
        .rightBumper()
        .whileTrue(createAimAtHubCommand(drivetrain, translationY, translationX));

    // Right Trigger - Hold to force-shoot (aim + feed when flywheel ready).
    // On release, rebuild intent from any still-held driver controls.
    controller
        .rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
        .onTrue(Commands.runOnce(
            () -> superstructure.setWantedSuperState(WantedSuperState.FORCE_SHOOT)))
        .onFalse(updateWantedStateFromDriverInputs(controller, superstructure));

    // Rumble while actively shooting
    new Trigger(() -> superstructure.getCurrentSuperState()
                == Superstructure.CurrentSuperState.FORCE_SHOOTING
            || superstructure.getCurrentSuperState() == Superstructure.CurrentSuperState.SHOOTING)
        .and(controller.rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD))
        .onTrue(Commands.runOnce(() -> controller
            .getHID()
            .setRumble(RumbleType.kBothRumble, Constants.StateMachineConstants.RUMBLE_STRONG)))
        .onFalse(
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // ==================== SHOOT ON THE MOVE ====================
    // Left Bumper - Hold to activate SOTM. Use a single trigger lifecycle so
    // onTrue/onFalse always pair correctly, even if the robot crosses a zone
    // boundary while held.
    final boolean[] useScoringZoneSotm = {true};
    Trigger sotmTrigger = controller.leftBumper();

    sotmTrigger
        .onTrue(Commands.sequence(
            // Latch which SOTM drive implementation to use for this hold to avoid
            // jittery command swapping near the split boundary.
            Commands.runOnce(() -> {
              var params = LaunchCalculator.getInstance().getParameters();
              // Passing=true means far-side ferrying mode; scoring-zone mode is
              // the inverse.
              useScoringZoneSotm[0] = params == null || !params.passing();
            }),
            Commands.runOnce(
                () -> superstructure.getShooterPivot().setTrenchAutoLowerEnabled(false)),
            Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.SOTM))))
        .onFalse(Commands.sequence(
            Commands.runOnce(
                () -> superstructure.getShooterPivot().setTrenchAutoLowerEnabled(true)),
            updateWantedStateFromDriverInputs(controller, superstructure)));

    // Scoring-zone SOTM (non-passing): use Oliver's ShootOnTheMoveDrive.
    sotmTrigger
        .and(() -> useScoringZoneSotm[0])
        .whileTrue(
            new ShootOnTheMoveDrive(drivetrain, vision, translationY::get, translationX::get));

    // Passing/ferrying SOTM: use branch drivetrain SOTM implementation.
    // Uses translationY/translationX so invertTranslation toggle is honored.
    sotmTrigger
        .and(() -> !useScoringZoneSotm[0])
        .whileTrue(drivetrain.shootOnTheMoveDriveCommand(
            () -> translationY.get(), () -> translationX.get()));

    // Rumble when SOTM is actively feeding (left bumper held + SOTM_SHOOTING)
    new Trigger(() -> superstructure.getCurrentSuperState() == CurrentSuperState.SOTM_SHOOTING)
        .and(controller.leftBumper())
        .onTrue(Commands.runOnce(() -> controller
            .getHID()
            .setRumble(RumbleType.kBothRumble, Constants.StateMachineConstants.RUMBLE_STRONG)))
        .onFalse(
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // ==================== VISION ALIGNMENT ====================
    // A - Align to AprilTag (CENTER)
    controller.a().whileTrue(new AlignToAprilTag(drivetrain, vision, AlignPosition.CENTER));

    controller.b().onTrue(Commands.runOnce(() -> invertTranslation[0] = !invertTranslation[0]));

    // ==================== TOF TUNING (D-PAD LEFT/RIGHT) ====================
    // D-pad Left - Increase TOF at nearest distance key
    // D-pad Right - Decrease TOF at nearest distance key
    // Prints current distance bucket and TOF value on each press.
    controller.povLeft().onTrue(Commands.runOnce(() -> {
      double dist = ShooterMath.getDistanceToHub(drivetrain.getState().Pose)
          .in(edu.wpi.first.units.Units.Meters);
      ShooterInterpolationTable.adjustTof(dist, true);
    }));
    controller.povRight().onTrue(Commands.runOnce(() -> {
      double dist = ShooterMath.getDistanceToHub(drivetrain.getState().Pose)
          .in(edu.wpi.first.units.Units.Meters);
      ShooterInterpolationTable.adjustTof(dist, false);
    }));
    // D-pad Up - Print current TOF at nearest key (read-only check)
    controller.povUp().onTrue(Commands.runOnce(() -> {
      double dist = ShooterMath.getDistanceToHub(drivetrain.getState().Pose)
          .in(edu.wpi.first.units.Units.Meters);
      ShooterInterpolationTable.printCurrentTof(dist);
    }));

    // ==================== STOW (through Superstructure) ====================
    // D-pad Down - Stow intake pivot + stop intake wheels
    controller.povDown().onTrue(Commands.runOnce(() -> superstructure.stowIntake()));

    // ======== UNJAM / EJECT (through Superstructure) ========
    // X - Hold reverse intake + indexer
    controller
        .x()
        .onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.UNJAM)))
        .onFalse(updateWantedStateFromDriverInputs(controller, superstructure));

    // ==================== X-STANCE ====================
    // X - Hold defensive wheel lock
    SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    controller.x().whileTrue(drivetrain.applyRequest(() -> brakeRequest));

    // ==================== DRIVER FEEDBACK ====================
    // Pulse rumble while loaded so driver knows they can leave loading zone.
    // Suppressed while SOTM is active (left bumper held) so the steady SOTM
    // readiness rumble isn't overwritten by the pulse's silence phase.
    new Trigger(() -> stateMachine.getFuelState() == FuelState.LOADED)
        .and(controller.leftBumper().negate())
        .whileTrue(Commands.repeatingSequence(
            Commands.runOnce(() -> controller
                .getHID()
                .setRumble(RumbleType.kBothRumble, Constants.StateMachineConstants.RUMBLE_LIGHT)),
            Commands.waitSeconds(0.15),
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)),
            Commands.waitSeconds(0.45)));
  }

  private static Command createAimAtHubCommand(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Double> translationY,
      Supplier<Double> translationX) {
    return ShooterFactory.aimAtHub(
        drivetrain,
        translationY::get,
        translationX::get,
        () -> ShooterMath.getHeadingToHub(drivetrain.getState().Pose),
        Constants.DrivetrainConstants.MAX_ALIGNING_SPEED_MPS,
        Constants.DrivetrainConstants.MAX_ALIGNING_ANGULAR_RATE_RAD_PER_SEC);
  }

  private static Command updateWantedStateFromDriverInputs(
      CommandXboxController controller, Superstructure superstructure) {
    return Commands.runOnce(() -> {
      // Rebuild intent from live button states because trigger onTrue is edge-only.
      if (controller
          .rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
          .getAsBoolean()) {
        superstructure.setWantedSuperState(WantedSuperState.FORCE_SHOOT);
      } else if (controller.rightBumper().getAsBoolean()) {
        superstructure.setWantedSuperState(WantedSuperState.AIM);
      } else if (controller.leftBumper().getAsBoolean()) {
        superstructure.setWantedSuperState(WantedSuperState.SOTM);
      } else {
        superstructure.setWantedSuperState(WantedSuperState.IDLE);
      }
    });
  }
}
