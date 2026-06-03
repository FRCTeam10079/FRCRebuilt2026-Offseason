// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.AlignPosition;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;

/**
 * Testing controller bindings (Port 2). Only active when NOT connected to the FMS — all bindings
 * are no-ops at competition. Provides SysId routines, mechanism joggers, and state overrides for
 * pit/practice testing.
 */
public final class TestingBindings {

  private TestingBindings() {} // Static utility class

  /**
   * Bind all testing controls. Does nothing if FMS is attached (competition safe).
   *
   * @param controller the testing Xbox controller (Port 2)
   * @param drivetrain swerve drivetrain subsystem
   * @param intake intake wheels subsystem
   * @param pivot pivot arm subsystem
   * @param indexer indexer subsystem
   * @param shooter shooter subsystem
   * @param vision vision subsystem
   */
  public static void configure(
      CommandXboxController controller,
      CommandSwerveDrivetrain drivetrain,
      IntakeWheelsSubsystem intake,
      PivotSubsystem pivot,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      Vision vision) {

    // Gate ALL test bindings behind FMS check — at competition these are inert
    if (DriverStation.isFMSAttached()) {
      return;
    }

    // ==================== SYSID CHARACTERIZATION ====================
    // A - SysId quasistatic forward
    controller.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    // B - SysId quasistatic reverse
    controller.b().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // X - SysId dynamic forward
    controller.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));

    // Y - SysId dynamic reverse
    controller.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // ==================== DISPLACED DRIVER/OPERATOR CONTROLS ====================
    // Left/Right Bumper - AprilTag align variants moved from driver
    controller.leftBumper().whileTrue(new AlignToAprilTag(drivetrain, vision, AlignPosition.LEFT));
    controller
        .rightBumper()
        .whileTrue(new AlignToAprilTag(drivetrain, vision, AlignPosition.RIGHT));

    // Start - Reset heading moved from driver
    controller.start().onTrue(Commands.runOnce(drivetrain::resetFieldHeading));

    // POV Up - Pathfind test moved from operator
    controller.povUp().whileTrue(drivetrain.pathfindToAprilTag(18));

    // POV Down - Stow pivot jogger
    controller.povDown().onTrue(pivot.stowCommand());
    controller.povRight().onTrue(pivot.deployCommand());

    Trigger backHeld = controller.back();
    Trigger rightTrigger = controller.rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD);
    Trigger leftTrigger = controller.leftTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD);

    // Right Trigger (without Back) - Feed indexer moved from driver
    rightTrigger.and(backHeld.negate()).whileTrue(indexer.feedCommand());

    // Left Trigger (without Back) - Slow mode moved from driver
    leftTrigger
        .and(backHeld.negate())
        .whileTrue(Commands.startEnd(
            () -> {
              drivetrain.setTeleopVelocityCoefficient(
                  Constants.DrivetrainConstants.SLOW_MODE_COEFFICIENT);
              drivetrain.setRotationVelocityCoefficient(
                  Constants.DrivetrainConstants.SLOW_MODE_COEFFICIENT);
            },
            () -> {
              drivetrain.setTeleopVelocityCoefficient(
                  Constants.DrivetrainConstants.NORMAL_SPEED_COEFFICIENT);
              drivetrain.setRotationVelocityCoefficient(
                  Constants.DrivetrainConstants.NORMAL_SPEED_COEFFICIENT);
            }));

    // Back + X - Heading lock to 0° moved from operator
    backHeld
        .and(controller.x())
        .whileTrue(drivetrain.headingLockedDriveCommand(
            controller::getLeftY,
            controller::getLeftX,
            0.0,
            Constants.DrivetrainConstants.MAX_SPEED_MPS,
            Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // Back + Right Trigger - Heading lock to AprilTag moved from operator
    rightTrigger
        .and(backHeld)
        .whileTrue(drivetrain.headingLockedDriveCommand(
            controller::getLeftY,
            controller::getLeftX,
            () -> computeAprilTagHeading(drivetrain, vision),
            Constants.DrivetrainConstants.MAX_SPEED_MPS,
            Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // Back + Left Trigger - Shooter spin-up test moved from operator
    leftTrigger
        .and(backHeld)
        .whileTrue(shooter.holdRPMCommand(Constants.ShooterConstants.SHOOTER_SPINUP_SPEED));

    new Trigger(shooter::isReady)
        .and(leftTrigger)
        .and(backHeld)
        .onTrue(Commands.runOnce(() -> controller
            .getHID()
            .setRumble(RumbleType.kBothRumble, Constants.StateMachineConstants.RUMBLE_STRONG)))
        .onFalse(
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }

  /**
   * Compute heading to face the currently visible AprilTag. Returns current heading if no tag is
   * visible (maintains position).
   */
  private static double computeAprilTagHeading(CommandSwerveDrivetrain drivetrain, Vision vision) {
    double currentHeading = drivetrain.getState().Pose.getRotation().getDegrees();
    for (String name : VisionConstants.LIMELIGHT_NAMES) {
      if (LimelightHelpers.getTV(name)) {
        return MathUtil.inputModulus(currentHeading - LimelightHelpers.getTX(name), -180.0, 180.0);
      }
    }
    return MathUtil.inputModulus(currentHeading, -180.0, 180.0);
  }
}
