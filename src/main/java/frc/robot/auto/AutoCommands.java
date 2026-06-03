// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.AlignPosition;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CurrentSuperState;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Factory for autonomous command compositions. Each method returns a <b>new</b> Command instance so
 * it can be safely registered with both PathPlanner NamedCommands and Choreo AutoFactory bindings
 * (WPILib commands cannot be shared across multiple triggers).
 *
 * <p>This class eliminates the massive duplication that previously existed between
 * registerNamedCommands() and registerChoreoBindings() in RobotContainer.
 */
public class AutoCommands {

  private final Superstructure superstructure;
  private final IntakeWheelsSubsystem intake;
  private final PivotSubsystem pivot;
  private final IndexerSubsystem indexer;
  private final ShooterSubsystem shooter;
  private final ShooterPivotSubsystem shooterPivot;
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;

  private final ClimberSubsystem climber;
  private final Supplier<ShooterSetpoint> setpointSupplier;

  public AutoCommands(
      Superstructure superstructure,
      IntakeWheelsSubsystem intake,
      PivotSubsystem pivot,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      ShooterPivotSubsystem shooterPivot,
      CommandSwerveDrivetrain drivetrain,
      Vision vision,
      ClimberSubsystem climber,
      Supplier<ShooterSetpoint> setpointSupplier) {
    this.superstructure = superstructure;
    this.intake = intake;
    this.pivot = pivot;
    this.indexer = indexer;
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.climber = climber;
    this.setpointSupplier = setpointSupplier;
  }

  // ==================== INTAKE WHEELS ====================

  /** Run intake motors to collect game pieces (runs until cancelled). */
  public Command intake() {
    return intake.intakeInCommand();
  }

  /** Reverse intake wheels (unjam / eject). */
  public Command intakeOut() {
    return intake.intakeOutCommand();
  }

  /** Stop intake wheels immediately. */
  public Command stopIntake() {
    return intake.stopCommand();
  }

  // ==================== PIVOT ====================

  /** Deploy the intake pivot arm to pickup position, waits until setpoint reached. */
  public Command deployPivot() {
    return pivot.deployCommand();
  }

  /** Stow the intake pivot arm, waits until setpoint reached. */
  public Command stowPivot() {
    return pivot.stowCommand();
  }

  /** Deploy pivot + spin intake via Superstructure COLLECT state. Runs until cancelled. */
  public Command intakeFuel() {
    return Commands.startEnd(
        () -> superstructure.setWantedSuperState(WantedSuperState.COLLECT),
        () -> superstructure.setWantedSuperState(WantedSuperState.IDLE));
  }

  /** Return to IDLE (stow pivot + stop intake) via Superstructure. */
  public Command endIntaking() {
    return Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.STOW));
  }

  // ==================== INDEXER ====================

  /** Feed game pieces forward through the indexer. */
  public Command runIndexer() {
    return indexer.feedCommand();
  }

  /** Reverse indexer to unjam. */
  public Command reverseIndexer() {
    return indexer.reverseCommand();
  }

  /** Stop indexer immediately. */
  public Command stopIndexer() {
    return indexer.stopCommand();
  }

  // ==================== SHOOTER ====================

  /** Spin up flywheel to target RPM and hold. */
  public Command spinUpShooter() {
    return shooter.holdRPMCommand(Constants.ShooterConstants.SHOOTER_SPINUP_SPEED);
  }

  /** Stop shooter immediately. */
  public Command stopShooter() {
    return shooter.stopCommand();
  }

  /**
   * Distance-based auto shoot via Superstructure. Sets SHOOT state and waits for the Superstructure
   * to transition to SHOOTING (on-target), then holds for a feed duration before returning to IDLE.
   * In auto, heading is assumed correct from the trajectory, so this uses FORCE_SHOOT.
   */
  public Command shoot() {
    return Commands.sequence(
            Commands.runOnce(
                () -> superstructure.setWantedSuperState(WantedSuperState.FORCE_SHOOT)),
            Commands.waitUntil(
                () -> superstructure.getCurrentSuperState() == CurrentSuperState.FORCE_SHOOTING),
            Commands.waitSeconds(Constants.ShooterConstants.AUTO_SHOOT_TIMEOUT.in(
                edu.wpi.first.units.Units.Seconds)))
        .finallyDo(() -> superstructure.setWantedSuperState(WantedSuperState.IDLE));
  }

  /** Fixed fender shot via Superstructure. Uses force-shoot for close-range. */
  public Command fenderShot() {
    return shoot(); // Same flow - Superstructure picks setpoint from distance
  }

  // ==================== VISION ALIGNMENT ====================

  /** Align to AprilTag - CENTER position. */
  public Command alignCenter() {
    return align(AlignPosition.CENTER);
  }

  /** Align to AprilTag - LEFT position. */
  public Command alignLeft() {
    return align(AlignPosition.LEFT);
  }

  /** Align to AprilTag - RIGHT position. */
  public Command alignRight() {
    return align(AlignPosition.RIGHT);
  }

  private Command align(AlignPosition position) {
    return new AlignToAprilTag(drivetrain, vision, position);
  }

  // ==================== STOP ALL ====================

  /** Stop all mechanisms via Superstructure IDLE state. */
  public Command stopAll() {
    return Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.IDLE));
  }

  // ==================== CLIMBER ====================

  /** Extend the climber and brake at the top. */
  public Command extendClimber() {
    return climber.extendCommand();
  }

  /** Enter CLIMB mode through Superstructure, then extend the climber. */
  public Command startClimbExtend() {
    return Commands.sequence(
        Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.CLIMB)),
        climber.extendCommand());
  }

  /** Retract the climber from the extended position. */
  public Command retractClimber() {
    return climber.retractCommand();
  }

  /** Retract to the mechanical zero hard stop and re-zero the encoder. */
  public Command retractClimberToZero() {
    return climber.retractToZeroCommand();
  }

  /** Full climb sequence: extend, brake at top, then retract. */
  public Command climb() {
    return climber.climbCommand();
  }

  /** Abort/stop the climber. */
  public Command abortClimber() {
    return climber.abortCommand();
  }

  /** Abort the climber and return Superstructure to IDLE. */
  public Command abortClimberAndIdle() {
    return Commands.sequence(
        climber.abortCommand(),
        Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.IDLE)));
  }

  // ==================== BULK REGISTRATION ====================

  /**
   * Register all named commands for PathPlanner autonomous routines. Must be called BEFORE any
   * PathPlannerAuto or AutoBuilder.buildAutoChooser() calls.
   */
  public void registerPathPlannerCommands() {
    // Intake
    NamedCommands.registerCommand("intake", intake());
    NamedCommands.registerCommand("intakeOut", intakeOut());
    NamedCommands.registerCommand("stopIntake", stopIntake());

    // Pivot
    NamedCommands.registerCommand("deployPivot", deployPivot());
    NamedCommands.registerCommand("stowPivot", stowPivot());

    // Combined intake + pivot
    NamedCommands.registerCommand("intakeFuel", intakeFuel());
    NamedCommands.registerCommand("endIntaking", endIntaking());

    // Indexer
    NamedCommands.registerCommand("runIndexer", runIndexer());
    NamedCommands.registerCommand("reverseIndexer", reverseIndexer());
    NamedCommands.registerCommand("stopIndexer", stopIndexer());

    // Shooter
    NamedCommands.registerCommand("spinUpShooter", spinUpShooter());
    NamedCommands.registerCommand("stopShooter", stopShooter());
    NamedCommands.registerCommand("shoot", shoot());
    NamedCommands.registerCommand("fenderShot", fenderShot());

    // Vision alignment
    NamedCommands.registerCommand("alignCenter", alignCenter());
    NamedCommands.registerCommand("alignLeft", alignLeft());
    NamedCommands.registerCommand("alignRight", alignRight());

    // Stop all
    NamedCommands.registerCommand("stopAll", stopAll());

    // Climber
    NamedCommands.registerCommand("extendClimber", extendClimber());
    NamedCommands.registerCommand("startClimbExtend", startClimbExtend());
    NamedCommands.registerCommand("retractClimber", retractClimber());
    NamedCommands.registerCommand("retractClimberToZero", retractClimberToZero());
    NamedCommands.registerCommand("climb", climb());
    NamedCommands.registerCommand("abortClimber", abortClimber());
    NamedCommands.registerCommand("abortClimberAndIdle", abortClimberAndIdle());

    // Aliases matching capitalized names used in PathPlanner auto files
    // (Right_OutPost.auto uses "Intake down", "Shoot", "Intake")
    NamedCommands.registerCommand("Intake down", intakeFuel());
    NamedCommands.registerCommand("Shoot", shoot());
    NamedCommands.registerCommand("Intake", intake());

    Logger.recordOutput("Events/AutoCommands/Last", "Named commands registered for PathPlanner");
  }

  /**
   * Register Choreo global marker bindings for subsystem actions. These bindings are evaluated from
   * event markers inside Choreo trajectories.
   */
  public void registerChoreoBindings(AutoFactory factory) {
    factory
        // Intake
        .bind("intake", intake())
        .bind("intakeOut", intakeOut())
        .bind("stopIntake", stopIntake())
        // Pivot
        .bind("deployPivot", deployPivot())
        .bind("stowPivot", stowPivot())
        // Combined intake + pivot
        .bind("intakeFuel", intakeFuel())
        .bind("endIntaking", endIntaking())
        // Indexer
        .bind("runIndexer", runIndexer())
        .bind("reverseIndexer", reverseIndexer())
        .bind("stopIndexer", stopIndexer())
        // Shooter
        .bind("spinUpShooter", spinUpShooter())
        .bind("stopShooter", stopShooter())
        .bind("shoot", shoot())
        .bind("fenderShot", fenderShot())
        // Vision alignment
        .bind("alignCenter", alignCenter())
        .bind("alignLeft", alignLeft())
        .bind("alignRight", alignRight())
        // Stop all
        .bind("stopAll", stopAll())
        // Aliases matching auto file names
        .bind("Intake down", intakeFuel())
        .bind("Shoot", shoot())
        .bind("Intake", intake());

    Logger.recordOutput("Events/AutoCommands/Last", "Marker bindings registered for Choreo");
  }
}
