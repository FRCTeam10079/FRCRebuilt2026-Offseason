// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Central auto-mode configuration. Builds the SmartDashboard chooser for PathPlanner autos and
 * provides the selected autonomous command to Robot.java.
 *
 * <p>Uses AutoFactory.trajectoryCmd() / resetOdometry() for simple command composition instead of
 * AutoRoutine.
 */
public class Autos {

  private final AutoFactory choreoAutoFactory;
  private final AutoCommands autoCommands;

  private final SendableChooser<Command> pathPlannerChooser;
  private final SendableChooser<String> choreoChooser = new SendableChooser<>();

  /**
   * Create the auto configuration and publish choosers to SmartDashboard.
   *
   * @param choreoAutoFactory the Choreo AutoFactory (already created in RobotContainer)
   */
  public Autos(AutoFactory choreoAutoFactory, AutoCommands autoCommands) {
    this.choreoAutoFactory = choreoAutoFactory;
    this.autoCommands = autoCommands;

    // PathPlanner chooser — populates from deploy/pathplanner/autos/
    pathPlannerChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", pathPlannerChooser);

    // Removed because the project no longer uses Choreo, and this extra chooser was
    // causing
    // confusion on the dashboard.
    configureChoreoChooser();
  }

  // ==================== CHOOSER BUILDERS ====================

  private void configureChoreoChooser() {
    choreoChooser.setDefaultOption("RS", "RS");
    choreoChooser.addOption("LS_Depot", "LS_Depot");
    choreoChooser.addOption("LS_Neutral", "LS_Neutral");
    choreoChooser.addOption("MS_Depot_Climb", "MS_Depot_Climb");
    choreoChooser.addOption("None", "");
    choreoChooser.addOption("Right_OutPost", "Right_OutPost");
    choreoChooser.addOption("LeftSideDepot", "LeftSideDepot");
  }

  // ==================== AUTO COMMAND SELECTION ====================

  /**
   * Returns the autonomous command selected by the dashboard. Priority:
   *
   * <ol>
   *   <li>PathPlanner auto (from the auto chooser)
   *   <li>Fallback: no-op command
   * </ol>
   */
  public Command getSelected() {

    // 2) PathPlanner
    Command ppAuto = pathPlannerChooser.getSelected();
    if (ppAuto == null) {
      // AD*-based pathfinding can take several seconds to compute before movement and
      // must never
      // be the default at auto start; when selection is null, do nothing rather than
      // guess.
      return Commands.none().withName("Auto: No Selection");
    }
    return ppAuto;
  }

  /**
   * Build a simple Choreo autonomous command for a single trajectory using command composition (no
   * AutoRoutine).
   *
   * @param trajectoryName the Choreo trajectory file name (e.g. "RS_Outpost")
   * @return a command that resets odometry and follows the trajectory
   */
  public Command getChoreoAuto(String trajectoryName) {
    return Commands.sequence(
            choreoAutoFactory.resetOdometry(trajectoryName),
            choreoAutoFactory.trajectoryCmd(trajectoryName))
        .withName("Choreo: " + trajectoryName);
  }

  // OLD AutoRoutine-based implementation:
  // public Command getChoreoAuto(String trajectoryName) {
  // AutoRoutine routine = choreoAutoFactory.newRoutine("SelectedChoreo");
  // AutoTrajectory trajectory = routine.trajectory(trajectoryName);
  // routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd()));
  // return routine.cmd().withName("Choreo: " + trajectoryName);
  // }

  // ==================== CHOREO COMMAND COMPOSITIONS ====================

  /**
   * Right-side depot auto: drive to outpost, deploy pivot, score, return to neutral. Uses simple
   * command composition with AutoFactory.trajectoryCmd().
   */
  public Command rightSideOp() {
    return Commands.sequence(
            choreoAutoFactory.resetOdometry("rStartToOp"),
            Commands.parallel(
                choreoAutoFactory.trajectoryCmd("rStartToOp"), autoCommands.deployPivot()),
            Commands.parallel(
                choreoAutoFactory.trajectoryCmd("OpToRScore"), autoCommands.spinUpShooter()),
            Commands.parallel(autoCommands.runIndexer(), autoCommands.shoot().withTimeout(5.0)),
            choreoAutoFactory
                .trajectoryCmd("RScoreToNeutral")
                .withTimeout(4.0)
                .andThen(autoCommands.intake()))
        .withName("Choreo: RightSideOp");
  }

  // OLD AutoRoutine-based implementation:
  // public AutoRoutine RightSideOp() {
  // AutoRoutine routine = choreoAutoFactory.newRoutine("RighSideOp");
  // AutoTrajectory rStartToOp = routine.trajectory("rStartToOp");
  // AutoTrajectory OpToRScore = routine.trajectory("OpToRScore");
  // AutoTrajectory RScoreToNeutral = routine.trajectory("RScoreToNeutral");
  // routine.active().onTrue(
  // Commands.sequence(
  // rStartToOp.resetOdometry(),
  // Commands.parallel(rStartToOp.cmd(), autoCommands.deployPivot()),
  // Commands.parallel(OpToRScore.cmd(), autoCommands.spinUpShooter()),
  // Commands.parallel(autoCommands.runIndexer(),
  // autoCommands.shoot().withTimeout(5.0)),
  // RScoreToNeutral.cmd().withTimeout(4.0).andThen(autoCommands.intake())));
  // return routine;
  // }

  /**
   * Left-side depot auto: drive to neutral, score, cycle through depot. Uses simple command
   * composition with AutoFactory.trajectoryCmd().
   */
  public Command leftSideDepot() {
    return Commands.sequence(
            choreoAutoFactory.resetOdometry("lStartToNeutral"),
            Commands.parallel(
                choreoAutoFactory.trajectoryCmd("lStartToNeutral"), autoCommands.deployPivot()),
            Commands.parallel(
                choreoAutoFactory.trajectoryCmd("LneutralToLscore"), autoCommands.spinUpShooter()),
            Commands.parallel(autoCommands.runIndexer(), autoCommands.shoot().withTimeout(5.0)),
            Commands.sequence(
                choreoAutoFactory.trajectoryCmd("LscoreToDepot"),
                choreoAutoFactory.trajectoryCmd("DepotToLscore"),
                autoCommands.spinUpShooter(),
                autoCommands.runIndexer(),
                autoCommands.shoot()))
        .withName("Choreo: LeftSideDepot");
  }

  // OLD AutoRoutine-based implementation:
  // public AutoRoutine LeftSideDepot() {
  // AutoRoutine routine = choreoAutoFactory.newRoutine("LeftSideDepot");
  // AutoTrajectory lStartToNeutral = routine.trajectory("lStartToNeutral");
  // AutoTrajectory LneutralToLscore = routine.trajectory("LneutralToLscore");
  // AutoTrajectory LscoreToDepot = routine.trajectory("LscoreToDepot");
  // AutoTrajectory DepotToLscore = routine.trajectory("DepotToLscore");
  // routine.active().onTrue(
  // Commands.sequence(
  // lStartToNeutral.resetOdometry(),
  // Commands.parallel(lStartToNeutral.cmd(), autoCommands.deployPivot()),
  // Commands.parallel(LneutralToLscore.cmd(), autoCommands.spinUpShooter()),
  // Commands.parallel(autoCommands.runIndexer(),
  // autoCommands.shoot().withTimeout(5.0)),
  // Commands.sequence(
  // LscoreToDepot.cmd(),
  // DepotToLscore.cmd(),
  // autoCommands.spinUpShooter(),
  // autoCommands.runIndexer(),
  // autoCommands.shoot())));
  // return routine;
  // }

}
