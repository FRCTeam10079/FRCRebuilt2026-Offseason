// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.Elastic;
import frc.robot.lib.HubShiftTracker;
import frc.robot.lib.LaunchCalculator;
import frc.robot.lib.PowerDiagnosticsLogger;
import frc.robot.lib.ShooterMath;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.statemachine.MatchState;
import frc.robot.statemachine.RobotStateMachine;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Robot class for FRC 2026 REBUILT season Integrates with the Master State Machine for
 * comprehensive robot control
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // MASTER STATE MACHINE - Controls EVERYTHING
  private final RobotStateMachine m_stateMachine;
  private final PowerDiagnosticsLogger m_powerDiagnosticsLogger;
  private int m_periodicCycleCount = 0;
  private int m_robotEventSequence = 0;

  public Robot() {
    // In sim/replay, missing extra controllers are expected and warning spam can
    // stall the loop enough to trigger stale CAN status signals.
    if (Constants.currentMode != Constants.Mode.REAL) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // ==================== ADVANTAGEKIT LOGGING ====================
    Logger.recordMetadata("ProjectName", "FRCRebuilt2026-COMP");

    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    m_robotContainer = new RobotContainer();
    m_stateMachine = RobotStateMachine.getInstance();
    Elastic.sendNotification(new Elastic.Notification(
        Elastic.NotificationLevel.INFO, "Robot Code", "Code initialized", 2000));

    m_powerDiagnosticsLogger = new PowerDiagnosticsLogger(
        m_robotContainer.getIntake(),
        m_robotContainer.getPivot(),
        m_robotContainer.getIndexer(),
        m_robotContainer.shooter,
        m_robotContainer.shooterPivot);

    // ==================== LIMELIGHT CAMERA STREAMS FOR ELASTIC DASHBOARD
    // ====================
    var nt = NetworkTableInstance.getDefault();
    for (String llName : Constants.VisionConstants.LIMELIGHT_NAMES) {
      StringArrayPublisher pub = nt.getTable("/CameraPublisher/" + llName)
          .getStringArrayTopic("streams")
          .publish();
      pub.set(new String[] {"mjpg:http://" + llName + ".local:5800/stream.mjpg"});
      recordRobotEvent(llName + " stream URL published to NetworkTables");
    }
  }

  @Override
  public void robotPeriodic() {
    // Clear and immediately update LaunchCalculator with fresh drivetrain state.
    // Parameters must be computed BEFORE triggers evaluate, so shootOnTheMove can
    // work.
    var driveState = m_robotContainer.drivetrain.getState();
    var currentPose = driveState.Pose;
    LaunchCalculator calc = LaunchCalculator.getInstance();
    calc.clearParameters();
    calc.update(currentPose, driveState.Speeds, currentPose.getRotation());

    Logger.recordOutput(
        "Shooter/DistanceToHub", ShooterMath.getDistanceToHub(currentPose).in(Meters));

    // Update HubShiftTracker (FMS game data + shift phase detection)
    HubShiftTracker.getInstance().periodic();

    // Update master state machine
    m_stateMachine.periodic();

    // Run command scheduler
    CommandScheduler.getInstance().run();

    // Update dashboard publisher (NT4 telemetry for Elastic Dashboard)
    m_robotContainer.getDashboardPublisher().periodic();

    // Throttle power diagnostics to every 5th cycle (~100ms) to reduce loop
    // overruns
    if (m_periodicCycleCount++ >= 4) {
      m_periodicCycleCount = 0;
      m_powerDiagnosticsLogger.logPeriodic();
    }
  }

  @Override
  public void disabledInit() {
    // State machine transition: Robot disabled
    m_stateMachine.setMatchState(MatchState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.shooterPivot.reZeroIfNeeded();
  }

  @Override
  public void disabledExit() {
    // Leaving disabled state
    recordRobotEvent("Exiting disabled mode");
  }

  @Override
  public void autonomousInit() {
    // State machine transition: Autonomous starting
    m_stateMachine.setMatchState(MatchState.AUTO_INIT);

    // Vision uses Mode 0 (EXTERNAL_ONLY) - no IMU mode switch needed.
    // Orientation is published every frame in VisionIOLimelight.updateInputs().
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // Critical for post-match log review: confirms what auto actually executed vs
    // what the driver
    // selected.
    Logger.recordOutput(
        "Auto/SelectedCommandName",
        m_autonomousCommand != null ? m_autonomousCommand.getName() : "<null>");

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
      // Transition to running state
      m_stateMachine.setMatchState(MatchState.AUTO_RUNNING);
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Autonomous is running - state machine tracks this
  }

  @Override
  public void autonomousExit() {
    // Autonomous ending
    recordRobotEvent("Autonomous period ended");
  }

  @Override
  public void teleopInit() {
    // State machine transition: Teleop starting
    m_stateMachine.setMatchState(MatchState.TELEOP_INIT);
    ShooterSetpoint.resetOffsets();
    Logger.recordOutput("ShooterTuning/EventType", "TeleopReset");
    Logger.recordOutput("ShooterTuning/OffsetsReset", true);
    Logger.recordOutput("ShooterTuning/AppliedOffsetRPM", 0.0);
    Logger.recordOutput("ShooterTuning/AppliedOffsetAngleDeg", 0.0);
    Logger.recordOutput("ShooterTuning/OffsetRPMAfterReset", 0.0);
    Logger.recordOutput("ShooterTuning/OffsetAngleDegAfterReset", 0.0);
    scheduleShooterTuningEventClear();

    // Vision orientation is published every frame in
    // VisionIOLimelight.updateInputs().

    // Reset Superstructure before cancelling auto to prevent stale state from
    // persisting
    m_robotContainer
        .getSuperstructure()
        .setWantedSuperState(frc.robot.subsystems.Superstructure.WantedSuperState.IDLE);

    // Cancel autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Transition to running state
    m_stateMachine.setMatchState(MatchState.TELEOP_RUNNING);
  }

  @Override
  public void teleopPeriodic() {
    // Teleop is running - state machine tracks endgame and hub shifts automatically
  }

  @Override
  public void teleopExit() {
    // Teleop ending
    recordRobotEvent("Teleop period ended");
  }

  @Override
  public void testInit() {
    // State machine transition: Test mode starting
    m_stateMachine.setMatchState(MatchState.TEST_INIT);

    CommandScheduler.getInstance().cancelAll();

    // Transition to running state
    m_stateMachine.setMatchState(MatchState.TEST_RUNNING);
  }

  @Override
  public void testPeriodic() {
    // Test mode is running
  }

  @Override
  public void testExit() {
    // Test mode ending
    recordRobotEvent("Test mode ended");
  }

  @Override
  public void simulationPeriodic() {
    // Simulation running
  }

  private void scheduleShooterTuningEventClear() {
    CommandScheduler.getInstance()
        .schedule(Commands.waitSeconds(0.02)
            .andThen(Commands.runOnce(() -> Logger.recordOutput("ShooterTuning/EventType", ""))));
  }

  private void recordRobotEvent(String message) {
    Logger.recordOutput("Events/Robot/Last", message);
    Logger.recordOutput("Events/Robot/Sequence", ++m_robotEventSequence);
  }
}
