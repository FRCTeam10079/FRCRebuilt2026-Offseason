// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.SwerveHeadingController.HeadingControllerState;
import java.util.function.DoubleSupplier;

/**
 * Command for automated heading step response testing.
 *
 * <p>Usage: Bind to a button to capture step responses for PID tuning.
 * Performs a sequence of heading changes (e.g., 0° -> 90° -> -90° -> 0°)
 * while the CSV logging captures data for analysis.
 *
 * <p>CSV output format (from SwerveHeadingController.logCSV):
 * HDG_CSV,timestamp_s,state,goal_deg,current_deg,error_deg,kP,kD,p_output,d_approx,total_output,omega_cmd_radps,omega_actual_radps
 */
public class HeadingStepTest {

  /** Default step sequence: 0 -> 90 -> -90 -> 0 degrees */
  private static final double[] DEFAULT_STEPS = {0.0, 90.0, -90.0, 0.0};
  private static final double STEP_SETTLE_TIME = 2.0; // seconds per step
  private static final double MIN_SETTLE_ERROR = 2.0; // degrees - consider step done when error < this

  /**
   * Creates a command that performs a heading step response test.
   *
   * @param drivetrain The drivetrain subsystem
   * @param xInputSupplier Translation X input (usually 0 for pure rotation test)
   * @param yInputSupplier Translation Y input (usually 0 for pure rotation test)
   * @param maxVelocity Max translation velocity (m/s)
   * @param maxAngularVelocity Max angular velocity (rad/s)
   * @param steps Array of heading targets in degrees. Default: {0, 90, -90, 0}
   * @return Command that executes the step test
   */
  public static Command headingStepTest(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier xInputSupplier,
      DoubleSupplier yInputSupplier,
      double maxVelocity,
      double maxAngularVelocity,
      double... steps) {

    double[] stepSequence = steps.length > 0 ? steps : DEFAULT_STEPS;

    return new Command() {
      private int m_stepIndex = 0;
      private double m_stepStartTime = 0;
      private boolean m_firstStep = true;

      {
        addRequirements(drivetrain);
      }

      @Override
      public void initialize() {
        m_stepIndex = 0;
        m_stepStartTime = 0;
        m_firstStep = true;

        // Enable heading lock at first target
        double initialTarget = stepSequence[0];
        drivetrain.enableHeadingLock(initialTarget);

        // Reset CSV logging header
        drivetrain.getHeadingController().reset();
      }

      @Override
      public void execute() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // Initialize step start time on first execute
        if (m_firstStep) {
          m_stepStartTime = currentTime;
          m_firstStep = false;
        }

        double targetHeading = stepSequence[m_stepIndex];
        double currentHeading = drivetrain.getState().Pose.getRotation().getDegrees();
        double error = MathUtil.inputModulus(targetHeading - currentHeading, -180, 180);
        double absError = Math.abs(error);

        // Update heading controller target (in case we're using dynamic targets)
        drivetrain.updateHeadingLockTarget(targetHeading);

        // Apply translation inputs (usually 0 for pure rotation test)
        double xInput = xInputSupplier.getAsDouble();
        double yInput = yInputSupplier.getAsDouble();

        drivetrain.driveWithHeadingLock(xInput, yInput, maxVelocity, maxAngularVelocity);

        // Check if step is complete (settled or timeout)
        boolean settled = absError < MIN_SETTLE_ERROR;
        boolean timeout = (currentTime - m_stepStartTime) > STEP_SETTLE_TIME;

        if (settled || timeout) {
          m_stepIndex++;
          m_stepStartTime = currentTime;

          if (m_stepIndex >= stepSequence.length) {
            // Test complete
          } else {
            // Move to next target
            drivetrain.updateHeadingLockTarget(stepSequence[m_stepIndex]);
          }
        }
      }

      @Override
      public boolean isFinished() {
        return m_stepIndex >= stepSequence.length;
      }

      @Override
      public void end(boolean interrupted) {
        drivetrain.disableHeadingLock();
      }
    };
  }

  /**
   * Simplified version with default step sequence and zero translation.
   *
   * @param drivetrain The drivetrain subsystem
   * @param maxAngularVelocity Max angular velocity (rad/s)
   * @return Command that executes the step test
   */
  public static Command headingStepTest(CommandSwerveDrivetrain drivetrain, double maxAngularVelocity) {
    return headingStepTest(drivetrain, () -> 0.0, () -> 0.0, 0.0, maxAngularVelocity);
  }

  /**
   * Creates a command for a SINGLE step (useful for manual testing).
   *
   * @param drivetrain The drivetrain subsystem
   * @param targetHeading Target heading in degrees
   * @param maxAngularVelocity Max angular velocity (rad/s)
   * @param settleTime Time to wait at target before ending (seconds)
   * @return Command that performs a single step
   */
  public static Command singleHeadingStep(
      CommandSwerveDrivetrain drivetrain,
      double targetHeading,
      double maxAngularVelocity,
      double settleTime) {

    return new Command() {
      private double m_startTime = 0;
      private boolean m_initialized = false;

      {
        addRequirements(drivetrain);
      }

      @Override
      public void initialize() {
        m_startTime = 0;
        m_initialized = false;
        drivetrain.enableHeadingLock(targetHeading);
      }

      @Override
      public void execute() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (!m_initialized) {
          m_startTime = currentTime;
          m_initialized = true;
        }
        drivetrain.driveWithHeadingLock(0.0, 0.0, 0.0, maxAngularVelocity);
      }

      @Override
      public boolean isFinished() {
        return (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - m_startTime) > settleTime;
      }

      @Override
      public void end(boolean interrupted) {
        drivetrain.disableHeadingLock();
      }
    };
  }
}