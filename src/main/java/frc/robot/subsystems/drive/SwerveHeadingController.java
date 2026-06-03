// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.HeadingControllerConstants;
import org.littletonrobotics.junction.Logger;

/**
 * This class controls the rotational heading of the drivetrain seperately from translation control.
 *
 * <p>Some features I added: 1. State Machine: SNAP (high gains to quickly reach target) vs MAINTAIN
 * (low gains to hold) 2. Automatic state transitions: After snapping to target, automatically
 * switches to maintain 3. Separate from drivetrain: Can be used with any translation source (driver
 * input, auto path)
 *
 * <p>Usage: - Call setGoal() to set target heading in degrees (field-relative) - Call update() each
 * loop with current heading to get rotation output - The output is a normalized value (-1 to 1)
 * representing rotational velocity demand
 */
public class SwerveHeadingController {

  // ==================== STATE MACHINE ====================
  public enum HeadingControllerState {
    OFF, // No heading control - return 0 rotation
    SNAP, // Actively snapping to a target heading (higher gains)
    MAINTAIN // Holding current heading with lower gains
  }

  private HeadingControllerState m_state = HeadingControllerState.OFF;

  private final PIDController m_pidController;

  private double m_goalDegrees = 0.0;

  // ==================== CSV LOGGING ====================
  private boolean m_csvHeaderPrinted = false;
  private double m_lastErrorDeg = 0.0;
  private double m_lastTimestamp = 0.0;
  private double m_loggingMaxAngularVelocity = 0.0;
  private double m_loggingMeasuredOmega = 0.0;

  /** Creates a new SwerveHeadingController */
  public SwerveHeadingController() {
    // Initialize PID with SNAP gains (will be updated based on state)
    m_pidController = new PIDController(
        HeadingControllerConstants.SNAP_KP,
        HeadingControllerConstants.SNAP_KI,
        HeadingControllerConstants.SNAP_KD);

    // Enable continuous input for angle wrapping (-180 to 180)
    m_pidController.enableContinuousInput(-180, 180);

    // Set tolerance for "at goal" detection
    m_pidController.setTolerance(HeadingControllerConstants.HEADING_TOLERANCE_DEGREES);
  }

  // ==================== CONFIGURATION METHODS ====================

  /**
   * Set the heading controller state
   *
   * @param state The desired state (OFF, SNAP, or MAINTAIN)
   */
  public void setHeadingControllerState(HeadingControllerState state) {
    if (m_state != state) {
      m_state = state;
      updateGains();
    }
  }

  /** Get the current heading controller state */
  public HeadingControllerState getHeadingControllerState() {
    return m_state;
  }

  /**
   * Set the target heading goal
   *
   * @param goalDegrees Target heading in degrees (field-relative, -180 to 180)
   */
  public void setGoal(double goalDegrees) {
    // Normalize to -180 to 180 range
    m_goalDegrees = MathUtil.inputModulus(goalDegrees, -180, 180);
    m_pidController.setSetpoint(m_goalDegrees);
  }

  /** Get the current heading goal */
  public double getGoal() {
    return m_goalDegrees;
  }

  // ==================== UPDATE LOOP ====================

  /**
   * Update the heading controller and calculate rotation output.
   *
   * <p>This should be called every robot loop with the current heading. The output is a normalized
   * rotation demand (-1 to 1) that should be multiplied by max angular velocity before applying to
   * the drivetrain.
   *
   * @param currentHeadingDegrees Current robot heading in degrees (field-relative)
   * @return Rotation output (-1 to 1), where positive is counter-clockwise
   */
  public double update(double currentHeadingDegrees) {
    // Normalize input to -180 to 180 range
    currentHeadingDegrees = MathUtil.inputModulus(currentHeadingDegrees, -180, 180);

    // Handle state machine
    switch (m_state) {
      case OFF:
        m_csvHeaderPrinted = false;
        return 0.0;

      case SNAP:
        // Calculate output with high gains
        double snapOutput = m_pidController.calculate(currentHeadingDegrees);

        // Check if we've reached the goal - if so, transition to MAINTAIN
        if (isAtGoal()) {
          setHeadingControllerState(HeadingControllerState.MAINTAIN);
        }

        // Clamp output to valid range
        double clampedSnap = MathUtil.clamp(snapOutput, -1.0, 1.0);
        logCSV(currentHeadingDegrees, clampedSnap);
        return clampedSnap;

      case MAINTAIN:
        // Calculate output with lower gains
        double maintainOutput = m_pidController.calculate(currentHeadingDegrees);

        // Re-snap: if error has grown beyond what MAINTAIN can handle efficiently
        // (e.g. driver changed direction fast, or tracking target moved a lot),
        // switch back to SNAP's higher gains so we converge quickly.
        if (Math.abs(getError()) > HeadingControllerConstants.RESNAP_THRESHOLD_DEGREES) {
          setHeadingControllerState(HeadingControllerState.SNAP);
        }

        // Clamp output to valid range
        double clampedMaintain = MathUtil.clamp(maintainOutput, -1.0, 1.0);
        logCSV(currentHeadingDegrees, clampedMaintain);
        return clampedMaintain;

      default:
        return 0.0;
    }
  }

  /**
   * Check if the heading controller is at the goal within tolerance
   *
   * @return true if current heading is within tolerance of goal
   */
  public boolean isAtGoal() {
    return m_pidController.atSetpoint();
  }

  /**
   * Get the current heading error in degrees
   *
   * @return Position error in degrees (positive = need to rotate CCW)
   */
  public double getError() {
    return m_pidController.getError();
  }

  // HELPER METHODS

  /** Update PID gains based on current state */
  private void updateGains() {
    switch (m_state) {
      case SNAP:
        m_pidController.setPID(
            HeadingControllerConstants.SNAP_KP,
            HeadingControllerConstants.SNAP_KI,
            HeadingControllerConstants.SNAP_KD);
        break;

      case MAINTAIN:
        m_pidController.setPID(
            HeadingControllerConstants.MAINTAIN_KP,
            HeadingControllerConstants.MAINTAIN_KI,
            HeadingControllerConstants.MAINTAIN_KD);
        break;

      case OFF:
      default:
        // No gains needed when off
        break;
    }

    // Log gain changes for debugging
    Logger.recordOutput("HeadingController/State", m_state.toString());
    Logger.recordOutput("HeadingController/ActiveKP", m_pidController.getP());
  }

  /** Reset the heading controller Clears accumulated integral error and resets state */
  public void reset() {
    m_pidController.reset();
    m_state = HeadingControllerState.OFF;
    m_goalDegrees = 0.0;
  }

  /** Set context values needed for CSV logging. Call each loop from driveWithHeadingLock. */
  public void setLoggingContext(double maxAngularVelocity, double measuredOmegaRadPerSec) {
    m_loggingMaxAngularVelocity = maxAngularVelocity;
    m_loggingMeasuredOmega = measuredOmegaRadPerSec;
  }

  /** Print one CSV data line with all heading controller state. */
  private void logCSV(double currentHeadingDegrees, double totalOutput) {
    /*
     * if (!m_csvHeaderPrinted) {
     * System.out.println(
     * "HDG_CSV,timestamp_s,state,goal_deg,current_deg,error_deg,"
     * + "kP,kD,p_output,d_approx,total_output,"
     * + "omega_cmd_radps,omega_actual_radps");
     * m_csvHeaderPrinted = true;
     * m_lastErrorDeg = getError();
     * m_lastTimestamp = Timer.getFPGATimestamp();
     * }
     */

    double now = Timer.getFPGATimestamp();
    double dt = now - m_lastTimestamp;
    double errorDeg = getError();

    // Approximate P and D contributions
    double kP = m_pidController.getP();
    double kD = m_pidController.getD();
    double pOutput = kP * errorDeg;
    double dApprox = (dt > 0.001) ? kD * (errorDeg - m_lastErrorDeg) / dt : 0.0;

    double omegaCmd = totalOutput * m_loggingMaxAngularVelocity;

    System.out.printf(
        "HDG_CSV,%.4f,%s,%.2f,%.2f,%.2f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f%n",
        now,
        m_state.toString(),
        m_goalDegrees,
        currentHeadingDegrees,
        errorDeg,
        kP,
        kD,
        pOutput,
        dApprox,
        totalOutput,
        omegaCmd,
        m_loggingMeasuredOmega);

    m_lastErrorDeg = errorDeg;
    m_lastTimestamp = now;
  }

  /** Log telemetry data via AdvantageKit. Call this from a subsystem's periodic() method */
  public void logTelemetry(double currentHeadingDegrees) {
    Logger.recordOutput("HeadingController/State", m_state.toString());
    Logger.recordOutput("HeadingController/GoalDeg", m_goalDegrees);
    Logger.recordOutput("HeadingController/CurrentDeg", currentHeadingDegrees);
    Logger.recordOutput("HeadingController/ErrorDeg", getError());
    Logger.recordOutput("HeadingController/AtGoal", isAtGoal());
  }
}
