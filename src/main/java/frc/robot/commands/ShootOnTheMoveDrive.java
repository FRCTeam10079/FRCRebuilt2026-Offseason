// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GameConstants;
import frc.robot.lib.ShooterInterpolationTable;
import frc.robot.lib.ShooterMath;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.statemachine.DrivetrainMode;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Command to align the robot to the hub */
public class ShootOnTheMoveDrive extends Command {
  static Supplier<ShooterSetpoint> m_sotmSetpointSupplier;

  // Subsystems
  private final CommandSwerveDrivetrain m_drivetrain;
  private final RobotStateMachine m_stateMachine;

  // PID Controllers for position control
  private final PIDController m_pidRotate;

  // Input suppliers
  DoubleSupplier m_xInputSupplier;
  DoubleSupplier m_yInputSupplier;

  // Swerve drive request - field centric with velocity control
  private final SwerveRequest.FieldCentric m_driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /**
   * Creates a new AlignToAprilTag command
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param xInputSupplier Supplier for driver translation controls - X
   * @param yInputSupplier Suppplier for driver translation controls - Y
   */
  public ShootOnTheMoveDrive(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier xInputSupplier,
      DoubleSupplier yInputSupplier) {
    this.m_drivetrain = drivetrain;
    this.m_stateMachine = RobotStateMachine.getInstance();

    this.m_yInputSupplier = yInputSupplier;
    this.m_xInputSupplier = xInputSupplier;

    // Create the memoized setpoint supplier (caches by pose X/Y/theta)
    m_sotmSetpointSupplier = ShooterMath.createSetpointSupplier(() -> this.getCorrectedRobotPose());

    // Initialize PID controller for yaw only
    m_pidRotate = new PIDController(4, 0, 0); // tune as needed

    // Enable continuous input for rotation (-PI to PI are same point)
    m_pidRotate.enableContinuousInput(-Math.PI, Math.PI);

    // This command requires the drivetrain
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Set state machine to vision tracking mode
    m_stateMachine.setDrivetrainMode(DrivetrainMode.VISION_TRACKING);
    m_stateMachine.setAlignedToTarget(false);
  }

  @Override
  public void execute() {
    // Get current robot pose
    Pose2d currentPose = m_drivetrain.getState().Pose;
    if (currentPose == null) {
      return;
    }

    Translation2d correctedHubPos = getCorrectedHubPos();

    // Calculate yaw rate using PID
    Yaw yaw = calculateYawInfo(currentPose, correctedHubPos);

    SmartDashboard.putNumberArray("CorrectedPos", new double[] {
      correctedHubPos.getX(), correctedHubPos.getY(),
    });

    // Apply velocities to drivetrain
    m_drivetrain.setControl(m_driveRequest
        .withVelocityX(MetersPerSecond.of(-m_xInputSupplier.getAsDouble()))
        .withVelocityY(MetersPerSecond.of(-m_yInputSupplier.getAsDouble()))
        .withRotationalRate(yaw.rate));

    // Calculate yaw error to target heading
    double yawError = Math.abs(
        MathUtil.angleModulus(yaw.target.minus(currentPose.getRotation()).getRadians()));
    SmartDashboard.putNumber("AlignToAprilTag/YawErrorDeg", Math.toDegrees(yawError));
  }

  private record Yaw(Rotation2d target, AngularVelocity rate) {}

  private Yaw calculateYawInfo(Pose2d currentPose, Translation2d targetLookPos) {
    double targetRotation = Math.atan2(
        targetLookPos.getY() - currentPose.getY(), targetLookPos.getX() - currentPose.getX());

    Rotation2d targetYaw = new Rotation2d(targetRotation);
    m_pidRotate.setSetpoint(targetRotation);

    double velocityYaw = m_pidRotate.calculate(currentPose.getRotation().getRadians());
    // Clamp to reasonable range
    velocityYaw = MathUtil.clamp(velocityYaw, -2.0, 2.0);

    SmartDashboard.putNumber("AlignToAprilTag/TargetYaw", Math.toDegrees(targetRotation));
    SmartDashboard.putNumber("AlignToAprilTag/ErrorYaw", Math.toDegrees(m_pidRotate.getError()));

    return new Yaw(targetYaw, RadiansPerSecond.of(velocityYaw));
  }

  public Pose2d getCorrectedRobotPose() {
    Pose2d currentPose = m_drivetrain.getState().Pose;
    if (currentPose == null) {
      return null;
    }
    double distanceToAprilTag = currentPose.getTranslation().getDistance(getCorrectedHubPos());

    ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
        m_drivetrain.getState().Speeds, currentPose.getRotation());
    double xVelFieldCentric = fieldVelocity.vxMetersPerSecond;
    double yVelFieldCentric = fieldVelocity.vyMetersPerSecond;

    return new Pose2d(
        currentPose.getX()
            + (xVelFieldCentric * ShooterInterpolationTable.getTimeOfFlight(distanceToAprilTag)),
        currentPose.getY()
            + (yVelFieldCentric * ShooterInterpolationTable.getTimeOfFlight(distanceToAprilTag)),
        currentPose.getRotation());
  }

  private Translation2d getCorrectedHubPos() {
    // Get current robot pose
    Pose2d currentPose = m_drivetrain.getState().Pose;
    Translation2d hubPos = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? GameConstants.RED_HUB_CENTER
        : GameConstants.BLUE_HUB_CENTER;
    // Base distance to hub (non-negative)
    double distanceToHub = currentPose.getTranslation().getDistance(hubPos);

    ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
        m_drivetrain.getState().Speeds, currentPose.getRotation());
    double xVelFieldCentric = fieldVelocity.vxMetersPerSecond;
    double yVelFieldCentric = fieldVelocity.vyMetersPerSecond;

    Translation2d correctedHubPos = new Translation2d(
        hubPos.getX()
            - (ShooterInterpolationTable.getTimeOfFlight(distanceToHub) * xVelFieldCentric),
        hubPos.getY()
            - (ShooterInterpolationTable.getTimeOfFlight(distanceToHub) * yVelFieldCentric));

    // Change the number of loops for more precision in estimating the pose
    for (int i = 0; i < 11; i++) {
      distanceToHub = currentPose.getTranslation().getDistance(correctedHubPos);
      correctedHubPos = new Translation2d(
          hubPos.getX()
              - (ShooterInterpolationTable.getTimeOfFlight(distanceToHub) * xVelFieldCentric),
          hubPos.getY()
              - (ShooterInterpolationTable.getTimeOfFlight(distanceToHub) * yVelFieldCentric));
    }
    SmartDashboard.putNumber("AlignToAprilTag/Final Target Dist", distanceToHub);
    SmartDashboard.putNumber(
        "AlignToAprilTag/ToF", ShooterInterpolationTable.getTimeOfFlight(distanceToHub));

    return correctedHubPos;
  }

  public static Supplier<ShooterSetpoint> getShooterSetpointSupplier() {
    return m_sotmSetpointSupplier;
  }

  // This shouldn't be called unless the command is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    m_drivetrain.setControl(new SwerveRequest.Idle());

    // Return to field-centric drive mode
    m_stateMachine.setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);
    m_stateMachine.setAlignedToTarget(!interrupted);

    // Log completion
    SmartDashboard.putBoolean("AlignToAprilTag/Completed", !interrupted);
  }
}
