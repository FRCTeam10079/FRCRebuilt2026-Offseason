// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterPivotSubsystem extends SubsystemBase {

  private final ShooterPivotIO io;
  private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  // ==================== STATE MACHINE ====================

  public enum WantedState {
    HOME,
    IDLE,
    TRACK_ANGLE,
    HOLD_ANGLE,
    MANUAL
  }

  private enum SystemState {
    HOMING,
    HOMED,
    IDLE,
    TRACKING,
    AT_ANGLE,
    TRENCH_LOWERED,
    MANUAL_OVERRIDE
  }

  private WantedState wantedState = WantedState.IDLE;
  private WantedState previousWantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  // State tracking
  private boolean isHomed = true;
  private Angle targetAngle = ShooterPivotConstants.MIN_ANGLE;
  private double manualOutput = 0.0;

  // Homing
  private int homingStallCounter = 0;

  // Trench auto-lower
  private final Supplier<Pose2d> poseSupplier;
  private boolean trenchMode = false;
  private boolean trenchAutoLowerEnabled = true;

  // Angle supplier for TRACK_ANGLE mode
  private Supplier<Angle> angleSupplier = () -> ShooterPivotConstants.MIN_ANGLE;

  public ShooterPivotSubsystem(ShooterPivotIO io, Supplier<Pose2d> poseSupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterPivot", inputs);

    systemState = handleStateTransitions();
    applyStates();

    previousWantedState = wantedState;

    Logger.recordOutput("ShooterPivot/WantedState", wantedState);
    Logger.recordOutput("ShooterPivot/SystemState", systemState);
    Logger.recordOutput("ShooterPivot/isInTrenchZone", isInTrenchZone());
    Logger.recordOutput("ShooterPivot/AngleDegrees", getCurrentAngle().in(Degrees));
    Logger.recordOutput("ShooterPivot/TargetAngleDegrees", targetAngle.in(Degrees));
    Logger.recordOutput("ShooterPivot/Position", inputs.positionRotations);
    Logger.recordOutput("ShooterPivot/Velocity", inputs.velocityRPS);
    Logger.recordOutput("ShooterPivot/IsHomed", isHomed);
    Logger.recordOutput("ShooterPivot/AtTarget", isAtTarget());
    Logger.recordOutput("ShooterPivot/TrenchMode", trenchMode);
    Logger.recordOutput("ShooterPivot/HomingStallCounter", homingStallCounter);
    Logger.recordOutput(
        "ShooterPivot/HomingCurrentExceeded",
        Amps.of(inputs.statorCurrentAmps).gt(ShooterPivotConstants.HOMING_CURRENT_THRESHOLD));
    Logger.recordOutput(
        "ShooterPivot/ClosedLoopReferenceRotations", inputs.closedLoopReferenceRotations);
    Logger.recordOutput("ShooterPivot/ClosedLoopErrorRotations", inputs.closedLoopErrorRotations);
    Logger.recordOutput("ShooterPivot/MotionMagicAtTarget", inputs.motionMagicAtTarget);
    Logger.recordOutput("ShooterPivot/MotionMagicIsRunning", inputs.motionMagicIsRunning);
    Logger.recordOutput("ShooterPivot/FaultField", inputs.faultField);
  }

  // ==================== STATE TRANSITIONS ====================

  private SystemState handleStateTransitions() {
    switch (wantedState) {
      case HOME:
        if (previousWantedState != WantedState.HOME) {
          isHomed = false;
          homingStallCounter = 0;
        }
        if (Amps.of(inputs.statorCurrentAmps).gt(ShooterPivotConstants.HOMING_CURRENT_THRESHOLD)) {
          homingStallCounter++;
        } else {
          homingStallCounter = 0;
        }
        if (homingStallCounter >= ShooterPivotConstants.HOMING_STALL_CYCLES) {
          io.setEncoderPosition(0);
          isHomed = true;
          enableSoftwareLimits();
          targetAngle = ShooterPivotConstants.MIN_ANGLE;
          homingStallCounter = 0;
          wantedState = WantedState.IDLE;
          return SystemState.HOMED;
        }
        return SystemState.HOMING;

      case TRACK_ANGLE:
        targetAngle = Constants.clamp(
            angleSupplier.get(), ShooterPivotConstants.MIN_ANGLE, ShooterPivotConstants.MAX_ANGLE);
        return evaluateAngleTrackingState();

      case HOLD_ANGLE:
        return evaluateAngleTrackingState();

      case MANUAL:
        return SystemState.MANUAL_OVERRIDE;

      case IDLE:
      default:
        trenchMode = false;
        return SystemState.IDLE;
    }
  }

  private SystemState evaluateAngleTrackingState() {
    if (isInTrenchZone()) {
      trenchMode = true;
      if (targetAngle.gt(ShooterPivotConstants.TRENCH_LOWER_ANGLE)) {
        targetAngle = ShooterPivotConstants.TRENCH_LOWER_ANGLE;
      }
      return SystemState.TRENCH_LOWERED;
    }
    trenchMode = false;
    if (isAtAngle(targetAngle)) {
      return SystemState.AT_ANGLE;
    }
    return SystemState.TRACKING;
  }

  private void applyStates() {
    switch (systemState) {
      case HOMING:
        io.setDutyCycle(ShooterPivotConstants.HOMING_SPEED);
        break;
      case HOMED:
      case IDLE:
        io.setNeutral();
        break;
      case TRACKING:
      case AT_ANGLE:
      case TRENCH_LOWERED:
        io.setMotionMagicPosition(ShooterPivotConstants.degreesToMotorRotations(
                targetAngle.minus(ShooterPivotConstants.MIN_ANGLE))
            .in(Rotations));
        break;
      case MANUAL_OVERRIDE:
        io.setDutyCycle(manualOutput);
        break;
    }
  }

  // ==================== PUBLIC API ====================

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public void setWantedState(WantedState state, Angle angle) {
    this.wantedState = state;
    Angle clamped =
        Constants.clamp(angle, ShooterPivotConstants.MIN_ANGLE, ShooterPivotConstants.MAX_ANGLE);
    this.targetAngle = clamped;
    this.angleSupplier = () -> clamped;
  }

  public void setAngleSupplier(Supplier<Angle> supplier) {
    this.angleSupplier = supplier;
  }

  public void setManualOutput(double output) {
    this.manualOutput = MathUtil.clamp(
        output, -ShooterPivotConstants.MANUAL_MAX_OUTPUT, ShooterPivotConstants.MANUAL_MAX_OUTPUT);
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public boolean isHomed() {
    return isHomed;
  }

  public boolean hasHomeCompleted() {
    return isHomed;
  }

  public void setTrenchAutoLowerEnabled(boolean enabled) {
    this.trenchAutoLowerEnabled = enabled;
    if (!enabled) {
      trenchMode = false;
    }
  }

  // ==================== TRENCH ZONE DETECTION ====================

  private static boolean isWithinXBand(Distance x, Distance centerX, Distance halfWidth) {
    return x.gte(centerX.minus(halfWidth)) && x.lte(centerX.plus(halfWidth));
  }

  private static boolean isInTrenchXWindow(Distance x, Distance halfWidth) {
    return isWithinXBand(x, ShooterPivotConstants.TRENCH_BLUE_X_CENTER, halfWidth)
        || isWithinXBand(x, ShooterPivotConstants.TRENCH_RED_X_CENTER, halfWidth);
  }

  public boolean isInTrenchZone() {
    if (!trenchAutoLowerEnabled) {
      trenchMode = false;
      return false;
    }
    if (poseSupplier == null) {
      return false;
    }
    Pose2d pose = poseSupplier.get();
    if (pose == null) {
      return false;
    }

    Distance x = Meters.of(pose.getX());
    Distance y = Meters.of(pose.getY());
    Distance fieldW = ShooterPivotConstants.FIELD_WIDTH_METERS;
    Distance approachMarginX = ShooterPivotConstants.TRENCH_APPROACH_MARGIN;
    Distance approachMarginY = ShooterPivotConstants.TRENCH_Y_APPROACH_MARGIN;
    Distance strictHalfWidthX = ShooterPivotConstants.TRENCH_X_HALF_WIDTH;

    if (trenchMode) {
      boolean inX = isInTrenchXWindow(x, strictHalfWidthX);
      boolean inY = y.lte(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD)
          || y.gte(fieldW.minus(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD));
      return inX && inY;
    } else {
      boolean inX = isInTrenchXWindow(x, strictHalfWidthX.plus(approachMarginX));
      boolean inY = y.lte(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD.plus(approachMarginY))
          || y.gte(
              fieldW.minus(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD).minus(approachMarginY));
      return inX && inY;
    }
  }

  // ==================== POSITION QUERIES ====================

  public Angle getCurrentAngle() {
    return ShooterPivotConstants.motorRotationsToDegrees(Rotations.of(inputs.positionRotations))
        .plus(ShooterPivotConstants.MIN_ANGLE);
  }

  public boolean isAtAngle(Angle target) {
    return getCurrentAngle().isNear(target, ShooterPivotConstants.SHOOTING_TOLERANCE);
  }

  public boolean isAtTarget() {
    return isAtAngle(targetAngle);
  }

  public Angle getTargetAngle() {
    return targetAngle;
  }

  public void reZeroIfNeeded() {
    if (inputs.positionRotations < 0.0) {
      io.setEncoderPosition(0);
    }
  }

  // ==================== HOMING HELPERS ====================

  private void enableSoftwareLimits() {
    var softLimits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ShooterPivotConstants.degreesToMotorRotations(
            ShooterPivotConstants.MAX_ANGLE.minus(ShooterPivotConstants.MIN_ANGLE)));
    io.applySoftwareLimits(softLimits);
  }

  // ==================== COMMAND FACTORIES ====================

  public Command homeCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> setWantedState(WantedState.HOME)),
            Commands.waitUntil(this::isHomed),
            Commands.runOnce(() -> setWantedState(WantedState.IDLE)))
        .withName("ShooterPivot Home");
  }

  public Command trackAngleCommand(Supplier<Angle> supplier) {
    return Commands.runEnd(
            () -> {
              setAngleSupplier(supplier);
              setWantedState(WantedState.TRACK_ANGLE);
            },
            () -> setWantedState(WantedState.IDLE),
            this)
        .withName("ShooterPivot Track Angle");
  }

  public Command goToAngleCommand(Angle angle) {
    return Commands.sequence(
            Commands.runOnce(() -> setWantedState(WantedState.HOLD_ANGLE, angle)),
            Commands.waitUntil(this::isAtTarget))
        .finallyDo(interrupted -> setWantedState(WantedState.IDLE))
        .withName("ShooterPivot GoTo " + angle.in(Degrees) + "deg");
  }

  public Command manualControlCommand(DoubleSupplier axisSupplier) {
    return run(() -> {
          double raw = axisSupplier.getAsDouble();
          double deadbanded = MathUtil.applyDeadband(raw, ShooterPivotConstants.MANUAL_DEADBAND);
          setManualOutput(deadbanded * ShooterPivotConstants.MANUAL_MAX_OUTPUT);
          setWantedState(WantedState.MANUAL);
        })
        .finallyDo(interrupted -> setWantedState(WantedState.IDLE))
        .withName("ShooterPivot Manual");
  }

  public Command zeroEncoderCommand() {
    return runOnce(() -> io.setEncoderPosition(0)).withName("ShooterPivot Zero Encoder");
  }

  // ==================== TELEMETRY ====================

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getStatorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  public double getMotorVoltageVolts() {
    return inputs.voltageVolts;
  }
}
