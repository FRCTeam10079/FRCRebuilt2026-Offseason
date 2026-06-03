package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterPivotConstants;

public class ShooterPivotIOTalonFX implements ShooterPivotIO {

  private final TalonFX pivotMotor;
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public ShooterPivotIOTalonFX() {
    pivotMotor = new TalonFX(ShooterPivotConstants.MOTOR_ID);
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(ShooterPivotConstants.SUPPLY_CURRENT_LIMIT)
        .withStatorCurrentLimit(ShooterPivotConstants.STATOR_CURRENT_LIMIT);

    config.Slot0 = new Slot0Configs()
        .withKP(ShooterPivotConstants.KP)
        .withKI(ShooterPivotConstants.KI)
        .withKD(ShooterPivotConstants.KD)
        .withKS(ShooterPivotConstants.KS)
        .withKV(ShooterPivotConstants.KV)
        .withKG(ShooterPivotConstants.KG)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    config.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(ShooterPivotConstants.MOTION_MAGIC_CRUISE_VELOCITY)
        .withMotionMagicAcceleration(ShooterPivotConstants.MOTION_MAGIC_ACCELERATION)
        .withMotionMagicJerk(ShooterPivotConstants.MOTION_MAGIC_JERK);

    config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ShooterPivotConstants.degreesToMotorRotations(
            ShooterPivotConstants.MAX_ANGLE.minus(ShooterPivotConstants.MIN_ANGLE)));

    pivotMotor.getConfigurator().apply(config);
    configureSignalRates();
  }

  private void configureSignalRates() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotMotor.getPosition(),
        pivotMotor.getVelocity(),
        pivotMotor.getSupplyCurrent(),
        pivotMotor.getStatorCurrent(),
        pivotMotor.getMotorVoltage(),
        pivotMotor.getDutyCycle(),
        pivotMotor.getClosedLoopReference(),
        pivotMotor.getClosedLoopError());

    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        pivotMotor.getDeviceTemp(),
        pivotMotor.getFaultField(),
        pivotMotor.getStickyFaultField(),
        pivotMotor.getMotionMagicAtTarget(),
        pivotMotor.getMotionMagicIsRunning());

    pivotMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterPivotIOInputs inputs) {
    inputs.positionRotations = pivotMotor.getPosition().getValueAsDouble();
    inputs.velocityRPS = pivotMotor.getVelocity().getValueAsDouble();
    inputs.supplyCurrentAmps = pivotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.voltageVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.dutyCycle = pivotMotor.getDutyCycle().getValueAsDouble();
    inputs.closedLoopReferenceRotations = pivotMotor.getClosedLoopReference().getValueAsDouble();
    inputs.closedLoopErrorRotations = pivotMotor.getClosedLoopError().getValueAsDouble();
    inputs.deviceTempCelsius = pivotMotor.getDeviceTemp().getValueAsDouble();
    inputs.faultField = pivotMotor.getFaultField().getValue().intValue();
    inputs.stickyFaultField = pivotMotor.getStickyFaultField().getValue().intValue();
    inputs.motionMagicAtTarget = pivotMotor.getMotionMagicAtTarget().getValue();
    inputs.motionMagicIsRunning = pivotMotor.getMotionMagicIsRunning().getValue();
  }

  @Override
  public void setMotionMagicPosition(double positionRotations) {
    pivotMotor.setControl(motionMagicRequest.withPosition(positionRotations));
  }

  @Override
  public void setDutyCycle(double output) {
    pivotMotor.setControl(dutyCycleRequest.withOutput(output));
  }

  @Override
  public void setNeutral() {
    pivotMotor.setControl(neutralRequest);
  }

  @Override
  public void setEncoderPosition(double positionRotations) {
    pivotMotor.setPosition(positionRotations);
  }

  @Override
  public void applySoftwareLimits(SoftwareLimitSwitchConfigs config) {
    pivotMotor.getConfigurator().apply(config);
  }
}
