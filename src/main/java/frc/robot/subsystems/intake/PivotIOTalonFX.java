package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.networked.NetworkedTalonFX;

public class PivotIOTalonFX implements PivotIO {

  private final NetworkedTalonFX pivotMotor;
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public PivotIOTalonFX() {
    pivotMotor = new NetworkedTalonFX(IntakeConstants.Pivot.MOTOR_ID, Constants.kCANBus);
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.withGravityType(GravityTypeValue.Arm_Cosine)
        .withKA(IntakeConstants.Pivot.KA)
        .withKV(IntakeConstants.Pivot.KV)
        .withKD(IntakeConstants.Pivot.KD)
        .withKG(IntakeConstants.Pivot.KG)
        .withKS(IntakeConstants.Pivot.KS)
        .withKI(IntakeConstants.Pivot.KI)
        .withKP(IntakeConstants.Pivot.KP);

    config.MotionMagic.withMotionMagicCruiseVelocity(IntakeConstants.Pivot.MM_CRUISE_VELOCITY);
    config.MotionMagic.withMotionMagicAcceleration(IntakeConstants.Pivot.MM_ACCELERATION);
    config.MotionMagic.withMotionMagicJerk(IntakeConstants.Pivot.MM_JERK);

    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Pivot.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.Pivot.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(IntakeConstants.Pivot.INTAKE_POSITION);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(IntakeConstants.Pivot.STOWED_POSITION);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotMotor.applyConfiguration(config);
    configureSignalRates();
  }

  private void configureSignalRates() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotMotor.getRotorPosition(),
        pivotMotor.getRotorVelocity(),
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
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRotations = pivotMotor.getRotorPosition().getValueAsDouble();
    inputs.velocityRPS = pivotMotor.getRotorVelocity().getValueAsDouble();
    inputs.statorCurrentAmps = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = pivotMotor.getSupplyCurrent().getValueAsDouble();
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
  public void setMotionMagicPosition(Angle position) {
    pivotMotor.setControl(motionMagicVoltage.withPosition(position));
  }

  @Override
  public void setNeutral() {
    pivotMotor.setControl(neutralRequest);
  }

  @Override
  public void periodic() {
    pivotMotor.periodic();
  }
}
