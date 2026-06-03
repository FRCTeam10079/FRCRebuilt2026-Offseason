package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeWheelsIOTalonFX implements IntakeWheelsIO {

  private final TalonFX intakeMotor;
  private final TalonFX slaveMotor;
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(false);
  private final NeutralOut neutralRequest = new NeutralOut();

  public IntakeWheelsIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.Wheels.MOTOR_ID, Constants.kCANBus);
    slaveMotor = new TalonFX(IntakeConstants.Wheels.SLAVE_MOTOR_ID, Constants.kCANBus);
    configureMotors();

    // Slave follows master with opposed alignment (motors face opposite directions)
    slaveMotor.setControl(
        new Follower(IntakeConstants.Wheels.MOTOR_ID, MotorAlignmentValue.Opposed));
  }

  private void configureMotors() {
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    masterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    masterConfig.CurrentLimits.withSupplyCurrentLimit(IntakeConstants.Wheels.SUPPLY_CURRENT_LIMIT);
    masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    masterConfig.CurrentLimits.withStatorCurrentLimit(IntakeConstants.Wheels.STATOR_CURRENT_LIMIT);
    masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    masterConfig
        .Slot0
        .withKA(IntakeConstants.Wheels.KA)
        .withKV(IntakeConstants.Wheels.KV)
        .withKD(IntakeConstants.Wheels.KD)
        .withKS(IntakeConstants.Wheels.KS)
        .withKI(IntakeConstants.Wheels.KI)
        .withKP(IntakeConstants.Wheels.KP);

    intakeMotor.getConfigurator().apply(masterConfig);

    // Slave only needs neutral mode - PID/current limits follow from the Follower
    // control
    TalonFXConfiguration slaveConfig = masterConfig;
    slaveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    slaveMotor.getConfigurator().apply(slaveConfig);

    configureSignalRates();
  }

  private void configureSignalRates() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeMotor.getVelocity(),
        intakeMotor.getSupplyCurrent(),
        intakeMotor.getStatorCurrent(),
        intakeMotor.getMotorVoltage(),
        intakeMotor.getDutyCycle(),
        intakeMotor.getClosedLoopReference(),
        intakeMotor.getClosedLoopError(),
        slaveMotor.getVelocity(),
        slaveMotor.getSupplyCurrent(),
        slaveMotor.getStatorCurrent(),
        slaveMotor.getMotorVoltage(),
        slaveMotor.getDutyCycle());

    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        intakeMotor.getDeviceTemp(),
        intakeMotor.getFaultField(),
        intakeMotor.getStickyFaultField(),
        slaveMotor.getDeviceTemp(),
        slaveMotor.getFaultField(),
        slaveMotor.getStickyFaultField());

    intakeMotor.optimizeBusUtilization();
    slaveMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeWheelsIOInputs inputs) {
    inputs.velocityRPS = intakeMotor.getVelocity().getValueAsDouble();
    inputs.supplyCurrentAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = intakeMotor.getStatorCurrent().getValueAsDouble();
    inputs.voltageVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.dutyCycle = intakeMotor.getDutyCycle().getValueAsDouble();
    inputs.closedLoopReferenceRPS = intakeMotor.getClosedLoopReference().getValueAsDouble();
    inputs.closedLoopErrorRPS = intakeMotor.getClosedLoopError().getValueAsDouble();
    inputs.deviceTempCelsius = intakeMotor.getDeviceTemp().getValueAsDouble();
    inputs.faultField = intakeMotor.getFaultField().getValue().intValue();
    inputs.stickyFaultField = intakeMotor.getStickyFaultField().getValue().intValue();
    inputs.slaveVelocityRPS = slaveMotor.getVelocity().getValueAsDouble();
    inputs.slaveSupplyCurrentAmps = slaveMotor.getSupplyCurrent().getValueAsDouble();
    inputs.slaveStatorCurrentAmps = slaveMotor.getStatorCurrent().getValueAsDouble();
    inputs.slaveVoltageVolts = slaveMotor.getMotorVoltage().getValueAsDouble();
    inputs.slaveDutyCycle = slaveMotor.getDutyCycle().getValueAsDouble();
    inputs.slaveDeviceTempCelsius = slaveMotor.getDeviceTemp().getValueAsDouble();
    inputs.slaveFaultField = slaveMotor.getFaultField().getValue().intValue();
    inputs.slaveStickyFaultField = slaveMotor.getStickyFaultField().getValue().intValue();
  }

  @Override
  public void setVelocity(double rps) {
    intakeMotor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void stop() {
    intakeMotor.setControl(neutralRequest);
  }
}
