package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {

  private final TalonFX feederMotor;
  private final TalonFX spindexerMotor;
  private final VelocityVoltage feederRequest = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage spindexerRequest = new VelocityVoltage(0).withSlot(0);

  public IndexerIOTalonFX() {
    feederMotor = new TalonFX(IndexerConstants.kFeederMotorID, new CANBus("rio"));
    spindexerMotor = new TalonFX(IndexerConstants.kSpindexerMotorID, new CANBus("rio"));
    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit;
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feederConfig.Slot0 = new Slot0Configs()
        .withKP(IndexerConstants.kFeederKP)
        .withKI(IndexerConstants.kFeederKI)
        .withKD(IndexerConstants.kFeederKD)
        .withKS(IndexerConstants.kFeederKS)
        .withKV(IndexerConstants.kFeederKV)
        .withKA(IndexerConstants.kFeederKA)
        .withKG(IndexerConstants.kFeederKG);
    feederMotor.getConfigurator().apply(feederConfig);

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    spindexerConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit;
    spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spindexerConfig.Slot0 = new Slot0Configs()
        .withKP(IndexerConstants.kSpindexerKP)
        .withKI(IndexerConstants.kSpindexerKI)
        .withKD(IndexerConstants.kSpindexerKD)
        .withKS(IndexerConstants.kSpindexerKS)
        .withKV(IndexerConstants.kSpindexerKV)
        .withKA(IndexerConstants.kSpindexerKA)
        .withKG(IndexerConstants.kSpindexerKG);
    spindexerMotor.getConfigurator().apply(spindexerConfig);

    configureSignalRates();
  }

  private void configureSignalRates() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        feederMotor.getVelocity(),
        feederMotor.getSupplyCurrent(),
        feederMotor.getStatorCurrent(),
        feederMotor.getMotorVoltage(),
        feederMotor.getDutyCycle(),
        feederMotor.getClosedLoopReference(),
        feederMotor.getClosedLoopError(),
        spindexerMotor.getVelocity(),
        spindexerMotor.getSupplyCurrent(),
        spindexerMotor.getStatorCurrent(),
        spindexerMotor.getMotorVoltage(),
        spindexerMotor.getDutyCycle(),
        spindexerMotor.getClosedLoopReference(),
        spindexerMotor.getClosedLoopError());

    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        feederMotor.getDeviceTemp(),
        feederMotor.getFaultField(),
        feederMotor.getStickyFaultField(),
        spindexerMotor.getDeviceTemp(),
        spindexerMotor.getFaultField(),
        spindexerMotor.getStickyFaultField());

    feederMotor.optimizeBusUtilization();
    spindexerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.feederVelocityRPS = feederMotor.getVelocity().getValueAsDouble();
    inputs.spindexerVelocityRPS = spindexerMotor.getVelocity().getValueAsDouble();
    inputs.feederSupplyCurrentAmps = feederMotor.getSupplyCurrent().getValueAsDouble();
    inputs.feederStatorCurrentAmps = feederMotor.getStatorCurrent().getValueAsDouble();
    inputs.feederVoltageVolts = feederMotor.getMotorVoltage().getValueAsDouble();
    inputs.feederDutyCycle = feederMotor.getDutyCycle().getValueAsDouble();
    inputs.feederClosedLoopReferenceRPS = feederMotor.getClosedLoopReference().getValueAsDouble();
    inputs.feederClosedLoopErrorRPS = feederMotor.getClosedLoopError().getValueAsDouble();
    inputs.feederDeviceTempCelsius = feederMotor.getDeviceTemp().getValueAsDouble();
    inputs.feederFaultField = feederMotor.getFaultField().getValue().intValue();
    inputs.feederStickyFaultField = feederMotor.getStickyFaultField().getValue().intValue();
    inputs.spindexerSupplyCurrentAmps = spindexerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.spindexerStatorCurrentAmps = spindexerMotor.getStatorCurrent().getValueAsDouble();
    inputs.spindexerVoltageVolts = spindexerMotor.getMotorVoltage().getValueAsDouble();
    inputs.spindexerDutyCycle = spindexerMotor.getDutyCycle().getValueAsDouble();
    inputs.spindexerClosedLoopReferenceRPS =
        spindexerMotor.getClosedLoopReference().getValueAsDouble();
    inputs.spindexerClosedLoopErrorRPS = spindexerMotor.getClosedLoopError().getValueAsDouble();
    inputs.spindexerDeviceTempCelsius = spindexerMotor.getDeviceTemp().getValueAsDouble();
    inputs.spindexerFaultField = spindexerMotor.getFaultField().getValue().intValue();
    inputs.spindexerStickyFaultField =
        spindexerMotor.getStickyFaultField().getValue().intValue();
  }

  @Override
  public void setFeederVelocity(double rps) {
    feederMotor.setControl(feederRequest.withVelocity(rps));
  }

  @Override
  public void setSpindexerVelocity(double rps) {
    spindexerMotor.setControl(spindexerRequest.withVelocity(rps));
  }

  @Override
  public void stop() {
    feederMotor.stopMotor();
    spindexerMotor.stopMotor();
  }
}
