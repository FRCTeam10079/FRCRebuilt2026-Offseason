package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX motor;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Temperature> temperatureSignal;
  private final StatusSignal<Double> dutyCycleSignal;
  private final StatusSignal<Voltage> supplyVoltageSignal;

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(ClimberConstants.MOTOR_CONNECTED_DEBOUNCE_SECONDS);

  public ClimberIOTalonFX() {
    motor =
        new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID, new CANBus(ClimberConstants.CLIMBER_CANBUS));
    configureMotor();

    // Zero the encoder at construction time.
    // Assumption: mechanism is fully retracted when the robot powers on.
    motor.setPosition(0.0);

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    supplyCurrentSignal = motor.getSupplyCurrent();
    statorCurrentSignal = motor.getStatorCurrent();
    voltageSignal = motor.getMotorVoltage();
    temperatureSignal = motor.getDeviceTemp();
    dutyCycleSignal = motor.getDutyCycle();
    supplyVoltageSignal = motor.getSupplyVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ClimberConstants.STATUS_SIGNAL_UPDATE_HZ,
        positionSignal,
        velocitySignal,
        supplyCurrentSignal,
        statorCurrentSignal,
        voltageSignal,
        temperatureSignal,
        dutyCycleSignal,
        supplyVoltageSignal);
    ParentDevice.optimizeBusUtilizationForAll(motor);

    voltageRequest.EnableFOC = ClimberConstants.ENABLE_FOC;
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Brake mode is critical - holds robot weight when motor is neutral
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = ClimberConstants.MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    config.Voltage.PeakForwardVoltage = ClimberConstants.PEAK_FORWARD_VOLTAGE;
    config.Voltage.PeakReverseVoltage = ClimberConstants.PEAK_REVERSE_VOLTAGE;

    // Current limits to protect the motor under sustained climbing load
    config.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(ClimberConstants.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(ClimberConstants.STALL_PROTECTION_LOWER_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerTime(ClimberConstants.STALL_PROTECTION_TIMEOUT_SECONDS)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(ClimberConstants.STATOR_CURRENT_LIMIT);

    // No software limits - stall detection handles stopping.
    // This also eliminates software limits as a possible failure cause.
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        supplyCurrentSignal,
        statorCurrentSignal,
        voltageSignal,
        temperatureSignal,
        dutyCycleSignal,
        supplyVoltageSignal);

    inputs.motorConnected = motorConnectedDebouncer.calculate(status.isOK());
    inputs.positionRotations = positionSignal.getValueAsDouble();
    inputs.velocityRPS = velocitySignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
    inputs.appliedVoltage = voltageSignal.getValueAsDouble();
    inputs.tempCelsius = temperatureSignal.getValueAsDouble();
    inputs.dutyCycle = dutyCycleSignal.getValueAsDouble();
    inputs.supplyVoltage = supplyVoltageSignal.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }

  @Override
  public void setEncoderPosition(double rotations) {
    motor.setPosition(rotations);
  }
}
