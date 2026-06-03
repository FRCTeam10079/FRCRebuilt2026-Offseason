package frc.robot.subsystems.climber;

public final class NoOpClimberIO implements ClimberIO {
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVoltage = appliedVolts;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
