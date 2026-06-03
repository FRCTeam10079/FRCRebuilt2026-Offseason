package frc.robot.lib;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

public class PowerDiagnosticsLogger {
  private static final double LOG_PERIOD_SECONDS = 0.1;
  private static final double FAILURE_LOG_PERIOD_SECONDS = 2.0;
  private static final double PDH_RETRY_BASE_SECONDS = 5.0;
  private static final double PDH_RETRY_MAX_SECONDS = 30.0;
  private static final int PDH_MAX_FAILURES = 3;
  private static final double SUMMARY_LOG_PERIOD_SECONDS = 1.0;
  private static final double BATTERY_LOW_THRESHOLD_VOLTS = 9.0;
  private static final double HIGH_MECHANISM_CURRENT_THRESHOLD_AMPS = 120.0;
  private static final double CURRENT_SPIKE_THRESHOLD_AMPS = 40.0;

  private final PowerDistribution m_powerDistribution;
  private final IntakeWheelsSubsystem m_intake;
  private final PivotSubsystem m_intakePivot;
  private final IndexerSubsystem m_indexer;
  private final ShooterSubsystem m_shooter;
  private final ShooterPivotSubsystem m_shooterPivot;

  private double m_lastLogTimestampSeconds = Double.NEGATIVE_INFINITY;
  private double m_lastFailureLogTimestampSeconds = Double.NEGATIVE_INFINITY;
  private double m_lastSummaryTimestampSeconds = Double.NEGATIVE_INFINITY;
  private boolean m_pdhTelemetryEnabled = true;
  private int m_pdhFailureCount = 0;
  private double m_nextPdhRetryTimestampSeconds = 0.0;
  private double m_lastPdhTotalCurrentAmps = Double.NaN;
  private double m_lastPdhVoltageVolts = Double.NaN;
  private double m_lastMechanismTotalAmps = Double.NaN;
  private boolean m_batteryLowActive = false;
  private boolean m_highCurrentActive = false;
  private int m_eventSequence = 0;

  public PowerDiagnosticsLogger(
      IntakeWheelsSubsystem intake,
      PivotSubsystem intakePivot,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      ShooterPivotSubsystem shooterPivot) {
    m_intake = intake;
    m_intakePivot = intakePivot;
    m_indexer = indexer;
    m_shooter = shooter;
    m_shooterPivot = shooterPivot;

    PowerDistribution pdh = null;
    try {
      pdh = new PowerDistribution();
    } catch (RuntimeException ex) {
      recordEvent("[PowerDiagnostics] PD init failed: " + ex.getMessage());
    }
    m_powerDistribution = pdh;

    recordEvent("[PowerDiagnostics] Power diagnostics logger initialized");
    if (m_powerDistribution == null) {
      recordEvent("[PowerDiagnostics] PD telemetry disabled until PD can be read");
      m_pdhTelemetryEnabled = false;
      m_nextPdhRetryTimestampSeconds = 0.0;
    }
  }

  public void logPeriodic() {
    double nowSeconds = RobotController.getFPGATime() / 1_000_000.0;
    if (nowSeconds - m_lastLogTimestampSeconds < LOG_PERIOD_SECONDS) {
      return;
    }
    m_lastLogTimestampSeconds = nowSeconds;

    double batteryVoltage = RobotController.getBatteryVoltage();
    double brownoutVoltage = RobotController.getBrownoutVoltage();
    boolean batteryLow = batteryVoltage <= BATTERY_LOW_THRESHOLD_VOLTS;

    Logger.recordOutput("Power/Robot/BatteryVoltageVolts", batteryVoltage);
    Logger.recordOutput("Power/Robot/BrownoutVoltageVolts", brownoutVoltage);

    // Canonical outputs used by existing AdvantageScope layouts.
    Logger.recordOutput("Power/BatteryVoltage", batteryVoltage);
    Logger.recordOutput("Power/BrownoutVoltage", brownoutVoltage);
    Logger.recordOutput("Power/BatteryLow", batteryLow);

    logPdhTelemetry(nowSeconds);

    double intakeSupply = m_intake.getSupplyCurrentAmps();
    double intakeStator = m_intake.getStatorCurrentAmps();
    double intakeVoltage = m_intake.getMotorVoltageVolts();

    double intakePivotSupply = m_intakePivot.getSupplyCurrentAmps();
    double intakePivotStator = m_intakePivot.getStatorCurrentAmps();
    double intakePivotVoltage = m_intakePivot.getMotorVoltageVolts();

    double feederSupply = 0.0;
    double spindexerSupply = 0.0;
    double feederStator = 0.0;
    double spindexerStator = 0.0;
    double feederVoltage = 0.0;
    double spindexerVoltage = 0.0;

    double shooterMasterSupply = 0.0;
    double shooterSlaveSupply = 0.0;
    double shooterMasterStator = 0.0;
    double shooterSlaveStator = 0.0;
    double shooterMasterVoltage = 0.0;
    double shooterSlaveVoltage = 0.0;

    double shooterPivotSupply = m_shooterPivot.getSupplyCurrentAmps();
    double shooterPivotStator = m_shooterPivot.getStatorCurrentAmps();
    double shooterPivotVoltage = m_shooterPivot.getMotorVoltageVolts();

    Logger.recordOutput("Power/Subsystems/Intake/SupplyCurrentAmps", intakeSupply);
    Logger.recordOutput("Power/Subsystems/Intake/StatorCurrentAmps", intakeStator);
    Logger.recordOutput("Power/Subsystems/Intake/MotorVoltageVolts", intakeVoltage);

    Logger.recordOutput("Power/Subsystems/IntakePivot/SupplyCurrentAmps", intakePivotSupply);
    Logger.recordOutput("Power/Subsystems/IntakePivot/StatorCurrentAmps", intakePivotStator);
    Logger.recordOutput("Power/Subsystems/IntakePivot/MotorVoltageVolts", intakePivotVoltage);

    try {
      feederSupply = m_indexer.getFeederSupplyCurrentAmps();
      spindexerSupply = m_indexer.getSpindexerSupplyCurrentAmps();
      feederStator = m_indexer.getFeederStatorCurrentAmps();
      spindexerStator = m_indexer.getSpindexerStatorCurrentAmps();
      feederVoltage = m_indexer.getFeederVoltageVolts();
      spindexerVoltage = m_indexer.getSpindexerVoltageVolts();
    } catch (RuntimeException ex) {
      logFailureRateLimited(
          nowSeconds, "[PowerDiagnostics] Indexer telemetry read failed: " + ex.getMessage());
    }

    Logger.recordOutput("Power/Subsystems/Indexer/FeederSupplyCurrentAmps", feederSupply);
    Logger.recordOutput("Power/Subsystems/Indexer/SpindexerSupplyCurrentAmps", spindexerSupply);
    Logger.recordOutput("Power/Subsystems/Indexer/FeederStatorCurrentAmps", feederStator);
    Logger.recordOutput("Power/Subsystems/Indexer/SpindexerStatorCurrentAmps", spindexerStator);
    Logger.recordOutput("Power/Subsystems/Indexer/FeederVoltageVolts", feederVoltage);
    Logger.recordOutput("Power/Subsystems/Indexer/SpindexerVoltageVolts", spindexerVoltage);
    Logger.recordOutput(
        "Power/Subsystems/Indexer/TotalSupplyCurrentAmps", feederSupply + spindexerSupply);

    try {
      shooterMasterSupply = m_shooter.getMasterSupplyCurrentAmps();
      shooterSlaveSupply = m_shooter.getSlaveSupplyCurrentAmps();
      shooterMasterStator = m_shooter.getMasterStatorCurrentAmps();
      shooterSlaveStator = m_shooter.getSlaveStatorCurrentAmps();
      shooterMasterVoltage = m_shooter.getMasterVoltageVolts();
      shooterSlaveVoltage = m_shooter.getSlaveVoltageVolts();
    } catch (RuntimeException ex) {
      logFailureRateLimited(
          nowSeconds, "[PowerDiagnostics] Shooter telemetry read failed: " + ex.getMessage());
    }

    Logger.recordOutput("Power/Subsystems/Shooter/MasterSupplyCurrentAmps", shooterMasterSupply);
    Logger.recordOutput("Power/Subsystems/Shooter/SlaveSupplyCurrentAmps", shooterSlaveSupply);
    Logger.recordOutput("Power/Subsystems/Shooter/MasterStatorCurrentAmps", shooterMasterStator);
    Logger.recordOutput("Power/Subsystems/Shooter/SlaveStatorCurrentAmps", shooterSlaveStator);
    Logger.recordOutput("Power/Subsystems/Shooter/MasterVoltageVolts", shooterMasterVoltage);
    Logger.recordOutput("Power/Subsystems/Shooter/SlaveVoltageVolts", shooterSlaveVoltage);
    Logger.recordOutput(
        "Power/Subsystems/Shooter/TotalSupplyCurrentAmps",
        shooterMasterSupply + shooterSlaveSupply);

    Logger.recordOutput("Power/Subsystems/ShooterPivot/SupplyCurrentAmps", shooterPivotSupply);
    Logger.recordOutput("Power/Subsystems/ShooterPivot/StatorCurrentAmps", shooterPivotStator);
    Logger.recordOutput("Power/Subsystems/ShooterPivot/MotorVoltageVolts", shooterPivotVoltage);

    double mechanismsTotalSupplyCurrent = intakeSupply
        + intakePivotSupply
        + feederSupply
        + spindexerSupply
        + shooterMasterSupply
        + shooterSlaveSupply
        + shooterPivotSupply;
    Logger.recordOutput(
        "Power/Subsystems/TotalMechanismSupplyCurrentAmps", mechanismsTotalSupplyCurrent);

    double[] mechanismCurrents = {
      intakeSupply,
      intakePivotSupply,
      feederSupply,
      spindexerSupply,
      shooterMasterSupply,
      shooterSlaveSupply,
      shooterPivotSupply
    };
    String[] mechanismNames = {
      "intake",
      "intakePivot",
      "indexerFeeder",
      "indexerSpindexer",
      "shooterMaster",
      "shooterSlave",
      "shooterPivot"
    };

    logDiagnosticEvents(
        nowSeconds,
        batteryVoltage,
        mechanismsTotalSupplyCurrent,
        mechanismNames,
        mechanismCurrents);
    logDiagnosticSummary(
        nowSeconds,
        batteryVoltage,
        mechanismsTotalSupplyCurrent,
        mechanismNames,
        mechanismCurrents);

    m_lastMechanismTotalAmps = mechanismsTotalSupplyCurrent;
  }

  private void logPdhTelemetry(double nowSeconds) {
    if (m_powerDistribution == null) {
      return;
    }

    if (!m_pdhTelemetryEnabled
        && (m_pdhFailureCount >= PDH_MAX_FAILURES || nowSeconds < m_nextPdhRetryTimestampSeconds)) {
      return;
    }

    try {
      double pdhVoltage = m_powerDistribution.getVoltage();
      double pdhTempCelsius = m_powerDistribution.getTemperature();
      double pdhTotalCurrent = m_powerDistribution.getTotalCurrent();
      double pdhTotalPower = m_powerDistribution.getTotalPower();
      double pdhTotalEnergy = m_powerDistribution.getTotalEnergy();
      boolean switchableEnabled = m_powerDistribution.getSwitchableChannel();

      Logger.recordOutput("Power/PDH/VoltageVolts", pdhVoltage);
      Logger.recordOutput("Power/PDH/TemperatureCelsius", pdhTempCelsius);
      Logger.recordOutput("Power/PDH/TotalCurrentAmps", pdhTotalCurrent);
      Logger.recordOutput("Power/PDH/TotalPowerWatts", pdhTotalPower);
      Logger.recordOutput("Power/PDH/TotalEnergyJoules", pdhTotalEnergy);
      Logger.recordOutput("Power/PDH/SwitchableChannelEnabled", switchableEnabled);

      int channels = m_powerDistribution.getNumChannels();
      for (int channel = 0; channel < channels; channel++) {
        Logger.recordOutput(
            "Power/PDH/Channel" + channel + "CurrentAmps", m_powerDistribution.getCurrent(channel));
      }

      if (!m_pdhTelemetryEnabled) {
        recordEvent("[PowerDiagnostics] PD telemetry recovered");
      }
      m_pdhTelemetryEnabled = true;
      m_pdhFailureCount = 0;
      m_lastPdhTotalCurrentAmps = pdhTotalCurrent;
      m_lastPdhVoltageVolts = pdhVoltage;
    } catch (RuntimeException ex) {
      m_pdhTelemetryEnabled = false;
      m_pdhFailureCount++;

      double retryDelaySeconds = Math.min(
          PDH_RETRY_MAX_SECONDS, PDH_RETRY_BASE_SECONDS * Math.pow(2.0, m_pdhFailureCount - 1));
      m_nextPdhRetryTimestampSeconds = nowSeconds + retryDelaySeconds;

      logFailureRateLimited(
          nowSeconds,
          "[PowerDiagnostics] PD read failed (retry in "
              + String.format("%.2f", retryDelaySeconds)
              + "s): "
              + ex.getMessage());
    }
  }

  private void logDiagnosticSummary(
      double nowSeconds,
      double batteryVoltage,
      double mechanismsTotalSupplyCurrent,
      String[] mechanismNames,
      double[] mechanismCurrents) {
    if (nowSeconds - m_lastSummaryTimestampSeconds < SUMMARY_LOG_PERIOD_SECONDS) {
      return;
    }
    m_lastSummaryTimestampSeconds = nowSeconds;

    int[] top = getTopThreeIndices(mechanismCurrents);
    String pdhTotal = Double.isNaN(m_lastPdhTotalCurrentAmps)
        ? "na"
        : String.format("%.1f", m_lastPdhTotalCurrentAmps);
    String pdhVoltage =
        Double.isNaN(m_lastPdhVoltageVolts) ? "na" : String.format("%.2f", m_lastPdhVoltageVolts);

    String flags = "none";
    if (batteryVoltage <= BATTERY_LOW_THRESHOLD_VOLTS) {
      flags = "batteryLow";
    }
    if (mechanismsTotalSupplyCurrent >= HIGH_MECHANISM_CURRENT_THRESHOLD_AMPS) {
      flags = flags.equals("none") ? "highMechCurrent" : flags + "|highMechCurrent";
    }
    if (!m_pdhTelemetryEnabled) {
      flags = flags.equals("none") ? "pdhUnavailable" : flags + "|pdhUnavailable";
    }

    String summary = String.format(
        "t=%.2f,batt=%.2f,mechTotal=%.1f,pdhTotal=%s,pdhVolt=%s,top1=%s:%.1f,top2=%s:%.1f,top3=%s:%.1f,flags=%s",
        nowSeconds,
        batteryVoltage,
        mechanismsTotalSupplyCurrent,
        pdhTotal,
        pdhVoltage,
        mechanismNames[top[0]],
        mechanismCurrents[top[0]],
        mechanismNames[top[1]],
        mechanismCurrents[top[1]],
        mechanismNames[top[2]],
        mechanismCurrents[top[2]],
        flags);
    Logger.recordOutput("Power/Diagnostics/Summary", summary);
  }

  private void logDiagnosticEvents(
      double nowSeconds,
      double batteryVoltage,
      double mechanismsTotalSupplyCurrent,
      String[] mechanismNames,
      double[] mechanismCurrents) {
    int[] top = getTopThreeIndices(mechanismCurrents);

    boolean batteryLowNow = batteryVoltage <= BATTERY_LOW_THRESHOLD_VOLTS;
    if (batteryLowNow && !m_batteryLowActive) {
      recordEvent(String.format(
          "t=%.2f,event=batteryLowStart,batt=%.2f,top=%s:%.1f",
          nowSeconds, batteryVoltage, mechanismNames[top[0]], mechanismCurrents[top[0]]));
    } else if (!batteryLowNow && m_batteryLowActive) {
      recordEvent(
          String.format("t=%.2f,event=batteryLowEnd,batt=%.2f", nowSeconds, batteryVoltage));
    }
    m_batteryLowActive = batteryLowNow;

    boolean highCurrentNow = mechanismsTotalSupplyCurrent >= HIGH_MECHANISM_CURRENT_THRESHOLD_AMPS;
    if (highCurrentNow && !m_highCurrentActive) {
      recordEvent(String.format(
          "t=%.2f,event=highMechanismCurrentStart,total=%.1f,top=%s:%.1f",
          nowSeconds,
          mechanismsTotalSupplyCurrent,
          mechanismNames[top[0]],
          mechanismCurrents[top[0]]));
    } else if (!highCurrentNow && m_highCurrentActive) {
      recordEvent(String.format(
          "t=%.2f,event=highMechanismCurrentEnd,total=%.1f",
          nowSeconds, mechanismsTotalSupplyCurrent));
    }
    m_highCurrentActive = highCurrentNow;

    if (!Double.isNaN(m_lastMechanismTotalAmps)) {
      double delta = mechanismsTotalSupplyCurrent - m_lastMechanismTotalAmps;
      if (delta >= CURRENT_SPIKE_THRESHOLD_AMPS) {
        recordEvent(String.format(
            "t=%.2f,event=mechanismCurrentSpike,delta=%.1f,total=%.1f,top=%s:%.1f",
            nowSeconds,
            delta,
            mechanismsTotalSupplyCurrent,
            mechanismNames[top[0]],
            mechanismCurrents[top[0]]));
      }
    }
  }

  private int[] getTopThreeIndices(double[] values) {
    int first = 0;
    int second = values.length > 1 ? 1 : 0;
    int third = values.length > 2 ? 2 : second;

    if (values[second] > values[first]) {
      int temp = first;
      first = second;
      second = temp;
    }
    if (values[third] > values[second]) {
      int temp = second;
      second = third;
      third = temp;
      if (values[second] > values[first]) {
        temp = first;
        first = second;
        second = temp;
      }
    }

    for (int i = 3; i < values.length; i++) {
      if (values[i] > values[first]) {
        third = second;
        second = first;
        first = i;
      } else if (values[i] > values[second]) {
        third = second;
        second = i;
      } else if (values[i] > values[third]) {
        third = i;
      }
    }

    return new int[] {first, second, third};
  }

  private void logFailureRateLimited(double nowSeconds, String message) {
    if (nowSeconds - m_lastFailureLogTimestampSeconds < FAILURE_LOG_PERIOD_SECONDS) {
      return;
    }
    m_lastFailureLogTimestampSeconds = nowSeconds;
    recordEvent(message);
  }

  private void recordEvent(String message) {
    Logger.recordOutput("Power/Diagnostics/Event", message);
    Logger.recordOutput("Power/Diagnostics/EventSequence", ++m_eventSequence);
  }
}
