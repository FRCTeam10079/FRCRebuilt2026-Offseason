// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.GameConstants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks hub active/inactive state using FMS Game Data and match time.
 *
 * <p>The FMS sends a single character ('R' or 'B') via `DriverStation.getGameSpecificMessage()` at
 * the start of TELEOP indicating which alliance scored more FUEL in AUTO (and thus gets their hub
 * set to INACTIVE first in SHIFT 1).
 *
 * <p>According to the 2026 REBUILT Game Manual (Section 6.4.1 Table 6-3):
 *
 * <p>- AUTO: Both hubs active - TRANSITION (2:20-2:10): Both hubs active - SHIFT 1 (2:10-1:45):
 * Winner of AUTO -> hub INACTIVE; Loser -> hub ACTIVE - SHIFT 2 (1:45-1:20): Alternates - SHIFT 3
 * (1:20-0:55): Alternates - SHIFT 4 (0:55-0:30): Alternates - END GAME (0:30-0:00): Both hubs
 * active
 *
 * <p>If game data is unavailable (practice mode, no FMS), falls back to operator manual override.
 */
public class HubShiftTracker {

  public enum ShiftPhase {
    AUTO,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME,
    UNKNOWN
  }

  private static HubShiftTracker instance;

  private boolean operatorOverrideActive = false;
  private boolean operatorOverrideHubActive = false;
  private boolean gameDataReceived = false;
  private boolean myAllianceScoredMoreInAuto = false;

  private HubShiftTracker() {}

  public static HubShiftTracker getInstance() {
    if (instance == null) {
      instance = new HubShiftTracker();
    }
    return instance;
  }

  /** Call every cycle from robotPeriodic to process FMS game data. */
  public void periodic() {
    if (!gameDataReceived) {
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData != null && !gameData.isEmpty()) {
        processGameData(gameData);
      }
    }

    // Telemetry
    Logger.recordOutput("HubShift/Phase", getCurrentPhase().name());
    Logger.recordOutput("HubShift/MyHubActive", isMyHubActive());
    Logger.recordOutput("HubShift/GameDataReceived", gameDataReceived);
    Logger.recordOutput("HubShift/TimeUntilNextShift", getTimeUntilNextShift());
    Logger.recordOutput("HubShift/OperatorOverride", operatorOverrideActive);
  }

  /**
   * Process the FMS game-specific message.
   *
   * <p>According to the game manual: The alliance that scores MORE fuel in AUTO gets their hub set
   * to INACTIVE for SHIFT 1. The FMS relays which alliance this is.
   *
   * <p>Format: 'R' means Red scored more (Red inactive in Shift 1), 'B' means Blue scored more
   * (Blue inactive in Shift 1).
   */
  private void processGameData(String gameData) {
    char data = gameData.charAt(0);
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return;

    // The alliance indicated by the game data scored more in AUTO
    // and gets their hub INACTIVE in SHIFT 1
    boolean redScoredMore = (data == 'R');

    if (alliance.get() == Alliance.Red) {
      // If red scored more, red hub is INACTIVE in Shift 1
      // So "my alliance scored more in auto" = red scored more
      myAllianceScoredMoreInAuto = redScoredMore;
    } else {
      // If blue scored more, blue hub is INACTIVE in Shift 1
      myAllianceScoredMoreInAuto = !redScoredMore;
    }

    gameDataReceived = true;
    Logger.recordOutput(
        "HubShift/GameData",
        String.format(
            "Data='%c' Alliance=%s MyScoredMore=%b",
            data, alliance.get().name(), myAllianceScoredMoreInAuto));
  }

  /** Get the current shift phase based on match time. */
  public ShiftPhase getCurrentPhase() {
    if (DriverStation.isAutonomousEnabled()) {
      return ShiftPhase.AUTO;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return ShiftPhase.UNKNOWN;
    }

    double matchTime = DriverStation.getMatchTime();
    if (matchTime <= 0) return ShiftPhase.UNKNOWN;

    if (matchTime > GameConstants.TRANSITION_END_TIME) {
      return ShiftPhase.TRANSITION; // 140 - 130
    } else if (matchTime > GameConstants.SHIFT_1_END) {
      return ShiftPhase.SHIFT_1; // 130 - 105
    } else if (matchTime > GameConstants.SHIFT_2_END) {
      return ShiftPhase.SHIFT_2; // 105 - 80
    } else if (matchTime > GameConstants.SHIFT_3_END) {
      return ShiftPhase.SHIFT_3; // 80 - 55
    } else if (matchTime > GameConstants.SHIFT_4_END) {
      return ShiftPhase.SHIFT_4; // 55 - 30
    } else {
      return ShiftPhase.ENDGAME; // 30 - 0
    }
  }

  /**
   * Determine if our hub is currently active.
   *
   * <p>If operator override is active, uses the override value. Otherwise uses FMS data + match
   * time. If no game data is available, defaults to true (assume active).
   */
  public boolean isMyHubActive() {
    if (operatorOverrideActive) {
      return operatorOverrideHubActive;
    }
    return computeHubActive(getCurrentPhase());
  }

  /**
   * Predict if our hub will be active at a future match time.
   *
   * @param futureMatchTime the DriverStation.getMatchTime() value to check (countdown value)
   * @return true if our hub would be active at that time
   */
  public boolean isMyHubActiveAtTime(double futureMatchTime) {
    if (operatorOverrideActive) {
      return operatorOverrideHubActive;
    }
    return computeHubActive(getPhaseAtTime(futureMatchTime));
  }

  /**
   * Get the time (in seconds) until our hub becomes active.
   *
   * @return seconds until hub activates, or 0 if already active, or -1 if unknown
   */
  public double getTimeUntilHubActive() {
    if (isMyHubActive()) return 0.0;
    if (!DriverStation.isTeleopEnabled()) return -1.0;

    double matchTime = DriverStation.getMatchTime();
    if (matchTime <= 0) return -1.0;

    ShiftPhase currentPhase = getCurrentPhase();

    // Find the next shift boundary where our hub becomes active
    double nextActiveTime = getNextActiveShiftBoundary(currentPhase, matchTime);
    if (nextActiveTime < 0) return -1.0;

    return matchTime - nextActiveTime;
  }

  /**
   * Get the time (in seconds) until the next shift change (any change).
   *
   * @return seconds until shift changes, or -1 if unknown
   */
  public double getTimeUntilNextShift() {
    if (!DriverStation.isTeleopEnabled()) return -1.0;

    double matchTime = DriverStation.getMatchTime();
    if (matchTime <= 0) return -1.0;

    // Return time until next boundary
    if (matchTime > GameConstants.TRANSITION_END_TIME) {
      return matchTime - GameConstants.TRANSITION_END_TIME;
    } else if (matchTime > GameConstants.SHIFT_1_END) {
      return matchTime - GameConstants.SHIFT_1_END;
    } else if (matchTime > GameConstants.SHIFT_2_END) {
      return matchTime - GameConstants.SHIFT_2_END;
    } else if (matchTime > GameConstants.SHIFT_3_END) {
      return matchTime - GameConstants.SHIFT_3_END;
    } else if (matchTime > GameConstants.SHIFT_4_END) {
      return matchTime - GameConstants.SHIFT_4_END;
    } else {
      return matchTime; // Time until match ends
    }
  }

  // ==================== OPERATOR OVERRIDE ====================

  /** Enable operator override to manually set hub state. */
  public void setOperatorOverride(boolean hubActive) {
    operatorOverrideActive = true;
    operatorOverrideHubActive = hubActive;
  }

  /** Disable operator override and return to FMS-based tracking. */
  public void clearOperatorOverride() {
    operatorOverrideActive = false;
  }

  public boolean isOperatorOverrideActive() {
    return operatorOverrideActive;
  }

  public boolean hasGameData() {
    return gameDataReceived;
  }

  /** Reset state for a new match. */
  public void reset() {
    gameDataReceived = false;
    myAllianceScoredMoreInAuto = false;
    operatorOverrideActive = false;
    operatorOverrideHubActive = false;
  }

  // ==================== INTERNAL LOGIC ====================

  /**
   * Compute whether our hub is active for a given shift phase.
   *
   * <p>According to Table 6-3 in the game manual: The alliance that scored MORE in AUTO has their
   * hub INACTIVE in Shifts 1 and 3, and ACTIVE in Shifts 2 and 4.
   */
  private boolean computeHubActive(ShiftPhase phase) {
    switch (phase) {
      case AUTO, TRANSITION, ENDGAME:
        return true; // Both hubs active during these phases
      case SHIFT_1, SHIFT_3:
        if (!gameDataReceived) return true; // Default to active if no data
        // If my alliance scored more in auto, my hub is INACTIVE in shifts 1 & 3
        return !myAllianceScoredMoreInAuto;
      case SHIFT_2, SHIFT_4:
        if (!gameDataReceived) return true;
        // If my alliance scored more in auto, my hub is ACTIVE in shifts 2 & 4
        return myAllianceScoredMoreInAuto;
      default:
        return true; // Default to active
    }
  }

  private ShiftPhase getPhaseAtTime(double matchTime) {
    if (matchTime > GameConstants.TELEOP_DURATION) return ShiftPhase.AUTO;
    if (matchTime > GameConstants.TRANSITION_END_TIME) return ShiftPhase.TRANSITION;
    if (matchTime > GameConstants.SHIFT_1_END) return ShiftPhase.SHIFT_1;
    if (matchTime > GameConstants.SHIFT_2_END) return ShiftPhase.SHIFT_2;
    if (matchTime > GameConstants.SHIFT_3_END) return ShiftPhase.SHIFT_3;
    if (matchTime > GameConstants.SHIFT_4_END) return ShiftPhase.SHIFT_4;
    if (matchTime > 0) return ShiftPhase.ENDGAME;
    return ShiftPhase.UNKNOWN;
  }

  /**
   * Find the match time value at which our hub next becomes active.
   *
   * @return the match time countdown value, or -1 if it won't become active
   */
  private double getNextActiveShiftBoundary(ShiftPhase currentPhase, double currentMatchTime) {
    // Check each upcoming phase boundary to see if our hub becomes active
    double[] boundaries = {
      GameConstants.TRANSITION_END_TIME, // 130 - start of Shift 1
      GameConstants.SHIFT_1_END, // 105 - start of Shift 2
      GameConstants.SHIFT_2_END, // 80 - start of Shift 3
      GameConstants.SHIFT_3_END, // 55 - start of Shift 4
      GameConstants.SHIFT_4_END // 30 - start of Endgame
    };

    for (double boundary : boundaries) {
      if (boundary < currentMatchTime) {
        ShiftPhase nextPhase = getPhaseAtTime(boundary - 0.01);
        if (computeHubActive(nextPhase)) {
          return boundary;
        }
      }
    }
    return -1.0;
  }
}
