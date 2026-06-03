package frc.robot.constants;

public class HeadingControllerConstants {
  // SNAP gains - used for initial target acquisition (high aggression).
  // 3x increase from originals (0.02/0.001). Data showed SNAP never ran because
  // the old 2 deg tolerance was hit instantly. These higher gains ensure a fast,
  // well-damped convergence when SNAP actually engages.
  public static final double SNAP_KP = 0.06;
  public static final double SNAP_KI = 0.0;
  public static final double SNAP_KD = 0.003;

  // MAINTAIN gains - used to hold heading while driving.
  // 4x increase from originals (0.01/0.0005). Data analysis showed 12-14 deg
  // steady-
  // state error while driving with the old gains because:
  // steady_state_error = disturbance_rate / (KP * maxAngularVelocity)
  // old: 0.4 / (0.01 * pi) ~ 12.7 deg <-- matched data exactly
  // new: 0.4 / (0.04 * pi) ~ 3.2 deg <-- acceptable for shooting
  // KI stays zero - a moving target would cause integral wind-up.
  // KD scaled proportionally to damp oscillation from the higher P.
  public static final double MAINTAIN_KP = 0.04;
  public static final double MAINTAIN_KI = 0.0;
  public static final double MAINTAIN_KD = 0.002;

  // Tightened from 2.0 to 1.5 deg. The old tolerance caused SNAP to exit on the
  // very first cycle (before the robot had actually settled), dumping it into
  // the weaker MAINTAIN gains prematurely.
  public static final double HEADING_TOLERANCE_DEGREES = 1.5;

  // If the heading error exceeds this while in MAINTAIN, re-enter SNAP.
  // This handles sudden large heading changes (e.g. driver reverses direction)
  // that the lower MAINTAIN gains can't recover from quickly enough.
  // 8 deg is conservative - can tighten to 5 deg after initial validation.
  public static final double RESNAP_THRESHOLD_DEGREES = 8.0;

  // ==================== LAUNCH MODE HEADING CONTROL ====================
  // These are used by the shoot-on-the-move heading controller.
  // The launch controller uses PD + angular velocity feedforward (NOT the
  // SNAP/MAINTAIN PID).
  // Gains are in radians (not degrees) - much more aggressive than SNAP/MAINTAIN.
  //
  // MA (6328) uses kP=8.0, kD=0.5 with radians. We use the same as a starting
  // point.

  /** P gain for launch-mode heading control (rad/s output per radian of error). */
  public static final double LAUNCH_KP = 8.2;

  /** D gain for launch-mode heading control (rad/s output per rad/s velocity error). */
  public static final double LAUNCH_KD = 0.7;

  protected HeadingControllerConstants() {}
}
