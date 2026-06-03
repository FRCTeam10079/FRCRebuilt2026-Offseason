// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoCommands;
import frc.robot.auto.Autos;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ClimbConstants.ClimbLane;
import frc.robot.controllers.DriverControls;
import frc.robot.controllers.OperatorControls;
import frc.robot.controllers.TestingBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.DashboardPublisher;
import frc.robot.lib.HubShiftTracker;
import frc.robot.lib.LaunchCalculator;
import frc.robot.lib.ShooterMath;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.lib.SmartShootController;
import frc.robot.pathfinding.Pathfinding;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.NoOpClimberIO;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.NoOpIndexerIO;
import frc.robot.subsystems.intake.IntakeWheelsIOTalonFX;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.NoOpIntakeWheelsIO;
import frc.robot.subsystems.intake.NoOpPivotIO;
import frc.robot.subsystems.intake.PivotIOTalonFX;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.NoOpShooterIO;
import frc.robot.subsystems.shooter.NoOpShooterPivotIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterPivotIOTalonFX;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * RobotContainer for FRC 2026 REBUILT season This class is where the robot's subsystems, commands,
 * and button bindings are defined.
 */
public class RobotContainer {

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final CommandXboxController m_testController = new CommandXboxController(2);

  // State Machine
  private final RobotStateMachine m_stateMachine = RobotStateMachine.getInstance();

  // ==================== SUBSYSTEMS ====================
  // Drivetrain - created from TunerConstants
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Vision
  public final Vision vision;
  // Subsystems (initialized in constructor based on mode)
  private final IndexerSubsystem indexer;
  public final ShooterSubsystem shooter;
  public final ShooterPivotSubsystem shooterPivot;
  private final IntakeWheelsSubsystem intake;
  private final PivotSubsystem pivot;
  private final ClimberSubsystem climber;

  private final Telemetry m_telemetry =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  // ==================== SUPERSTRUCTURE ====================
  private final Superstructure superstructure;

  // ==================== SMART SHOOT + DASHBOARD ====================
  private final SmartShootController smartShootController;
  private final DashboardPublisher dashboardPublisher;

  // ==================== AUTO ====================
  private final AutoFactory choreoAutoFactory;
  private final AutoCommands autoCommands;
  private final Autos autos;

  // ==================== DISTANCE-BASED SHOOTING ====================
  /** Memoized setpoint supplier that caches by robot pose. */
  private final Supplier<ShooterSetpoint> m_setpointSupplier;

  // ==================== CLIMB LANE CHOOSER ====================
  private final LoggedDashboardChooser<String> m_climbLaneChooser =
      new LoggedDashboardChooser<>("Climb Lane");

  public RobotContainer() {
    // ==================== IO MODE SWITCHING ====================
    switch (Constants.currentMode) {
      case REAL:
        indexer = new IndexerSubsystem(new IndexerIOTalonFX());
        shooter = new ShooterSubsystem(new ShooterIOTalonFX());
        shooterPivot = new ShooterPivotSubsystem(
            new ShooterPivotIOTalonFX(), () -> drivetrain.getState().Pose);
        intake = new IntakeWheelsSubsystem(new IntakeWheelsIOTalonFX());
        pivot = new PivotSubsystem(new PivotIOTalonFX());
        climber = new ClimberSubsystem(new ClimberIOTalonFX());
        break;

      case SIM:
        // Sim IOs - first-order physics models for each mechanism
        indexer = new IndexerSubsystem(new frc.robot.subsystems.indexer.IndexerIOSim());
        shooter = new ShooterSubsystem(new frc.robot.subsystems.shooter.ShooterIOSim());
        shooterPivot = new ShooterPivotSubsystem(
            new frc.robot.subsystems.shooter.ShooterPivotIOSim(), () -> drivetrain.getState().Pose);
        intake = new IntakeWheelsSubsystem(new frc.robot.subsystems.intake.IntakeWheelsIOSim());
        pivot = new PivotSubsystem(new frc.robot.subsystems.intake.PivotIOSim());
        climber = new ClimberSubsystem(new frc.robot.subsystems.climber.ClimberIOSim());
        break;

      case REPLAY:
        // Replay - same as SIM (Logger will inject real data from the log file)
        indexer = new IndexerSubsystem(new NoOpIndexerIO());
        shooter = new ShooterSubsystem(new NoOpShooterIO());
        shooterPivot =
            new ShooterPivotSubsystem(new NoOpShooterPivotIO(), () -> drivetrain.getState().Pose);
        intake = new IntakeWheelsSubsystem(new NoOpIntakeWheelsIO());
        pivot = new PivotSubsystem(new NoOpPivotIO());
        climber = new ClimberSubsystem(new NoOpClimberIO());
        break;

      default:
        indexer = new IndexerSubsystem(new NoOpIndexerIO());
        shooter = new ShooterSubsystem(new NoOpShooterIO());
        shooterPivot =
            new ShooterPivotSubsystem(new NoOpShooterPivotIO(), () -> drivetrain.getState().Pose);
        intake = new IntakeWheelsSubsystem(new NoOpIntakeWheelsIO());
        pivot = new PivotSubsystem(new NoOpPivotIO());
        climber = new ClimberSubsystem(new NoOpClimberIO());
        break;
    }

    // Create vision subsystem using Mercer Island architecture
    // VisionConsumer = drivetrain::addVisionMeasurement, with per-camera
    // VisionIOLimelight instances
    vision = new Vision(
        drivetrain::addVisionMeasurement,
        new VisionIOLimelight("limelight-left", () -> drivetrain.getState().Pose.getRotation()),
        new VisionIOLimelight(
            "limelight-right", () -> drivetrain.getState().Pose.getRotation()));

    drivetrain.registerTelemetry(m_telemetry::telemeterize);

    // Create the memoized \[]
    // setpoint supplier (caches by pose X/Y/theta)
    m_setpointSupplier = ShooterMath.createSetpointSupplier(() -> drivetrain.getState().Pose);

    // Register controllers with state machine for haptic feedback
    m_stateMachine.registerControllers(m_driverController, m_operatorController);

    // ==================== SMART SHOOT ====================
    HubShiftTracker hubTracker = HubShiftTracker.getInstance();
    smartShootController = new SmartShootController(hubTracker, () -> drivetrain.getState().Pose);

    // ==================== DASHBOARD PUBLISHER ====================
    dashboardPublisher = new DashboardPublisher(m_stateMachine, hubTracker, smartShootController);

    // Wire subsystem state into the state machine so isReadyToFire() works
    m_stateMachine.registerShooterSuppliers(shooter::isReady, () -> {
      // Heading is "aligned" when robot faces the hub within tolerance
      double targetHeading =
          ShooterMath.getHeadingToHub(drivetrain.getState().Pose).in(Radians);
      double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
      Angle error = Radians.of(
          Math.abs(MathUtil.inputModulus(currentHeading - targetHeading, -Math.PI, Math.PI)));
      return error.lt(Constants.ShooterConstants.HEADING_TOLERANCE);
    });

    // ==================== SUPERSTRUCTURE ====================
    superstructure = new Superstructure(
        shooter,
        shooterPivot,
        indexer,
        intake,
        pivot,
        climber,
        m_stateMachine,
        m_setpointSupplier,
        () -> {
          double targetHeading =
              ShooterMath.getHeadingToHub(drivetrain.getState().Pose).in(Radians);
          double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
          Angle error = Radians.of(
              Math.abs(MathUtil.inputModulus(currentHeading - targetHeading, -Math.PI, Math.PI)));
          return error.lt(Constants.ShooterConstants.HEADING_TOLERANCE);
        },
        smartShootController,
        () -> LaunchCalculator.getInstance().getParameters(),
        () -> drivetrain.isAtLaunchHeadingGoal());

    // Initialize the pathfinding system
    initializePathfinding();

    choreoAutoFactory = new AutoFactory(
        () -> drivetrain.getState().Pose,
        drivetrain::resetPose,
        drivetrain::followTrajectory,
        false,
        drivetrain);

    // ==================== REGISTER NAMED COMMANDS ====================
    // AutoCommands provides factory methods used by both PathPlanner and
    // Choreo.
    // Must be registered BEFORE any PathPlanner autos/paths are created.
    autoCommands = new AutoCommands(
        superstructure,
        intake,
        pivot,
        indexer,
        shooter,
        shooterPivot,
        drivetrain,
        vision,
        climber,
        m_setpointSupplier);
    autoCommands.registerPathPlannerCommands();
    autoCommands.registerChoreoBindings(choreoAutoFactory);

    // ==================== BUILD AUTO CHOOSER ====================
    autos = new Autos(choreoAutoFactory, autoCommands);

    // ==================== CLIMB LANE CHOOSER ====================
    m_climbLaneChooser.addDefaultOption("Center", "CENTER");
    m_climbLaneChooser.addOption("Left", "LEFT");
    m_climbLaneChooser.addOption("Right", "RIGHT");

    // Configure button bindings
    configureBindings();
  }

  /**
   * Initialize the pathfinding system. This loads the navgrid and starts the background AD*
   * planning thread.
   */
  private void initializePathfinding() {
    Logger.recordOutput("Events/RobotContainer/Last", "Initializing pathfinding system");
    Pathfinding.ensureInitialized();
    Logger.recordOutput("Events/RobotContainer/Last", "Pathfinding system ready");
  }

  /**
   * Configure button bindings for driver and operator controllers. Delegates to dedicated binding
   * classes for clean separation.
   */
  private void configureBindings() {

    // Build climb pathfind commands: two-phase approach to avoid routing through
    // the climb structure.
    // Phase 1: Pathfind to an approach waypoint in the open field (AD* avoids
    // obstacles).
    // Phase 2: Drive straight from the approach point into the climb structure to
    // the final
    // pose.
    Supplier<Command> climbApproachCommandFactory = () -> drivetrain.pathfindToPose(() -> {
      ClimbLane lane = resolveClimbLane();
      boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
      return ClimbConstants.getClimbApproachPose(lane, isRed);
    });
    Supplier<Command> climbEntryCommandFactory = () -> drivetrain.pathfindToPose(() -> {
      ClimbLane lane = resolveClimbLane();
      boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
      return ClimbConstants.getClimbPose(lane, isRed);
    });

    DriverControls.configure(
        m_driverController, drivetrain, vision, superstructure, m_stateMachine, m_setpointSupplier);
    OperatorControls.configure(
        m_operatorController,
        superstructure,
        intake,
        shooterPivot,
        climber,
        m_stateMachine,
        m_setpointSupplier,
        () -> ShooterMath.getDistanceToHub(drivetrain.getState().Pose),
        climbApproachCommandFactory,
        climbEntryCommandFactory,
        drivetrain);
    TestingBindings.configure(
        m_testController, drivetrain, intake, pivot, indexer, shooter, vision);
  }

  /** Get the driver controller for use in commands/subsystems */
  public CommandXboxController getDriverController() {
    return m_driverController;
  }

  /** Get the operator controller for use in commands/subsystems */
  public CommandXboxController getOperatorController() {
    return m_operatorController;
  }

  /** Get the state machine instance */
  public RobotStateMachine getStateMachine() {
    return m_stateMachine;
  }

  public IntakeWheelsSubsystem getIntake() {
    return intake;
  }

  public PivotSubsystem getPivot() {
    return pivot;
  }

  public IndexerSubsystem getIndexer() {
    return indexer;
  }

  public ClimberSubsystem getClimber() {
    return climber;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  public DashboardPublisher getDashboardPublisher() {
    return dashboardPublisher;
  }

  /**
   * Returns the autonomous command selected via SmartDashboard.
   *
   * @see Autos#getSelected()
   */
  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
  /** Resolve the currently selected climb lane from the dashboard chooser. */
  private ClimbLane resolveClimbLane() {
    String selected = m_climbLaneChooser.get();
    if (selected != null) {
      try {
        return ClimbLane.valueOf(selected);
      } catch (IllegalArgumentException ignored) {
        // fall through
      }
    }
    return ClimbLane.CENTER;
  }
}
