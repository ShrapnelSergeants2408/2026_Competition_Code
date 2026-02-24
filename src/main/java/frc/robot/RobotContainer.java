// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Climber subsystem
  private final Climber climber = new Climber();

  // Vision must be constructed before DriveTrain (passed into its constructor).
  // DriveTrain must be constructed before Shooter (both are injected into Shooter).
  private final Vision vision = new Vision();
  private final DriveTrain drivetrain = new DriveTrain(vision);

  // Shooter subsystem (unified — owns shooter wheel, feeder, and intake).
  // Receives vision and drivetrain for live distance resolution and zone enforcement.
  private final Shooter shooter = new Shooter(vision, drivetrain);

  // Driver camera (USB webcam) — may be null if no camera is present at startup.
  private final UsbCamera driverCamera;

  // Driver controller — drive motions only (port 0)
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  // Operator controller – all intake and shooting operations (port 1)
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Autonomous chooser
  private final Command doNothingAuto =
      Commands.waitUntil(DriverStation::isTeleopEnabled).withName("Do Nothing");
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize driver camera. Guard against missing hardware (simulation,
    // camera unplugged) so a missing USB camera does not crash the constructor.
    UsbCamera tempCamera = null;
    try {
      tempCamera = CameraServer.startAutomaticCapture(VisionConstants.DRIVER_CAMERA_NAME, 0);
      tempCamera.setResolution(320, 240);
      tempCamera.setFPS(30);
    } catch (Exception e) {
      DriverStation.reportWarning(
          "Driver camera not found on USB port 0 — running without driver feed: " + e.getMessage(),
          false);
    }
    driverCamera = tempCamera;

    // Build auto chooser – must run after DriveTrain constructor calls AutoBuilder.configure()
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", doNothingAuto);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureDefaultCommands();
    configureBindings();
  }

  /**
   * Set up default commands for subsystems. Each subsystem can have one default command
   * that runs whenever no other command is using that subsystem.
   */
  private void configureDefaultCommands() {
    // Single unified drive command — DriveTrain internally handles all 4 mode combinations
    // (field-oriented tank, field-oriented arcade, robot-relative tank, robot-relative arcade).
    // Defaults to field-oriented tank.
    drivetrain.setDefaultCommand(
        drivetrain.teleopDriveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getRightY()
        )
    );
  }

  /**
   * Configure button-to-command bindings.
   *
   * Driver (port 0) — drive motions only:
   *   Back  = toggle Tank / Arcade drive mode
   *   Start = toggle Field-Oriented / Robot-Relative orientation
   *
   * Operator (port 1) — intake and shooting:
   *   Y        = toggle shooter spin-up to distance-resolved RPM (zone-locked outside offensive zone)
   *   X        = shoot: spin up + auto-feed when at speed (hold; zone-locked)
   *   RB       = feed if shooter is at speed (hold)
   *   B        = stop all – immediately stops shooter and feeder
   *   A        = clear distance preset (return to automatic distance resolution)
   *   LB       = toggle intake (draw ball in; dashboard indicator)
   *   LT       = toggle eject (reverse feeder to expel ball)
   *   POV Up   = stage distance preset 15.0 ft (no spin-up)
   *   POV Down = stage distance preset 10.0 ft
   *   POV Left = stage distance preset  7.5 ft
   *   POV Right= stage distance preset 12.5 ft
   *
   * Zone enforcement: shooter spin-up is blocked outside the offensive zone in
   * teleop/auto. Test mode bypasses the zone lock for bench and pit testing.
   */
  private void configureBindings() {
    // ── Driver ────────────────────────────────────────────────────────────────
    m_driverController.back().onTrue(
        Commands.runOnce(() -> drivetrain.toggleDriveMode(), drivetrain)
    );
    m_driverController.start().onTrue(
        Commands.runOnce(() -> drivetrain.toggleOrientationMode(), drivetrain)
    );

    // ── Operator ──────────────────────────────────────────────────────────────

    // Y: toggle shooter spin-up to distance-resolved RPM — no feeding, just spin.
    // First press starts spinning; second press stops. Zone-locked outside offensive zone.
    m_operatorController.y().toggleOnTrue(shooter.spinUpCommand());

    // X: full shoot sequence — spin up then auto-feed when at speed. Hold to shoot.
    m_operatorController.x().whileTrue(shooter.shootCommand());

    // RB: feed only if shooter is already at target speed (hold).
    m_operatorController.rightBumper().whileTrue(shooter.feedCommand());

    // B: emergency stop – cancels all shooter/feeder activity immediately.
    m_operatorController.b().onTrue(shooter.stopAllCommand());

    // A: clear staged distance preset, returning to automatic distance resolution.
    m_operatorController.a().onTrue(
        Commands.runOnce(shooter::clearDistancePreset)
    );

    // LB: toggle intake – first press draws ball in, second press stops.
    // "Intake Active" boolean on dashboard shows current state.
    m_operatorController.leftBumper().toggleOnTrue(shooter.intakeCommand());

    // LT: toggle eject — first press reverses feeder to expel ball, second press stops.
    m_operatorController.leftTrigger().toggleOnTrue(shooter.ejectCommand());

    // POV: stage a distance preset without spinning the motor.
    // Acts as priority-3 fallback when vision and odometry are unavailable.
    m_operatorController.povUp().onTrue(
        Commands.runOnce(() -> shooter.setDistancePreset(15.0))
    );
    m_operatorController.povDown().onTrue(
        Commands.runOnce(() -> shooter.setDistancePreset(10.0))
    );
    m_operatorController.povLeft().onTrue(
        Commands.runOnce(() -> shooter.setDistancePreset(7.5))
    );
    m_operatorController.povRight().onTrue(
        Commands.runOnce(() -> shooter.setDistancePreset(12.5))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selectedAuto = autoChooser.getSelected();
    if (selectedAuto == null) {
      selectedAuto = doNothingAuto;
    }
    drivetrain.initializePose(selectedAuto); // seed pose from vision or auto path before running
    return selectedAuto;
  }
}
