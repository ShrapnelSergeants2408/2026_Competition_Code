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

  // Shooter subsystem (unified — owns shooter wheel, feeder, and intake)
  private final Shooter shooter = new Shooter();

  // Vision subsystem (must be constructed before DriveTrain)
  private final Vision vision = new Vision();

  // Drivetrain subsystem
  private final DriveTrain drivetrain = new DriveTrain(vision);

  // Driver camera (USB webcam) — may be null if no camera is present at startup.
  private final UsbCamera driverCamera;

  // Driver controller — drive motions only (port 0)
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  // Operator controller — all intake and shooting operations (port 1)
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Autonomous chooser
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

    // Build auto chooser — must run after DriveTrain constructor calls AutoBuilder.configure()
    autoChooser = AutoBuilder.buildAutoChooser();
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
   * Operator (port 1) — all intake and shooting operations:
   *   X        = shoot (spin up + feed when ready, hold)
   *   B        = stop all (shooter + feeder)
   *   Y        = manual feed (bypasses speed gate, hold)
   *   RT       = feed-when-ready (hold — requires shooter at speed)
   *   LT       = intake (draw ball in, hold)
   *   LB       = eject (reverse feeder, hold)
   *   POV Up   = distance preset 15 ft
   *   POV Down = distance preset 10 ft
   *   POV Left = distance preset  7.5 ft
   *   POV Right= distance preset 12.5 ft
   */
  private void configureBindings() {
    // ── Driver ────────────────────────────────────────────────────────────────
    // Back (select): toggle Tank <-> Arcade
    m_driverController.back().onTrue(
        Commands.runOnce(() -> drivetrain.toggleDriveMode(), drivetrain)
    );
    // Start (menu): toggle Field-Oriented <-> Robot-Relative
    m_driverController.start().onTrue(
        Commands.runOnce(() -> drivetrain.toggleOrientationMode(), drivetrain)
    );

    // ── Operator ──────────────────────────────────────────────────────────────
    // X: spin up to target RPM and feed automatically when ready
    m_operatorController.x().whileTrue(shooter.shootCommand());

    // B: immediately stop shooter and feeder
    m_operatorController.b().onTrue(shooter.stopAllCommand());

    // Y: manual feed — bypasses speed gate, use with caution
    m_operatorController.y().whileTrue(shooter.manualFeedCommand());

    // RT: feed-when-ready — feeder runs only while shooter is at speed
    m_operatorController.rightTrigger().whileTrue(shooter.feedCommand());

    // LT: intake — draw ball in
    m_operatorController.leftTrigger().whileTrue(shooter.intakeCommand());

    // LB: eject — reverse feeder to clear ball
    m_operatorController.leftBumper().whileTrue(shooter.ejectCommand());

    // POV: distance presets — stages the RPM target without spinning the motor
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
    drivetrain.initializePose(selectedAuto); // seed pose from vision or auto path before running
    return selectedAuto;
  }
}
