// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ShooterConstants.NOMINAL_VOLTAGE;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
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
import frc.robot.subsystems.Intake;
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

  // Intake subsystem
  private final Intake intake = new Intake();

  // Shooter subsystem
  private final Shooter shooter = new Shooter(NOMINAL_VOLTAGE);

  // Vision subsystem (must be constructed before DriveTrain)
  private final Vision vision = new Vision();

  // Drivetrain subsystem
  private final DriveTrain drivetrain = new DriveTrain(vision);

  // Driver camera (USB webcam)
  private final UsbCamera driverCamera;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Autonomous chooser
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize driver camera (USB webcam on front)
    driverCamera = CameraServer.startAutomaticCapture(
        VisionConstants.DRIVER_CAMERA_NAME,
        0  // USB port 0 (TODO: adjust if needed)
    );

    // TODO: Configure resolution and FPS for low latency
    driverCamera.setResolution(320, 240);  // Low res for speed
    driverCamera.setFPS(30);

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
   * Back button: toggle between Tank and Arcade drive modes.
   * Start button: toggle between Field-Oriented and Robot-Relative orientation.
   */
  private void configureBindings() {
    // Back button (select): toggle Tank <-> Arcade
    m_driverController.back().onTrue(
        Commands.runOnce(() -> drivetrain.toggleDriveMode(), drivetrain)
    );

    // Start button (menu): toggle Field-Oriented <-> Robot-Relative
    m_driverController.start().onTrue(
        Commands.runOnce(() -> drivetrain.toggleOrientationMode(), drivetrain)
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
