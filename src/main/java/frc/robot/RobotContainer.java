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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveTrain.OrientationMode;

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

  // Vision subsystem
  private final VisionSubsystem vision = new VisionSubsystem();

  // Drivetrain subsystem
  private final DriveTrain drivetrain = new DriveTrain(vision);

  // Driver camera (USB webcam)
  private final UsbCamera driverCamera;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  // Add Autonomous chooser
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

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    /*
     *new JoystickButton(m_driverController, 8) //start button = 8
        .toggleOnTrue(drivetrain.teleopArcadeCommand())
        .toggleOnFalse(teleopTankCommand());
     */
    
      drivetrain.setDefaultCommand(
        new RunCommand(() ->
        drivetrain.teleopArcadeCommand(
          -m_driverController.getLeftY(),
          -m_driverController.getRightY()),
        drivetrain));

      drivetrain.setDefaultCommand(
        new RunCommand(() ->
        drivetrain.teleopTankCommand(
          -m_driverController.getLeftY(),
          -m_driverController.getRightY()),
        drivetrain));
        
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
