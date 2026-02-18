package frc.robot;

import static frc.robot.Constants.ShooterConstants.NOMINAL_VOLTAGE;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
  // TODO: cleanup - Shooter was replaced by ShooterSubsystem in shooter branch; remove one
  private final Shooter shooter = new Shooter(NOMINAL_VOLTAGE);

  // Drivetrain subsystem
  private final DriveTrain drivetrain = new DriveTrain();

  // Vision subsystem
  private final VisionSubsystem vision = new VisionSubsystem();

  // Driver camera (USB webcam)
  private final UsbCamera driverCamera;

  // TODO: cleanup - ShooterSubsystem is the shooter branch replacement for Shooter; remove one
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);





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

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings.
   */

   
  private void configureBindings() {

    // Xbox buttons for shooter
    

    // X button → spin shooter to fixed 2950 RPM (open-loop)
    m_driverController.x()
        .whileTrue(Commands.run(() -> m_shooterSubsystem.setTargetRPM(2000.0), m_shooterSubsystem))
        .onFalse(Commands.run(() -> m_shooterSubsystem.stop(), m_shooterSubsystem));

    // Y button → stop shooter immediately when pressed
    m_driverController.y()
        .onTrue(Commands.run(() -> m_shooterSubsystem.stop(), m_shooterSubsystem));

    // Right trigger: run feeder while held, stop when released
   m_driverController.rightTrigger()
    .whileTrue(
        m_shooterSubsystem.shooterCommand()  // uses canShoot() internally
    )
    .onFalse(
        Commands.run(() -> m_shooterSubsystem.stopFeeder(), m_shooterSubsystem)
    );


    m_driverController.b()
    .onTrue(Commands.run(() -> {
        m_shooterSubsystem.stop();       // stop shooter
        m_shooterSubsystem.stopFeeder(); // stop feeder
    }, m_shooterSubsystem));
      
    
    
     new Trigger(() -> m_driverController.getHID().getPOV() == 0)
    .onTrue(Commands.run(() -> m_shooterSubsystem.setTargetDistance(15.0), m_shooterSubsystem));

    m_driverController.povDown()
        .onTrue(Commands.run(() -> m_shooterSubsystem.setTargetDistance(10.0), m_shooterSubsystem));

    m_driverController.povLeft()
        .onTrue(Commands.run(() -> m_shooterSubsystem.setTargetDistance(7.5), m_shooterSubsystem));

    m_driverController.povRight()
        .onTrue(Commands.run(() -> m_shooterSubsystem.setTargetDistance(12.5), m_shooterSubsystem));
}
 
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /* 
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
  */

