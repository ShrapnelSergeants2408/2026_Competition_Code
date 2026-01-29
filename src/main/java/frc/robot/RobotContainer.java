package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands; // <-- for run()

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Instantiate the shooter subsystem
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {

    // -------------------------------
    // Xbox buttons for shooter
    // -------------------------------

    // X button → spin shooter to fixed 2950 RPM (open-loop)
    m_driverController.x()
        .whileTrue(Commands.run(() -> m_shooterSubsystem.setTargetRPM(2950.0), m_shooterSubsystem))
        .onFalse(Commands.run(() -> m_shooterSubsystem.stop(), m_shooterSubsystem));

    // Y button → stop shooter immediately when pressed
    m_driverController.y()
        .onTrue(Commands.run(() -> m_shooterSubsystem.stop(), m_shooterSubsystem));
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
}
