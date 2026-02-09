package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

    // Instantiate Intake subsystem
    private final Intake m_intake = new Intake();

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // --- Shooter bindings ---
        m_driverController.x()
            .whileTrue(Commands.run(() -> m_shooterSubsystem.setTargetRPM(2000.0), m_shooterSubsystem))
            .onFalse(Commands.run(() -> m_shooterSubsystem.stop(), m_shooterSubsystem));

        m_driverController.y()
            .onTrue(Commands.run(() -> m_shooterSubsystem.stop(), m_shooterSubsystem));

        m_driverController.rightTrigger()
            .whileTrue(m_shooterSubsystem.shooterCommand())
            .onFalse(Commands.run(() -> m_shooterSubsystem.stopFeeder(), m_shooterSubsystem));

        m_driverController.b()
            .onTrue(Commands.run(() -> {
                m_shooterSubsystem.stop();
                m_shooterSubsystem.stopFeeder();
            }, m_shooterSubsystem));

        new Trigger(() -> m_driverController.getHID().getPOV() == 0)
            .onTrue(Commands.run(() -> m_shooterSubsystem.setTargetDistance(15.0), m_shooterSubsystem));

        m_driverController.povDown()
            .onTrue(Commands.run(() -> m_shooterSubsystem.setTargetDistance(10.0), m_shooterSubsystem));

        m_driverController.povLeft()
            .onTrue(Commands.run(() -> m_shooterSubsystem.setTargetDistance(7.5), m_shooterSubsystem));

        m_driverController.povRight()
            .onTrue(Commands.run(() -> m_shooterSubsystem.setTargetDistance(12.5), m_shooterSubsystem));

        // --- Intake binding ---
       // Left trigger → run intake forward while held
        m_driverController.leftTrigger()
            .whileTrue(m_intake.intakeCommand());

// Left bumper → run intake backward (eject) while held
m_driverController.leftBumper()
    .whileTrue(m_intake.ejectCommand());

    }
}
