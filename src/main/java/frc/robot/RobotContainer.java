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
//import frc.robot.subsystems.ExampleSubsystem;

public class RobotContainer {

    // Climber subsystem
    private final Climber climber = new Climber();

    // TODO: cleanup - both 'intake' (main) and 'm_intake' (Intake branch) reference the same subsystem; remove one
    private final Intake intake = new Intake();

    // TODO: cleanup - Shooter (main) and ShooterSubsystem (Intake branch) are duplicates; remove one
    private final Shooter shooter = new Shooter(NOMINAL_VOLTAGE);

    // Drivetrain subsystem
    private final DriveTrain drivetrain = new DriveTrain();

    // Vision subsystem
    private final VisionSubsystem vision = new VisionSubsystem();

    // Driver camera (USB webcam)
    private final UsbCamera driverCamera;

    // TODO: cleanup - ShooterSubsystem is the Intake branch's replacement for Shooter; remove one
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

    // TODO: cleanup - m_intake duplicates 'intake' above; remove one
    private final Intake m_intake = new Intake();

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        // Initialize driver camera (USB webcam on front)
        driverCamera = CameraServer.startAutomaticCapture(
            VisionConstants.DRIVER_CAMERA_NAME,
            0  // USB port 0 (TODO: adjust if needed)
        );
        // TODO: Configure resolution and FPS for low latency
        driverCamera.setResolution(320, 240);  // Low res for speed
        driverCamera.setFPS(30);

        configureBindings();
    }

    private void configureBindings() {
        // --- Shooter bindings ---
        m_driverController.x()
            .whileTrue(Commands.run(() -> m_shooterSubsystem.setTargetRPM(2000.0), m_shooterSubsystem))
            .onFalse(Commands.run(() -> m_shooterSubsystem.stop(), m_shooterSubsystem));

        // Right trigger -> feed balls only if shooter is at target speed
        m_driverController.rightTrigger()
    .whileTrue(
        Commands.run(() -> {
            // Update RPM from AprilTag vision distance
            m_shooterSubsystem.updateFromVision(m_visionSubsystem.getShootingDistanceFeet());

            // Feed only when ready
            if (m_shooterSubsystem.canShoot()) {
                m_shooterSubsystem.startFeeder();
            } else {
                m_shooterSubsystem.stopFeeder();
            }
        }, m_shooterSubsystem)
    )
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

        //  Intake bindings 
        // Left trigger -> run intake forward while held
        m_driverController.leftTrigger()
            .whileTrue(m_intake.intakeCommand());

        // Left bumper -> run intake backward (eject) while held
        m_driverController.leftBumper()
            .whileTrue(m_intake.ejectCommand());
    }
}
