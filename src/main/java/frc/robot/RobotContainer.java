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
import frc.robot.subsystems.Feeder;
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

  // Shooter subsystem (flywheel wheel only — distance resolution and zone enforcement).
  // Feeder subsystem (intake roller + trigger/hopper — ball path).
  // Separated so that intake/eject can run concurrently with the shooter wheel spinning.
  private final Shooter shooter = new Shooter(vision, drivetrain);
  private final Feeder feeder = new Feeder();

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

  // ── Compound shooter commands ──────────────────────────────────────────────
  //
  // These commands coordinate both the Shooter (flywheel) and Feeder (ball path)
  // subsystems. They are defined here because both subsystem instances are needed.

  /**
   * Full shoot sequence: spin up the flywheel, then auto-feed when at speed.
   * Requires both Shooter and Feeder subsystems.
   * Uses kCancelIncoming so that accidental LB/LT presses during an active shot
   * are rejected rather than aborting the sequence. The B-button emergency stop
   * (stopAllCommand) bypasses this by having no requirements and canceling directly.
   */
  private Command shootCommand() {
    return Commands.run(() -> {
      if (!shooter.canSpinShooter()) {
        shooter.stopShooter();
        feeder.stopAll();
        return;
      }
      shooter.resolveDistanceAndSpin();
      if (shooter.canShoot(feeder.hasBall())) {
        feeder.startFeed();
      }
    }, shooter, feeder)
    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    .finallyDo(interrupted -> {
      shooter.stopShooter();
      feeder.stopAll();
    });
  }

  /**
   * Feed only — runs the feeder in feed direction when the shooter is at target speed
   * AND the robot is in the offensive zone. Requires only Feeder, so it runs
   * concurrently with spinUpCommand (Y button).
   * Intended workflow: press Y to pre-spin, then hold RT to feed when ready.
   */
  private Command feedCommand() {
    return Commands.run(() -> {
      //if (shooter.canSpinShooter() && shooter.canShoot(feeder.hasBall())) {
      if (shooter.canSpinShooter()) {
        feeder.startFeed();
      } else {
        feeder.stopAll();
      }
    }, feeder)
    .finallyDo(interrupted -> feeder.stopAll());
  }

  /**
   * Emergency stop — immediately halts shooter wheel and feeder motors.
   * Has NO subsystem requirements so it is always schedulable regardless of what
   * is running, including commands with kCancelIncoming. It directly cancels any
   * active commands on both subsystems, then stops the motors immediately.
   */
  private Command stopAllCommand() {
    return Commands.runOnce(() -> {
      // Force-cancel active commands on both subsystems (works even with kCancelIncoming).
      Command sc = shooter.getCurrentCommand();
      if (sc != null) sc.cancel();
      Command fc = feeder.getCurrentCommand();
      if (fc != null) fc.cancel();
      // Stop motors directly (immediate effect, does not wait for scheduler).
      shooter.stopShooter();
      feeder.stopAll();
    });
  }

  /**
   * Configure button-to-command bindings.
   *
   * Driver (port 0) — drive motions only:
   *   Back  = toggle Tank / Arcade drive mode
   *   Start = toggle Field-Oriented / Robot-Relative orientation
   *
   * Operator (port 1) — intake and shooting:
   *   Y        = toggle shooter spin-up to distance-resolved RPM / coast stop
   *   X        = full auto-shoot: spin up + auto-feed when at speed (hold; zone-locked)
   *   RT       = feed while held (only runs when at speed AND in offensive zone)
   *   B        = stop all – immediately stops shooter and feeder (always works)
   *   A        = clear distance preset (return to automatic distance resolution)
   *   LB       = toggle intake (clockwise — both intake & trigger at 100%)
   *   RB       = toggle removal (counterclockwise — both intake & trigger at 100%)
   *   POV 0°   = stage distance preset  7.5 ft (test mode only)
   *   POV 90°  = stage distance preset 10.0 ft (test mode only)
   *   POV 180° = stage distance preset 12.5 ft (test mode only)
   *   POV 270° = stage distance preset 15.0 ft (test mode only)
   *
   * Concurrent operation:
   *   Y (spinUpCommand, requires Shooter) and LT/LB/RT (require only Feeder) do NOT
   *   conflict — they run on different subsystems simultaneously.
   *
   * No interference with drivetrain:
   *   Shooter requires the Shooter subsystem, Feeder requires the Feeder subsystem,
   *   and DriveTrain requires its own subsystem. They are fully independent.
   *
   * Zone enforcement: shooter spin-up and feed are blocked outside the offensive zone
   * in teleop/auto. Test mode bypasses the zone lock for bench and pit testing.
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

    // Y: toggle shooter spin-up to distance-resolved RPM — first press spins up,
    // second press coasts to a stop. Requires only Shooter subsystem.
    m_operatorController.y().toggleOnTrue(shooter.spinUpCommand());

    // X: full auto-shoot — spin up then auto-feed when at speed and in zone. Hold to shoot.
    // Requires both Shooter and Feeder; kCancelIncoming guards against accidental
    // LT/LB presses aborting the sequence. B-button stop still works (no requirements).
    m_operatorController.x().whileTrue(shootCommand());

    // RT: manual feed — hold to run feeder into shooter.
    // Only activates when shooter is at target speed AND robot is in offensive zone.
    // Requires only Feeder, so it runs concurrently with Y spin-up.
    m_operatorController.rightTrigger().whileTrue(feedCommand());

    // B: emergency stop – cancels all shooter/feeder activity immediately.
    // No subsystem requirements — always schedulable, even mid-shoot.
    m_operatorController.b().onTrue(stopAllCommand());

    // A: clear staged distance preset, returning to automatic distance resolution.
    /* 
    m_operatorController.a().onTrue(
        Commands.runOnce(shooter::clearDistancePreset)
    );
    */

    m_operatorController.a().toggleOnTrue(feeder.shootCommand());
    //inhale
    // LB: toggle intake — clockwise, both intake & trigger motors at 100%.
    // Requires only Feeder — runs concurrently with Y spin-up.
    m_operatorController.leftBumper().toggleOnTrue(feeder.intakeCommand());

    //exhale
    // RB: toggle removal — counterclockwise, both intake & trigger motors at 100%.
    // Requires only Feeder — runs concurrently with Y spin-up.
    m_operatorController.rightBumper().toggleOnTrue(feeder.ejectCommand());

    // POV: stage a distance preset (test mode only).
    // 0°=7.5ft, 90°=10ft, 180°=12.5ft, 270°=15ft
    m_operatorController.povUp().onTrue(
        Commands.runOnce(() -> { if (DriverStation.isTest()) shooter.setDistancePreset(7.5); })
    );
    m_operatorController.povRight().onTrue(
        Commands.runOnce(() -> { if (DriverStation.isTest()) shooter.setDistancePreset(10.0); })
    );
    m_operatorController.povDown().onTrue(
        Commands.runOnce(() -> { if (DriverStation.isTest()) shooter.setDistancePreset(12.5); })
    );
    m_operatorController.povLeft().onTrue(
        Commands.runOnce(() -> { if (DriverStation.isTest()) shooter.setDistancePreset(15.0); })
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
