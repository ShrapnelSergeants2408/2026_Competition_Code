# Hiebert Review Planning Document

## 1) Subsystem + RobotContainer commented skeleton (with inline command factories)

> Use this as a **structure template**. It is intentionally commented and partial so your team can fill in robot-specific values.

```java
// ===============================
// Drivetrain subsystem skeleton
// ===============================

// --- library imports ---
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  // ----- enums/state -----
  public enum DriveMode { ROBOT_RELATIVE, FIELD_RELATIVE }
  private DriveMode driveMode = DriveMode.ROBOT_RELATIVE;

  // ----- hardware declarations -----
  // motor controllers
  // encoders
  // gyro
  // differential drive helper

  // ----- math/model declarations -----
  // track width, wheel radius, conversion factors
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(/* trackWidthMeters */);

  // estimator (odometry + vision fusion)
  private final DifferentialDrivePoseEstimator poseEstimator =
      new DifferentialDrivePoseEstimator(
          kinematics,
          getHeading(),
          getLeftDistanceMeters(),
          getRightDistanceMeters(),
          new Pose2d()
      );

  // dashboard field widget
  private final Field2d field = new Field2d();

  public Drivetrain() {
    // configure inversion/current limits/idle modes
    // set encoder conversion factors
    // reset sensors

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // 1) update odometry/pose estimator
    poseEstimator.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());

    // 2) pull vision measurements (if available) and fuse
    // Optional<PoseEstimate> vision = visionSubsystem.getEstimatedGlobalPose(getPose());
    // vision.ifPresent(v -> poseEstimator.addVisionMeasurement(v.pose(), v.timestampSeconds()));

    // 3) push telemetry
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Drivetrain/HeadingDeg", getHeading().getDegrees());
  }

  // ----- basic methods -----
  public void arcade(double fwd, double rot) {
    // differentialDrive.arcadeDrive(fwd, rot);
  }

  public void tank(double left, double right) {
    // differentialDrive.tankDrive(left, right);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    // convert wheel speeds -> voltage/percent output
    // set left/right motor outputs
  }

  public void driveFieldRelative(double xSpeed, double rotSpeed) {
    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed,
        0.0,
        rotSpeed,
        getHeading()
    );
    driveRobotRelative(robotRelative);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
  }

  public Rotation2d getHeading() {
    // return gyro angle as Rotation2d with your sign convention
    return new Rotation2d();
  }

  public double getLeftDistanceMeters() { return 0.0; }
  public double getRightDistanceMeters() { return 0.0; }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() { return new DifferentialDriveWheelSpeeds(); }

  // ----- command factories (inline command creation) -----
  public Command teleopArcadeCommand(
      java.util.function.DoubleSupplier fwd,
      java.util.function.DoubleSupplier rot
  ) {
    return Commands.run(() -> arcade(fwd.getAsDouble(), rot.getAsDouble()), this);
  }

  public Command teleopTankCommand(
      java.util.function.DoubleSupplier left,
      java.util.function.DoubleSupplier right
  ) {
    return Commands.run(() -> tank(left.getAsDouble(), right.getAsDouble()), this);
  }

  public Command resetHeadingCommand() {
    return Commands.runOnce(() -> {
      // gyro.reset();
    }, this);
  }

  public Command setDriveModeCommand(DriveMode mode) {
    return Commands.runOnce(() -> driveMode = mode, this);
  }
}
```

```java
// =================================
// RobotContainer skeleton
// =================================

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // ----- controller declarations -----
  private final CommandXboxController driver = new CommandXboxController(0);

  // ----- subsystem instantiation -----
  private final VisionSubsystem vision = new VisionSubsystem();
  private final Drivetrain drivetrain = new Drivetrain(/* optionally inject vision */);

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
    configureAutos();
  }

  private void configureDefaultCommands() {
    // one default command per subsystem
    drivetrain.setDefaultCommand(
      drivetrain.teleopTankCommand(
        () -> -driver.getLeftY(),
        () -> -driver.getRightY()
      )
    );
  }

  private void configureBindings() {
    // driver.y().onTrue(drivetrain.resetHeadingCommand());
    // driver.a().onTrue(drivetrain.setDriveModeCommand(DriveMode.FIELD_RELATIVE));
    // driver.b().onTrue(drivetrain.setDriveModeCommand(DriveMode.ROBOT_RELATIVE));
  }

  private void configureAutos() {
    // PathPlanner NamedCommands.registerCommand(...)
    // AutoBuilder config usually done in drivetrain constructor/init method
  }

  public Command getAutonomousCommand() {
    // return AutoBuilder.buildAuto("Your Auto Name");
    return null;
  }
}
```

---

## 2) Basic sequence of events in an FRC program (example: both joysticks forward for tank drive)

### High-level timeline
1. `Robot` starts, `RobotContainer` is constructed.
2. `RobotContainer` creates subsystems/controllers and binds commands.
3. `RobotContainer` sets drivetrain default command (tank/arcade).
4. During teleop, scheduler runs every ~20 ms.

### Detailed logic flow for tank drive joystick input
1. **Driver pushes joysticks forward** on controller.
2. **Default drivetrain command executes** each scheduler tick.
3. That command reads `leftY/rightY` suppliers from `CommandXboxController`.
4. Command calls drivetrain method (e.g., `tank(left, right)`).
5. Drivetrain method sends outputs to motor controllers.
6. Robot physically moves.
7. Independent of command, drivetrain `periodic()` runs every tick:
   - updates odometry/pose estimator from encoders + gyro,
   - fuses vision if available,
   - publishes telemetry (`Field2d`, dashboard).

### Which class owns what?
- **RobotContainer**: wiring and bindings.
- **Subsystem methods**: hardware actuation + state updates.
- **Command**: “when/how to call methods continuously or once.”
- **Subsystem `periodic()`**: sensor fusion + telemetry + maintenance updates.

---

## 3) What declarations do you need?

### Core declarations
- `Drivetrain drivetrain;`
- `VisionSubsystem vision;`
- `CommandXboxController driver;`
- Motor controllers, encoders, gyro in drivetrain.
- `DifferentialDriveKinematics` and `DifferentialDrivePoseEstimator`.
- `Field2d` for on-field pose display.

### Constants declarations (recommended groups)
- CAN IDs / PWM ports.
- Encoder conversion factors (rotations -> meters).
- Track width meters, wheel radius meters.
- Feedforward/PID values (if using velocity control).
- Camera transforms (`robotToFrontCam`, `robotToRearCam`) and camera names.
- Vision covariance/std-dev settings for near/far tags.

### PathPlanner declarations
- Robot config from GUI settings.
- AutoBuilder callbacks:
  - pose supplier,
  - pose reset,
  - chassis speed supplier,
  - robot-relative drive consumer,
  - controller (`PPLTVController` for differential),
  - alliance mirroring supplier.

### PhotonVision declarations
- `PhotonCamera frontCam`, `PhotonCamera rearCam`.
- `AprilTagFieldLayout`.
- `PhotonPoseEstimator` per camera.
- Optional structure/class to return best estimated pose + timestamp + confidence.

---

## 4) What methods do you need?

### 4a) Basic driver control (robot-oriented + field-oriented) during teleop
- `arcade(fwd, rot)`
- `tank(left, right)`
- `driveRobotRelative(ChassisSpeeds speeds)`
- `driveFieldRelative(xSpeed, rotSpeed)`
- `setDriveMode(...)`
- `resetHeading()`
- deadband/slew helpers (optional but recommended)

> Note: For differential drive, “field oriented” means converting desired field frame forward/turn into robot frame using gyro heading, then applying differential drive wheel outputs.

### 4b) Differential kinematics + odometry + SmartDashboard
- `getLeftDistanceMeters()`, `getRightDistanceMeters()`
- `getWheelSpeeds()`
- `getHeading()`
- `getPose()`
- `resetPose(Pose2d pose)`
- `updateOdometryOrEstimator()` (called from `periodic()`)
- `publishTelemetry()` (field pose, heading, wheel speeds, command outputs)

### 4c) PathPlanner integration with LTV + config
- `configureAutoBuilder()` (run once in subsystem init)
- `driveRobotRelative(ChassisSpeeds)` callback for AutoBuilder
- `getRobotRelativeSpeeds()` callback
- `followPathCommand(String autoName)` convenience method (optional)

#### LTV (what it is)
- **LTV = Linear Time-Varying controller**.
- In PathPlanner differential context, it computes control effort using a linearized model that changes with robot state/speed.
- It tends to track trajectories more robustly than very simple fixed-gain methods when dynamics vary across path segments.
- You still need good units, feedforward, and wheel-speed conversion for it to perform well.

### 4d) PhotonVision AprilTag detection use-cases

#### i) Determine initial pose
- `Optional<Pose2d> getInitialPoseFromTags()` in Vision subsystem.
- `seedInitialPoseIfConfident()` in drivetrain/robot init.

#### ii) Correct odometry / determine field position continuously
- `Optional<VisionMeasurement> getVisionMeasurement()` in Vision subsystem.
- `addVisionMeasurementToEstimator(...)` in drivetrain periodic.
- method to gate poor estimates (ambiguity, distance, few tags).

#### iii) Determine distance/alignment to field landmarks (hub/depot/etc)
- `Optional<Double> getDistanceToPose(Pose2d target)`
- `Optional<Rotation2d> getYawToPose(Pose2d target)`
- `boolean isAlignedForShot(...)`
- `Command autoAlignToTarget(...)` (optional command)

#### Should this be in a Vision subsystem?
**Yes, recommended split:**

- **VisionSubsystem owns** camera objects, tag detections, pose estimation, filtering/scoring measurement quality.
- **Drivetrain owns** pose estimator fusion and all driving/actuation.

This keeps drivetrain testable and avoids camera/network code mixed with motor code.

With 2 Pi cameras + 1 driver webcam:
- front cam estimator (tag solve)
- rear cam estimator (tag solve)
- driver webcam **not** used for pose solve (display only)
- fuse best front/rear estimate each cycle (or both if quality-gated)

---

## 5) What commands do you need?

### Driver commands
- `TeleopTankCommand` (default)
- `TeleopArcadeCommand` (if needed)
- `SetDriveModeFieldRelativeCommand`
- `SetDriveModeRobotRelativeCommand`
- `ResetHeadingCommand`

### Pose/vision utility commands
- `SeedPoseFromVisionCommand` (one-shot at start)
- `EnableVisionFusionCommand` / `DisableVisionFusionCommand` (optional)
- `AlignToTargetCommand` (turn robot to desired yaw/pose)

### Autonomous/PathPlanner commands
- Named commands for mechanisms (intake/shooter actions)
- `AutoBuilder.buildAuto("...")` from chooser
- Optional “drive to pose” command for testing

---

## 6) What are teams commonly forgetting?

1. **Unit conversions** (encoder rotations to meters, meters/sec everywhere).
2. **Consistent gyro sign convention** (CW/CCW inversion bugs).
3. **One source of truth for geometry constants** (track width mismatch between code and PathPlanner).
4. **Vision measurement gating** (don’t fuse bad/far/ambiguous tag solves).
5. **Timestamp correctness** for vision measurements.
6. **Default command ownership** (forgot to set default => drivetrain does nothing in teleop).
7. **Subsystem responsibilities** (camera logic leaking into drive actuation).
8. **Simulation/telemetry checks** before on-field testing.
9. **AutoBuilder callbacks wired to correct methods** (no recursion, no placeholders).
10. **Fail-safe behavior** when cameras disconnect.

---

## Suggested implementation order (practical)
1. Clean compile + constants + conversion factors.
2. Basic tank teleop command and default command wiring.
3. Pose estimator with encoders+gyro only.
4. Dashboard/Field2d telemetry.
5. PathPlanner AutoBuilder + test auto.
6. Add VisionSubsystem with front/rear PhotonPoseEstimators.
7. Fuse vision into drivetrain estimator with gating.
8. Add alignment/distance helper methods + commands.

