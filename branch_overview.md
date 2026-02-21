# Branch Overview: Drivetrain, Vision, and PathPlanner (Beginner-Friendly)

This document reviews the code in this branch that is related to:
- **Drivetrain**
- **Vision (PhotonVision + AprilTags)**
- **PathPlanner autonomous driving**

I will walk through each class and method like you are learning Java and robot code for the first time. For each part, I include a small code snippet (“highlight”) and then explain what it does.

---

## 1) Dependency and Version Check (2026 API alignment)

### Highlight
```gradle
plugins {
    id "java"
    id "edu.wpi.first.GradleRIO" version "2026.2.1"
}
```

### Explanation
This line sets the project to use the **WPILib 2026 GradleRIO plugin**. That is the base of your FRC robot framework, so this is correct for a 2026 project.

### Highlight
```json
// vendordeps/PathplannerLib-2026.1.2.json
"version": "2026.1.2",
"frcYear": "2026"
```
```json
// vendordeps/photonlib.json
"version": "v2026.2.2",
"frcYear": "2026"
```
```json
// vendordeps/REVLib.json
"version": "2026.0.3",
"frcYear": "2026"
```
```json
// vendordeps/Phoenix6-26.1.1.json
"version": "26.1.1",
"frcYear": "2026"
```

### Explanation
These vendordep files are how third-party robot libraries are installed. The key thing is that they all target **frcYear 2026**, which they do.

### Concern
- Both **Phoenix 5 and Phoenix 6** vendordeps are present in the repo. That is not automatically wrong, but if your code only uses one family, extra vendordeps can increase complexity and cause confusion.

---

## 2) `VisionMeasurement` record (shared data object)

File: `src/main/java/frc/robot/VisionMeasurement.java`

### Highlight
```java
public record VisionMeasurement(
    Pose2d estimatedPose,
    double timestampSeconds,
    Matrix<N3, N1> standardDeviations,
    int numTagsUsed,
    double averageDistance
) {}
```

### Explanation
A `record` in Java is a compact way to store data. Think of it like a labeled box.

This object packages one vision estimate so other code can trust and use it:
- `estimatedPose`: where the camera thinks robot is on field
- `timestampSeconds`: when measurement happened
- `standardDeviations`: how uncertain the measurement is (lower is more trusted)
- `numTagsUsed`: how many AprilTags contributed
- `averageDistance`: average tag distance

This is a good structure because it cleanly separates “vision produces data” from “drivetrain consumes data.”

---

## 3) `VisionSubsystem` (PhotonVision + measurement filtering)

File: `src/main/java/frc/robot/subsystems/VisionSubsystem.java`

### Constructor and setup

#### Highlight
```java
fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
rearCamera = new PhotonCamera(VisionConstants.REAR_CAMERA_NAME);
```

#### Explanation
- Loads the field's official AprilTag map for 2026.
- Creates two PhotonVision camera objects by name.

For beginners: camera names must exactly match what PhotonVision uses on the coprocessor.

#### Highlight
```java
frontPoseEstimator = new PhotonPoseEstimator(
    fieldLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    VisionConstants.ROBOT_TO_FRONT_CAM
);
```

#### Explanation
This creates a pose estimator per camera. It tells PhotonVision:
1. Which AprilTags exist on field (`fieldLayout`)
2. Which math strategy to use (`MULTI_TAG_PNP_ON_COPROCESSOR`)
3. Where camera is mounted on robot (`ROBOT_TO_FRONT_CAM` transform)

### Main measurement method

#### Highlight
```java
public Optional<VisionMeasurement> getBestVisionMeasurement() { ... }
```

#### Explanation
`Optional` means “maybe there is a result, maybe not.” It prevents null pointer issues.

This method:
1. Gets one possible measurement from front camera
2. Gets one from rear camera
3. Chooses best result:
   - prefers multi-tag estimates
   - if equal, prefers closer average distance
4. Returns whichever exists

This is a sensible camera arbitration strategy for beginners and practical robot code.

### Camera processing helper

#### Highlight
```java
var result = camera.getLatestResult();
updateCameraFreshness(camera, result);
if (!result.hasTargets()) return Optional.empty();
var estimatedPose = poseEstimator.update(result);
if (estimatedPose.isEmpty()) return Optional.empty();
```

#### Explanation
This is a step-by-step filter pipeline:
- Read newest frame result
- Track whether frames are still arriving (camera health)
- If no AprilTag targets, stop
- Ask PhotonPoseEstimator for robot pose
- If pose estimate failed, stop

### Quality gating

#### Highlight
```java
if (!shouldUseMeasurement(pose, result)) {
    SmartDashboard.putString("Vision/" + cameraName + "CamStatus", "Rejected (quality gate)");
    return Optional.empty();
}
```

#### Explanation
Even if PhotonVision gives a pose, this code can reject it if low quality.
That is very important, because one bad vision update can hurt robot pose estimation.

#### `shouldUseMeasurement(...)` checks
- Pose is on the field (x/y bounds)
- Single-tag ambiguity under max threshold
- Average tag distance not too far

This is good defensive engineering.

### Standard deviation tuning

#### Highlight
```java
if (numTags >= VisionConstants.MIN_TAGS_FOR_MULTI_TAG) {
    return VisionConstants.MULTI_TAG_STDDEVS;
}
if (averageDistance < 2.0) {
    return VisionConstants.SINGLE_TAG_CLOSE_STDDEVS;
} else {
    return VisionConstants.SINGLE_TAG_FAR_STDDEVS;
}
```

#### Explanation
This is how trust is assigned:
- Multi-tag: high trust
- Single-tag near: medium trust
- Single-tag far: low trust

These values are later passed into `poseEstimator.addVisionMeasurement(...)` by drivetrain.

### Utility methods
- `getBestTarget()`: returns lowest-ambiguity target from both cameras
- `getVisibleTags()`: list of unique visible tag IDs
- `getDistanceToPose(...)`, `getYawToPose(...)`, `isAlignedWithTarget(...)`: pose-to-target helper math

These are useful building blocks for commands like auto-align or shooter aiming.

### Telemetry
`updateTelemetry()` sends camera connectivity, pose, distances, and alignment booleans to SmartDashboard.

### Concerns in this class
1. **Potential high CPU/network use**: `getBestVisionMeasurement()` is called from both `VisionSubsystem.periodic()` (telemetry path) and `DriveTrain.periodic()` when vision is enabled. This may duplicate work each loop.
2. **Stale dashboard values**: distance fields are only updated when a value is present (`ifPresent`) and are not explicitly zeroed/cleared when absent, so old values could remain displayed.
3. **Hardcoded game element poses** in constants are marked TODO and may be placeholders from older games.

---

## 4) `DriveTrain` subsystem (motors, teleop drive modes, odometry, path following)

File: `src/main/java/frc/robot/subsystems/DriveTrain.java`

### Core objects and constructor

#### Highlight
```java
private final SparkMax leftMotorLead   = new SparkMax(LEFT_LEAD_CAN_ID,   MotorType.kBrushless);
...
private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
private final DifferentialDrive driver = new DifferentialDrive(leftMotorLead, rightMotorLead);
```

#### Explanation
This creates motor controllers, gyro sensor, and WPILib helper object for differential drive.

For beginners:
- “Lead” motors are directly commanded.
- “Follow” motors mirror a lead motor.
- `DifferentialDrive` computes combined motor outputs for tank/arcade commands.

#### Highlight
```java
private final DifferentialDriveOdometry poseOdometry = ...
private final DifferentialDrivePoseEstimator poseEstimator = ...
```

#### Explanation
There are two position trackers:
- `poseOdometry`: encoder + gyro only
- `poseEstimator`: encoder + gyro + optional vision corrections

Having both helps debugging.

### Motor config

#### Highlight
```java
rightLeadConfig.inverted(true);
rightLeadConfig.encoder.positionConversionFactor(WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO);
rightLeadConfig.encoder.velocityConversionFactor(WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO / 60.0);
```

#### Explanation
Important beginner concept: Spark encoder native units are rotations and RPM.
These conversion factors convert directly into **meters** and **meters per second**, which simplifies all later math.

### PathPlanner configuration

#### Highlight
```java
robotConfig = RobotConfig.fromGUISettings();
AutoBuilder.configure(
    this::getPose,
    this::resetPose,
    this::getRobotRelativeSpeeds,
    this::driveRobotRelative,
    ltvController,
    robotConfig,
    () -> { ... red alliance flip ... },
    this
);
```

#### Explanation
This wires your drivetrain into PathPlanner’s auto system.

You are giving PathPlanner function references:
- how to read robot pose
- how to reset pose
- how to read speed
- how to command speed

The alliance lambda flips paths for red side. Great feature.

### Pose initialization before autonomous

#### Highlight
```java
if (visionSubsystem != null) {
    Optional<VisionMeasurement> visionMeasurement = visionSubsystem.getBestVisionMeasurement();
    if (visionMeasurement.isPresent()) {
        initialPose = visionMeasurement.get().estimatedPose();
    }
}
if (initialPose == null && autoCommand instanceof PathPlannerAuto ppAuto) {
    initialPose = ppAuto.getStartingPose();
}
if (initialPose == null) initialPose = new Pose2d();
resetPose(initialPose);
```

#### Explanation
Before auto starts, it chooses best starting pose source:
1. live AprilTag vision now
2. auto file’s starting pose
3. origin fallback

This is beginner-friendly and robust.

### Teleop drive architecture

#### Highlight
```java
public Command teleopDriveCommand(DoubleSupplier leftYSupplier, DoubleSupplier rightYSupplier) {
    return new RunCommand(
        () -> drive(leftYSupplier.getAsDouble(), rightYSupplier.getAsDouble()),
        this
    );
}
```

#### Explanation
This command continuously calls your `drive(...)` method. `DoubleSupplier` means joystick values are read live each loop.

#### Highlight
```java
if (orientationMode == OrientationMode.FIELD_ORIENTED) {
    switch (driveMode) {
        case ARCADE -> fieldOrientedArcade(lY, rY, headingRad);
        case TANK   -> fieldOrientedTank(lY, rY, headingRad);
    }
} else {
    switch (driveMode) {
        case ARCADE -> driver.arcadeDrive(lY, rY);
        case TANK   -> driver.tankDrive(lY, rY);
    }
}
```

#### Explanation
This supports 4 states:
- robot-relative arcade
- robot-relative tank
- field-oriented arcade
- field-oriented tank

Deadband is applied first to ignore tiny stick noise.

### Path following drive call

#### Highlight
```java
ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(targetSpeeds);
wheelSpeeds.desaturate(MAX_MODULE_SPEED);
driver.tankDrive(
    wheelSpeeds.leftMetersPerSecond  / MAX_MODULE_SPEED,
    wheelSpeeds.rightMetersPerSecond / MAX_MODULE_SPEED,
    false
);
```

#### Explanation
PathPlanner gives desired chassis motion. This converts to left/right wheel speeds, limits them, then scales to percent motor commands.

### Vision fusion

#### Highlight
```java
poseEstimator.addVisionMeasurement(
    measurement.estimatedPose(),
    measurement.timestampSeconds(),
    measurement.standardDeviations()
)
```

#### Explanation
This is where camera pose updates are merged into drivetrain pose estimator.
Using timestamp + std dev is exactly the right pattern.

### Concerns in this class
1. `visionEnabled` defaults to `false`, and no binding currently enables it, so vision fusion may never run unless called elsewhere.
2. Path following output is open-loop percent (`tankDrive`), not velocity closed-loop. This can still work, but may track worse under battery sag/load.
3. `testDriveTrain()` is a placeholder dummy test.

---

## 5) `RobotContainer` (wiring commands, controls, auto chooser)

File: `src/main/java/frc/robot/RobotContainer.java`

### Key setup flow

#### Highlight
```java
private final VisionSubsystem vision = new VisionSubsystem();
private final DriveTrain drivetrain = new DriveTrain(vision);
```

#### Explanation
Vision is created first and injected into drivetrain so drivetrain can request measurements.

#### Highlight
```java
autoChooser = AutoBuilder.buildAutoChooser();
SmartDashboard.putData("Auto Chooser", autoChooser);
```

#### Explanation
This uses PathPlanner's generated auto chooser and publishes it to dashboard.

### Default drive command

#### Highlight
```java
drivetrain.setDefaultCommand(
    drivetrain.teleopDriveCommand(
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getRightY()
    )
);
```

#### Explanation
Default command means drivetrain always responds to joystick in teleop unless interrupted.
Negative sign flips joystick direction to match expected forward stick behavior.

### Button bindings

#### Highlight
```java
m_driverController.back().onTrue(
    Commands.runOnce(() -> drivetrain.toggleDriveMode(), drivetrain)
);
m_driverController.start().onTrue(
    Commands.runOnce(() -> drivetrain.toggleOrientationMode(), drivetrain)
);
```

#### Explanation
Simple “press once to toggle mode” controls.
This is clean and easy for students to understand.

### Autonomous command selection

#### Highlight
```java
Command selectedAuto = autoChooser.getSelected();
drivetrain.initializePose(selectedAuto);
return selectedAuto;
```

#### Explanation
Before returning auto command to Robot.java, drivetrain pose is seeded first.
Good order of operations.

### Concern
- No explicit UI/control is present for `drivetrain.setVisionEnabled(true)`. If you expect vision fusion in matches, this should be enabled intentionally (for example in autonomousInit and/or teleopInit).

---

## 6) `Constants` relevant to drivetrain and vision

File: `src/main/java/frc/robot/Constants.java`

### Important drivetrain constants
- CAN IDs, current limit
- gear ratio, wheel diameter/circumference
- track width

These values drive encoder conversion and kinematics. If wrong, all motion math is wrong.

### Auto constants
- `MAX_MODULE_SPEED`, acceleration, angular limits used by path following logic.

### Vision constants
- camera names
- robot-to-camera transforms
- gating limits (`MAX_TAG_DISTANCE_METERS`, `MAX_AMBIGUITY`)
- estimator covariance presets (`*_STDDEVS`)
- field element poses

### Concerns
1. Many values are marked TODO and may be estimates.
2. Field-element target poses are likely placeholders and should be updated for actual 2026 game tasks.

---

## 7) PathPlanner settings file

File: `src/main/deploy/pathplanner/settings.json`

### Highlight
```json
"holonomicMode": false,
"robotTrackwidth": 0.546,
"driveGearing": 8.46,
"maxDriveSpeed": 3.0,
"driveMotorType": "NEO"
```

### Explanation
This says robot is configured as **differential drive** (`holonomicMode: false`) and gives PathPlanner physical model data.

### Concern
The file still contains swerve module coordinate fields (`flModuleX`, etc.). They are ignored for differential mode but can confuse new students reading settings.

---

## 8) Overall architecture summary (beginner view)

1. **VisionSubsystem** reads AprilTag cameras and outputs a cleaned `VisionMeasurement`.
2. **DriveTrain** handles motor driving, tracks pose from encoders/gyro, and optionally mixes in vision pose corrections.
3. **PathPlanner AutoBuilder** plugs into drivetrain through method references.
4. **RobotContainer** wires controls and autonomous chooser.
5. **Robot** schedules whichever auto command is selected.

This is a strong architecture for a command-based robot, with good separation of responsibilities.

---

## 9) Suggested next cleanup tasks (if you want to improve reliability)

1. Enable `visionEnabled` by default or with explicit mode control.
2. Cache single-cycle vision result to avoid repeated camera processing in one loop.
3. Clear dashboard values when data is absent, not only when present.
4. Replace TODO camera transform and field pose constants with measured values.
5. Decide whether Phoenix 5 vendordep is actually needed.
6. Add real subsystem tests (remove dummy test).
