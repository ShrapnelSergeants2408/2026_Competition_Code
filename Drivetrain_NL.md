# Drivetrain Subsystem – Method Completion Guide (Natural Language)

This document describes every empty or incomplete method in the **TP-Drivetrain** branch
`DriveTrain.java` and explains in plain English what each method must do to function correctly.
It is organized by the logical group each method belongs to.

---

## Known Bugs to Fix Before Filling Methods

Before implementing the methods below, three copy-paste / typo bugs in the existing skeleton must be corrected:

| Location | Bug | Fix |
|---|---|---|
| `leftMotorFollow` declaration | Uses `LEFT_MOTOR_PORT` (lead ID) instead of follow ID | Change to `LEFT_FOLLOW_CANID` |
| `rightMotorFollow` declaration | Uses `RIGHT_MOTOR_PORT` (lead ID) instead of follow ID | Change to `RIGHT_FOLLOW_CANID` |
| `getSpeeds()` body | Calls `kinematics.toChassisSpeeds(getSpeeds())` – infinite recursion | Pass `getWheelSpeeds()` instead |
| `driveFieldRelative()` body | Calls `getPose.getRotation2d()` – missing parentheses | Change to `getPose().getRotation()` |
| `poseOdometry` field declaration | References undefined variables `gyroAngle`, `leftDistance`, etc. | Remove this redundant field; use `odometry` already declared above |
| `getAnalogGyroAngle()` | Declared as a static helper that returns `double` but has no return statement | Remove this stub entirely; the AHRS gyro is accessed via `gyro.getAngle()` |
| `getLeftDistanceMeters()` / `getRightDistanceMeters()` | Return raw encoder position ticks without gear-ratio or circumference conversion | Multiply `encoder.getPosition()` by `WHEEL_CIRCUMFERENCE_METERS` divided by the gear ratio (8.46) |

---

## Group 1 – Constructor & Motor Configuration

### `DriveTrain()` constructor
Currently only inverts the right leader motor.  It must also call `configureMotors()` to finish
hardware setup, and call `addPath()` to register the PathPlanner AutoBuilder.  If a
`VisionSubsystem` reference is later added as a constructor parameter, it should be stored in a
field at this point.

### `configureMotors()`
This method is responsible for all one-time motor configuration that should happen at startup.
It needs to:
1. Set the right-side **follower** motors (`leftMotorFollow`, `rightMotorFollow`) to follow their
   respective leaders using SparkMax's `.follow()` call.  The right follower should follow the
   right leader **with output inverted** so that the inversion set on the leader propagates
   correctly.
2. Set a **current limit** on all four motors (the PathPlanner settings use 40 A; this protects the
   NEOs during stall).
3. Set the encoder **position conversion factor** so that one encoder revolution equals one wheel
   circumference in meters divided by the gear ratio (8.46 : 1), giving encoder readings in meters
   rather than raw revolutions.  This is critical for odometry accuracy.
4. Set the encoder **velocity conversion factor** in the same way, converting RPM to meters per
   second.
5. Reset the encoder positions to zero.

---

## Group 2 – `periodic()` Override

### `periodic()`
This method is called every 20 ms by the WPILib scheduler.  It must:
1. Update the **odometry** by calling `odometry.update()` with the current gyro heading and the
   current left/right encoder distances (in meters).
2. Update the **pose estimator** (`poseEstimator.update()`) the same way, so that vision
   measurements can later be fused into it.
3. Push the current robot pose to the **Field2d** widget so it appears on the SmartDashboard field
   visualization.
4. Call `updateTelemetry()` to publish all other diagnostics.
5. If vision is enabled, call `updateVisionMeasurements()` to fuse any new camera observations.

---

## Group 3 – Basic Drive Helpers

### `stop()`
Stops all drivetrain motion immediately.  Call `driver.stopMotor()` (the DifferentialDrive helper
method), or equivalently set both motor leaders to zero output.  This is the emergency-stop
primitive used by commands.

### `applyDeadband(double value, double deadband)`
Takes a joystick axis value (−1.0 to 1.0) and a deadband threshold.  If the absolute value of the
input is less than the deadband, return 0.0.  Otherwise, return the value unchanged.  This prevents
motor jitter when the joystick is near center.  Note: the method signature in the skeleton declares
`private void` but it needs to **return a double**.  The signature must be changed to
`private double applyDeadband(double value, double deadband)`.

### `setOrientationMode(OrientationMode mode)`
Simply stores the provided `OrientationMode` enum value into the `orientationMode` field.  This is
the setter that allows other code (commands, button bindings) to switch between robot-oriented and
field-oriented driving at runtime.

---

## Group 4 – Teleop Command Factories

### `teleopArcadeCommand(DoubleSupplier fwd, DoubleSupplier rot)`
Returns a `Command` (specifically a `RunCommand`) that, while scheduled, continuously reads the
two supplier values, applies the deadband to each, and calls `arcadeDrive()`.  The command should
require `this` subsystem.  The method is currently private; it is intended for use inside the
class as a factory method called by `RobotContainer` or the constructor's button-binding section.

### `teleopTankCommand(DoubleSupplier left, DoubleSupplier right)`
Same structure as `teleopArcadeCommand` but calls `tankDrive()` instead, passing the left and
right values after deadband is applied.

### `fieldOrientedArcadeCommand(DoubleSupplier fwd, DoubleSupplier rot)`
Returns a `RunCommand` that reads the two joystick suppliers, applies deadband, then calls
`driveFieldRelative()` with a `ChassisSpeeds` built from the forward speed (scaled to max velocity)
and rotation rate (scaled to max angular velocity).  The field-to-robot heading comes from the
current gyro heading so the robot always moves relative to the field regardless of its facing
direction.

---

## Group 5 – Odometry & Pose

### `getPose()`
The skeleton has a partial implementation that reads from `odometry`.  However the code should
prefer the **pose estimator** (which fuses vision) when available.  Change the return to
`poseEstimator.getEstimatedPosition()` so that vision fusion is reflected in the reported pose.

### `resetPose(Pose2d pose)`
Resets both the simple `odometry` object and the `poseEstimator` to the given pose.  Use the
current gyro heading and the current encoder distances (left and right in meters) as the second
and third arguments to each `resetPosition()` / `resetPose()` call.  The skeleton currently calls
`getPosition()` which does not exist; it should call `getLeftDistanceMeters()` and
`getRightDistanceMeters()`.

### `getLeftDistanceMeters()` and `getRightDistanceMeters()`
These already exist but return raw encoder revolutions.  They should multiply the raw
`encoder.getPosition()` value by `WHEEL_CIRCUMFERENCE_METERS` and divide by the gear ratio
(8.46) so that the result is actual wheel travel in meters.  These corrections are required for
accurate odometry and PathPlanner following.

---

## Group 6 – Wheel Speeds & Chassis Speeds

### `getWheelSpeeds()`
Returns a `DifferentialDriveWheelSpeeds` object.  The left speed comes from the left encoder
velocity converted to meters per second (encoder velocity in RPM × circumference ÷ 60 ÷ gear
ratio), and the right speed comes from the right encoder the same way.  With the velocity
conversion factor set in `configureMotors()`, you can read `leftEncoder.getVelocity()` and
`rightEncoder.getVelocity()` directly in m/s.

### `getSpeeds()`
Currently calls itself recursively.  It should call
`kinematics.toChassisSpeeds(getWheelSpeeds())` to convert the left/right wheel speeds into a
single `ChassisSpeeds` object that PathPlanner and other callers can consume.

### `getRobotRelativeSpeeds()`
This is correct in intent – it simply delegates to `getSpeeds()`.  No change needed once
`getSpeeds()` is fixed.

### `driveRobotRelative(ChassisSpeeds robotRelativeSpeeds)`
Converts a `ChassisSpeeds` object into left/right `DifferentialDriveWheelSpeeds` using the
kinematics object, then passes those wheel speeds to `setSpeedsVoltage()` for closed-loop voltage
control.  Use `ChassisSpeeds.discretize()` first with `dt = 0.02` seconds to compensate for
loop-time skew, as the skeleton already shows.

### `driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds)`
Converts field-relative speeds to robot-relative speeds by calling
`ChassisSpeeds.fromFieldRelativeSpeeds()` using the current robot heading (from `getPose()
.getRotation()`), then delegates to `driveRobotRelative()`.  The skeleton has the right idea but
calls `getPose.getRotation2d()` with a missing `()` and the wrong method name (should be
`.getRotation()`).

---

## Group 7 – Motor Output

### `setSpeedsVoltage(DifferentialDriveWheelSpeeds speeds)`
Converts target wheel speeds (m/s) into voltages using a **SimpleMotorFeedforward** (or the
values from PathPlanner's robot config).  The simplest implementation calls
`leftMotorLead.setVoltage(kS + kV * speed)` for each side, where `kS` is the static friction
voltage and `kV` is the velocity gain.  The return type in the skeleton is
`DifferentialDriveWheelSpeeds` but the method has no logical reason to return; it should be
changed to `void`.  Alternatively it can use `leftMotorLead.set()` with a fraction of `12 V`
(nominal voltage from Constants) until feedforward constants are tuned.

### `setSpeedsOpenLoop(DifferentialDriveWheelSpeeds speeds)`
Similar to `setSpeedsVoltage` but uses open-loop control: divides each speed by the maximum drive
speed (from PathPlanner settings: 3.0 m/s) to get a −1 to 1 throttle value and passes it to
`leftMotorLead.set()` / `rightMotorLead.set()`.  Return type should also be changed to `void`.

---

## Group 8 – PathPlanner Integration

### `addPath()` (AutoBuilder configuration)
The skeleton has the overall structure correct but the braces are misaligned: the `try/catch`
block sits inside `addPath()` but the closing brace of `addPath()` is missing, causing a
compile error.  The method should:
1. Register a callback with `PathPlannerLogging.setLogActivePathCallback()` to push the active
   path to the `Field2d` object.
2. Load `robotConfig` from `RobotConfig.fromGUISettings()` inside a `try/catch`.
3. Call `AutoBuilder.configure()` with the six required arguments:
   - Pose supplier: `this::getPose`
   - Pose resetter: `this::resetPose`
   - Speed supplier: `this::getRobotRelativeSpeeds`
   - Speed consumer (robot-relative): `this::driveRobotRelative`
   - Path-following controller: `ltvController`
   - Robot config: `robotConfig`
   - Alliance flip predicate (returns `true` when the current alliance is Red)
   - Subsystem requirement: `this`
4. This method must be called from the **constructor** so AutoBuilder is ready before any auto
   commands are requested.

### `getAutoCommand(String autoName)`
Returns the command that runs a named PathPlanner autonomous routine.  Simply call and return
`AutoBuilder.buildAuto(autoName)`.  If the name is invalid or AutoBuilder has not been configured,
this will throw; wrap it in a try/catch and return `Commands.none()` as a safe fallback.

---

## Group 9 – Vision Integration

### `updateVisionMeasurements()`
Asks the `VisionSubsystem` (which must be stored as a field) for its best measurement by calling
`visionSubsystem.getBestVisionMeasurement()`.  If a measurement is present, pass it to
`poseEstimator.addVisionMeasurement()` using the measurement's estimated pose, timestamp in
seconds, and standard deviation matrix.  This is only called when `visionEnabled` is `true`.

### `getVisionSeededPose()`
Returns an `Optional<Pose2d>`.  Calls `visionSubsystem.getBestVisionMeasurement()` and maps the
result to `measurement.estimatedPose()`.  Returns `Optional.empty()` if no measurement is
available.  This method is used at startup to seed the initial pose estimate from vision before
the robot has moved.

### `setVisionEnabled(Boolean enabled)`
Stores the value into the `visionEnabled` field and returns it.  The current skeleton already
returns the argument but forgets to save it to the field.  Add `this.visionEnabled = enabled`
before the return.

---

## Group 10 – Telemetry

### `updateTelemetry()`
Publishes robot state to SmartDashboard and updates the Field2d widget.  Should output:
- Current robot pose (X, Y, heading in degrees) to SmartDashboard number entries
- Current drive mode (ARCADE / TANK) as a string
- Current orientation mode (ROBOT / FIELD oriented) as a string
- Left and right encoder distances in meters
- Left and right wheel speeds in m/s
- Gyro heading in degrees
- Whether vision is enabled
- Update the `field` Field2d object with `field.setRobotPose(getPose())` so the robot icon moves
  on the dashboard

---

## Summary Table

| Method | Status in Skeleton | What Is Missing |
|---|---|---|
| `DriveTrain()` constructor | Partial | Call `configureMotors()` and `addPath()` |
| `configureMotors()` | Empty | Follower setup, current limits, encoder conversion factors |
| `periodic()` | Empty | Odometry update, pose estimator update, Field2d update, telemetry call |
| `stop()` | Empty | Call `driver.stopMotor()` |
| `applyDeadband()` | Empty + wrong return type | Return the deadbanded value (or 0) |
| `setOrientationMode()` | Empty | Assign `orientationMode = mode` |
| `teleopArcadeCommand()` | Empty | Return `RunCommand` calling `arcadeDrive` with deadband |
| `teleopTankCommand()` | Empty | Return `RunCommand` calling `tankDrive` with deadband |
| `fieldOrientedArcadeCommand()` | Empty | Return `RunCommand` calling `driveFieldRelative` |
| `getPose()` | Reads from `odometry` | Switch to reading from `poseEstimator` |
| `resetPose()` | Calls nonexistent `getPosition()` | Use `getLeftDistanceMeters()` / `getRightDistanceMeters()` |
| `getLeftDistanceMeters()` / `getRightDistanceMeters()` | Returns raw ticks | Multiply by `WHEEL_CIRCUMFERENCE_METERS`, divide by gear ratio |
| `getWheelSpeeds()` | Empty | Build and return `DifferentialDriveWheelSpeeds` from encoder velocities |
| `getSpeeds()` | Infinite recursion | Call `getWheelSpeeds()` instead of `getSpeeds()` |
| `getRobotRelativeSpeeds()` | Delegates to `getSpeeds()` | No change needed once `getSpeeds()` is fixed |
| `driveRobotRelative()` | Has `????` placeholder | Convert `ChassisSpeeds` to wheel speeds, call `setSpeedsVoltage()` |
| `driveFieldRelative()` | Syntax bug | Fix method call to `getPose().getRotation()` |
| `addPath()` / AutoBuilder | Brace mismatch, not called | Fix braces, call from constructor |
| `getAutoCommand()` | No return | Return `AutoBuilder.buildAuto(autoName)` |
| `setSpeedsVoltage()` | No return / no body | Apply feedforward voltage to each motor lead |
| `setSpeedsOpenLoop()` | No return / no body | Set motor output as fraction of max speed |
| `updateVisionMeasurements()` | Empty | Get measurement from VisionSubsystem, add to pose estimator |
| `getVisionSeededPose()` | Empty | Map VisionSubsystem measurement to `Optional<Pose2d>` |
| `setVisionEnabled()` | Returns arg but does not store | Add `this.visionEnabled = enabled` |
| `updateTelemetry()` | Empty | Publish pose, speeds, modes, gyro to SmartDashboard + Field2d |
