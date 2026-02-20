# Drivetrain Branch — Main Integration Evaluation

This document evaluates the readiness of the `TP-Drivetrain` branch for integration into `main`, identifies blocking issues that must be resolved beforehand, and lists desirable improvements that can follow integration in a subsequent branch.

---

## What This Branch Does

The `TP-Drivetrain` branch is a complete architectural overhaul of the drivetrain subsystem. The prior implementation on `main` was a minimal ~60-line class using two PWM-based motor controllers with basic arcade and tank drive modes. This branch replaces it with a full-featured 420+ line drivetrain that includes:

- Four CAN-based brushless SparkMax motors in a leader/follower configuration
- NavX2 gyro integration via the SPI port
- Encoder-based odometry for position tracking
- A `DifferentialDrivePoseEstimator` with vision sensor fusion
- Full PathPlanner autonomous integration via `AutoBuilder`
- A three-tier pose initialization strategy at autonomous start (vision → PathPlanner path → field origin)
- A dropdown auto routine selector on SmartDashboard
- Field visualization via `Field2d`
- A partial implementation of field-oriented driving
- An `OrientationMode` enum for future robot-vs-field orientation toggling

The branch also includes multiple PathPlanner `.path` and `.auto` files for game positions, a set of test paths, robot parameter configuration for PathPlanner, and architecture documentation files.

---

## What Changed Outside of DriveTrain.java

### `Constants.java`
The `DriveTrainConstants` class was substantially expanded. New entries cover encoder port assignments, a joystick deadband value, and gear ratio confirmation. A new `Auto` inner class was added with wheel geometry (diameter, circumference), track width, gear ratio, and PathPlanner motion limits (maximum speed, acceleration, angular velocity, and angular acceleration).

Several placeholder constants were added — notably `CURRENT_LIMIT`, `POSITION_FACTOR`, and `VELOCITY_FACTOR` — all initialized to zero. These appear to be stubs for values that were later hardcoded directly in `DriveTrain.java`. Additionally, `OPEN_LOOP_RAMP`, `CLOSED_LOOP_RAMP`, `LEFT_INVERTED`, and `RIGH_INVERTED` were defined but are never referenced in code. There is also a typo in the name `RIGH_INVERTED` (missing a `T`).

### `RobotContainer.java`
The order of subsystem instantiation was updated so that `VisionSubsystem` is constructed before `DriveTrain`, because `DriveTrain` now requires a `VisionSubsystem` reference in its constructor. A `SendableChooser` for autonomous routines was added and published to SmartDashboard via `AutoBuilder.buildAutoChooser()`. The `getAutonomousCommand()` method was implemented to retrieve the selected auto, call `drivetrain.initializePose()` to seed the pose before the auto runs, and then return the command.

### `Robot.java`
A single previously commented-out line was uncommented to enable autonomous command scheduling. This allows PathPlanner routines to actually execute during the autonomous period.

---

## Evaluation: What Is Ready

The following areas are solid and ready for integration:

- **Motor hardware setup.** The four-motor SparkMax configuration with follower relationships, brake mode, current limiting, and encoder conversion factors is correctly structured. The leader/follower topology is correct.
- **Odometry and pose estimation.** Both the basic `DifferentialDriveOdometry` and the `DifferentialDrivePoseEstimator` are initialized correctly with gyro and encoder inputs and updated every periodic cycle.
- **Gyro heading.** The NavX2 is initialized and its heading is correctly negated for WPILib's counter-clockwise-positive convention.
- **PathPlanner integration.** `AutoBuilder` is configured with the correct callbacks (pose supplier, pose reset, chassis speed supplier, chassis speed consumer, controller, and alliance flip flag). The `PPLTVController` is used for trajectory following.
- **Pose initialization at auto start.** The three-tier fallback (vision → auto starting pose → origin) is a robust pattern that prevents odometry from starting in an unknown state.
- **Vision sensor fusion.** The optional vision measurement pattern correctly guards against a null `VisionSubsystem` reference and integrates measurements with timestamps and standard deviations.
- **Auto selector.** Dashboard-driven auto selection via `AutoBuilder.buildAutoChooser()` is complete and functional.
- **Autonomous path and auto files.** Several competition-relevant paths and autos have been created and are organized clearly.
- **Arcade and tank drive.** The basic drive mode toggle and dispatch logic carries over correctly from the prior implementation.

---

## Blocking Issues — Must Resolve Before Merging

These issues must be addressed before this branch can be merged into `main`. They either represent incomplete implementations that could cause confusion or incorrect behavior, or they are cleanup items that will become harder to fix after other branches also merge.

### 1. Encoder Configuration: Alternate Encoder vs. Primary Encoder

The motor configuration code sets conversion factors on `alternateEncoder`, but the code that reads encoder values calls `.getEncoder()` (the primary integrated encoder). These are different encoder objects. If the robot uses the SparkMax's internal hall-effect encoder (which is the default), `alternateEncoder` configuration will be ignored and the conversion factors will have no effect, meaning all position and velocity readings will be in raw units rather than meters and meters per second. The team must verify which encoder is physically connected and update the configuration to match.

### 2. `OrientationMode` Is Defined But Not Functional

An `OrientationMode` enum with `ROBOT_ORIENTED` and `FIELD_ORIENTED` values is declared. Methods to get and set the mode exist. A `fieldOrientedArcadeCommand()` method exists that applies a rotation matrix to the joystick inputs. However, the `drive()` method and the standard arcade/tank drive methods do not consult `orientationMode` at all — they always drive robot-relative. The `fieldOrientedArcadeCommand()` method is never called from `RobotContainer`.

Before merging, the team must make one of two decisions:
- **If field-oriented drive is wanted for this season:** Wire `fieldOrientedArcadeCommand()` into `RobotContainer` with a button or toggle, and make the `drive()` dispatcher respect the `orientationMode` field.
- **If field-oriented drive is not needed yet:** Remove the partial implementation entirely — the enum, the field, the getter/setter methods, and the unused command method — to prevent confusion during competition. It can be re-added properly in a future branch.

Leaving it half-implemented is the worst outcome because it suggests a feature works when it does not.

### 3. Teleop Drive Commands Are Not Wired

`DriveTrain` exposes `teleopArcadeCommand()` and `teleopTankCommand()` as named command factory methods, but `RobotContainer.configureBindings()` does not call them or bind them to any controller input. The robot has no teleop drive control at present. Drive commands must be wired to controller axes in `RobotContainer` before this branch is driveable.

### 4. Current Limit Is Inconsistent

`DriveTrainConstants.CURRENT_LIMIT` is set to `0.0`, which is a placeholder value that was never updated. The actual motor configuration code bypasses it and hardcodes `40` amps directly in four separate places. The constant should be updated to the intended value (likely 40A or whatever the electrical team specifies), and the four hardcoded values should be replaced with the constant so there is a single place to change it.

### 5. Placeholder and Unused Constants Should Be Cleaned Up

Several constants in `DriveTrainConstants` and the new `Auto` class are either zero-placeholder values that are unused (`POSITION_FACTOR`, `VELOCITY_FACTOR`) or are defined but never referenced in any code (`OPEN_LOOP_RAMP`, `CLOSED_LOOP_RAMP`, `LEFT_INVERTED`, `RIGH_INVERTED`, `WHEEL_DISTANCE`). These should be removed before merging to keep `Constants.java` clean. The typo `RIGH_INVERTED` should be corrected if the constant is kept, or deleted if it is removed.

### 6. Deadband Implementation Does Not Remap Output

The `applyDeadband()` helper returns zero for inputs below the threshold but returns the raw input value unchanged for inputs above it. This means a joystick at just above the deadband threshold produces a very small output (near zero), and the full range from deadband to maximum maps to the full output range only in theory — in practice the small values near the threshold feel like dead zone even after it. A correct deadband implementation remaps the post-threshold range so that just above the threshold produces a proportionally small but non-zero output, and maximum input still produces maximum output. This should be corrected before the robot is used in competition, as the current behavior can feel sluggish or unresponsive in the low-input range.

---

## Unresolved TODOs

The following TODO comments in `DriveTrain.java` represent open design questions that should be answered before or alongside the blocking fixes above:

- **LTV controller tuning:** Whether the default `PPLTVController` constructor parameters are appropriate for this robot's dynamics. PathPlanner's default tuning may not produce tight path following on this robot's weight and wheel configuration.
- **Deprecated motor modes:** A comment references needing to investigate deprecated configuration modes. This should be resolved to avoid future API breakage.
- **Telemetry completeness:** Whether the current set of SmartDashboard outputs (pose, encoder distances, heading, drive mode) is sufficient for drive team and pit crew use during competition.

---

## Improvements Desirable Before Integration (Non-Blocking but Recommended)

The following items do not prevent the branch from functioning but would meaningfully improve robustness or clarity before the code reaches `main`:

- **Hardware test checklist.** Before merging, verify on the physical robot that all four motors spin in the correct direction, that follower motors correctly mirror their leaders, that the gyro produces stable heading readings, and that encoder position and velocity values are in the expected units (meters and meters per second) after conversion factor fixes.
- **Remove dead command methods.** If `teleopArcadeCommand()` and `teleopTankCommand()` are not going to be used (perhaps the drive is being wired with `driveCommand()` instead), remove them to avoid confusion. If they are the intended interface, use them in `RobotContainer`.
- **Validate PathPlanner robot configuration.** Confirm that the `pathplanner/settings.json` values (wheel diameter, track width, gear ratio) match the physical robot exactly. Incorrect values will cause autonomous paths to under- or overshoot.
- **Test pose initialization at auto start.** Verify that when vision tags are visible, the pose estimator is seeded correctly. Verify fallback to the PathPlanner starting pose when no tags are visible. Verify the origin fallback does not cause autonomous paths to execute from wrong positions.

---

## Improvements to Add After Integration (Future Branch)

The following are enhancements that would improve the drivetrain but are not needed for initial competition use. They are good candidates for a follow-on branch after the merge.

**Field-oriented drive (if removed before merge)**
If the partial field-oriented implementation is removed as part of resolving the blocking issue, it can be properly re-implemented in a dedicated branch once teleop driving is validated in robot-oriented mode.

**Adaptive deadband and input shaping**
Replace the simple deadband cutoff with a proper remapped deadband and optionally add an input curve (quadratic or cubic mapping) for finer low-speed control. This can make the robot significantly easier to drive precisely.

**Drive characterization**
Run the WPILib `SysId` characterization tool on the physical robot to measure the feedforward constants (`kS`, `kV`, `kA`) for the drivetrain. Use those values to update the PathPlanner robot configuration. This will substantially improve autonomous path accuracy.

**Motion profiling for teleop**
Add acceleration limiting (slew rate) to teleop drive inputs to reduce wheel slip during hard starts and stops, which improves odometry accuracy and reduces mechanical stress.

**Expanded telemetry and logging**
Add per-motor voltage, current, and temperature outputs to SmartDashboard. Add logging of vision measurement quality (tag count, ambiguity, distance) separately from pose. Consider AdvantageKit integration for post-match replay analysis.

**Automated drive-to-target commands**
Implement a command that drives the robot to a specific field position using the pose estimator and PathPlanner's on-the-fly path generation. This enables consistent game piece pickup and scoring position approaches.

**Gyro drift monitoring**
Add a SmartDashboard indicator that alerts when the gyro has been running for a long time without a reset or when the vision and gyro headings diverge beyond a threshold, indicating drift.

---

## Merge Conflict Assessment

The expected merge complexity is low. `DriveTrain.java` is a complete rewrite and will not conflict with the `main` version. `Robot.java` differs by only a single uncommented line. `RobotContainer.java` changes are additive. `Constants.java` is the most likely source of merge conflicts because other branches (shooter, intake) have also expanded it. A manual merge of `Constants.java` will be needed to combine the `Auto` class and expanded `DriveTrainConstants` from this branch with the `IntakeConstants`, `ClimberConstants`, and updated `ShooterConstants` from those branches. The new path and auto files have no counterparts on `main` and will merge automatically.

---

## Summary

| Area | Status |
|---|---|
| Motor hardware (4x CAN SparkMax, leader/follower) | Ready |
| Odometry and pose estimation | Ready |
| Gyro heading | Ready |
| PathPlanner `AutoBuilder` integration | Ready |
| Auto routine selector (SmartDashboard) | Ready |
| Pose initialization at auto start | Ready |
| Vision sensor fusion | Ready |
| PathPlanner path and auto files | Ready |
| Teleop drive commands wired to controller | **Blocking — not done** |
| Encoder configuration (alternate vs primary) | **Blocking — verify hardware** |
| OrientationMode implementation | **Blocking — decide and act** |
| Current limit constant | **Blocking — clean up** |
| Unused/placeholder constants | **Blocking — clean up** |
| Deadband implementation correctness | **Blocking — fix before competition** |
| Hardware tested on robot | **Required before merge** |
| PathPlanner config validated against robot | **Recommended before merge** |

**Overall readiness: Not yet — resolve the six blocking issues, then this branch is ready to merge.**
