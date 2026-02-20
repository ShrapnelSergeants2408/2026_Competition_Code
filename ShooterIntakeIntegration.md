# Shooter + Intake Integration Plan

This document describes the changes required to merge the `intake` branch into the `shooter` branch, consolidate overlapping subsystems, and replace manual distance-selection buttons with automatic vision-based distance measurement. No code is included — this is a natural-language description of the work to be done.

---

## Background and Goals

The robot currently has two branches that have diverged significantly:

- The **shooter branch** contains a working `ShooterSubsystem` with a distance-to-RPM lookup table and basic vision integration scaffolding, but it also still references a deprecated `Shooter` class. The `Intake` and `Climber` subsystems are empty stubs.
- The **intake branch** contains a fully implemented `Intake` subsystem with jam detection, a fully implemented `Climber` subsystem with encoder-based position control, and an improved `ShooterSubsystem` that uses true closed-loop RPM feedback via TalonFX velocity control rather than simulated open-loop output.

The goals of this integration are:

1. Eliminate the deprecated `Shooter` class and standardize on `ShooterSubsystem`.
2. Bring the full `Intake` and `Climber` implementations from the intake branch into the shooter branch.
3. Use the intake branch's `ShooterSubsystem` as the authoritative version (it has actual motor velocity feedback).
4. Replace manual D-pad distance selection buttons with automatic vision-based distance calculation using the `VisionSubsystem`.
5. Resolve all constant conflicts between the two branches, particularly motor CAN IDs.
6. Clean up `RobotContainer` to remove duplicate subsystem instantiations and organize button bindings.

---

## Step 1 — Resolve Hardware Conflicts Before Touching Code

**This must be done first.** The two branches disagree on CAN bus IDs for the shooter and feeder motors. The shooter branch assigns the shooter motor ID as 30 and the feeder motor ID as 31. The intake branch assigns them as 31 and 32 respectively. Using the wrong ID could command the wrong physical motor, which could damage hardware.

Before any code changes are made, the electrical or mechanical team must confirm the actual CAN IDs assigned on the physical robot for:

- The shooter flywheel motor (TalonFX)
- The feeder motor (SparkMax)
- The intake motor (SparkMax, currently assigned ID 42 on the intake branch)
- The climber motor (SparkMax, currently assigned ID 6 on the intake branch)

Once confirmed, these IDs must be set correctly in `Constants.java` and must not be changed without re-verifying against the robot's wiring.

---

## Step 2 — Delete the Deprecated `Shooter` Class

The `Shooter.java` file is an older, simpler implementation that uses only open-loop percentage output and has no RPM feedback, no ball detection integration, and no vision integration. It is already absent from the intake branch. It should be deleted from the shooter branch as well. Any references to it in `RobotContainer` or elsewhere must also be removed.

`ShooterSubsystem.java` is the only shooter implementation that should remain.

---

## Step 3 — Replace `ShooterSubsystem` with the Intake Branch Version

The intake branch version of `ShooterSubsystem` is strictly better than the shooter branch version in one critical way: it uses true closed-loop velocity control via TalonFX's velocity voltage mode, which means the motor controller actively tracks and corrects its RPM using encoder feedback. The shooter branch version uses open-loop percentage output and only simulates RPM by tracking the last commanded percentage.

The intake branch version should become the authoritative implementation. The key behavioral differences are:

- `spinAtSpeed` will command a true target velocity to the motor controller rather than setting a duty cycle percentage.
- `stop` will command a zero-velocity target rather than cutting motor output directly.
- `getCurrentRPM` will return the actual measured motor velocity from the encoder, not just whatever was last commanded.
- `setTargetRPM` will store the target for later use by the enable/disable flow.
- A new `enableShooter` method will exist that conditionally applies or clears the stored target RPM, allowing the shooter to be armed and disarmed cleanly.

The PID gain constant `SHOOTER_KP` must be updated from its current value of 0.0 (unused in the shooter branch) to the intake branch's value of 1.0, which is required for the closed-loop controller to function. The other gains (`SHOOTER_KI`, `SHOOTER_KD`, `SHOOTER_KV`) should also be reviewed.

---

## Step 4 — Integrate the Full `Intake` Subsystem

The shooter branch currently has an empty `Intake.java` stub. This needs to be replaced with the fully implemented version from the intake branch.

The implemented intake subsystem includes:

- A SparkMax motor controlled at a configurable forward speed for ball collection, a slow intake speed, and a reverse speed for ejecting.
- Current spike monitoring to detect jams. When the motor current exceeds a configurable threshold for a debounce window, the subsystem automatically reverses briefly to clear the jam before resuming.
- Two command factory methods: one for forward intake operation and one for ejecting.

No structural changes to the intake subsystem itself are needed during this merge. It should be brought in as-is, with its constants populated in `Constants.java` as described in Step 6.

---

## Step 5 — Integrate the Full `Climber` Subsystem

The shooter branch currently has an empty `Climber.java` stub. This needs to be replaced with the fully implemented version from the intake branch.

The implemented climber subsystem includes:

- A SparkMax motor with a relative encoder.
- Open-loop control methods for climbing up, climbing down, and stopping.
- A position-controlled method that drives the climber to a target encoder position using a proportional controller, intended for reaching the level-one climb bar.
- Motor configuration with brake idle mode, current limiting, and voltage compensation.

No structural changes to the climber subsystem are needed during this merge.

---

## Step 6 — Consolidate `Constants.java`

The two branches have diverged significantly in `Constants.java`. The merge must produce a single file that includes all of the following:

**`ShooterConstants`**
- Correct motor IDs (verified with hardware team per Step 1).
- `SHOOTER_KP` updated to 1.0 to enable closed-loop control.
- All existing distance-to-RPM lookup table entries.
- All RPM tolerance and tuning values.

**`IntakeConstants`** (currently empty in shooter branch — populate from intake branch)
- Motor ID, current limit, and voltage compensation.
- Forward speed, slow speed, reverse speed, and eject speed values.
- Jam detection: current spike threshold and debounce cycle count.
- Jam reversal: reverse speed and reversal duration in seconds.

**`ClimberConstants`** (currently empty in shooter branch — populate from intake branch)
- Motor ID, current limit, and voltage compensation.
- Climb speed.
- Level-one target encoder position.
- Proportional gain constant for position control.

**`SensorConstants`**
- The existing light sensor DIO port used by the shooter for ball detection.
- The intake branch adds a ball sensor DIO port and an inversion flag — include these as well.

**`DriveTrainConstants`**
- The intake branch has a more complete version of this class, including encoder port assignments, joystick deadband, and additional motor IDs. Merge these in.

**`Auto`** (new class from intake branch)
- Wheel diameter, circumference, and gear ratio constants.
- Track width for differential drive kinematics.
- Maximum module speed, acceleration, and angular velocity/acceleration values for autonomous path following.

---

## Step 7 — Replace Manual Distance Buttons with Vision-Based Distance

Currently, the driver uses the D-pad to manually select a shooting distance from a fixed set of preset values (7.5, 10, 12.5, and 15 feet). This is slow, requires the driver to judge distance visually, and is error-prone.

The `VisionSubsystem` already has all the infrastructure needed to provide distance automatically:

- It calculates the robot's pose on the field using AprilTag detection.
- It has a method that returns the current distance to the hub in feet.
- It has a method that checks whether the robot is rotationally aligned with the hub within a configurable tolerance.

The `ShooterSubsystem` already has an `updateFromVision` method that accepts an optional distance value in feet and feeds it into the distance-to-RPM lookup table.

The integration work in this step involves changes to `RobotContainer`:

1. Obtain the live distance from `VisionSubsystem` on each control loop cycle when the driver is holding the shoot button.
2. Pass that distance into `ShooterSubsystem.updateFromVision` continuously so the RPM target tracks the robot's actual distance in real time.
3. Only feed the ball (run the feeder motor) when `ShooterSubsystem.canShoot()` returns true — that is, when the shooter is at its target speed and a ball is detected.
4. Keep the D-pad manual preset buttons as a fallback for situations where vision is unavailable or no tag is visible. The driver should have an indicator on the dashboard (via SmartDashboard) showing whether the shooter is operating in vision mode or manual mode, the current detected distance, and whether the robot is aligned with the hub.

The driver workflow will become: drive to approximately the correct position, hold the shoot button, wait for the "ready to shoot" indicator, then trigger the feeder. The system will handle distance computation and RPM selection automatically.

---

## Step 8 — Clean Up `RobotContainer`

Both branches have accumulated technical debt in `RobotContainer` that must be addressed as part of this merge:

**Remove deprecated references**
- All references to the deleted `Shooter` class must be removed, including imports and field declarations.
- The duplicate `Intake` instantiation in the intake branch (two separate fields pointing to two separate objects) must be collapsed to a single instance.

**Unify subsystem field naming**
- Decide on a consistent naming convention for subsystem fields (e.g., drop the `m_` prefix or apply it consistently) and apply it across all subsystems.

**Add missing bindings**
- The climber currently has no button bindings in either branch. Decide on which buttons will control climb up, climb down, and automated level-one positioning, then add those bindings.
- The intake bindings from the intake branch (left trigger for intake, left bumper for eject) should be preserved.

**Reorganize binding configuration**
- Group all bindings by subsystem with clear comments separating each group (drivetrain, shooter, intake, climber).
- The vision-driven shooter command should be wired to the right trigger. The D-pad fallback presets should remain but should only be used when the driver explicitly overrides.

**Add subsystem accessor methods**
- Add getter methods for each subsystem so they can be accessed from `Robot.java` if needed for periodic telemetry or autonomous configuration.

---

## Step 9 — Verify `VisionSubsystem`

The `VisionSubsystem` is identical between both branches, so no merge is needed. However, before relying on it for shooter distance control, confirm the following:

- The `HUB_POSE` constant is set to the correct field coordinates for the 2026 game.
- Camera-to-robot transform values are correctly calibrated.
- The `MAX_TAG_DISTANCE_METERS` gate is set to a value that covers all shooting positions the team will use.
- The `getShootingDistanceFeet` method's conversion factor (meters to feet) is correct.

The deprecation warnings on `getLatestResult`, `PhotonPoseEstimator` constructor, and `update` should be addressed separately, but they do not block this integration.

---

## Step 10 — Testing Sequence

After completing the code changes, test each subsystem in order before integration testing:

1. **DriveTrain** — Confirm no regressions. Verify encoder constants if intake branch version is used.
2. **Climber** — Test open-loop up/down, verify current limiting, test level-one positioning.
3. **Intake** — Test forward intake, ejection, and jam detection/reversal with actual game pieces.
4. **Shooter** — Test RPM feedback accuracy, verify the lookup table produces expected RPMs at known distances, confirm ball detection logic.
5. **Vision** — Confirm camera connectivity, tag detection, and distance calculation against a physical measurement at several distances.
6. **Integration** — Test the full intake-to-feeder-to-shooter sequence, then the vision-driven shooter flow end-to-end.

---

## Summary of Files to Modify

| File | Action |
|---|---|
| `Shooter.java` | **Delete** — deprecated, replaced by `ShooterSubsystem` |
| `ShooterSubsystem.java` | **Replace** with intake branch version (VelocityVoltage, real RPM feedback) |
| `Intake.java` | **Replace** empty stub with intake branch implementation |
| `Climber.java` | **Replace** empty stub with intake branch implementation |
| `VisionSubsystem.java` | **No change** — identical on both branches; verify calibration values |
| `DriveTrain.java` | **No change** — identical on both branches |
| `Constants.java` | **Merge** — populate IntakeConstants, ClimberConstants, update ShooterConstants, add Auto class |
| `RobotContainer.java` | **Rewrite** — remove duplicates, add all bindings, wire vision-driven shooter |

---

## Open Questions for the Team

- **CAN IDs**: What are the actual CAN bus IDs for the shooter motor, feeder motor, intake motor, and climber motor on the physical robot? This must be resolved before any merge attempt.
- **Climber buttons**: Which controller buttons should control the climber? Are there enough buttons remaining after shooter and intake bindings?
- **Vision alignment**: Should the robot automatically rotate to align with the hub when the shoot button is held, or should the driver manually aim?
- **Feeder + Intake coordination**: Is there a ball in the hopper/magazine between the intake and the feeder? If so, does there need to be a separate sensor or logic to prevent double-loading?
- **SHOOTER_KP tuning**: The intake branch uses a KP of 1.0, but this has not been tuned on the physical robot. Plan time to characterize and tune the closed-loop controller before competition.
