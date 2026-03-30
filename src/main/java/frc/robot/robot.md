# Robot Architecture

This document explains how the robot code is structured at the top level. It assumes you have read [repo.md](../../../../../repo.md) and understand what the repository contains.

---

## Command-Based Programming

WPILib robots are written using the **Command-Based** programming paradigm. If you are coming from a Python or block-based background this may seem unusual, so it is worth understanding before diving into the code.

### The Problem It Solves

A robot needs to do many things simultaneously — drive forward, spin a flywheel, run a conveyor belt — and these actions need to be coordinated and interruptible. Simply putting everything in a loop is hard to manage. Command-Based solves this with two key abstractions:

### Subsystems

A **Subsystem** represents a physical group of components on the robot (drivetrain, shooter, etc.). Each subsystem:
- Owns its hardware objects (motors, sensors, encoders).
- Has a `periodic()` method that runs every 20 ms regardless of what commands are active.
- Enforces **exclusive access** — only one command can use a subsystem at a time.

### Commands

A **Command** represents an *action* the robot takes. Every command declares which subsystems it *requires*. The WPILib **CommandScheduler** ensures that:
- Only one command runs on each subsystem at a time.
- If a new command requires a subsystem that is already in use, it either cancels the running command (default) or is rejected.
- When a command finishes or is cancelled, its `finallyDo()` cleanup runs automatically.

Commands have a lifecycle:
1. `initialize()` — runs once when scheduled
2. `execute()` — runs every 20 ms loop while active
3. `isFinished()` — returns true when the command should end
4. `end(interrupted)` — runs once when the command ends (finished or cancelled)

### Why This Matters

The separation of subsystems and commands is what allows, for example, the shooter flywheel to spin up (a Shooter command) at the same time as the intake runs (a Feeder command) — they require *different* subsystems so they are fully independent and run concurrently without any special coordination code.

---

## Robot Lifecycle

The `Robot.java` class manages the robot's operating modes. You do not need to edit this file, but understanding it helps you understand when your code runs.

| Method | When It Runs |
|--------|-------------|
| `robotInit()` | Once at startup — creates `RobotContainer` |
| `robotPeriodic()` | Every 20 ms always — runs the CommandScheduler |
| `autonomousInit()` | Once when auto period begins — calls `getAutonomousCommand()` |
| `autonomousPeriodic()` | Every 20 ms during auto |
| `teleopInit()` | Once when teleop begins |
| `teleopPeriodic()` | Every 20 ms during teleop |
| `testInit()` | Once when test mode begins |
| `testPeriodic()` | Every 20 ms in test mode |
| `disabledInit()` | Once when robot is disabled |

The most important thing to know: **the CommandScheduler runs in `robotPeriodic()`**, so it runs in every mode. Default commands, button bindings, and subsystem `periodic()` methods are all driven by this single scheduler call.

---

## RobotContainer

`RobotContainer.java` is the **wiring harness** of the robot. It is the first thing created by `Robot.java` and is responsible for:

1. **Instantiating all subsystems** in the correct dependency order.
2. **Configuring default commands** — the command each subsystem runs when nothing else is using it.
3. **Registering named commands** for PathPlanner autos.
4. **Mapping controller buttons to commands** (`configureBindings()`).
5. **Building the autonomous chooser** from available PathPlanner routines.

### Subsystem Dependency Order

Subsystems must be constructed in the right order because some are passed as dependencies to others:

```
Vision
  └── DriveTrain(vision)
        └── Shooter(vision, drivetrain)

Feeder  ← independent
```

`Vision` must exist before `DriveTrain` is constructed. `DriveTrain` must exist before `Shooter` is constructed. `Feeder` has no dependencies.

### Controllers

| Controller | Port | Role |
|-----------|------|------|
| Driver (Xbox) | 0 | Drive motions only |
| Operator (Xbox) | 1 | Intake, shooting, distance overrides |

See the Javadoc in `RobotContainer.java` (`configureBindings()`) for the full button map.

---

## Constants

`Constants.java` holds all robot-wide configuration values. It is organized into **static inner classes** so constants are namespaced:

| Inner Class | Contents |
|-------------|---------|
| `OperatorConstants` | Controller port numbers |
| `DriveTrainConstants` | CAN IDs, gear ratios, speeds, encoder factors |
| `ShooterConstants` | CAN IDs, PID gains, RPM map, current limits |
| `SensorConstants` | Photo sensor DIO port and enable flag |
| `Auto` | PathPlanner velocity/acceleration limits, LTV controller tuning |
| `VisionConstants` | Camera names, transforms, standard deviations, zone boundaries |
| `IntakeConstants` | Reserved for future use |
| `ClimberConstants` | Reserved for future use |

**Never put logic in Constants.** It should only contain `public static final` values. Import specific inner classes with `import static frc.robot.Constants.DriveTrainConstants.*;` where needed to avoid verbose `Constants.DriveTrainConstants.SOME_VALUE` references throughout the code.

---

## Utility Classes

### `Result.java`

A lightweight test result container used in subsystem self-tests. Each `Result` holds:
- `unit` — the name of what was tested
- `success` — pass or fail
- `reports` — list of detail strings

Factory methods `Result.pass(unit)` and `Result.fail(unit, messages...)` keep construction concise. `toString()` outputs colored text (green/red) to the console for easy visual scanning. Currently only used by a placeholder test in `DriveTrain`.

### `VisionMeasurement.java`

A Java **record** (an immutable data container) that packages all relevant data from a single vision pose estimate into one object:

| Field | Type | Meaning |
|-------|------|---------|
| `estimatedPose` | `Pose2d` | Robot position on the field |
| `timestampSeconds` | `double` | FPGA time when the image was captured |
| `standardDeviations` | `Matrix<N3,N1>` | Trust level (lower = more trusted) for x, y, heading |
| `bestTargetAmbiguity` | `double` | How ambiguous the best AprilTag detection was (0 = perfect, 1 = useless) |
| `numTagsUsed` | `int` | How many tags contributed to this estimate |
| `averageDistance` | `double` | Average distance to detected tags (meters) |

Records automatically generate a constructor, getters, `equals()`, `hashCode()`, and `toString()`. You access fields with method calls: `measurement.estimatedPose()`, `measurement.timestampSeconds()`, etc.

---

## AdvantageKit Logging

This robot uses **AdvantageKit** for data logging. The robot extends `LoggedRobot` instead of `TimedRobot`. AdvantageKit records all inputs and outputs to a `.wpilog` file on the roboRIO, which can be replayed later in simulation for debugging. This is especially useful for diagnosing problems that only occur during matches.

Log files are saved to `/home/lvuser/logs/` on the roboRIO and can be downloaded via the Driver Station or FTP.
