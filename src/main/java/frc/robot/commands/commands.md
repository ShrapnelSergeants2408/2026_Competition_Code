# Commands

This document explains how commands work in WPILib's Command-Based framework, why the `commands/` directory is no longer the primary home for commands, and how to write commands using the modern approach used in this codebase.

---

## What Is a Command?

A **command** is an action the robot performs. Every command:
- Declares which **subsystems** it requires (so the scheduler can prevent conflicts).
- Has an **execute** phase that runs every 20 ms while active.
- Has an **end** phase that runs when the command finishes or is cancelled.
- Has an `isFinished()` condition that determines when it ends automatically.

Commands are scheduled by the **CommandScheduler**, which runs every robot loop. They can be triggered by:
- **Button bindings** — a button press schedules a command.
- **Default commands** — a subsystem runs a command whenever nothing else is using it.
- **Auto routines** — PathPlanner composes commands into sequences.

---

## The Old Pattern: Separate Command Classes

In older WPILib versions (pre-2022), each command was a separate `.java` file in the `commands/` directory. A typical command looked like this:

```java
// Old pattern — one file per command
public class DriveForwardCommand extends CommandBase {
    private final DriveTrain drivetrain;

    public DriveForwardCommand(DriveTrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(0.5, 0.5);
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
```

This pattern requires a new file for every action. For simple commands it is verbose and unnecessary.

### Why the `commands/` Directory Is Deprecated

The `commands/` directory in this project contains only a placeholder file. **All commands are defined inline in `RobotContainer.java`** using the modern lambda-based API. This is the recommended approach for most FRC teams because:

- Short, focused commands do not need their own file.
- Less boilerplate — you do not need four method overrides for a simple action.
- Commands are defined right next to the button bindings that trigger them, making the code easier to follow.

You should still create a separate command class if a command has significant internal logic, multiple states, or is long enough that keeping it inline makes `RobotContainer` hard to read.

---

## The Modern Pattern: Inline Lambdas and Command Factories

WPILib provides **static factory methods** in the `Commands` class that create commands from lambda functions. A lambda is an anonymous function written with `() -> { ... }` syntax.

### `Commands.runOnce(action, requirements...)`

Runs the action **once**, then immediately finishes. Use for one-shot actions.

```java
// Run once when button is pressed — re-seeds drivetrain pose from vision
m_driverController.back().onTrue(
    Commands.runOnce(() -> drivetrain.initializePose(null), drivetrain)
);
```

### `Commands.run(action, requirements...)`

Runs the action **every 20 ms** until the command is interrupted. Never finishes on its own. Use for continuous actions like driving or spinning a motor.

```java
// Runs every loop while the button is held
m_operatorController.leftBumper().whileTrue(
    Commands.run(() -> feeder.startFeed(), feeder)
        .finallyDo(() -> feeder.stopAll())
);
```

### `Commands.startEnd(onStart, onEnd, requirements...)`

Runs `onStart` once when the command starts, runs `onEnd` once when it ends. Use for toggle-like actions where you set something on and clear it off.

```java
// While held: set distance preset to 10 ft. On release: clear preset.
m_operatorController.pov(90).whileTrue(
    Commands.startEnd(
        () -> shooter.setDistancePreset(10.0),
        shooter::clearDistancePreset
    )
);
```

### Subsystem Command Factory Methods

Subsystems can define methods that **return** a `Command`. This keeps command logic co-located with the hardware it controls.

```java
// In Feeder.java
public Command intakeCommand() {
    return Commands.run(() -> {
        intakeMotor.set(INTAKE_SPEED);
        triggerMotor.set(TRIGGER_INTAKE_SPEED);
    }, this)
    .finallyDo(() -> stopAll());
}

// In RobotContainer.java — clean, readable
m_operatorController.leftBumper().whileTrue(feeder.intakeCommand());
```

The `this` in `.run(..., this)` declares the subsystem requirement — the scheduler sees that this command needs `Feeder` and manages conflicts automatically.

### `.finallyDo(action)`

Attaches cleanup code that runs when a command ends, whether it finished normally or was cancelled. This ensures motors are always stopped:

```java
Commands.run(shooter::resolveDistanceAndSpin, shooter)
    .finallyDo(interrupted -> shooter.stopShooter());
```

---

## Command Composition

Multiple commands can be combined into sequences and parallel groups.

### `Commands.sequence(c1, c2, c3, ...)`

Runs commands **one after another**. Each command runs to completion before the next starts.

```java
// PathPlanner auto: spin up, wait for speed, then feed
Commands.sequence(
    Commands.run(shooter::resolveDistanceAndSpin, shooter)
        .until(shooter::isAtTargetSpeed)
        .withTimeout(2.0),
    Commands.run(() -> {
        shooter.resolveDistanceAndSpin();
        feeder.startFeed();
    }, shooter, feeder)
    .withTimeout(8.0)
).finallyDo(interrupted -> {
    shooter.stopShooter();
    feeder.stopAll();
})
```

### `.until(condition)`

Ends a `run()` command when a condition becomes true (turns it from "runs forever" to "runs until ready").

```java
Commands.run(shooter::resolveDistanceAndSpin, shooter)
    .until(shooter::isAtTargetSpeed)
```

### `.withTimeout(seconds)`

Ends a command after a maximum time, regardless of `isFinished()`. Prevents getting stuck.

```java
.withTimeout(2.0)  // give shooter at most 2 seconds to spin up
```

### `Commands.waitSeconds(seconds)` / `Commands.waitUntil(condition)`

Pauses a sequence.

```java
NamedCommands.registerCommand("Wait3Sec", Commands.waitSeconds(3.0));
```

---

## Button Binding Methods

These methods on `CommandXboxController` determine *when* a command is scheduled:

| Method | Behavior |
|--------|---------|
| `.onTrue(command)` | Schedules command once when button is pressed |
| `.whileTrue(command)` | Schedules when pressed, cancels when released |
| `.toggleOnTrue(command)` | First press schedules, second press cancels |
| `.onFalse(command)` | Schedules when button is released |

Examples from `RobotContainer.java`:

```java
// Toggle — press Y once to spin up, press again to stop
m_operatorController.y().toggleOnTrue(shooter.spinUpCommand());

// Hold — feed while RT is held, stop when released
m_operatorController.rightTrigger().whileTrue(feeder.shootCommand());

// One-shot — re-seed pose when Back is pressed
m_driverController.back().onTrue(
    Commands.runOnce(() -> drivetrain.initializePose(null), drivetrain)
);
```

---

## Named Commands for PathPlanner

PathPlanner autos can trigger commands by name using event markers. To make a command available by name in PathPlanner, register it with `NamedCommands.registerCommand()` in `RobotContainer.configureDefaultCommands()`:

```java
NamedCommands.registerCommand("Shoot5Sec", Commands.sequence(...));
NamedCommands.registerCommand("StartIntake", Commands.runOnce(...));
```

Named commands must be registered **before any auto is run** (i.e., before `autonomousInit()`). Registration in `configureDefaultCommands()` — which is called from the `RobotContainer` constructor — satisfies this requirement.

---

## When to Use a Separate Command Class

Create a class in `commands/` when:
- The command has **multiple internal states** that need tracking (e.g., a multi-phase scoring sequence with state variables).
- The command is **long enough** that keeping it inline would make `RobotContainer` hard to read.
- The command is **reused** in many places across the codebase.

For everything else, prefer inline lambdas or subsystem command factory methods. The `commands/` directory exists but is effectively unused in this codebase — that is intentional, not an oversight.
