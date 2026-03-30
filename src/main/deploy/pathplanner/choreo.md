# Choreo — Alternative Trajectory Tool

This document describes **Choreo**, an alternative to PathPlanner for generating autonomous trajectories. ChoreoLib is already installed as a vendordep in this project, so no additional installation is required to start using it.

---

## What Is Choreo?

**Choreo** (full name: Choreo Trajectory Optimizer) is a trajectory generation tool developed by SleipnirGroup. Unlike PathPlanner — which follows user-defined waypoints and applies motion constraints post-hoc — Choreo generates **time-optimal trajectories** using direct collocation optimization. This means Choreo mathematically finds the fastest possible path through your waypoints given the robot's physical limits.

Choreo is particularly useful for:
- Maximizing auto speed (less wasted time)
- Precise, repeatable trajectories with tight tolerances
- Robots where PathPlanner's path-following controller is producing undesirable oscillation or spin

---

## PathPlanner vs. Choreo: Comparison

| Feature | PathPlanner | Choreo |
|---------|------------|--------|
| **Path creation** | Visual GUI, drag-and-drop waypoints | Visual GUI, waypoint-based |
| **Trajectory type** | Spline-based, constrained | Time-optimal (mathematically optimized) |
| **Speed** | May not be globally optimal | Provably fastest given robot limits |
| **Differential drive support** | Full support | Full support |
| **Swerve support** | Full support | Full support |
| **Event markers** | Yes (named commands at position/time) | Yes |
| **Integration complexity** | Moderate (AutoBuilder pattern) | Moderate (SwerveAutoFactory or differential helpers) |
| **Tuning required** | PPLTVController Q/R matrices | Less controller tuning needed |
| **File format** | `.path` / `.auto` (JSON) | `.traj` (JSON) |
| **Already installed** | Yes (PathPlannerLib) | Yes (ChoreoLib) |

**When to choose PathPlanner:** You want a quick, visual workflow and are comfortable with the AutoBuilder pattern already set up in this codebase.

**When to choose Choreo:** PathPlanner's paths are not smooth or precise enough, you are losing time to non-optimal trajectories, or you are experiencing control instability like the end-of-path spinning issue documented in [pathplanner.md](./pathplanner.md).

---

## Installation

ChoreoLib is **already installed** — see `vendordeps/ChoreoLib2026.json`. No additional steps are needed in the robot code.

### Install the Choreo GUI

Download the Choreo desktop application from [choreo.autos](https://choreo.autos). Choreo is available for Windows, macOS, and Linux.

---

## Creating a Trajectory in Choreo

### 1. Open Choreo and Create a Project

1. Open the Choreo application.
2. Click **"New Project"**.
3. Select the field for the current year (2026 REBUILT).
4. Configure your robot's physical properties:
   - **Drive type:** Differential (Tank)
   - **Mass:** 61.235 kg (from `settings.json`)
   - **MOI:** 6.883 kg·m²
   - **Wheel radius:** 0.0762 m (3 inches)
   - **Track width:** 0.546 m
   - **Max drive velocity:** 4.572 m/s (15 ft/s)
   - **Max drive acceleration:** 2.0 m/s²

### 2. Place Waypoints

1. Click on the field to place **waypoints** (control points the robot passes through).
2. Each waypoint has:
   - **Position** (x, y on the field)
   - **Heading** (robot facing direction)
   - **Constraint options** (stop point, max velocity override)

3. For differential drive, the robot's heading must match its travel direction — it cannot strafe. Choreo enforces this automatically.

### 3. Add Constraints

Right-click on a waypoint to add constraints:
- **Stop point** — robot comes to a complete stop (useful between pickup and shoot)
- **Max velocity** — limit speed through a section (e.g., near a field element)

### 4. Generate the Trajectory

Click **"Generate"**. Choreo runs the optimizer and produces a `.traj` file containing time-stamped states (position, velocity, acceleration, heading) at each control point.

Save the `.traj` file to `src/main/deploy/choreo/` (create this directory if it does not exist — it will be deployed to the roboRIO automatically).

### 5. Add Event Markers

Click on the trajectory timeline to add **event markers** at specific timestamps. These trigger named commands (same names as PathPlanner's `NamedCommands` map) at that moment during auto.

---

## Integrating Choreo into Robot Code

ChoreoLib provides helper classes to follow trajectories. Below is a minimal example for a differential drive using ChoreoLib's trajectory follower:

### Import the Library

```java
import choreo.Choreo;
import choreo.trajectory.DifferentialSample;
import choreo.trajectory.Trajectory;
import choreo.auto.AutoFactory;
```

### Create an AutoFactory (in DriveTrain or RobotContainer)

```java
// In RobotContainer constructor — after AutoBuilder is already configured:
AutoFactory choreoFactory = Choreo.createAutoFactory(
    drivetrain,
    drivetrain::getPose,
    // Left and right speed controller (PID or feedforward)
    (leftSampleSpeeds, rightSampleSpeeds) -> drivetrain.driveRobotRelative(
        new ChassisSpeeds(leftSampleSpeeds, 0, rightSampleSpeeds)
    ),
    () -> {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    },
    new AutoBindings()  // register named commands here
);
```

### Load and Follow a Trajectory

```java
// Load by name (matches the .traj filename without extension)
var trajectory = Choreo.loadTrajectory("my_auto");

// Create a command that follows it
Command followCommand = choreoFactory.trajectoryCommand("my_auto");
```

### Register Named Commands with AutoBindings

```java
AutoBindings bindings = new AutoBindings();
bindings.bind("Shoot5Sec", Commands.sequence(...));
bindings.bind("StartIntake", Commands.runOnce(...));

AutoFactory choreoFactory = Choreo.createAutoFactory(..., bindings);
```

---

## Coexisting with PathPlanner

ChoreoLib and PathPlannerLib can coexist in the same project. You can use PathPlanner for some autos and Choreo for others. There is no conflict between the libraries — they use different file formats and separate auto-building APIs.

If you decide to migrate primarily to Choreo, note that `AutoBuilder.configure()` in `DriveTrain.java` is specific to PathPlanner. You would keep it in place for any remaining PathPlanner autos, or remove it if fully migrating.

---

## Further Reading

- Choreo documentation: [choreo.autos/docs](https://choreo.autos/docs)
- ChoreoLib GitHub: [github.com/SleipnirGroup/Choreo](https://github.com/SleipnirGroup/Choreo)
- WPILib trajectory following overview: [docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/index.html)
