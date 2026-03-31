# PathPlanner Autonomous Paths

This document covers the PathPlanner autonomous system used by Team 2408 in the 2026 REBUILT season — the naming conventions, what each autonomous routine does, how to create and edit paths, and a known issue observed at competition.

---

## What Is PathPlanner?

**PathPlanner** is a path planning and autonomous framework for FRC robots. It provides:

1. **A GUI** (PathPlanner desktop app) to visually create and edit paths on a field image.
2. **PathPlannerLib** (the library integrated into robot code) to follow those paths using trajectory tracking.
3. **Named command events** that can trigger robot actions (like shooting or intaking) at specific points along a path.

PathPlanner is configured for this robot as a **differential drive** (tank drive). The robot configuration is stored in `settings.json` in this directory. The PathPlanner GUI reads this file to display the correct robot footprint and apply motion constraints.

### How It Integrates with the Robot

In `DriveTrain.java`, `AutoBuilder.configure()` is called once at startup. This registers the robot's current pose, speed reading, and drive functions with PathPlanner. When an auto routine runs, PathPlanner calls these functions directly to command the drivetrain.

The `PPLTVController` (Linear Time-Varying controller) handles path following. Its tuning parameters (`Q` and `R` matrices) are adjustable in test mode from SmartDashboard under `DriveTrain/PPLTV/`.

---

## File Structure

```
pathplanner/
├── settings.json       ← Robot configuration (dimensions, max speed, motor type)
├── navgrid.json        ← Field navigation grid for obstacle avoidance
├── autos/              ← Complete autonomous routines (.auto files)
└── paths/              ← Individual reusable path segments (.path files)
```

**`.path` files** define a single trajectory segment — a list of waypoints and constraints. They are reusable building blocks.

**`.auto` files** define a complete autonomous routine — a sequence of path segments and named command events composed together. Autos are what appear in the auto chooser on SmartDashboard.

---

## Naming Convention

All paths and autos follow a consistent naming pattern:

```
[POSITION] [DISTANCE] [OBJECTIVE]
```

### Starting Position Codes

| Code | Position on Field | Description |
|------|------------------|-------------|
| **TL** | Trench Left | Robot starts at the left trench opening |
| **BL** | Bump Left | Robot starts to the left of center, at the bump |
| **C** | Center | Robot starts at the center of the field |
| **BR** | Bump Right | Robot starts to the right of center, at the bump |
| **TR** | Trench Right | Robot starts at the right trench opening |

### Distance

The distance number refers to **how far the shooter is from the center of the hub at the shooting position** — not the robot's starting distance from the wall. Every auto that includes a shot drives the robot to a position where the shooter is exactly this far from the hub, rotates to align the shooter with the hub opening, and then fires.

- **10** — Shooter is 10 feet from the hub center at the shooting position
- **7.5** — Shooter is 7.5 feet from the hub center at the shooting position

These values feed directly into the distance-to-RPM interpolation table in `ShooterConstants`. A `SpinUpShooter` named command spins the flywheel to the correct RPM for that distance before the robot arrives at the shooting position, and a `Shoot` named command (or `Shoot5Sec`) feeds fuel for 8 seconds once the robot is in position (targeting approximately 8 fuel at ~1 fuel per second).

### Objective (End Goal)

| Code | Destination |
|------|------------|
| **fwd** | Drive straight forward only (no end-game objective) |
| **depot** | Navigate to the alliance depot |
| **neutral** | Navigate to the neutral zone |
| **outpost** | Navigate to the outpost (human player station) |

### Phase Prefixes

Some paths use a phase prefix when a multi-segment auto requires separate path files:

| Prefix | Meaning |
|--------|---------|
| (none) | Single-phase auto, or the complete p1 segment |
| **p2** | Second phase path — used after the initial movement |

**Example names:**
- `bl 10 fwd` — Start Bump Left, drive to shooting position 10 ft from hub, shoot, stop
- `tr 7.5 to depot` — Start Trench Right, drive to shooting position 7.5 ft from hub, shoot, then navigate to depot
- `p2 c 10 to depot` — Second phase path for the Center 10 ft auto, continuing to depot after shooting

---

## Autonomous Routines

All 25 autonomous routines available in the 2026 season:

### Auto Sequence

Every auto that includes a shot follows this sequence:
1. **Drive** from the starting line to the shooting position (shooter aligned with hub at the specified distance)
2. **SpinUpShooter** — flywheel ramps to the RPM for that distance while traveling (pre-spun before arrival)
3. **Shoot** or **Shoot5Sec** — feeds fuel into the shooter for 8 seconds (~8 fuel at 1/second)
4. **Drive** to the post-shot objective (depot, neutral zone, outpost), or stop if `fwd`

### Forward Only (Drive to Shooting Position, Shoot, Stop)

| Auto Name | Description |
|-----------|-------------|
| `bl 10 fwd` | Start at Bump Left starting line. Drive to shooting position 10 ft from hub, align shooter with hub, spin up and shoot. Stop — no post-shot movement. |
| `bl 7.5 fwd` | Start at Bump Left starting line. Drive to shooting position 7.5 ft from hub, align, spin up and shoot. Stop. |
| `br 10 fwd` | Start at Bump Right starting line. Drive to shooting position 10 ft from hub, align, spin up and shoot. Stop. |
| `br 7.5 fwd` | Start at Bump Right starting line. Drive to shooting position 7.5 ft from hub, align, spin up and shoot. Stop. |
| `c 10 fwd` | Start at Center starting line. Drive to shooting position 10 ft from hub, align, spin up and shoot. Stop. |
| `c 7.5 fwd` | Start at Center starting line. Drive to shooting position 7.5 ft from hub, align, spin up and shoot. Stop. |
| `tl 10 fwd` | Start at Trench Left starting line. Drive to shooting position 10 ft from hub, align, spin up and shoot. Stop. |
| `tr 10 fwd` | Start at Trench Right starting line. Drive to shooting position 10 ft from hub, align, spin up and shoot. Stop. |

### To Depot (Shoot, Then Navigate to Alliance Depot)

| Auto Name | Description |
|-----------|-------------|
| `bl 10 to depot` | Start at Bump Left. Drive to 10 ft shooting position, align, spin up and shoot. Then navigate to the alliance depot. |
| `bl 7.5 to depot` | Start at Bump Left. Drive to 7.5 ft shooting position, align, spin up and shoot. Then navigate to the alliance depot. |
| `br 10 to depot` | Start at Bump Right. Drive to 10 ft shooting position, align, spin up and shoot. Then navigate to the alliance depot. |
| `br 7.5 to depot` | Start at Bump Right. Drive to 7.5 ft shooting position, align, spin up and shoot. Then navigate to the alliance depot. |
| `tl 10 to depot` | Start at Trench Left. Drive to 10 ft shooting position, align, spin up and shoot. Then navigate to the alliance depot. |
| `tr 10 to depot` | Start at Trench Right. Drive to 10 ft shooting position, align, spin up and shoot. Then navigate to the alliance depot. |

### To Neutral Zone (Shoot, Then Move to Neutral Zone)

| Auto Name | Description |
|-----------|-------------|
| `bl 10 to neutral` | Start at Bump Left. Drive to 10 ft shooting position, align, spin up and shoot. Then move to the neutral zone. |
| `bl 7.5 to neutral` | Start at Bump Left. Drive to 7.5 ft shooting position, align, spin up and shoot. Then move to the neutral zone. |
| `br 10 to neutral` | Start at Bump Right. Drive to 10 ft shooting position, align, spin up and shoot. Then move to the neutral zone. |
| `br 7.5 to neutral` | Start at Bump Right. Drive to 7.5 ft shooting position, align, spin up and shoot. Then move to the neutral zone. |
| `c 7.5 to neutral` | Start at Center. Drive to 7.5 ft shooting position, align, spin up and shoot. Then move to the neutral zone. |
| `tl 10 to neutral` | Start at Trench Left. Drive to 10 ft shooting position, align, spin up and shoot. Then move to the neutral zone. |
| `tr 10 to neutral` | Start at Trench Right. Drive to 10 ft shooting position, align, spin up and shoot. Then move to the neutral zone. |
| `tr 10 to neutral (fix)` | Trench Right 10 ft to neutral — revised version of the path above with a corrected route (created during competition to address a path problem). |

### Center to Neutral (Multi-Route Variants)

These four autos all start at Center and shoot from 7.5 ft, but take different routes through the field to reach the neutral zone. The route suffix indicates which field obstacle the robot passes by.

| Auto Name | Description |
|-----------|-------------|
| `c 7.5 to neutral (lb)` | Start at Center. Shoot from 7.5 ft. Navigate to neutral zone via the left bump. |
| `c 7.5 to neutral (lt)` | Start at Center. Shoot from 7.5 ft. Navigate to neutral zone via the left trench. |
| `c 7.5 to neutral (rb)` | Start at Center. Shoot from 7.5 ft. Navigate to neutral zone via the right bump. |
| `c 7.5 to neutral (rt)` | Start at Center. Shoot from 7.5 ft. Navigate to neutral zone via the right trench. |

### To Outpost (Shoot, Then Navigate to Outpost)

| Auto Name | Description |
|-----------|-------------|
| `bl 10 to outpost` | Start at Bump Left. Drive to 10 ft shooting position, align, spin up and shoot. Then navigate to the outpost (human player station). |
| `br 10 to outpost` | Start at Bump Right. Drive to 10 ft shooting position, align, spin up and shoot. Then navigate to the outpost. |
| `tr 10 to outpost` | Start at Trench Right. Drive to 10 ft shooting position, align, spin up and shoot. Then navigate to the outpost. |

---

## How to Create or Edit Paths in PathPlanner

### 1. Install PathPlanner

Download the PathPlanner desktop app from the [PathPlanner GitHub releases page](https://github.com/mjansen4857/pathplanner/releases). Install and open it.

### 2. Open the Project

Open PathPlanner and point it to this project's `src/main/deploy/pathplanner/` directory. PathPlanner will read `settings.json` and load the existing paths and autos.

### 3. Understanding the GUI

- **Field view** — the main editing area showing the 2026 field image.
- **Waypoints** — blue dots on the path. Drag them to reposition. Each waypoint has:
  - **Position** (where the robot is)
  - **Heading** (which direction the robot is facing at that point)
  - **Control handles** (gray dots) — adjusting these changes the curve of the path.
- **Constraints** — maximum velocity and acceleration for path segments. Set lower on tight curves.
- **Event markers** — orange flags placed on the path timeline. These trigger `NamedCommands` at a specific position. Available commands are listed in [commands.md](../../java/frc/robot/commands/commands.md).

### 4. Creating a New Path

1. Click **"New Path"** in the left panel.
2. Name the path following the naming convention above.
3. Click on the field to place waypoints.
4. Adjust control handles to shape curves.
5. Set start and end headings.
6. Add event markers if needed (e.g., to trigger `SpinUpShooter` before arrival).
7. Save (`Ctrl+S`).

### 5. Creating a New Auto

1. Click **"New Auto"** in the left panel.
2. Name it following the naming convention.
3. Drag path segments from the left panel into the auto timeline.
4. Add named command events between or within path segments.
5. Set the starting pose (must match the robot's physical starting position).
6. Save.

### 6. Testing

After saving, rebuild and deploy the robot code. The new auto will appear in the SmartDashboard auto chooser. Test in an open area before competition.

---

## Known Issue: Robot Spins Multiple Times at End of Path Segments

### Symptom

During competition matches, the robot would **spin 2–3 full rotations** at the conclusion of each path segment before proceeding to the next step of the autonomous routine. This caused significant time loss and unpredictable robot positioning.

### Root Cause

PathPlanner paths on a differential drive include a **goal end state** for heading — the direction the robot should be facing when it reaches the final waypoint. If this heading is set to 0° (the default) but the robot is actually pointing at a different angle at the end of the path, the controller continues correcting the heading after the translation portion of the path is complete, causing the spinning behavior.

### Attempted Fix

A fix was implemented in commit `0f5e89a` (*"Reapply: adjusted path goalEndStates to match path angle instead of 0 to control spin"*). This change sets the `goalEndState` heading of each path to match the actual path exit angle rather than defaulting to 0°.

This fix was originally committed (`c51327c`), then reverted (`0fffe48`), then re-applied (`0f5e89a`). The final state of the repository has the fix applied. Full evaluation during a match was not possible within the season.

### Additional Approaches to Investigate

If spinning continues, future teams should investigate:

1. **Verify `goalEndState` headings in PathPlanner GUI** — Open each path and confirm the endpoint heading arrow matches the direction the robot is actually traveling at the end of the path. Mismatched headings are the most likely cause.

2. **PPLTVController `Qtheta` tuning** — The `Qtheta` parameter controls how aggressively the controller corrects heading error. Increasing it increases heading correction; reducing it makes the controller less aggressive about heading. Accessible in test mode at `DriveTrain/PPLTV/Qtheta` on SmartDashboard.

3. **Check path holonomic rotation targets** — In PathPlanner's path editor, each waypoint can have a "rotation target" that specifies what heading the robot should maintain while traveling through that point. For a differential drive, these may be interfering with the LTV controller's heading tracking.

4. **Consider Choreo** — Choreo generates time-optimal trajectories differently and may handle end-state heading more cleanly for differential drives. See [choreo.md](./choreo.md).
