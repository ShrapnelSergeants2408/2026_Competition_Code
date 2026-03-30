# FRC Team 2408 — REBUILT 2026 Competition Code

This repository contains the competition robot code for **FRC Team 2408** for the **2026 FIRST Robotics Competition season: REBUILT™ presented by Haas**.

---

## The Game: REBUILT™

REBUILT is a game where two three-robot alliances compete to score **fuel** (5.91-inch foam balls) into their **hub**, cross obstacles, and climb a **tower** at the end of the match.

### Match Structure

| Period | Duration | Goal |
|--------|----------|------|
| Autonomous (Auto) | 20 seconds | Robots operate without driver input — score fuel, move, optionally climb |
| Teleop | 2 min 20 sec | Drivers control robots — score fuel into hub when it is active |
| End Game | Final seconds | All hubs active; robots attempt tower climbs for bonus points |

### Key Field Elements

- **Hub** — One per alliance. A 47"×47" rectangular structure with a 41.7" hexagonal opening 72" off the ground. Fuel scored here is redistributed to the neutral zone through four exits at the base. Hub light bars indicate whether it is currently *active* (accepting scores) or *inactive*.
- **Tower** — One per alliance, integrated into the alliance wall. Three rungs at 27" (Low), 45" (Mid), and 63" (High) for end-game climbing.
- **Depot** — A low barrier structure along the alliance wall holding extra fuel for robots to collect.
- **Outpost** — Assembly at the alliance end through which human players feed fuel onto the field. Robots can also deliver fuel back to human players here.
- **Trench** — Structures along the field edges that robots can drive beneath.
- **Bumps** — Orange HDPE ramps on either side of the hub that robots drive over.

### Field Zones

- **Alliance Zone** — Behind the robot starting line, contains the tower and depot.
- **Neutral Zone** — Center of the field, contains the bumps, trenches, and hubs.
- **Offensive Zone** — The portion of the field between an alliance's driver station wall and their hub. This is where shooting is most effective.

### Scoring

Points are earned for fuel scored in the hub, tower climbs, and meeting scoring thresholds (Energized and Supercharged ranking points).

### How Team 2408 Played

- **Auto:** Five starting positions (Trench Left, Bump Left, Center, Bump Right, Trench Right) at 7.5 ft or 10 ft from the starting line. Autos drove forward, shot pre-loaded fuel, then proceeded to the depot, outpost, or neutral zone depending on strategy.
- **Teleop:** Primary goal was to intake as much fuel as possible and shoot/score during active hub periods. The team also spent significant match time **herding fuel** into the alliance offensive zone, either to outpost human players or to alliance partner robots.
- **Climbing:** Not implemented for the 2026 season.

---

## Robot Capabilities

| Capability | Status | Notes |
|-----------|--------|-------|
| Tank (differential) drive | Implemented | 4 NEO motors, SparkMax controllers |
| Distance-based shooting | Implemented | TalonFX flywheel, 7-point RPM interpolation table |
| Ball intake/feed/eject | Implemented | SparkMax intake roller + trigger/hopper |
| Vision pose estimation | Implemented (limited) | Dual PhotonVision cameras; see Vision Known Issues below |
| PathPlanner autonomous | Implemented | 25 routines, 5 starting positions |
| Tower climbing | Not implemented | Planned but not completed for 2026 |

### Vision Known Issues

The vision subsystem was coded and integrated but **did not function reliably during competition matches**. During testing, cameras showed as connected, but the Driver Station regularly reported errors for `Front_Camera` and `Rear_Camera` not transmitting data. The underlying cause was not determined due to time constraints. Possible causes include:

- Camera disconnects during robot brownouts (voltage sag tripping the Raspberry Pi)
- PhotonVision pipelines not being refreshed or applied correctly at robot startup
- Network Tables connection instability between the coprocessors and the roboRIO

Future teams should verify PhotonVision pipeline configuration, camera power wiring, and coprocessor boot reliability before relying on vision during matches.

---

## Repository Structure

```
2026_Competition_Code/
├── README.md                        ← You are here
├── repo.md                          ← Repo structure, libraries, git workflow
├── WPILib-License.md                ← Required BSD license (do not remove)
├── build.gradle / settings.gradle   ← Gradle build system (rarely touched)
├── gradlew / gradlew.bat            ← Gradle wrapper scripts
├── vendordeps/                      ← External library manifests
└── src/
    └── main/
        ├── deploy/
        │   └── pathplanner/
        │       ├── pathplanner.md   ← PathPlanner paths, autos, how-to
        │       ├── choreo.md        ← Choreo alternative trajectory tool
        │       ├── settings.json    ← Robot config for PathPlanner GUI
        │       ├── autos/           ← 25 complete autonomous routines
        │       └── paths/           ← Individual reusable path segments
        └── java/frc/robot/
            ├── robot.md             ← Command-based architecture overview
            ├── Constants.java       ← All robot-wide constants
            ├── RobotContainer.java  ← Subsystem wiring, bindings, auto chooser
            ├── Result.java          ← Test result utility class
            ├── VisionMeasurement.java ← Vision data record
            ├── subsystems/
            │   ├── subsystems.md    ← Subsystem documentation
            │   ├── DriveTrain.java
            │   ├── Shooter.java
            │   ├── Feeder.java
            │   └── Vision.java
            └── commands/
                └── commands.md      ← Command pattern documentation
```

See [repo.md](./repo.md) for a detailed explanation of each directory and file.

---

## Getting Started

### Prerequisites

- **WPILib 2026** — Install from [docs.wpilib.org](https://docs.wpilib.org). This includes VS Code with WPILib extensions, Java 17, and Gradle.
- **Git** — For version control. Install from [git-scm.com](https://git-scm.com).
- **REV Hardware Client** — For configuring SparkMax motor controllers.
- **Phoenix Tuner X** — For configuring TalonFX (Kraken/Falcon) motor controllers.
- **PhotonVision** — Running on coprocessors (Raspberry Pi) attached to the robot.

### Building

Open the project in WPILib VS Code. To build without deploying:

```
Ctrl+Shift+P → WPILib: Build Robot Code
```

Or from a terminal in the project root:

```bash
./gradlew build          # Mac/Linux
gradlew.bat build        # Windows
```

### Deploying to the Robot

Connect to the robot over USB or Wi-Fi, then:

```
Ctrl+Shift+P → WPILib: Deploy Robot Code
```

Or from a terminal:

```bash
./gradlew deploy
```

---

## Source Control Conventions

We use **Git** with **GitHub** for version control. See [repo.md](./repo.md) for a full explanation of the workflow.

**Quick reference:**

| Convention | Rule |
|-----------|------|
| Branch naming | `INITIALS-Focus` — e.g., `TP-Drivetrain`, `JD-Shooter` |
| Start of each session | Pull latest changes from `main` |
| During session | Commit regularly with meaningful messages |
| End of session | Push all commits to remote |
| Merging to main | Open a pull request; code must be tested first |

---

## Libraries

This project uses the following external libraries. See [repo.md](./repo.md) for descriptions of all installed libraries including those not currently in use.

| Library | Used | Purpose |
|---------|------|---------|
| WPILib | Yes | Core FRC robot framework |
| REVLib | Yes | SparkMax motor controllers |
| Phoenix 6 | Yes | TalonFX (Kraken) motor controllers |
| PhotonLib | Yes | AprilTag vision pose estimation |
| PathPlannerLib | Yes | Autonomous path following |
| AdvantageKit | Yes | Data logging and replay |
| ChoreoLib | No | Alternative trajectory tool (see choreo.md) |
| YAGSL | No | Swerve drive library (if swerve is added) |
| YAMS | No | Swerve drive library alternative |

---

## Documentation Policy

- **Prefer markdown** for theory, architecture, and process explanations.
- **Use Javadoc** on classes and methods to explain purpose and parameters.
- **Avoid large multi-line block comments** inside method bodies; prefer clear naming.
- **No directly AI-generated Java code.** AI may be consulted for implementation ideas, but all Java code must be written by a team member.

---

## Key Documentation Links

| Topic | File |
|-------|------|
| Repository structure & git workflow | [repo.md](./repo.md) |
| Command-based architecture | [robot.md](src/main/java/frc/robot/robot.md) |
| Subsystems (DriveTrain, Shooter, etc.) | [subsystems.md](src/main/java/frc/robot/subsystems/subsystems.md) |
| Commands pattern | [commands.md](src/main/java/frc/robot/commands/commands.md) |
| PathPlanner paths & autos | [pathplanner.md](src/main/deploy/pathplanner/pathplanner.md) |
| Choreo (alternative to PathPlanner) | [choreo.md](src/main/deploy/pathplanner/choreo.md) |
| WPILib | [docs.wpilib.org](https://docs.wpilib.org) |
| REV Robotics (SparkMax) | [docs.revrobotics.com](https://docs.revrobotics.com) |
| CTRE Phoenix 6 (TalonFX) | [pro.docs.ctr-electronics.com](https://pro.docs.ctr-electronics.com) |
| PhotonVision | [docs.photonvision.org](https://docs.photonvision.org) |
| PathPlanner | [pathplanner.me](https://pathplanner.me) |
| AdvantageKit | [docs.advantagekit.org](https://docs.advantagekit.org) |
