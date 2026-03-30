# Repository Structure and Workflow

This document covers everything outside of the robot source code itself: how the repository is organized, what each tool and library does, and how the team uses Git and GitHub.

---

## Repository Layout

```
2026_Competition_Code/
├── .git/           ← Git version control metadata (never edit manually)
├── .vscode/        ← VS Code editor settings and WPILib task definitions
├── .wpilib/        ← WPILib project metadata (target platform, year)
├── .gradle/        ← Gradle download cache (auto-generated, not committed)
├── build/          ← Compiled class files (auto-generated, not committed)
├── bin/            ← Additional build output (auto-generated)
├── gradle/         ← Gradle wrapper files (committed — do not delete)
├── vendordeps/     ← External library manifest files
├── src/            ← All source code and deploy files
├── build.gradle    ← Build configuration
├── settings.gradle ← Project name
├── gradlew         ← Gradle wrapper script (Mac/Linux)
└── gradlew.bat     ← Gradle wrapper script (Windows)
```

### Hidden Directories

Files and directories starting with `.` are hidden on Mac/Linux (Windows may show them). You will almost never need to edit them:

- **`.git/`** — Git's internal database. Never edit this manually.
- **`.vscode/`** — VS Code settings including WPILib build and deploy tasks. If deploy stops working, check `launch.json` here.
- **`.wpilib/`** — Stores the WPILib project year and target platform. Checked by the WPILib extension to validate the project.
- **`.gradle/`** — Gradle's local cache for downloaded dependencies. Auto-regenerated. Safe to delete if builds behave strangely.

### Build System: Gradle

**Gradle** is the build and dependency management system used by WPILib. It works like `npm` in JavaScript or `pip` in Python — it downloads the libraries your project needs and knows how to compile and deploy the code.

You interact with Gradle through the **wrapper scripts** (`gradlew` / `gradlew.bat`), which download the correct Gradle version automatically. You should never need to install Gradle separately.

Common tasks (run from VS Code terminal in the project root):

```bash
./gradlew build         # Compile and check for errors (no deploy)
./gradlew deploy        # Compile and deploy to the connected roboRIO
./gradlew test          # Run any unit tests
```

The WPILib VS Code extension also provides `Ctrl+Shift+P → WPILib: Build Robot Code` and `WPILib: Deploy Robot Code` as GUI shortcuts that run the same commands.

**`build.gradle`** — Defines Java version (17), declares dependencies, and sets roboRIO deploy targets. Rarely needs editing.

**`settings.gradle`** — Just sets the project name. You will never need to edit this.

### `src/main/deploy/`

The contents of this directory are **copied onto the roboRIO** at `~/deploy/` when you run `gradlew deploy`. Libraries like PathPlanner look for their configuration files here at runtime. The PathPlanner GUI reads and writes to the `pathplanner/` subdirectory. See [pathplanner.md](src/main/deploy/pathplanner/pathplanner.md) for details.

### `WPILib-License.md`

This is the BSD open-source license inherited from the WPILib Command-Based project template. **Do not remove or modify this file.** It is a legal requirement of using the WPILib template.

---

## External Libraries (vendordeps)

External libraries are added by placing a `.json` manifest file in the `vendordeps/` directory. Gradle reads these manifests and downloads the actual library JAR files automatically. You can install a new vendor library by:
1. Using `Ctrl+Shift+P → WPILib: Manage Vendor Libraries → Install new libraries (online)` and pasting the JSON URL, **or**
2. Copying the `.json` file directly into `vendordeps/`.

The following libraries are installed in this project:

### Core / Always Used

| Library | Version | Purpose |
|---------|---------|---------|
| **WPILib New Commands** | 1.0.0 | The WPILib command-based framework itself — SubsystemBase, Command, CommandScheduler, controllers, kinematics, pose estimation. This is the foundation everything else is built on. |
| **REVLib** | 2026.0.5 | Official library for REV Robotics **SparkMax** and **SparkFlex** motor controllers. Used to configure and control the drive motors (CAN 20–23), intake roller (CAN 31), and trigger/hopper (CAN 32). |
| **CTRE Phoenix 6** | 26.1.3 | Official library for CTR Electronics **TalonFX** (Kraken X60/Falcon 500) motor controllers. Used for the shooter flywheel (CAN 30) with on-controller velocity PID. Phoenix 6 is the modern API; prefer it over Phoenix 5 for all new TalonFX work. |
| **PhotonLib** | v2026.3.2 | Client library for **PhotonVision** — the vision coprocessor software running on the Raspberry Pis. Used to read AprilTag detections and compute field-relative robot pose estimates. |
| **PathPlannerLib** | 2026.1.2 | Library for **PathPlanner** autonomous path following. Provides `AutoBuilder`, `NamedCommands`, and the `PPLTVController` used to follow trajectories with a differential drive. |
| **AdvantageKit** | 26.0.2 | Data logging and replay framework from FRC Team 6328. The robot extends `LoggedRobot` instead of `TimedRobot`. All sensor inputs and command outputs are logged to `.wpilog` files on the roboRIO for post-match replay and debugging. Docs: [docs.advantagekit.org](https://docs.advantagekit.org) |

### Installed, Not Currently Used

These libraries are installed in `vendordeps/` for potential future use. Removing an unused vendordep will not break the build — it just won't be available if you need it later.

| Library | Version | Purpose / When to Use |
|---------|---------|----------------------|
| **CTRE Phoenix 5** | 5.36.0 | Legacy API for older TalonSRX and Victor SPX motor controllers. Only needed if the robot uses those older CTRE devices. For TalonFX, use Phoenix 6 instead. |
| **ChoreoLib** | 2026.0.2 | Client library for **Choreo** — an alternative trajectory tool that generates time-optimal paths. See [choreo.md](src/main/deploy/pathplanner/choreo.md) for a comparison with PathPlanner. |
| **YAGSL** | 2026.3.12 | Yet Another Generic Swerve Library. A high-level library that greatly simplifies swerve drive implementation. If the team ever switches to swerve drive, start here. Docs: [docs.yagsl.com](https://docs.yagsl.com) |
| **YAMS** | 2026.3.11 | Yet Another Mechanism System — a companion library to YAGSL for configuring mechanisms with JSON files. Useful if using YAGSL. Docs: [yagsl.gitbook.io/yams](https://yagsl.gitbook.io/yams) |
| **DogLog** | 2026.5.0 | A simpler, annotation-based alternative to AdvantageKit for logging. If AdvantageKit proves too complex, DogLog is a lighter-weight option. Docs: [doglog.dev](https://doglog.dev) |
| **maple-sim** | 0.4.0-beta | Physics-based robot simulation library. Enables realistic simulation of robot motion, sensors, and game piece interactions in WPILib simulation without real hardware. |
| **Studica** | 2026.0.0 | Library for Studica hardware including the **NavX2** (AHRS gyroscope) which this robot uses. Note: the NavX is already working via this library — it is actively used even though the intent was to list it here as a reference. |
| **AndyMark AM Library (AmLib)** | 2026.0.3 | Library for AndyMark sensors and hardware components. Useful if the team purchases AndyMark encoders or pneumatics components. |
| **ReduxLib** | 2026.1.2 | Library for Redux Robotics hardware, including the **Canandgyro** (CAN-connected gyroscope) and the **Canandmag** (absolute encoder). Useful as an alternative or additional gyro/encoder option. |
| **ThriftyLib** | 2026.1.2 | Library for Thrifty Bot sensors and hardware. Relevant if using Thrifty encoders, sensors, or other hardware from that vendor. |
| **LumynLabs** | 2026.2.0 | Library for Lumyn Labs hardware and sensors. |
| **PlayingWithFusion** | 2026.3.05 | Library for Playing With Fusion sensors, including Time-of-Flight (ToF) distance sensors. Useful for detecting game pieces or measuring distances without vision. |
| **libgrapplefrc** | 2026.0.0 | Library for GrappleFRC hardware, including the Mitocan integrated motor controller. |

---

## Source Control with Git and GitHub

### Why Version Control?

Git tracks every change made to the code. This means:
- You can always recover a previous working version.
- Multiple people can work on different features simultaneously without overwriting each other.
- You can see exactly what changed, when, and why.

### Concepts

- **Repository (repo)** — The project folder tracked by Git, including all history.
- **Commit** — A saved snapshot of your changes with a message explaining what changed.
- **Branch** — An independent line of development. Changes on a branch don't affect `main` until they are merged.
- **Remote** — The copy of the repo on GitHub (`origin`). Your local copy and the remote stay in sync by pushing and pulling.
- **Pull Request (PR)** — A request on GitHub to merge your branch into `main`. Others can review the changes before they are merged.

### Branch Naming Convention

Each team member creates their own branch named:

```
INITIALS-Focus
```

Examples:
- `TP-Drivetrain` — Taaliyah Powell's work on the DriveTrain subsystem
- `JD-Shooter` — working on the Shooter subsystem
- `MR-Vision` — vision subsystem work

Create a new branch for each distinct feature or subsystem. Do not do unrelated work on the same branch.

### Session Workflow

Follow this pattern at every programming session:

**1. Start of session — pull latest changes from main**
```bash
git checkout main          # switch to main branch
git pull                   # download latest changes from GitHub
git checkout your-branch   # switch back to your branch
git merge main             # bring your branch up to date with main
```
Or if starting fresh work:
```bash
git checkout -b TP-NewFeature   # create and switch to a new branch
```

**2. During the session — commit regularly**

Commit every time you complete a logical unit of work. Do not wait until the end of the session — small, frequent commits make it easier to undo mistakes.

```bash
git add src/main/java/frc/robot/subsystems/DriveTrain.java
git commit -m "Add encoder position conversion factor to drive motors"
```

**Write meaningful commit messages.** A good commit message completes the sentence *"This commit will..."*

Good examples:
```
Add encoder position conversion factor to drive motors
Fix shooter RPM calculation using correct gear ratio
Implement jam detection debounce timer in Feeder
Register Shoot5Sec named command for PathPlanner autos
Tune PPLTV Q and R matrix values from test session
```

Bad examples:
```
fix stuff
wip
asdfgh
changes
```

**3. End of session — push to GitHub**
```bash
git push origin your-branch-name
```

If this is the first time pushing this branch:
```bash
git push -u origin your-branch-name
```

### Merging to Main

When your feature is complete and tested on the robot:

1. Push your final commits.
2. Go to GitHub and open a **Pull Request** from your branch to `main`.
3. Describe what you changed and how it was tested.
4. Once reviewed and approved, merge the PR.

**Never push directly to `main`.** All changes go through pull requests so they can be reviewed.

### Useful Git Commands

```bash
git status                    # see what files have changed
git diff                      # see what changed in tracked files
git log --oneline             # see recent commit history
git stash                     # temporarily save uncommitted changes
git stash pop                 # restore stashed changes
git checkout -- filename.java # discard changes to a specific file (careful!)
```
