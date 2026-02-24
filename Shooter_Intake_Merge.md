# Shooter + Intake Merge Plan (hieb-trial)

## Branch snapshots (what exists now)
- **hieb-trial:** `Shooter.java` (SparkMAX shooter + feeder, optional PID/tuning, SmartDashboard toggles), `Intake.java` is an empty stub. `Constants` has shooter IDs 30/31, minimal intake constants, `RobotContainer` only instantiates shooter/intake (no bindings).
- **shooter branch:** Two shooters (`Shooter.java` simple REV open-loop + `ShooterSubsystem.java` TalonFX shooter + SparkMAX feeder + light sensor + distance→RPM table). `Constants` add SensorConstants + RPM map + current limits (IDs 30/31). `RobotContainer` is messy (duplicates Shooter + ShooterSubsystem, DriveTrain without Vision).
- **intake branch:** Intake with beam-break ball sensor, current-spike jam clear, intake/eject commands, telemetry. ShooterSubsystem same as shooter branch. `Constants` define intake speeds/current limits/sensors (motor ID 42, BALL_SENSOR_DIO_PORT/LIGHT_SENSOR_DIO_PORT both 0, EJECT_SPEED = 0 bug), shooter IDs 31/32, PID gains set (KP=1). `RobotContainer` duplicates subsystems and binds shooter/intake controls.
- **Conflicts to expect:** CAN IDs (30/31 vs 31/32 vs 42), SensorConstants overlaps, REV API version (`com.revrobotics.spark` vs older `com.revrobotics`), duplicated subsystems in `RobotContainer`, DriveTrain ctor signature (hieb-trial needs Vision).

## Phase 1 — Pull branch sources onto hieb-trial (fix obvious conflicts)
- Create a working branch from `hieb-trial` (keep `git -c safe.directory=...` usage since global config fails).
- Bring reference files from both branches without overwriting yet: `git show shooter:src/.../ShooterSubsystem.java > /tmp/shooter_subsystem.java` etc. Do the same for intake `Intake.java`, Shooter constants, SensorConstants, and button bindings from `RobotContainer`.
- Normalize imports to the current vendor libs before staging: use the new REV API (`com.revrobotics.spark.*`) everywhere; if TalonFX stays, keep Phoenix6.
- Fold missing constants into `Constants.java` in a single pass (Shooter current limits/RPM map, SensorConstants, full IntakeConstants). Resolve CAN/DIO conflicts by choosing a single authoritative mapping after checking electrical notes.
- Clean up `RobotContainer` duplicates immediately: retain hieb-trial vision-first drivetrain/auto chooser structure; drop extra Shooter/ShooterSubsystem/Intake instances while keeping the binding logic for later adaptation.

## Phase 2 — Build one unified `Shooter` subsystem (owns shooter + intake/feeder)
- **Hardware/layout decisions:** Pick final hardware per mechanical/Electrical: (a) TalonFX shooter + SparkMAX feeder/intake, or (b) dual SparkMAX as in hieb-trial. Update motor IDs accordingly and remove unused device configs.
- **Control model:** Combine hieb-trial PID/tuning surface with shooter-branch distance→RPM map. Implement a single velocity controller (Spark PID or Talon velocity control) with SmartDashboard tuning gated to test mode. Ensure RPM comes from a real sensor (TalonFX velocity or Spark encoder).
- **State machine:** Manage shooter & feeder/intake together with explicit states (Idle, SpinUp, Feed, Intake, Eject, JamClear). Intake motor doubles as feeder; coordinate so feed only runs when shooter at speed & ball present.
- **Ball detection:** Merge light sensor (shooter branch) + beam-break (intake branch) into one `hasBall()` decision with configurable inversion and graceful null handling.
- **Jam handling:** Carry over intake branch spike detection & timed reverse, but make debounce time loop-aware (use counter with `Timer` or elapsed seconds).
- **Commands:** Provide commands for intake forward, eject, auto-shoot (spin-up then feed when ready), and manual stop. Keep readiness gates (`canShoot`) and integrate distance presets (POV) for RPM targets.
- **Telemetry & safety:** Publish RPM/targets, feeder state, ball detected, jam status, current draw. Add current limits per motor and ensure idle/brake modes match desired behavior.

## Phase 3 — Wire `RobotContainer`
- Instantiate only the unified `Shooter` (no standalone `Intake`). Keep `Vision` → `DriveTrain` construction from hieb-trial.
- Re-map controls using the new commands: X/B/Y for shoot/stop, RT for feed-when-ready, LT for intake, LB for eject, POV for distance presets. Remove legacy duplicate fields.
- Ensure default commands remain unchanged for drivetrain; re-add auto chooser wiring already present in hieb-trial.

## Phase 4 — Constants consolidation
- Decide final CAN/DIO IDs and stick to them (shooter branch 30/31 vs intake branch 31/32/42). Add comments noting hardware mapping and TODOs if unconfirmed.
- Merge ShooterConstants fields: current limits, RPM map, PID gains, nominal voltage, RPM tolerance. Merge IntakeConstants: speeds (forward, eject, jam reverse), spike thresholds, debounce timing, sensor inversion.
- Keep SensorConstants centralized; avoid overlapping ports; add separate constants for shooter light sensor vs intake beam break if both used.

## Phase 5 — Bugs/cleanup to fix during merge
- ShooterSubsystem `isAtTargetSpeed()` uses the cached target RPM (always “ready”); fix to read actual velocity.
- Intake branch `EJECT_SPEED` is 0 → eject command is a no-op; set to negative intake speed.
- `SPIKE_DEBOUNCE_CYCLES` comment says seconds but code uses loop counts; switch to time-based or document loop rate.
- Shooter uses same config for feeder/shooter current limit; give feeder its own limit and optional inversion.
- Shooter/Intake SensorConstants both set DIO = 0; choose distinct ports.
- Shooter branch RobotContainer lacks vision-aware drivetrain and duplicates subsystems; keep hieb-trial structure only.
- Normalize REV API usage (new `spark` package) to match current dependencies.

## Phase 6 — Validation
- Build checks: `./gradlew build` in simulation; fix compile errors from API/ID changes.
- Bench tests (robot): verify motor directions, current limits, jam clear timing, and ball sensor polarity. Tune PID via SmartDashboard in test mode; validate distance presets hit expected RPM.
- Driver station dry run: confirm bindings execute the correct commands and intake can’t feed unless shooter is ready (unless overriding eject).
