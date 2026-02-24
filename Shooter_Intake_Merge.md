# Shooter + Intake Merge Plan (hieb-trial)

## Branch snapshots (what exists now)
- **hieb-trial:** `Shooter.java` (SparkMAX shooter + feeder, optional PID/tuning, SmartDashboard toggles), `Intake.java` is an empty stub. `Constants` has shooter IDs 30/31, minimal intake constants, `RobotContainer` only instantiates shooter/intake (no bindings).
- **shooter branch:** Two shooters (`Shooter.java` simple REV open-loop + `ShooterSubsystem.java` TalonFX shooter + SparkMAX feeder + light sensor + distance→RPM table). `Constants` add SensorConstants + RPM map + current limits (IDs 30/31). `RobotContainer` is messy (duplicates Shooter + ShooterSubsystem, DriveTrain without Vision).
- **intake branch:** Intake with beam-break ball sensor, current-spike jam clear, intake/eject commands, telemetry. ShooterSubsystem same as shooter branch. `Constants` define intake speeds/current limits/sensors (motor ID 42, BALL_SENSOR_DIO_PORT/LIGHT_SENSOR_DIO_PORT both 0, EJECT_SPEED = 0 bug), shooter IDs 31/32, PID gains set (KP=1). `RobotContainer` duplicates subsystems and binds shooter/intake controls.
- **Conflicts to expect:** CAN IDs (30/31 vs 31/32 vs 42), SensorConstants overlaps, REV API version (`com.revrobotics.spark` vs older `com.revrobotics`), duplicated subsystems in `RobotContainer`, DriveTrain ctor signature (hieb-trial needs Vision).

## Hardware decisions (resolved — gates all coding phases)
| Item | Decision |
|---|---|
| Shooter motor | CAN ID 30, TalonFX, Phoenix 6 on-controller velocity PID |
| Feeder/intake motor | CAN ID 40, single SparkMAX (doubles as both intake and feeder) |
| Shooter inversion | `SHOOTER_INVERTED = false` — verify polarity on bench |
| Feeder inversion | `FEEDER_INVERTED = false` — verify polarity on bench |
| Photo sensor | DIO port 1, included in code but `PHOTO_SENSOR_ENABLED = false` until installed |
| Sensor polarity | `PHOTO_SENSOR_INVERTED = false` — verify on bench |
| Driver controller | Port 0, drive motions only |
| Operator controller | Port 1, all intake and shooting operations |
| Velocity PID location | Phoenix 6 on-controller (not Rio-side); SmartDashboard tuning in test mode |

## Execution order
Phases should be executed in this order to avoid rework:
> **Phase 4** (constants) → **Phase 1** (pull + normalize) → **Phase 2** (unified subsystem) → **Phase 3** (RobotContainer) → **Phase 6** (validation)
> Phase 5 bugs are integrated inline into Phases 2 and 4 — not a separate pass.

---

## Phase 4 — Constants consolidation *(complete)*
**Status: complete** — hardware decisions above are applied to `Constants.java` on `hieb-trial`.

Confirmed final constants:
- `SHOOTER_MOTOR_ID = 30` (TalonFX, Phoenix 6)
- `FEEDER_MOTOR_ID = 40` (SparkMAX, replaces old 31 and intake-branch 42)
- `SHOOTER_INVERTED = false`, `FEEDER_INVERTED = false` (flip in Constants if bench test shows wrong direction)
- `SHOOTER_CURRENT_LIMIT = 40` A stator limit (TalonFX), `FEEDER_CURRENT_LIMIT = 30` A smart limit (SparkMAX)
- `PHOTO_SENSOR_DIO_PORT = 1`, `PHOTO_SENSOR_ENABLED = false`, `PHOTO_SENSOR_INVERTED = false`
- `DRIVER_CONTROLLER_PORT = 0`, `OPERATOR_CONTROLLER_PORT = 1`
- Removed redundant `STALL_LIMIT` (was duplicate of `FEEDER_CURRENT_LIMIT`)
- Removed `SHOOTER_SPEED` (TalonFX uses velocity targets, not duty cycle), `SHOOTER_TARGET_RPM`, `SHOOTER_USE_PID`
- `IntakeConstants` left as empty shell — no separate intake subsystem; all intake behavior lives in `ShooterConstants`
- `SensorConstants` added as a new centralized class

Remaining items when other branches are merged:
- Confirm `KP/KI/KD/KV` values once TalonFX PID slot is exercised on bench
- Add `EJECT_SPEED` constant — set to negative of intake forward speed, **not zero** (intake branch bug fixed here)
- Add jam-detection constants: `FEEDER_SPIKE_THRESHOLD_AMPS`, `FEEDER_JAM_CLEAR_SECONDS`
- Verify distance→RPM map against actual shooter once hardware is running

---

## Phase 1 — Pull branch sources onto hieb-trial (fix obvious conflicts)
- Bring reference files from both branches without overwriting yet. Use a project-relative scratch location to avoid `/tmp/` issues on Windows:
  ```
  git show shooter:src/.../ShooterSubsystem.java > _scratch/shooter_subsystem.java
  git show intake:src/.../Intake.java > _scratch/intake.java
  ```
  Delete `_scratch/` after the merge is complete.
- Vendordeps confirmed present: `REVLib.json` (new `com.revrobotics.spark.*` API), `Phoenix6-26.1.1.json` (TalonFX). Phoenix 5 is installed but should not be used for new code.
- Normalize all imports to current vendor libs: use the new REV API (`com.revrobotics.spark.*`) everywhere; keep Phoenix 6 (`com.ctre.phoenix6.*`) for TalonFX.
- Clean up `RobotContainer` duplicates immediately: retain hieb-trial vision-first drivetrain/auto chooser structure; drop extra Shooter/ShooterSubsystem/Intake instances while keeping the binding logic for later adaptation.

---

## Phase 2 — Build one unified `Shooter` subsystem (owns shooter + intake/feeder)
- **Hardware layout (confirmed):** TalonFX (ID 30) drives the shooter wheel. Single SparkMAX (ID 40) drives the feeder/intake — same motor, same wheel, direction determines intake vs feed vs eject.
- **Control model:** TalonFX uses Phoenix 6 `VelocityVoltage` request with on-controller PID slot 0 (kP/kI/kD/kV from `ShooterConstants`). SmartDashboard tuning of PID gains gated to test mode. Distance→RPM table feeds the velocity target. SparkMAX feeder runs open-loop duty cycle.
  - **Bug fix inline:** `isAtTargetSpeed()` in shooter-branch ShooterSubsystem uses cached target instead of actual velocity — rewrite to read TalonFX `getVelocity().getValueAsDouble()` converted to RPM.
- **State machine:** Explicit states: Idle, SpinUp, Feed, Intake, Eject, JamClear. Feeder/intake runs: positive for intake, positive-slower for feed, negative for eject/jam-clear. Feed only runs when shooter is at speed AND (ball detected OR sensor disabled).
- **Ball detection:** Photo sensor gated by `PHOTO_SENSOR_ENABLED`. When disabled, `hasBall()` returns `false` so shoot always attempts feed on command. When enabled: read `DigitalInput` on DIO port 1 with `PHOTO_SENSOR_INVERTED` polarity.
  - **Bug fix inline:** Shooter/Intake-branch SensorConstants both set DIO = 0 — resolved, port 1 is now authoritative.
- **Jam handling:** Current-spike detection on SparkMAX (`getOutputCurrent() > FEEDER_SPIKE_THRESHOLD_AMPS`) with time-based debounce using `Timer.getFPGATimestamp()` (not loop-count-based).
  - **Bug fix inline:** `SPIKE_DEBOUNCE_CYCLES` comment vs code mismatch from intake branch — switch to elapsed seconds.
- **Commands:** Provide: `IntakeCommand` (run feeder forward while held), `EjectCommand` (run feeder negative while held), `ShootCommand` (spin up shooter, feed when ready), `StopAllCommand`. Keep `canShoot()` readiness gate. POV bindings set RPM target from distance presets.
  - **Bug fix inline:** Intake-branch `EJECT_SPEED = 0` makes eject a no-op — set to negative intake forward speed in Phase 4 constants.
- **Feeder config:** SparkMAX feeder uses its own current limit (`FEEDER_CURRENT_LIMIT`) and inversion flag (`FEEDER_INVERTED`) separate from shooter config.
  - **Bug fix inline:** Shooter-branch used same current limit for both shooter and feeder — now separate constants.
- **Telemetry & safety:** Publish shooter RPM, target RPM, feeder state, ball detected, jam status, feeder current draw. Idle: coast for shooter, brake for feeder (holds ball in place).

---

## Phase 3 — Wire `RobotContainer`
- Instantiate only the unified `Shooter` (no standalone `Intake`). Keep `Vision` → `DriveTrain` construction from hieb-trial.
- **Driver controller (port 0):** Drive motions only. Back = toggle tank/arcade. Start = toggle field-oriented. No shooter/intake bindings.
- **Operator controller (port 1):** All intake/shooting operations:
  - X = shoot (spin up + feed when ready), B = stop all, Y = manual feed
  - RT = feed-when-ready (hold), LT = intake (hold), LB = eject (hold)
  - POV up/down/left/right = distance presets for RPM target
- Remove all legacy duplicate subsystem fields. Re-add auto chooser wiring already present in hieb-trial.
- Ensure drivetrain default command unchanged.
  - **Bug fix inline:** Shooter-branch RobotContainer lacks vision-aware drivetrain — keep hieb-trial structure only.

---

## Phase 6 — Validation
- **Build (simulation):** `./gradlew build` — fix compile errors from API/ID changes first. No real hardware needed.
- **Bench tests (robot connected):** Verify motor directions (flip `SHOOTER_INVERTED`/`FEEDER_INVERTED` in Constants if wrong), current limits, jam clear timing, photo sensor polarity (`PHOTO_SENSOR_INVERTED`). Enable sensor by setting `PHOTO_SENSOR_ENABLED = true` once installed.
- **PID tuning:** Use SmartDashboard in test mode to tune kV first (feedforward sets baseline RPM), then kP for error correction. Validate distance presets hit expected RPM from the table.
- **Driver station dry run:** Confirm operator bindings execute correct commands. Confirm intake cannot feed unless shooter is at speed (unless overriding eject). Confirm driver controller has no unintended shooter effects.
