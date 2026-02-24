# Shooter + Intake Merge Plan (hieb-trial)

## Branch snapshots (what existed at merge start)
- **hieb-trial:** `Shooter.java` (SparkMAX shooter + feeder, optional PID/tuning, SmartDashboard toggles), `Intake.java` is an empty stub. `Constants` has shooter IDs 30/31, minimal intake constants, `RobotContainer` only instantiates shooter/intake (no bindings).
- **shooter branch:** Two shooters (`Shooter.java` simple REV open-loop + `ShooterSubsystem.java` TalonFX shooter + SparkMAX feeder + light sensor + distance→RPM table). `Constants` add SensorConstants + RPM map + current limits (IDs 30/31). `RobotContainer` is messy (duplicates Shooter + ShooterSubsystem, DriveTrain without Vision).
- **intake branch:** Intake with beam-break ball sensor, current-spike jam clear, intake/eject commands, telemetry. ShooterSubsystem same as shooter branch. `Constants` define intake speeds/current limits/sensors (motor ID 42, BALL_SENSOR_DIO_PORT/LIGHT_SENSOR_DIO_PORT both 0, EJECT_SPEED = 0 bug), shooter IDs 31/32, PID gains set (KP=1). `RobotContainer` duplicates subsystems and binds shooter/intake controls.

## Hardware decisions (resolved)
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

---

## Phase 4 — Constants consolidation ✅ complete

Applied to `Constants.java` on `hieb-trial`:
- `SHOOTER_MOTOR_ID = 30` (TalonFX), `FEEDER_MOTOR_ID = 40` (SparkMAX)
- `SHOOTER_INVERTED = false`, `FEEDER_INVERTED = false`
- `SHOOTER_CURRENT_LIMIT = 40` A stator (TalonFX), `FEEDER_CURRENT_LIMIT = 30` A smart (SparkMAX)
- `PHOTO_SENSOR_DIO_PORT = 1`, `PHOTO_SENSOR_ENABLED = false`, `PHOTO_SENSOR_INVERTED = false`
- `DRIVER_CONTROLLER_PORT = 0`, `OPERATOR_CONTROLLER_PORT = 1`
- `INTAKE_SPEED = 0.5`, `FEEDER_SPEED = 0.6`, `EJECT_SPEED = -0.5`, `JAM_REVERSE_SPEED = -0.5`
- `FEEDER_SPIKE_THRESHOLD_AMPS = 35.0`, `JAM_REVERSE_TIME_SEC = 0.25`
- Removed: `STALL_LIMIT`, `SHOOTER_SPEED`, `SHOOTER_TARGET_RPM`, `SHOOTER_USE_PID`
- `SensorConstants` added as a centralized class; `IntakeConstants` left as empty shell

**Pending (bench):**
- Confirm `KP/KI/KD/KV` values once TalonFX PID slot is exercised on hardware
- Verify distance→RPM map against actual shooter output
- Flip `SHOOTER_INVERTED` / `FEEDER_INVERTED` if bench test shows wrong direction

---

## Phase 1 — Pull branch sources and normalize ✅ complete

- Reference files pulled from `shooter` and `Intake` branches into `_scratch/` (9 files)
- Vendordeps confirmed: `REVLib.json` (`com.revrobotics.spark.*`), `Phoenix6-26.1.1.json`. Phoenix 5 present but not used.
- All imports in new code use current REV API (`com.revrobotics.spark.*`) and Phoenix 6 (`com.ctre.phoenix6.*`)
- `RobotContainer` duplicate subsystems (shooter branch's `ShooterSubsystem`, `VisionSubsystem`, dual `Intake` fields) identified and removed in Phase 3

**Cleanup remaining:**
- Delete `_scratch/` folder — reference files no longer needed now that merge is code-complete

---

## Phase 2 — Unified `Shooter` subsystem ✅ complete

`Shooter.java` fully rewritten (~210 lines). All logic from `shooter` and `Intake` branches merged into a single subsystem:

| Area | Implementation |
|---|---|
| Shooter motor | TalonFX (ID 30), Phoenix 6 `VelocityVoltage` slot 0. `NeutralOut` to coast at idle. |
| Feeder/intake motor | SparkMAX (ID 40), open-loop duty cycle, brake at idle. Direction = operation. |
| `isAtTargetSpeed()` | Reads `getVelocity().getValueAsDouble() * 60` — actual velocity, not cached target (bug fixed). |
| `canShoot()` | `isAtTargetSpeed() && (!PHOTO_SENSOR_ENABLED \|\| hasBall())` — never blocked when sensor disabled. |
| Ball sensor | DIO not allocated until `PHOTO_SENSOR_ENABLED = true`. Polarity via `PHOTO_SENSOR_INVERTED`. |
| Jam detection | SparkMAX current spike → 100 ms `Timer` debounce → `JAM_REVERSE_TIME_SEC` reverse. Time-based throughout. |
| State machine | `IDLE → SPIN_UP → FEED`, plus `INTAKE`, `EJECT`, `JAM_CLEAR`. Jam only triggers from INTAKE or FEED. |
| PID tuning | SmartDashboard reads push to TalonFX `Slot0Configs` in test mode only. Control requests reused (no per-loop allocation). |
| Commands | `shootCommand()`, `feedCommand()`, `manualFeedCommand()`, `intakeCommand()`, `ejectCommand()`, `stopAllCommand()`, `setDistancePreset()` |
| Telemetry | `Shooter/*` and `Feeder/*` keys: current RPM, target RPM, distance, motor currents, state, ball detected, jam clearing, can shoot |

**Bugs fixed inline:**
- `isAtTargetSpeed()` always-ready bug (shooter branch)
- `EJECT_SPEED = 0` no-op bug (intake branch)
- DIO port 0 conflict on both sensor constants (resolved: port 1)
- Loop-count spike debounce replaced with `Timer`-based elapsed seconds
- Separate current limits for shooter (TalonFX stator) and feeder (SparkMAX smart)

---

## Phase 3 — Wire `RobotContainer` ✅ complete

- `Intake` field and import removed — feeder/intake owned entirely by `Shooter`
- Construction order enforced: `Vision` → `DriveTrain` → `Shooter` (Shooter receives both as constructor args for distance resolution and zone enforcement)
- Auto chooser and drivetrain default command unchanged

**Driver controller (port 0):**
| Button | Action |
|---|---|
| Back | Toggle Tank ↔ Arcade |
| Start | Toggle Field-Oriented ↔ Robot-Relative |

**Operator controller (port 1):**
| Button | Command |
|---|---|
| Y (toggle) | `spinUpCommand()` — spin to distance-resolved RPM, no feed. Zone-locked. |
| X (hold) | `shootCommand()` — spin up + auto-feed when at speed. Zone-locked. |
| RB (hold) | `feedCommand()` — feed only if shooter is already at speed |
| B | `stopAllCommand()` — immediately stop shooter and feeder |
| LB (toggle) | `intakeCommand()` — draw ball in; `Shooter/Intake Active` shows state on dashboard |
| LT (toggle) | `ejectCommand()` — reverse feeder to expel ball |
| POV Up | Stage preset 15.0 ft (no spin-up) |
| POV Down | Stage preset 10.0 ft |
| POV Left | Stage preset  7.5 ft |
| POV Right | Stage preset 12.5 ft |

**Zone enforcement:** Y and X spin-up are blocked outside the offensive zone in teleop/auto. Test mode bypasses the zone lock for bench/pit testing from any field position. RB, LB, LT, B, and POV presets are never zone-restricted.

---

## Phase 5 — Distance resolution, zone enforcement, and field element positions ✅ complete

### Field zones (2026 Rebuilt Welded — derived from `2026-rebuilt-welded.json` AprilTag layout)
Field runs X = 0 m (blue DS wall) → X = 16.541 m (red DS wall).

| Zone | Blue Alliance | Red Alliance |
|---|---|---|
| **Offensive** | X ≤ 5.2 m (blue DS to blue hub) | X ≥ 11.3 m (red DS to red hub) |
| Neutral | 5.2 m < X < 11.3 m | 5.2 m < X < 11.3 m |
| Defensive | X ≥ 11.3 m | X ≤ 5.2 m |

Hub positions corrected from AprilTag cluster centroids in the layout JSON:
- `BLUE_HUB_POSE = (4.63, 4.03 m)` — inner hub tags 18–21, 24–27; spans X = 4.02–5.23 m
- `RED_HUB_POSE  = (11.92, 4.03 m)` — inner hub tags 2–5, 8–11; spans X = 11.31–12.52 m
- `BLUE_OFFENSIVE_MAX_X = 5.2 m`, `RED_OFFENSIVE_MIN_X = 11.3 m` added to `VisionConstants`
- Previous `HUB_POSE = (8.27, 4.1)` (center-field placeholder) was incorrect and has been removed

### Distance priority (`resolveShooterDistance()`)
Runs every loop while the shooter is spinning; updates `targetRPM` continuously.

| Priority | Source | Condition |
|---|---|---|
| 1 | **Vision** | Fresh AprilTag pose available AND robot in offensive zone (or test mode) |
| 2 | **Odometry** | `DriveTrain.getPose()` AND robot in offensive zone (or test mode) |
| 3 | **POV Preset** | Operator has pressed a D-pad direction since last `stopAllCommand()` |
| 4 | **Default** | 10.0 ft — always available as last resort |

`Vision.getAllianceHubPose()` returns `BLUE_HUB_POSE` or `RED_HUB_POSE` based on `DriverStation.getAlliance()`.

### SmartDashboard keys (full list)
| Key | Type | Notes |
|---|---|---|
| `Shooter/Current RPM` | Number | Live shooter wheel speed |
| `Shooter/Target RPM` | Number | RPM currently commanded |
| `Shooter/RPM At Speed` | Boolean | **Green = ready to shoot, Red = spinning up** |
| `Shooter/Can Shoot` | Boolean | At speed AND (sensor disabled OR ball present) |
| `Shooter/In Offensive Zone` | Boolean | Zone gate status — false blocks spin-up |
| `Shooter/Active Distance (ft)` | Number | Distance feeding the RPM interpolation table |
| `Shooter/Vision Distance (ft)` | Number | Vision-computed hub distance (−1 = unavailable) |
| `Shooter/Odometry Distance (ft)` | Number | Odometry-computed hub distance (−1 = unavailable) |
| `Shooter/POV Preset Distance (ft)` | Number | Last D-pad staged preset |
| `Shooter/Distance Source` | String | "Vision" / "Odometry" / "POV Preset" / "Default" |
| `Shooter/Intake Active` | Boolean | LB toggle — green when intake running |
| `Shooter/Eject Active` | Boolean | LT toggle — green when eject running |
| `Shooter/Ball Detected` | Boolean | Photo sensor (always false until sensor enabled) |
| `Shooter/Motor Current (A)` | Number | TalonFX supply current |
| `Shooter/State` | String | State machine: IDLE / SPIN_UP / FEED / INTAKE / EJECT / JAM_CLEAR |
| `Feeder/Motor Current (A)` | Number | SparkMAX output current |
| `Feeder/Jam Clearing` | Boolean | Active jam-clear reverse in progress |

---

## Phase 6 — Validation

**Build (simulation):** ✅ `./gradlew build` — clean, zero errors after all phases. 22 pre-existing deprecation warnings in `DriveTrain.java`, `Vision.java`, and `Shooter.java` (REV SparkMAX configure API, PhotonLib — unrelated to this work).

**Bench tests (robot connected):** ⬜ pending
- [ ] Verify shooter wheel spins in correct direction — flip `SHOOTER_INVERTED` in Constants if not
- [ ] Verify feeder/intake spins in correct direction for intake and feed — flip `FEEDER_INVERTED` if not
- [ ] Confirm current limits engage correctly at stall
- [ ] Confirm jam clear triggers on hard stall and reverses for ~0.25 s
- [ ] Set `PHOTO_SENSOR_ENABLED = true` once sensor installed; verify `hasBall()` polarity — flip `PHOTO_SENSOR_INVERTED` if backwards

**Zone enforcement (test mode — any field position):** ⬜ pending
- [ ] Y toggles shooter spin-up; RPM climbs to expected value for default 10 ft preset
- [ ] `Shooter/In Offensive Zone` reads false when robot is at field center (X ≈ 8.27 m)
- [ ] Y does NOT spin shooter outside offensive zone in teleop mode
- [ ] Y DOES spin shooter outside offensive zone in test mode
- [ ] `Shooter/Distance Source` reads "Vision" when tags visible and in offensive zone
- [ ] `Shooter/Distance Source` reads "Odometry" when no tags visible and in offensive zone
- [ ] `Shooter/Distance Source` reads "POV Preset" after pressing D-pad with no pose data
- [ ] Hub poses in Constants match physical hub position on the actual field — adjust if off

**PID tuning (test mode):** ⬜ pending
- [ ] Tune `SHOOTER_KV` first — feedforward should reach ~80–90% of target RPM open-loop
- [ ] Tune `SHOOTER_KP` to close remaining RPM error without oscillation
- [ ] Validate each POV distance preset hits expected RPM from the table
- [ ] Confirm `Shooter/RPM At Speed` turns green reliably at each tested distance
- [ ] Update `KP/KI/KD/KV` constants in `ShooterConstants` once values are confirmed

**Driver station dry run (on field):** ⬜ pending
- [ ] Y toggles shooter spin on/off; second press stops cleanly
- [ ] X spins up and auto-feeds; button release stops both shooter and feeder
- [ ] RB feeds only after shooter is at speed; no feed if shooter is idle
- [ ] LB toggles intake on/off; `Shooter/Intake Active` reflects state on dashboard
- [ ] LT toggles eject on/off; `Shooter/Eject Active` reflects state on dashboard
- [ ] B stops all from any active state (spin, feed, intake, eject)
- [ ] D-pad presets update `Shooter/POV Preset Distance (ft)` without spinning motor
- [ ] Crossing hub boundary mid-match stops spinner (zone gate fires) and restarts when back in zone
- [ ] Driver controller produces no unintended shooter effects
- [ ] Auto chooser populates and selected auto executes correctly
