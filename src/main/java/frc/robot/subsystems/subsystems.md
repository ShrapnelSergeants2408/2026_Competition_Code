# Subsystems

This document describes all active subsystems in this robot. For the general architecture of Command-Based programming and how subsystems fit in, read [robot.md](../robot.md) first.

---

## What Is a Subsystem?

A subsystem is a Java class that `extends SubsystemBase`. It represents a physical group of hardware on the robot. Key rules:

- **All hardware objects** (motors, encoders, sensors) for a mechanism live in the subsystem that owns that mechanism.
- **`periodic()`** runs every 20 ms unconditionally — use it for sensor updates, telemetry, and safety checks.
- **Exclusive access** — the CommandScheduler ensures only one command runs on a subsystem at a time. If two commands need the same subsystem, they conflict.
- **Commands live in** `RobotContainer` (as lambdas or command factory methods), or as methods on the subsystem class that return `Command`. They are bound to buttons in `RobotContainer.configureBindings()`.

---

## DriveTrain

**File:** `DriveTrain.java`

The DriveTrain subsystem manages everything related to moving the robot.

### Hardware

| Device | CAN ID | Role |
|--------|--------|------|
| SparkMax (NEO) | 20 | Left lead motor |
| SparkMax (NEO) | 21 | Left follower motor |
| SparkMax (NEO) | 22 | Right lead motor |
| SparkMax (NEO) | 23 | Right follower motor |
| NavX2 AHRS | MXP (SPI) | Gyroscope for heading |

The right lead motor is **inverted** so that positive output drives forward on both sides. Each follower mirrors its lead motor using SparkMax's `.follow()` configuration. Follower motors have their CAN telemetry periods slowed to 500 ms (from the default 20 ms) to reduce unnecessary CAN bus traffic, since the robot never reads data from followers.

### Encoder Conversion

The SparkMax built-in encoders are configured with **position and velocity conversion factors** so that:
- `getPosition()` returns **meters** (not motor rotations)
- `getVelocity()` returns **meters per second**

This means you can call `leftEncoder.getPosition()` and get a meaningful distance directly, without manual unit conversion elsewhere in the code.

### Pose Estimation

The DriveTrain maintains a `DifferentialDrivePoseEstimator` — a Kalman filter that fuses:
- **Wheel encoder odometry** (how far each wheel has traveled)
- **Gyroscope heading** (which direction the robot is facing)
- **Vision measurements** (AprilTag pose estimates from the Vision subsystem)

The result is a continuous estimate of the robot's **position and heading on the field**, represented as a `Pose2d` (x meters, y meters, rotation in degrees).

Vision measurements are only fused if `Vision.isAnyVisionAvailable()` returns true. Each measurement has **standard deviations** that tell the filter how much to trust it — multi-tag measurements with low ambiguity are trusted more than single-tag distant measurements.

### Pose Initialization

At the start of auto or teleop, `initializePose()` is called to seed the estimator with the best available starting position:

1. **Vision** — if a fresh AprilTag fix exists (up to 5 seconds old is accepted at init time)
2. **PathPlanner auto starting pose** — the known starting position of the selected auto routine
3. **Field origin** — last resort; a warning is issued on the dashboard

### PathPlanner Integration

`AutoBuilder.configure()` is called once in the `DriveTrain` constructor. It registers the robot's pose, speed, and drive functions with PathPlanner so that auto routines can command the drivetrain to follow trajectories.

The **PPLTVController** (Linear Time-Varying controller) is the path-following controller. Its `Q` and `R` matrix tuning parameters can be adjusted live from SmartDashboard in test mode under the `DriveTrain/PPLTV/` key prefix.

### Teleop Drive

The default command is a tank drive (`teleopDriveCommand`):
- Left Y stick → left wheels
- Right Y stick → right wheels
- Right trigger (analog) → speed boost from 70% to 100%
- Start button → toggle reverse driving (robot's rear becomes front)

A 0.05 deadband is applied to joystick inputs to prevent motor movement from controller drift.

---

## Shooter

**File:** `Shooter.java`

The Shooter subsystem owns the **flywheel motor** only. The intake roller and trigger/hopper were deliberately moved to the separate `Feeder` subsystem so that intake and shoot commands can run concurrently (they require different subsystems and thus do not conflict).

### Hardware

| Device | CAN ID | Role |
|--------|--------|------|
| TalonFX (Kraken X60) | 30 | Shooter flywheel wheel |

### Velocity PID

The TalonFX runs **on-controller closed-loop velocity PID** (Phoenix 6, Slot 0). The control request is a `VelocityVoltage` in rotations per second. The subsystem converts RPM → rotations/sec internally, accounting for the belt pulley gear ratio (26T motor : 32T wheel).

PID gains (`kP`, `kI`, `kD`, `kV`) can be tuned live from SmartDashboard in test mode under `Shooter/Tuning/`. They are only written to the TalonFX when a value actually changes (avoids blocking CAN calls every loop).

### Distance → RPM Table

The shooter uses a **7-point interpolation table** to translate shooting distance to target RPM. Values were derived from projectile physics (70° launch angle, 18" launcher height, 72" hub target height) and calibrated from test sessions.

| Distance (ft) | Target RPM |
|--------------|-----------|
| 5.0 | 2150 |
| 7.5 | 2400 |
| 10.0 | 2600 |
| 12.5 | 2850 |
| 15.0 | 3050 |
| 17.5 | 3200 |
| 18.75 | 3300 |

Distances outside this range clamp to the nearest endpoint. Intermediate distances are linearly interpolated.

### Distance Resolution Priority

Every time the shooter needs to determine its target RPM, it resolves the best available distance source in this priority order:

| Priority | Source | Condition |
|---------|--------|-----------|
| 1 | **POV Preset** | Operator is holding a POV direction on the operator controller |
| 2 | **Vision** | Fresh AprilTag measurement available (< 0.5 s old) |
| 3 | **Odometry** | Drivetrain pose estimator position |
| 4 | **Default 10 ft** | No other source available |

In **autonomous**, the distance is always fixed at 10 ft (no sensor input during auto).
In **test mode**, POV preset is sticky — it remains locked until a new POV is pressed.

### Commands

- **`spinUpCommand()`** — spins flywheel to distance-resolved RPM. Used by operator Y (toggle). Requires only the Shooter subsystem so Feeder commands can run concurrently.
- **`reverseShooter()`** — runs flywheel at 50% reverse. Used by operator LT to unjam.
- **`stopShooter()`** — coasts flywheel to stop.

---

## Feeder

**File:** `Feeder.java`

The Feeder subsystem owns the **intake roller** and **trigger/hopper** motors. It is deliberately separate from Shooter so intake/eject operations can run at the same time the flywheel is spinning.

### Hardware

| Device | CAN ID | Role |
|--------|--------|------|
| SparkMax (NEO) | 31 | Intake roller (CCW only — into robot) |
| SparkMax (NEO) | 32 | Trigger/hopper (bidirectional) |
| DigitalInput | DIO 1 | Ball-presence photo sensor (disabled — not yet installed) |

Both motors use **brake mode** and **voltage compensation** at 12 V for consistent behavior across battery charge states.

### State Machine

The Feeder tracks an internal `FeederState` enum:

```
IDLE → INTAKE  (LB pressed — draw ball in)
IDLE → FEED    (RT pressed — feed to shooter)
IDLE → EJECT   (RB pressed — expel ball)
INTAKE/FEED → JAM_CLEAR  (trigger current spike detected)
JAM_CLEAR → [previous state]  (after 0.25 s reverse)
```

### Jam Detection

The subsystem monitors the trigger motor's output current. If a sustained spike above the threshold is detected (debounced at 100 ms), it automatically:
1. Stops the intake motor
2. Runs the trigger motor in reverse for 0.25 seconds
3. Resumes the previous state (INTAKE or FEED)

> **Note:** Jam detection is currently **disabled** (`updateJamDetection()` is commented out in `periodic()`). It was disabled during competition due to unreliability. The spike threshold (50 A) may exceed the SparkMax's actual sustainable current limit (30 A) — this should be re-evaluated before re-enabling.

### Commands

| Method | Operator Button | Action |
|--------|----------------|--------|
| `intakeCommand()` | LB (hold) | Both motors draw ball inward |
| `ejectCommand()` | RB (hold) | Both motors reverse to expel ball |
| `shootCommand()` | RT (hold) | Intake holds ball while trigger feeds to shooter |
| `stopAll()` | B (emergency) | Stop both motors immediately |

All commands use `Commands.run()` (re-commands every 20 ms) rather than `Commands.runOnce()`, which ensures motors stay running even if a CAN frame is dropped or briefly current-limited at startup.

### Photo Sensor

A ball-detection photo sensor at DIO port 1 is wired but not yet physically installed. It is disabled via `SensorConstants.PHOTO_SENSOR_ENABLED = false`. When enabled, `hasBall()` will return `true` when the sensor detects a ball in the feeder path, which can be used to gate feed commands.

---

## Vision

**File:** `Vision.java`

The Vision subsystem interfaces with two **PhotonVision** coprocessors (Raspberry Pis) running AprilTag detection pipelines.

### Hardware

| Camera | Coprocessor | Lens | Mount Position |
|--------|------------|------|---------------|
| `Front_Camera` | Raspberry Pi 4 | Pi Camera v2 | Front of robot, tilted 30° down |
| `Rear_Camera` | Raspberry Pi 5 | OV9281 | Rear of robot, facing backward, tilted 30° down |
| `Driver_Camera` | USB (roboRIO) | Webcam | Driver feed only, no pose estimation |

Camera transforms (position and angle relative to robot center) are defined in `VisionConstants.ROBOT_TO_FRONT_CAM` and `ROBOT_TO_REAR_CAM`. These must be measured accurately on the physical robot — incorrect values will produce wrong pose estimates.

### AprilTag Field Layout

The 2026 field layout `k2026RebuiltWelded` is loaded at startup. This file contains the 3D positions of all 32 AprilTag targets on the field (IDs 1–32). The pose estimators use this layout to compute the robot's world position from tag detections.

### Per-Loop Caching

Camera processing is computationally expensive. To avoid processing each camera more than once per 20 ms loop, the Vision subsystem:
- Fetches both camera results once at the top of `periodic()`
- Caches them as `cachedFrontResult` / `cachedRearResult`
- Recomputes the best measurement only when a new frame arrives (timestamp changed)
- Stores the result in `cachedMeasurement` — all callers read from this cache

This means `getBestVisionMeasurement()` and `getBestVisionMeasurementIfFresh()` are O(1) operations safe to call from any command without causing redundant CAN or NetworkTables reads.

### Measurement Selection

When both cameras produce valid measurements in the same loop:
- **Multi-tag preferred over single-tag** (multi-tag is more accurate)
- **Closer tags preferred** when both are the same type

Quality gates reject measurements that:
- Place the robot outside the field boundary
- Have tag ambiguity > 0.3 (for single-tag only)
- Are farther than 4.0 m from the nearest tag
- Contain unknown tag IDs (returns `POSITIVE_INFINITY` distance, caught by distance check)

### Standard Deviations

Measurements that pass quality gating are assigned trust levels before being fed to the `DifferentialDrivePoseEstimator`:

| Condition | Std Dev (x, y, θ) |
|-----------|-------------------|
| Multi-tag (≥ 2 tags) | 0.2 m, 0.2 m, 5° |
| Single-tag, close (< 2 m) | 0.5 m, 0.5 m, 10° |
| Single-tag, far (≥ 2 m) | 1.0 m, 1.0 m, 20° |

Lower values = more trust. These are starting values — tune based on actual field testing.

### Known Issues: Vision Did Not Work During Matches

> **Important for future teams:** The vision subsystem was fully coded and integrated, but **did not provide usable data during competition matches**. During pre-match testing, cameras appeared connected, but the Driver Station displayed regular errors indicating `Front_Camera` and `Rear_Camera` were not transmitting data.
>
> The root cause was not identified due to time constraints. Possible causes:
>
> - **Brownout-induced camera disconnects** — If the robot's battery voltage drops suddenly (e.g., from hard acceleration), the Raspberry Pi coprocessors may lose power momentarily and disconnect. Adding capacitors or a dedicated battery for the coprocessors can help.
> - **PhotonVision pipelines not refreshed at startup** — PhotonVision pipelines (the processing configuration for each camera) may need to be re-saved or re-applied after a software update or coprocessor reboot. Verify that the correct pipeline is active and set as the default in the PhotonVision web interface.
> - **Network Tables instability** — The coprocessors communicate with the roboRIO via NetworkTables over the robot network. If the network is congested or the coprocessor hostname is not resolving correctly, the connection can fail intermittently.
>
> **Recommended steps for future teams:**
> 1. Verify camera names in PhotonVision (`Front_Camera`, `Rear_Camera`) match `VisionConstants.FRONT_CAMERA_NAME` and `REAR_CAMERA_NAME` exactly (case-sensitive).
> 2. Confirm pipelines are saved and set as default in PhotonVision web UI before each event.
> 3. Test vision on the actual competition field network, not just the pit — radio and network behavior differ.
> 4. Monitor `Vision/FrontCamConnected` and `Vision/RearCamConnected` on SmartDashboard during robot enable to verify connection before the match.
> 5. Check Raspberry Pi power wiring — coprocessors should be powered from a regulator (e.g., VRM or dedicated PDH port) that can sustain current during robot brownouts.

---

## Stub Subsystems

`Climber.java` and `Intake.java` exist in the repository as **empty stub files**. They were not implemented for the 2026 season. `Intake.java` was superseded by `Feeder.java`. These files can be used as starting points for future development but contain no functional code.
