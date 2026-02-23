# Drive, Vision & Autonomous Code Review
**Branch:** `hieb-trial` | **Date:** 2026-02-23
**Scope:** `DriveTrain.java`, `Vision.java`, `Constants.java`, `RobotContainer.java`, PathPlanner config

---

## Table of Contents
1. [Architecture Overview](#1-architecture-overview)
2. [Bugs](#2-bugs)
3. [Inefficiencies](#3-inefficiencies)
4. [Enhancement Track A — Computing Overhead & Speed](#4-enhancement-track-a--computing-overhead--speed)
5. [Enhancement Track B — Code Elegance](#5-enhancement-track-b--code-elegance)
6. [Summary Priority Table](#6-summary-priority-table)

---

## 1. Architecture Overview

| Component | Technology | Status |
|---|---|---|
| Drivetrain | 4-motor differential drive, NEO/SparkMax | Functional |
| Localization | `DifferentialDrivePoseEstimator` + NavX | Functional |
| Vision | Dual PhotonVision cameras, AprilTag PnP | Functional but always disabled |
| Autonomous | PathPlanner + `PPLTVController` | Functional |
| Telemetry | SmartDashboard + AdvantageKit | Functional |

---

## 2. Bugs

### BUG-01 — Vision Pose Fusion Is Permanently Disabled
**File:** `DriveTrain.java:98`, `DriveTrain.java:113–115`
**Severity:** Critical

`visionEnabled` defaults to `false` and `setVisionEnabled()` is never called from `RobotContainer` or anywhere else. The entire vision → pose-estimator pipeline is dead code at runtime.

```java
// DriveTrain.java:98
private boolean visionEnabled = false;  // Never flipped to true

// DriveTrain.java:113
if (visionEnabled) {                    // This block NEVER executes
    updateVisionMeasurements();
}
```

**Fix:** Either default to `true`, call `drivetrain.setVisionEnabled(true)` in `RobotContainer` after subsystem init, or remove the flag entirely if there is no operational reason to disable vision during a match.

---

### BUG-02 — Gyro Heading Convention Mismatch Between Odometry and Field-Oriented Drive
**File:** `DriveTrain.java:75, 83, 111–112, 365–367`
**Severity:** High

`getHeading()` negates `gyro.getAngle()` to enforce CCW-positive (WPILib standard):

```java
// DriveTrain.java:365
public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle()); // negated
}
```

However, `periodic()` passes `gyro.getRotation2d()` directly to both odometry and the pose estimator — bypassing `getHeading()` entirely:

```java
// DriveTrain.java:111–112
poseOdometry.update(gyro.getRotation2d(), ...);   // uses raw gyro convention
poseEstimator.update(gyro.getRotation2d(), ...);  // uses raw gyro convention
```

The Studica AHRS `getRotation2d()` already returns CCW-positive (it negates internally), so `getHeading()` double-negates and produces CW-positive — the wrong convention for field-oriented drive. Field-oriented math then rotates inputs in the wrong direction.

**Fix:** Replace `gyro.getRotation2d()` in `periodic()` with `getHeading()` so a single, tested heading source is used everywhere, or verify that `gyro.getRotation2d()` equals `getHeading()` on the actual hardware and remove the negation in `getHeading()` if they agree.

---

### BUG-03 — Field-Oriented Arcade Applies Rotation Matrix to a Non-Spatial Input
**File:** `DriveTrain.java:260–265`
**Severity:** High

The method treats `(fwd, rot)` as a 2D Cartesian vector and rotates it through the robot heading:

```java
private void fieldOrientedArcade(double fwd, double rot, double headingRad) {
    double robotFwd =  fwd * cos + rot * sin;   // rot is angular rate, not a displacement
    double robotRot = -fwd * sin + rot * cos;
    driver.arcadeDrive(robotFwd, robotRot);
}
```

`rot` is a normalized turning rate (−1 to +1), not a spatial axis. Injecting it into the rotation matrix introduces a heading-dependent coupling between forward speed and turn rate that has no physical basis. When the robot is at 90°, pushing the stick forward produces unexpected turning.

**Fix for field-oriented arcade:** Only rotate the linear (forward) component; leave the rotational command robot-relative since angular velocity has no field direction:

```java
// Correct field-oriented arcade for differential drive
private void fieldOrientedArcade(double fwd, double rot, double headingRad) {
    double robotFwd = fwd * Math.cos(headingRad); // project field-fwd onto robot-fwd axis
    driver.arcadeDrive(robotFwd, rot);             // rot stays robot-relative
}
```

---

### BUG-04 — `calculateAverageTagDistance()` Called Three Times Per Camera Frame
**File:** `Vision.java:138, 146–147, 153, 212`
**Severity:** Medium

For each accepted camera result, `calculateAverageTagDistance()` is called three separate times with the same inputs — once inside `shouldUseMeasurement()`, once to compute standard deviations, and once to store `avgDistance` in the measurement record:

```java
// processCameraResult() — Vision.java
if (!shouldUseMeasurement(pose, result)) { ... }  // calls calculateAverageTagDistance() ONCE (line 212)

Matrix<N3, N1> stdDevs = calculateStandardDeviations(
    pose,
    calculateAverageTagDistance(result.getTargets(), ...)  // SECOND call (line 146)
);

double avgDistance = calculateAverageTagDistance(result.getTargets(), ...); // THIRD call (line 153)
```

Each call iterates all visible targets and performs field layout lookups. Compute it once and pass the result.

---

### BUG-05 — `POSITIVE_INFINITY` Distance Can Reach `calculateStandardDeviations()`
**File:** `Vision.java:264–266, 236–240`
**Severity:** Medium

When `calculateAverageTagDistance()` finds no tags in the field layout, it returns `Double.POSITIVE_INFINITY`. The quality gate in `shouldUseMeasurement()` correctly rejects measurements where `avgDistance > MAX_TAG_DISTANCE_METERS` — so `POSITIVE_INFINITY` is rejected there. However, because `calculateAverageTagDistance()` is called separately for `calculateStandardDeviations()` (BUG-04), if a code path ever reaches `calculateStandardDeviations()` with an infinity distance, the `< 2.0` branch is skipped and `SINGLE_TAG_FAR_STDDEVS` is used silently rather than discarding the measurement. Consolidating calls (BUG-04 fix) eliminates this risk entirely.

---

### BUG-06 — Camera Freshness Check Mixes Wall Clock with FPGA Timestamps
**File:** `Vision.java:43–44, 172–173, 554–558`
**Severity:** Medium

`updateCameraFreshness()` compares PhotonVision's result timestamp (FPGA seconds, in robot-time) against `System.currentTimeMillis()` (UTC wall-clock milliseconds). At startup `lastFrontUpdateMs = 0`, meaning `nowMs - 0` is on the order of 1.7 × 10¹² ms — `isCameraConnected()` will return `false` for any camera that hasn't produced a new frame since the robot powered on, which is correct behavior. However, if the system clock and FPGA clock ever desynchronize (e.g., NTP correction mid-match) the freshness window silently breaks.

**Fix:** Use `Timer.getFPGATimestamp()` for both the freshness threshold and the "now" check, making the entire system use one time domain.

---

### BUG-07 — PathPlanner Settings Include Swerve Module Positions for a Differential Drive Robot
**File:** `settings.json:25–34`
**Severity:** Low (non-functional)

The PathPlanner project file includes swerve-specific fields (`flModuleX`, `flModuleY`, etc.) while `holonomicMode` is correctly `false`. These values are ignored at runtime but represent incorrect configuration state that could confuse future developers or cause problems if `holonomicMode` is ever accidentally enabled.

---

### BUG-08 — `getAutoCommand()` in DriveTrain Is Dead Code
**File:** `DriveTrain.java:417–419`
**Severity:** Low

```java
public Command getAutoCommand(String autoName) {
    return new PathPlannerAuto(autoName);
}
```

`RobotContainer` uses `AutoBuilder.buildAutoChooser()` and `autoChooser.getSelected()` exclusively. This method is never called, creating a misleading public API.

---

### BUG-09 — `testDriveTrain()` Returns a Meaningless Stub
**File:** `DriveTrain.java:321–323`
**Severity:** Low

```java
public static List<Result> testDriveTrain() {
    return List.of(Result.pass("dummy test"));
}
```

This always passes regardless of hardware state, providing false confidence in any test harness that calls it.

---

## 3. Inefficiencies

### INEFF-01 — Vision Processing Runs Twice Per Loop
**File:** `Vision.java:567–569`, `DriveTrain.java:113–116`, `Vision.java:451–529`

`Vision.periodic()` calls `updateTelemetry()`, which calls `getBestVisionMeasurement()`. Separately, `DriveTrain.periodic()` calls `updateVisionMeasurements()`, which also calls `getBestVisionMeasurement()`. Both executions call `processCameraResult()` on both cameras, each calling `camera.getLatestResult()`.

Result: **both cameras are queried and processed twice per 20 ms loop**.

```
Vision.periodic()
  └─ updateTelemetry()
       └─ getBestVisionMeasurement()          ← full processing cycle #1
            ├─ processCameraResult(front)
            └─ processCameraResult(rear)

DriveTrain.periodic()
  └─ updateVisionMeasurements()
       └─ getBestVisionMeasurement()          ← full processing cycle #2 (same frame data)
            ├─ processCameraResult(front)
            └─ processCameraResult(rear)
```

---

### INEFF-02 — Dual Odometry Runs Every Loop With No Consumer
**File:** `DriveTrain.java:111`

```java
poseOdometry.update(gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
```

`poseOdometry` is a `DifferentialDriveOdometry` (encoder+gyro only) running alongside `poseEstimator` (encoder+gyro+vision). No method in the codebase reads `poseOdometry`'s position. It is updated every loop for no operational benefit.

---

### INEFF-03 — `SmartDashboard.putData("Field", field)` Called Every Loop
**File:** `DriveTrain.java:425`

`SmartDashboard.putData()` registers a `Sendable` object. Calling it every loop re-registers the same `Field2d` on every periodic cycle (50 Hz). `putData` for Sendable objects should be called once (e.g., in the constructor), then the underlying `field` object updated each loop via `field.setRobotPose()`.

---

### INEFF-04 — `getVisibleTags()` Allocates an ArrayList and Boxes Integers Every Loop
**File:** `Vision.java:312–330`, `Vision.java:457`

`updateTelemetry()` calls `getVisibleTags()` every loop. That method allocates a new `ArrayList<Integer>`, then boxes each primitive `int` tag ID into an `Integer` object as it iterates both camera results. This produces per-loop heap pressure and GC activity.

---

### INEFF-05 — `getDriveStateString()` Allocates a New String Every Loop via Concatenation
**File:** `DriveTrain.java:439–444`

```java
return orient + " " + mode;  // new String object every 20ms
```

The state only changes on button press. The string can be computed once when the mode changes and cached.

---

### INEFF-06 — `isTagVisible()` Creates a Full ArrayList to Answer a Boolean Question
**File:** `Vision.java:305–307`

```java
public boolean isTagVisible(int tagId) {
    return getVisibleTags().contains(tagId);  // allocates ArrayList for a simple check
}
```

A direct iteration over camera results without allocation would be far cheaper.

---

### INEFF-07 — Public Vision Methods Re-Trigger Full Camera Processing
**File:** `Vision.java:335–339, 345–352`

The public `getDistanceToPose(Pose2d)` and `getYawToPose(Pose2d)` methods each call `getBestVisionMeasurement()` internally, triggering another full processing cycle if called from game code or commands. Any command using these APIs causes a third (or more) processing pass in the same loop.

---

## 4. Enhancement Track A — Computing Overhead & Speed

### A-01 — Cache Vision Results Once Per Loop (Addresses INEFF-01, INEFF-07)

Process each camera **once per loop** in `Vision.periodic()`, store the result, and expose it as a pre-computed field. All callers (telemetry, drivetrain update, game commands) read the cached value instead of triggering new processing.

```java
// Vision.java — new fields
private Optional<VisionMeasurement> cachedMeasurement = Optional.empty();
private List<Integer> cachedVisibleTags = List.of();

@Override
public void periodic() {
    // Process cameras exactly once
    cachedMeasurement = computeBestVisionMeasurement();
    cachedVisibleTags = computeVisibleTags();   // compute once, reuse
    updateTelemetry(cachedMeasurement, cachedVisibleTags);
}

// All public getters return the cache — zero re-processing
public Optional<VisionMeasurement> getBestVisionMeasurement() {
    return cachedMeasurement;
}
```

`DriveTrain.updateVisionMeasurements()` and all distance/alignment methods then read the cached result without any per-call overhead.

---

### A-02 — Compute Average Distance Once Per Frame (Addresses BUG-04, INEFF-01)

Compute `avgDistance` once per `processCameraResult()` call and pass it to both `shouldUseMeasurement()` and `calculateStandardDeviations()`:

```java
private Optional<VisionMeasurement> processCameraResult(...) {
    // ...
    double avgDistance = calculateAverageTagDistance(result.getTargets(), pose2d); // ONE call
    if (!shouldUseMeasurement(pose, result, avgDistance)) return Optional.empty();
    Matrix<N3, N1> stdDevs = calculateStandardDeviations(pose, avgDistance);
    // ...
}
```

---

### A-03 — Remove Redundant `DifferentialDriveOdometry` (Addresses INEFF-02)

Delete `poseOdometry` and its `update()` call. If encoder+gyro-only pose is needed for debugging, read it from `poseEstimator` before vision measurements are added (log it just before `addVisionMeasurement()`). Removing one full odometry update per loop eliminates unnecessary matrix math.

---

### A-04 — Register `Field2d` Once, Update Each Loop (Addresses INEFF-03)

Move `SmartDashboard.putData("Field", field)` to the `DriveTrain` constructor. In `updateTelemetry()`, only call `field.setRobotPose()`:

```java
// Constructor — register once
SmartDashboard.putData("Field", field);

// updateTelemetry() — only update the pose
field.setRobotPose(getPose());
```

---

### A-05 — Cache Drive State String (Addresses INEFF-05)

Compute the state string only when mode changes, not every loop:

```java
private String driveStateString = "Field-Oriented Tank"; // initial value

public void toggleDriveMode() {
    this.driveMode = ...;
    this.driveStateString = buildDriveStateString(); // update on change only
}

public void toggleOrientationMode() {
    this.orientationMode = ...;
    this.driveStateString = buildDriveStateString(); // update on change only
}
```

---

### A-06 — Replace `isTagVisible()` List Allocation With Direct Iteration (Addresses INEFF-06)

```java
public boolean isTagVisible(int tagId) {
    for (var t : frontCamera.getLatestResult().getTargets())
        if (t.getFiducialId() == tagId) return true;
    for (var t : rearCamera.getLatestResult().getTargets())
        if (t.getFiducialId() == tagId) return true;
    return false;
}
```

Or — better — after A-01 is implemented, check against the cached tag list using a primitive loop (no boxing).

---

### A-07 — Use `Timer.getFPGATimestamp()` for Camera Freshness (Addresses BUG-06)

Replace `System.currentTimeMillis()` with `Timer.getFPGATimestamp()` (which returns a `double` in seconds). Store `lastFrontTimestamp` and `lastRearTimestamp` as FPGA-time doubles and compare against a `CAMERA_STALE_TIMEOUT_SECONDS` constant. This eliminates the wall-clock/FPGA-time domain mismatch and eliminates `long` arithmetic:

```java
private static final double CAMERA_STALE_TIMEOUT_SECONDS = 0.5;
private double lastFrontFpgaTs = -1.0;
private double lastRearFpgaTs  = -1.0;

private boolean isCameraConnected(PhotonCamera camera) {
    double lastTs = (camera == frontCamera) ? lastFrontFpgaTs : lastRearFpgaTs;
    return (Timer.getFPGATimestamp() - lastTs) <= CAMERA_STALE_TIMEOUT_SECONDS;
}
```

---

### A-08 — Replace Hardcoded `0.02` Discretize dt With a Constant (Addresses fragility)
**File:** `DriveTrain.java:332`

```java
// Current
ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

// Better
ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, Robot.kDefaultPeriod);
```

If loop rate ever changes (e.g., for testing at 20 Hz), the discretization remains correct automatically.

---

## 5. Enhancement Track B — Code Elegance

### B-01 — Fix or Remove the `visionEnabled` Flag (Addresses BUG-01)

The flag is the most critical elegance issue because it creates a false mental model. Choose one:

**Option 1 (Recommended):** Remove the flag, always feed vision to the estimator:
```java
// DriveTrain.periodic()
updateVisionMeasurements(); // unconditional — vision quality gates handle bad data
```

**Option 2:** Expose it in RobotContainer with a clear enable call:
```java
// RobotContainer constructor
drivetrain.setVisionEnabled(true); // explicit, discoverable
```

---

### B-02 — Unify Field-Oriented Rotation Into a Single Helper (Addresses BUG-03, code duplication)

Both `fieldOrientedArcade` and `fieldOrientedTank` recompute `cos`/`sin` of the heading independently. Extract a shared rotation step:

```java
// Shared helper: rotate a field-frame (x, y) vector into robot frame
private double[] rotateToRobotFrame(double fieldX, double fieldY, double headingRad) {
    double cos = Math.cos(headingRad);
    double sin = Math.sin(headingRad);
    return new double[]{ fieldX * cos + fieldY * sin,
                        -fieldX * sin + fieldY * cos };
}
```

For field-oriented arcade (per BUG-03 fix), only the forward component is rotated; `rot` stays unmodified. For field-oriented tank, the `(fwd, turn)` vector is rotated. One rotation implementation, two callers, no duplication.

---

### B-03 — Prefer `getRotation2d()` Consistency Over a Parallel `getHeading()` (Addresses BUG-02)

After resolving BUG-02, consolidate to a single heading source. One clean option:

```java
// Verified once that gyro.getRotation2d() is CCW-positive on this hardware:
public Rotation2d getHeading() {
    return gyro.getRotation2d(); // no negation needed if AHRS handles it internally
}

// periodic() — now both odometry and field-oriented drive agree
poseEstimator.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
```

---

### B-04 — Collapse Vision Public API Into a Single Measurement-Based Interface

The current Vision class exposes two parallel sets of methods:
- Public: `getDistanceToPose(Pose2d)` → calls `getBestVisionMeasurement()` internally
- Private: `getDistanceToPose(Pose2d, Optional<VisionMeasurement>)` → accepts cached measurement

This split exists to prevent redundant processing in `updateTelemetry()`, but after A-01 (caching), the public methods can simply read the cache — making the private overloads unnecessary and collapsing to one clean set:

```java
// After caching (A-01), the public method is already O(1):
public Optional<Double> getDistanceToPose(Pose2d target) {
    return cachedMeasurement.map(m ->
        m.estimatedPose().getTranslation().getDistance(target.getTranslation()));
}
// No private overload needed — telemetry calls the same public method.
```

---

### B-05 — Replace `getBestVisionMeasurement()` Selection Logic With a Comparator

The current selection logic (lines 87–109) uses nested if-else branches. A `Comparator`-based approach is more readable and easier to extend:

```java
private static final Comparator<VisionMeasurement> MEASUREMENT_QUALITY = Comparator
    .comparingInt(VisionMeasurement::numTagsUsed).reversed()  // more tags = better
    .thenComparingDouble(VisionMeasurement::averageDistance); // closer = better

public Optional<VisionMeasurement> getBestVisionMeasurement() {
    return Stream.of(frontMeasurement, rearMeasurement)
        .filter(Optional::isPresent)
        .map(Optional::get)
        .min(MEASUREMENT_QUALITY);
}
```

This also correctly handles the case (currently missed) where front has 3 tags and rear has 2 — the current code treats them equally and falls through to distance comparison, while the comparator naturally picks 3 over 2.

---

### B-06 — Remove or Implement `getAutoCommand()` in DriveTrain (Addresses BUG-08)

Dead code should be removed. If the method is intended for direct-name auto selection (bypassing the chooser), document that intent clearly. Otherwise, delete it.

---

### B-07 — Replace Hardcoded `2.0` Distance Threshold in `calculateStandardDeviations()`

```java
// Current — magic number
if (averageDistance < 2.0) {
```

Move to `VisionConstants`:
```java
public static final double SINGLE_TAG_CLOSE_THRESHOLD_METERS = 2.0;
```

This makes the threshold tunable from one place alongside the related standard deviation constants.

---

### B-08 — Update Field Element Pose Names to Match 2026 Game

`VisionConstants` defines `HUB_POSE`, `TRENCH_POSE`, `DEPOT_POSE` — names from 2021 Infinite Recharge. These should be renamed and their coordinates updated to reflect the actual 2026 game field elements once the game manual is available. Leaving 2021 names active on a 2026 robot will cause confusion at competition.

---

### B-09 — Replace Shooter-Speed Magic Numbers With Named Constants

```java
// Constants.java:66–67
public static final double SHOOTER_SPEED = 0.9;
public static final double FEEDER_SPEED = 0.6;
```

These are already in `ShooterConstants` — good. Verify Shooter.java actually uses them (not hardcoded inline).

---

## 6. Summary Priority Table

| ID | Category | Title | Severity |
|---|---|---|---|
| BUG-01 | Bug | Vision pose fusion permanently disabled | **Critical** |
| BUG-02 | Bug | Gyro heading convention mismatch (odometry vs field-orient) | **High** |
| BUG-03 | Bug | Field-oriented arcade rotates angular rate as a spatial vector | **High** |
| BUG-04 | Bug | `calculateAverageTagDistance()` called 3× per frame | **Medium** |
| BUG-05 | Bug | `POSITIVE_INFINITY` distance can reach std-dev calculation | **Medium** |
| BUG-06 | Bug | Wall-clock vs FPGA-time mismatch in camera freshness | **Medium** |
| BUG-07 | Bug | Swerve module positions in differential-drive PathPlanner config | **Low** |
| BUG-08 | Bug | `getAutoCommand()` is dead code | **Low** |
| BUG-09 | Bug | `testDriveTrain()` always passes — provides no validation | **Low** |
| INEFF-01 | Inefficiency | Vision cameras processed twice per loop | **High** |
| INEFF-02 | Inefficiency | Dual odometry — `poseOdometry` updated but never read | **Medium** |
| INEFF-03 | Inefficiency | `SmartDashboard.putData("Field")` registered every loop | **Medium** |
| INEFF-04 | Inefficiency | `getVisibleTags()` allocates ArrayList + boxes ints every loop | **Medium** |
| INEFF-05 | Inefficiency | Drive state string allocated every loop | **Low** |
| INEFF-06 | Inefficiency | `isTagVisible()` creates ArrayList to answer a boolean | **Low** |
| INEFF-07 | Inefficiency | Public vision API re-triggers full camera processing | **Medium** |
| A-01 | Enhancement | Cache vision results once per loop | **High Impact** |
| A-02 | Enhancement | Compute average distance once per frame | **Medium Impact** |
| A-03 | Enhancement | Remove redundant `DifferentialDriveOdometry` | **Medium Impact** |
| A-04 | Enhancement | Register `Field2d` once, update each loop | **Low Impact** |
| A-05 | Enhancement | Cache drive state string on mode change | **Low Impact** |
| A-06 | Enhancement | Replace `isTagVisible()` list with direct iteration | **Low Impact** |
| A-07 | Enhancement | Use FPGA time for camera freshness | **Medium Impact** |
| A-08 | Enhancement | Use `Robot.kDefaultPeriod` for discretize dt | **Low Impact** |
| B-01 | Enhancement | Fix/remove `visionEnabled` flag | **High Impact** |
| B-02 | Enhancement | Unify field-oriented rotation into shared helper | **Medium Impact** |
| B-03 | Enhancement | Single heading source for odometry and field-orient | **High Impact** |
| B-04 | Enhancement | Collapse vision API to one set of methods | **Medium Impact** |
| B-05 | Enhancement | Comparator-based vision measurement selection | **Medium Impact** |
| B-06 | Enhancement | Remove dead `getAutoCommand()` | **Low Impact** |
| B-07 | Enhancement | Named constant for 2m distance threshold | **Low Impact** |
| B-08 | Enhancement | Rename field poses to match 2026 game | **Low Impact** |

---

*Document generated by code review of branch `hieb-trial` — all line references are to the current HEAD at commit `7612467`.*
