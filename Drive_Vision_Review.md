# Drive, Vision, and Autonomous Code Review

**Date:** 2026-02-23
**Branch:** `hieb-trial`
**Scope:** `DriveTrain.java`, `Vision.java`, `RobotContainer.java`, `Robot.java`, `Constants.java`, `VisionMeasurement.java`, `pathplanner/settings.json`

---

## Status of Previously Identified Issues

### Previously Identified Issues — RESOLVED

The following bugs and inefficiencies from the prior review pass have been corrected in the current branch:

| ID | Title | How It Was Fixed |
|---|---|---|
| BUG-01 | Field-oriented tank rotates turn component | `fieldOrientedTank()` now only projects `fwd` through `cos(heading)`; `turn` stays robot-relative |
| BUG-02 | `setVisionEnabled()` cannot disable vision | Public setter removed; `visionEnabled` is automatically set from `isAnyVisionAvailable()` each loop |
| BUG-03 | Auto pose seeding uses stale vision data | `initializePose()` now calls `getBestVisionMeasurementIfFresh()`, which gates on `MAX_VISION_AGE_SECONDS` |
| BUG-04 | Vision telemetry shows stale values | The no-measurement else-branch in `updateTelemetry()` now explicitly writes `NaN`/`0`/`false` to all distance and alignment keys |
| INEFF-01 | Multiple `getLatestResult()` calls per loop | `cachedFrontResult`/`cachedRearResult` are set once in `Vision.periodic()` and consumed via `getFrontResult()`/`getRearResult()` everywhere else |
| INEFF-02 | SmartDashboard updates at 50 Hz | Both `Vision` and `DriveTrain` now use a `telemetryLoopCounter` with `TELEMETRY_PERIOD_LOOPS = 5`, running at ≈10 Hz |

### Previously Identified Issues — STILL OPEN

| ID | Title | Notes |
|---|---|---|
| BUG-05 | Swerve module positions in diff-drive PathPlanner settings | Still present in `settings.json`; see detail below |
| INEFF-03 | O(n) `contains()` for tag deduplication and lookup | Building moved to `periodic()` but `List.contains()` is still used internally; see detail below |

---

## Active Bugs and Risks

### BUG-05 — Swerve module positions in differential drive PathPlanner settings *(carried forward)*
**Severity:** Low (configuration risk)
**File:** `src/main/deploy/pathplanner/settings.json:25–32`

`holonomicMode` is `false`, but the JSON still contains `flModuleX/Y`, `frModuleX/Y`, `blModuleX/Y`, `brModuleX/Y`. PathPlanner ignores these fields for differential drive today. However, if holonomic mode is accidentally re-enabled via the GUI, these values would produce incorrect swerve geometry that does not match the actual robot.

**Recommendation:** Remove the swerve-only keys, or regenerate the file from the PathPlanner GUI as a differential drive project.

---

### BUG-06 — `PPLTV_MAX_VELOCITY` is 3× the robot's actual top speed
**Severity:** Medium
**File:** `src/main/java/frc/robot/Constants.java:92` and `DriveTrain.java:257`

`Auto.PPLTV_MAX_VELOCITY = 9.0` m/s, but `Auto.MAX_MODULE_SPEED = 3.0` m/s. `PPLTVController` builds a gain table by linearizing the LTV system at the provided maximum velocity. Setting this value 3× higher than the robot's achievable speed means the controller's gains at the 0–3 m/s range where the robot actually operates are computed from a linearization point far outside the operating envelope, degrading path-tracking quality.

**Recommendation:** Set `PPLTV_MAX_VELOCITY` to `3.0` to match `MAX_MODULE_SPEED`, or at most 10–15% above it to give the gain table a small headroom margin.

---

### BUG-07 — `AutoBuilder.configure()` is called multiple times during test-mode LTV tuning
**Severity:** Medium
**File:** `src/main/java/frc/robot/subsystems/DriveTrain.java:211–229`

`refreshLtvControllerFromDashboard()` runs every 20 ms during test mode. When any dashboard value changes it calls `rebuildLtvController()` followed by `configurePathPlanner()`, which internally calls `AutoBuilder.configure()`. PathPlanner documents that `AutoBuilder.configure()` must be called exactly once; repeated calls register duplicate command factories and may overwrite existing auto bindings.

**Recommendation:** Decouple controller rebuilding from PathPlanner reconfiguration. Keep `AutoBuilder.configure()` only in the constructor. Reference `ltvController` by field—PathPlanner will use the current value of the field on each auto invocation—so rebuilding the object is sufficient without reconfiguring `AutoBuilder`.

---

### BUG-08 — Public vision geometry API uses non-fresh cached measurement
**Severity:** Low
**File:** `src/main/java/frc/robot/subsystems/Vision.java:391–409`

The public methods `getDistanceToPose(Pose2d)`, `getYawToPose(Pose2d)`, and their convenience wrappers (`getDistanceToHub()`, `isAlignedWithHub()`, etc.) call `getBestVisionMeasurement()`, which returns the cached measurement regardless of age. Commands invoking these for targeting or alignment could act on a measurement up to `MAX_VISION_AGE_SECONDS` (0.5 s) old.

**Recommendation:** Change the backing call inside `getDistanceToPose` and `getYawToPose` to `getBestVisionMeasurementIfFresh()`. The private overloads used in `updateTelemetry()` already receive the measurement directly and are unaffected.

---

### BUG-09 — Driver camera construction has no failure guard
**Severity:** Low
**File:** `src/main/java/frc/robot/RobotContainer.java:63–70`

`CameraServer.startAutomaticCapture(name, 0)` is called unconditionally in the `RobotContainer` constructor. If no USB camera is present at port 0 (simulation, missing hardware, or wrong port), `CameraServer` returns a `UsbCamera` in an error state rather than throwing. The subsequent `setResolution()` and `setFPS()` calls on the error-state object generate cscore warnings every loop, adding noise to Driver Station logs.

**Recommendation:** Wrap the camera initialization in a try/catch and call `DriverStation.reportWarning()` on failure, or check `driverCamera.isConnected()` after construction before calling configuration methods.

---

### BUG-10 — AdvantageKit `ProjectName` metadata is the WPILib template default
**Severity:** Low (logging quality)
**File:** `src/main/java/frc/robot/Robot.java:33`

`Logger.recordMetadata("ProjectName", "MyProject")` — the default template string was never replaced. AdvantageScope log files and NT4 feeds will identify as "MyProject", making it difficult to sort or identify match logs at competition.

**Recommendation:** Update to a team/season-specific identifier such as `"2408_2026_Competition"`.

---

## Remaining Inefficiencies

### INEFF-03 (partial) — `List.contains()` still used for deduplication and tag lookup
**File:** `src/main/java/frc/robot/subsystems/Vision.java:352–354, 368–386`

`cachedVisibleTags` is a `List<Integer>`. `isTagVisible(int tagId)` calls `cachedVisibleTags.contains(tagId)`, which does a linear scan and boxes the primitive `int` to `Integer` for each element comparison. Inside `computeVisibleTags()`, the rear-camera deduplication loop also uses `visibleTags.contains()`, making the rear-camera section O(n²) in the number of visible tags. At competition tag counts (typically 2–10 visible), the absolute CPU cost is small, but both the linear scan and the integer boxing are avoidable.

---

### INEFF-04 — Vision reprocesses the same camera frame every loop
**File:** `src/main/java/frc/robot/subsystems/Vision.java:627–638`

`computeBestVisionMeasurement()` and `computeVisibleTags()` are called every 20 ms regardless of whether a new frame has arrived. PhotonVision coprocessors typically publish at 30–50 Hz, meaning 1–2 loops out of every 3 will re-run `PhotonPoseEstimator.update()`, re-allocate an `ArrayList`, and re-compute all tag distances on the same data that was already processed.

**Recommendation:** Compare the incoming frame timestamp against `lastFrontTimestamp`/`lastRearTimestamp` before re-running the heavy path; if neither timestamp has advanced, leave `cachedMeasurement` and `cachedVisibleTags` unchanged.

---

### INEFF-05 — Telemetry computes geometry for all six field poses every cycle
**File:** `src/main/java/frc/robot/subsystems/Vision.java:507–578`

`updateTelemetry()` calls `getDistanceToPose()` and `isAlignedWithTarget()` for six field locations (hub, HP station, trench, depot, outpost, tower) every telemetry cycle. Each call computes a `Translation2d` difference and a `Rotation2d.minus()`. With a 10 Hz telemetry rate this is low-cost today, but all six targets are computed unconditionally even when the robot is nowhere near most of them.

**Recommendation:** Limit telemetry geometry computation to the one or two targets relevant to the current game phase, or cache the results across telemetry cycles.

---

## Enhancement Recommendations

### E1 — Efficiency: Use `boolean[]` or `Set` for visible tag tracking

Replace `List<Integer> cachedVisibleTags` with a `boolean[] cachedTagsVisible = new boolean[31]` (FRC tag IDs 1–30). `isTagVisible(int id)` becomes an O(1) bounds-checked array access with no boxing. `computeVisibleTags()` fills the array in a single pass. The existing `getVisibleTags()` API can return a compact `List<Integer>` built from the set positions.

```java
// O(1) lookup, no boxing
public boolean isTagVisible(int tagId) {
    return tagId >= 1 && tagId <= 30 && cachedTagsVisible[tagId];
}
```

---

### E2 — Efficiency: Skip vision recomputation when no new frame has arrived

Before calling `computeBestVisionMeasurement()` and `computeVisibleTags()` in `Vision.periodic()`, check whether either camera produced a newer timestamp than the last processed cycle. If neither has, the existing cache is still valid:

```java
double frontTs = cachedFrontResult.get().getTimestampSeconds();
double rearTs  = cachedRearResult.get().getTimestampSeconds();
boolean anyNew = frontTs > lastFrontTimestamp || rearTs > lastRearTimestamp;
if (anyNew) {
    cachedMeasurement = computeBestVisionMeasurement();
    cachedVisibleTags = computeVisibleTags();
}
```

This eliminates two `PhotonPoseEstimator.update()` calls and one `ArrayList` allocation on loops where the coprocessor has not published a new result (roughly half of all loops at 30 Hz vision / 50 Hz robot).

---

### E3 — Efficiency: Align `PPLTV_MAX_VELOCITY` with `MAX_MODULE_SPEED`

Change `PPLTV_MAX_VELOCITY` from `9.0` to `3.0` in `Constants.java`. This ensures the LTV gain table is built over the velocity range the robot actually uses, yielding better path-tracking accuracy without any additional computational cost.

---

### E4 — Efficiency: Decouple LTV controller rebuild from `AutoBuilder.configure()`

Store `ltvController` as a `volatile` field and reference it from PathPlanner by field rather than by value at configure time (verify against the PathPlanner API). This allows live PPLTV parameter updates in test mode to take effect without re-running `AutoBuilder.configure()`:

```java
// In constructor only:
AutoBuilder.configure(..., ltvController, ...);

// During live tuning (test mode only):
// Rebuild ltvController field — PathPlanner reads it on the next path invocation.
ltvController = new PPLTVController(...);
// No call to configurePathPlanner() needed.
```

---

### E5 — Elegance: Extract field-forward projection to a shared helper

`fieldOrientedArcade()` and `fieldOrientedTank()` both contain identical `fwd * Math.cos(headingRad)` logic. Extract to a single private method:

```java
/** Projects the driver's field-forward intent onto the robot's forward axis.
 *  For a differential drive, only the cosine component is usable;
 *  the sine component would require lateral motion the robot cannot produce. */
private double projectFieldForward(double fieldFwd, double headingRad) {
    return fieldFwd * Math.cos(headingRad);
}
```

This makes the heading math visible in exactly one place, clearly documents the intentional omission of the `sin` component, and simplifies any future changes (e.g., a switchover to swerve).

---

### E6 — Elegance: Replace multi-branch vision camera selection with a Comparator

`computeBestVisionMeasurement()` uses three nested if-else branches to pick the better measurement. Replace with a `Comparator`-based selection that is shorter, self-documenting, and trivially extensible (e.g., adding ambiguity as a tiebreaker):

```java
Comparator<VisionMeasurement> comp = Comparator
    .comparingInt(VisionMeasurement::numTagsUsed).reversed()
    .thenComparingDouble(VisionMeasurement::averageDistance);

return Stream.of(frontMeasurement, rearMeasurement)
    .filter(Optional::isPresent)
    .map(Optional::get)
    .min(comp);
```

---

### E7 — Elegance: Replace `visionEnabled` field with an inline check

`visionEnabled` in `DriveTrain` is set to `visionSubsystem.isAnyVisionAvailable()` and then immediately checked:

```java
visionEnabled = visionSubsystem.isAnyVisionAvailable();
if (visionEnabled) { updateVisionMeasurements(); }
```

The field serves no purpose beyond this two-line span—it is not read anywhere else, it is not settable, and it is not exposed via telemetry. Removing it in favor of an inline guard reduces mutable state and makes the intent clearer:

```java
if (visionSubsystem != null && visionSubsystem.isAnyVisionAvailable()) {
    updateVisionMeasurements();
}
```

---

### E8 — Elegance: Document field-oriented heading-loss behavior

Field-oriented tank/arcade projects driver-forward intent via `cos(heading)`. When the robot faces ±90° from field-forward, the joystick forward response drops to zero — correct physics for a non-holonomic drive, but potentially confusing to a driver who expects the robot to respond to input at any heading.

Recommended improvements:
- Add a `SmartDashboard.putNumber("DriveTrain/FOEfficiency", Math.abs(Math.cos(heading)))` key so drivers can see on the dashboard when forward responsiveness is reduced.
- Add a doc-comment to `fieldOrientedTank()` and `fieldOrientedArcade()` that explicitly states: "When heading is ±90° from field-forward, `cos(heading) ≈ 0` and forward output is suppressed by design; the robot must rotate to re-align before field-forward motion resumes."

---

### E9 — Elegance: Complete AdvantageKit integration or reduce its scope

`Robot` extends `LoggedRobot` and writes to `WPILOGWriter`/`NT4Publisher`, but subsystems write directly to `SmartDashboard` rather than through `Logger.recordOutput()`. This is a partial integration: the framework overhead of `LoggedRobot` is present, but subsystem-level replay in AdvantageScope is not available.

Two clear paths:
- **Minimal (reduce scope):** Revert to `TimedRobot`, keep `SmartDashboard` telemetry, and update the project name. Eliminates the AdvantageKit dependency cost with no loss of current functionality.
- **Full integration:** Replace `SmartDashboard.put*()` in periodic methods with `Logger.recordOutput()` and implement the IO layer per AdvantageKit conventions. Enables full match replay and structured logging.

---

## Priority Table

| ID | Category | Title | Severity | Status |
|---|---|---|---|---|
| BUG-01 | Bug | Field-oriented tank rotates turn component | High | **RESOLVED** |
| BUG-02 | Bug | `setVisionEnabled()` cannot disable vision | Medium | **RESOLVED** |
| BUG-03 | Bug | Auto pose seeding uses stale vision data | Medium | **RESOLVED** |
| BUG-04 | Bug | Vision telemetry shows stale values | Low | **RESOLVED** |
| INEFF-01 | Inefficiency | Multiple `getLatestResult()` calls per loop | Medium | **RESOLVED** |
| INEFF-02 | Inefficiency | SmartDashboard updates at 50 Hz | Medium | **RESOLVED** |
| BUG-06 | Bug | `PPLTV_MAX_VELOCITY = 9.0` vs actual max 3.0 m/s | Medium | Open |
| BUG-07 | Bug | `AutoBuilder.configure()` called multiple times | Medium | Open |
| BUG-05 | Config | Swerve fields in diff-drive PathPlanner settings | Low | Open |
| INEFF-03 | Inefficiency | `List.contains()` for tag dedup and lookup | Low | Partial |
| BUG-08 | Bug | Public vision geometry API uses non-fresh cache | Low | Open |
| BUG-09 | Bug | Driver camera construction has no failure guard | Low | Open |
| INEFF-04 | Inefficiency | Vision reprocesses same frame every loop | Low | Open |
| INEFF-05 | Inefficiency | Telemetry computes all 6 field-pose distances every cycle | Low | Open |
| BUG-10 | Quality | AdvantageKit `ProjectName` = `"MyProject"` (template default) | Low | Open |

---

*Updated 2026-02-23 against the `hieb-trial` branch. Previously resolved issues are retained for historical tracking.*
