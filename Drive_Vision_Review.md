# Drive, Vision, and Autonomous Code Review

**Date:** 2026-02-24 (updated)
**Branch:** `hieb-trial`
**Scope:** `DriveTrain.java`, `Vision.java`, `RobotContainer.java`, `Robot.java`, `Constants.java`, `VisionMeasurement.java`, `pathplanner/settings.json`

---

## Status of Previously Identified Issues

### Previously Identified Issues — RESOLVED

All bugs and inefficiencies identified across both review sessions are now resolved.

| ID | Title | How It Was Fixed |
|---|---|---|
| BUG-01 | Field-oriented tank rotates turn component | `fieldOrientedTank()` now only projects `fwd` through `cos(heading)`; `turn` stays robot-relative |
| BUG-02 | `setVisionEnabled()` cannot disable vision | Public setter removed; vision check inlined in `periodic()` each loop |
| BUG-03 | Auto pose seeding uses stale vision data | `initializePose()` now calls `getBestVisionMeasurementIfFresh()`, gated on `MAX_VISION_AGE_SECONDS` |
| BUG-04 | Vision telemetry shows stale values | The no-measurement else-branch in `updateTelemetry()` now explicitly writes `NaN`/`0`/`false` to all distance and alignment keys |
| BUG-05 | Swerve module positions in diff-drive PathPlanner settings | Swerve-only keys (`flModuleX/Y`, etc.) removed from `settings.json`; file is a clean differential drive config |
| BUG-06 | `PPLTV_MAX_VELOCITY` is 3× the robot's actual top speed | `PPLTV_MAX_VELOCITY` now references `MAX_ROBOT_VELOCITY_MPS` (4.572 m/s); both limits derived from the same constant |
| BUG-07 | `AutoBuilder.configure()` called multiple times | `configurePathPlanner()` removed from `refreshLtvControllerFromDashboard()`; `AutoBuilder.configure()` now called exactly once in the constructor |
| BUG-08 | Public vision geometry API uses non-fresh cached measurement | `getDistanceToPose(Pose2d)` and `getYawToPose(Pose2d)` now call `getBestVisionMeasurementIfFresh()` |
| BUG-09 | Driver camera construction has no failure guard | Camera initialization wrapped in try/catch; `DriverStation.reportWarning()` called on failure |
| BUG-10 | AdvantageKit `ProjectName` metadata is the WPILib template default | Updated to `"2408_2026_Competition"` in `Robot.java` |
| INEFF-01 | Multiple `getLatestResult()` calls per loop | `cachedFrontResult`/`cachedRearResult` set once in `Vision.periodic()` and consumed via `getFrontResult()`/`getRearResult()` everywhere else |
| INEFF-02 | SmartDashboard updates at 50 Hz | Both `Vision` and `DriveTrain` use `telemetryLoopCounter` with `TELEMETRY_PERIOD_LOOPS = 5`, running at ≈10 Hz |
| INEFF-03 | O(n) `contains()` for tag deduplication and lookup | `List<Integer> cachedVisibleTags` replaced by `boolean[] cachedTagsVisible[31]`; `isTagVisible()` is an O(1) bounds-checked array access; `computeVisibleTags()` is a single forward pass with no dedup loop |
| INEFF-04 | Vision reprocesses the same camera frame every loop | `periodic()` now compares incoming frame timestamps against `lastFrontTimestamp`/`lastRearTimestamp`; skips `computeBestVisionMeasurement()` and `computeVisibleTags()` when neither camera has a new frame |
| INEFF-05 | Telemetry computes geometry for all six field poses every cycle | `updateTelemetry()` uses private overloads that accept the pre-fetched `Optional<VisionMeasurement>` directly; no redundant public `getBestVisionMeasurement()` calls inside the telemetry path |
| E7 | `visionEnabled` field is unnecessary mutable state | Field removed from `DriveTrain`; check inlined as `if (visionSubsystem != null && visionSubsystem.isAnyVisionAvailable())` in `periodic()` |

### Previously Identified Issues — STILL OPEN

*(none — all identified bugs and inefficiencies resolved)*

---

## Active Bugs and Risks

*(none — all active bugs resolved as of 2026-02-24)*

---

## Remaining Inefficiencies

*(none — all identified inefficiencies resolved as of 2026-02-24)*

---

## Enhancement Recommendations (Open)

The following elegance and architecture improvements were identified but not yet implemented.
They are non-blocking and have no impact on robot correctness or performance at this time.

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

This makes the heading math visible in exactly one place, clearly documents the intentional omission of the `sin` component, and simplifies any future swerve migration.

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

### E8 — Elegance: Document field-oriented heading-loss behavior

Field-oriented tank/arcade projects driver-forward intent via `cos(heading)`. When the robot faces ±90° from field-forward, the joystick forward response drops to zero — correct physics for a non-holonomic drive, but potentially confusing to a driver.

Recommended improvements:
- Add `SmartDashboard.putNumber("DriveTrain/FOEfficiency", Math.abs(Math.cos(heading)))` so drivers can see on the dashboard when forward responsiveness is reduced.
- Add a doc-comment to `fieldOrientedTank()` and `fieldOrientedArcade()` stating: *"When heading is ±90° from field-forward, `cos(heading) ≈ 0` and forward output is suppressed by design; the robot must rotate to re-align before field-forward motion resumes."*

---

### E9 — Architecture: Complete AdvantageKit integration or reduce its scope

`Robot` extends `LoggedRobot` and writes to `WPILOGWriter`/`NT4Publisher`, but subsystems write directly to `SmartDashboard` rather than through `Logger.recordOutput()`. This is a partial integration: the `LoggedRobot` framework overhead is present, but subsystem-level replay in AdvantageScope is not available.

Two clear paths:
- **Minimal (reduce scope):** Revert to `TimedRobot`, keep `SmartDashboard` telemetry. Eliminates the AdvantageKit dependency cost with no loss of current functionality.
- **Full integration:** Replace `SmartDashboard.put*()` in periodic methods with `Logger.recordOutput()` and implement the IO layer per AdvantageKit conventions. Enables full match replay and structured logging.

---

## Priority Table

| ID | Category | Title | Severity | Status |
|---|---|---|---|---|
| BUG-01 | Bug | Field-oriented tank rotates turn component | High | **RESOLVED** |
| BUG-02 | Bug | `setVisionEnabled()` cannot disable vision | Medium | **RESOLVED** |
| BUG-03 | Bug | Auto pose seeding uses stale vision data | Medium | **RESOLVED** |
| BUG-04 | Bug | Vision telemetry shows stale values | Low | **RESOLVED** |
| BUG-06 | Bug | `PPLTV_MAX_VELOCITY` mismatched with actual max speed | Medium | **RESOLVED** |
| BUG-07 | Bug | `AutoBuilder.configure()` called multiple times | Medium | **RESOLVED** |
| BUG-05 | Config | Swerve fields in diff-drive PathPlanner settings | Low | **RESOLVED** |
| INEFF-01 | Inefficiency | Multiple `getLatestResult()` calls per loop | Medium | **RESOLVED** |
| INEFF-02 | Inefficiency | SmartDashboard updates at 50 Hz | Medium | **RESOLVED** |
| INEFF-03 | Inefficiency | `List.contains()` for tag dedup and lookup | Low | **RESOLVED** |
| BUG-08 | Bug | Public vision geometry API uses non-fresh cache | Low | **RESOLVED** |
| BUG-09 | Bug | Driver camera construction has no failure guard | Low | **RESOLVED** |
| INEFF-04 | Inefficiency | Vision reprocesses same frame every loop | Low | **RESOLVED** |
| INEFF-05 | Inefficiency | Telemetry computes all 6 field-pose distances every cycle | Low | **RESOLVED** |
| BUG-10 | Quality | AdvantageKit `ProjectName` = `"MyProject"` (template default) | Low | **RESOLVED** |
| E7 | Elegance | `visionEnabled` field is unnecessary mutable state | Low | **RESOLVED** |
| E5 | Elegance | Extract field-forward projection to a shared helper | Low | Open |
| E6 | Elegance | Replace multi-branch camera selection with Comparator | Low | Open |
| E8 | Elegance | Document field-oriented heading-loss behavior | Low | Open |
| E9 | Architecture | Complete AdvantageKit integration or reduce scope | Medium | Open |

---

*Updated 2026-02-24 against the `hieb-trial` branch. All bugs and inefficiencies from both review sessions are resolved. Remaining open items (E5, E6, E8, E9) are non-blocking elegance and architecture enhancements.*
