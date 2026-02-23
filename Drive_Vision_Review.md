# Drive, Vision, and Autonomous Code Review
**Date:** 2026-02-23
**Scope:** `src/main/java/frc/robot/subsystems/DriveTrain.java`, `src/main/java/frc/robot/subsystems/Vision.java`, `src/main/java/frc/robot/RobotContainer.java`, `src/main/deploy/pathplanner/settings.json`

---

## Findings: Bugs and Risks

### BUG-01 - Field-oriented tank rotates the turn component (non-spatial)
**Severity:** High
**File:** `src/main/java/frc/robot/subsystems/DriveTrain.java:272-279`

`fieldOrientedTank()` treats the derived `turn` value as a spatial axis and rotates it with the heading. `turn` is an angular rate, not a field-frame vector, so rotating it produces heading-dependent coupling between forward speed and turn rate. This makes the robot feel different at different headings and can cause unexpected turning when pushing straight.

**Fix:** Only rotate the forward component. Keep `turn` robot-relative, then recompose left/right from `robotFwd` and `turn`.

---

### BUG-02 - `setVisionEnabled()` cannot actually disable vision
**Severity:** Medium
**File:** `src/main/java/frc/robot/subsystems/DriveTrain.java:108-112, 411-413`

`visionEnabled` is overwritten every loop with `visionSubsystem.isAnyVisionAvailable()`. Any manual call to `setVisionEnabled(false)` is undone on the next periodic. The API implies a manual override, but it does not work.

**Fix:** Split into `manualVisionEnabled` and `cameraHealthy`, and compute `visionEnabled = manualVisionEnabled && cameraHealthy`, or remove `setVisionEnabled()` entirely if auto-enable is the only intended behavior.

---

### BUG-03 - Autonomous pose seeding can use stale vision data
**Severity:** Medium
**File:** `src/main/java/frc/robot/subsystems/DriveTrain.java:190-216`

`initializePose()` uses the latest cached vision measurement without checking its age. If the last vision measurement was captured during disabled or before the robot was positioned, the auto start pose can be wrong.

**Fix:** Reject measurements older than a small threshold (e.g., 0.2-0.5s using `Timer.getFPGATimestamp()`), or require `visionSubsystem.isAnyVisionAvailable()` and a fresh timestamp before seeding.

---

### BUG-04 - Vision telemetry values can go stale when measurement disappears
**Severity:** Low
**File:** `src/main/java/frc/robot/subsystems/Vision.java:473-548`

When no measurement is available, pose fields are cleared, but distance/alignment keys are only updated inside `ifPresent`. Old values remain on the dashboard and look valid.

**Fix:** On the no-measurement path, write NaN/0 and set all alignment booleans to false.

---

### BUG-05 - PathPlanner settings contain swerve module positions for a diff drive
**Severity:** Low (config risk)
**File:** `src/main/deploy/pathplanner/settings.json:25-32`

`holonomicMode` is false, but swerve module positions are still present. They are ignored today, but if holonomic mode is ever toggled by mistake, these values are wrong for a differential drive.

**Fix:** Remove swerve-only fields or regenerate settings as a differential drive project.

---

## Inefficiencies

### INEFF-01 - `getLatestResult()` is called multiple times per loop per camera
**File:** `src/main/java/frc/robot/subsystems/Vision.java:91-112, 334-354, 288-314, 603-605`

Each loop runs `computeBestVisionMeasurement()` and `computeVisibleTags()`, both of which call `getLatestResult()` for each camera. `getBestTarget()` also calls it again if used by commands. This doubles or triples camera result reads and allocations per loop.

---

### INEFF-02 - SmartDashboard updates run at full rate with many keys
**File:** `src/main/java/frc/robot/subsystems/Vision.java:473-548`, `src/main/java/frc/robot/subsystems/DriveTrain.java:425-434`

Dozens of `SmartDashboard.put*` calls run every 20ms. This creates unnecessary NetworkTables traffic and CPU load, especially on the Rio.

---

### INEFF-03 - Visible tag list uses repeated `contains()` checks
**File:** `src/main/java/frc/robot/subsystems/Vision.java:334-354`

`computeVisibleTags()` uses `List.contains()` in a loop, which is O(n^2) and boxes integers. The tag count is low, but the pattern is still avoidable.

---

## Enhancements: Reduce Computing Overhead / Improve Speed

1) **Cache camera results once per loop**
   - Add per-loop `PhotonPipelineResult` caches for front/rear in `Vision.periodic()` and pass them into `computeBestVisionMeasurement()`, `computeVisibleTags()`, and `getBestTarget()`.
   - This avoids duplicate result reads and repeated list creation each loop.

2) **Add vision measurement staleness gating**
   - In `Vision`, expose a method to check measurement age using `Timer.getFPGATimestamp()` and a constant (e.g., `MAX_VISION_AGE_SEC`).
   - Use it in `DriveTrain.updateVisionMeasurements()` and `initializePose()` to avoid fusing old data and wasting estimator updates.

3) **Throttle dashboard updates**
   - Update SmartDashboard at 5-10Hz using a counter or `Timer`, or move high-rate telemetry to AdvantageKit logs.
   - Reduces NT bandwidth and CPU usage during auto.

4) **Use a set or boolean map for visible tags**
   - Replace `List.contains()` with a simple boolean array or `IntOpenHashSet`-style structure to avoid per-loop boxing and quadratic checks.

---

## Enhancements: Improve Code Elegance

1) **Clarify vision enable behavior**
   - Replace the single `visionEnabled` flag with explicit `manualVisionEnabled` + `cameraHealthy` logic, or remove the public setter entirely.

2) **Make field-oriented driving consistent**
   - Share a helper that only rotates the forward component for both arcade and tank. Keep `turn` robot-relative to avoid heading-dependent coupling.

3) **Comparator-based vision selection**
   - Replace the current multi-branch selection with a comparator (prefer more tags, then lower avg distance, then lower ambiguity). This is shorter and easier to tune.

4) **Consolidate dashboard key updates**
   - Use small helper methods for "clear vision telemetry" and "write vision telemetry" to keep `updateTelemetry()` readable and consistent.

---

## Priority Table

| ID | Category | Title | Severity |
|---|---|---|---|
| BUG-01 | Bug | Field-oriented tank rotates turn component | High |
| BUG-02 | Bug | `setVisionEnabled()` cannot disable vision | Medium |
| BUG-03 | Bug | Auto pose seeding can use stale vision data | Medium |
| BUG-04 | Bug | Vision telemetry can show stale values | Low |
| BUG-05 | Config | Swerve fields in diff-drive PathPlanner settings | Low |
| INEFF-01 | Inefficiency | Multiple `getLatestResult()` calls per loop | Medium |
| INEFF-02 | Inefficiency | SmartDashboard spam at 50Hz | Medium |
| INEFF-03 | Inefficiency | O(n^2) tag list build with boxing | Low |

---

*Updated for the current code state in this workspace.*
