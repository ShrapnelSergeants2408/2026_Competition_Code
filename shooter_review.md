# Shooter Review

## Bugs and Risks
- High: Jam clear is unlikely to trigger because `FEEDER_SPIKE_THRESHOLD_AMPS` (35A) is higher than the SparkMAX current limit (30A), so the limiter can clamp before the threshold is exceeded. Lower the threshold or base jam detection on a different signal. See `src/main/java/frc/robot/Constants.java:79`, `src/main/java/frc/robot/Constants.java:82`, `src/main/java/frc/robot/subsystems/Shooter.java:328`.
- High: `getAutonomousCommand()` passes `null` to `drivetrain.initializePose(...)` when the auto chooser has no selection, which can lead to an NPE if `initializePose` does not handle null. Guard or provide a default auto. See `src/main/java/frc/robot/RobotContainer.java:175`, `src/main/java/frc/robot/RobotContainer.java:176`.
- Medium: Zone gating uses the freshest vision pose when available and never cross-checks odometry, so a single bad tag solve can block shooting in-zone or allow shooting out-of-zone. Consider validating vision against odometry or checking ambiguity before trusting it. See `src/main/java/frc/robot/subsystems/Shooter.java:214`.

## Inefficiencies
- Medium: In test mode, `updateSmartDashboardTuning()` applies slot 0 configs every 20ms even if values have not changed, which can spam CAN and slow down other traffic. Cache values or throttle updates. See `src/main/java/frc/robot/subsystems/Shooter.java:487`.
- Low: Telemetry publishes many SmartDashboard values every loop without throttling. Consider using a loop counter or `NetworkTableEntry` caching to reduce bandwidth. See `src/main/java/frc/robot/subsystems/Shooter.java:497`.

## Potential Enhancements
- `visionDistanceFt` and `odometryDistanceFt` are never reset when data is unavailable, so telemetry can show stale values. Reset them to -1 when measurements are missing or expired. See `src/main/java/frc/robot/subsystems/Shooter.java:242`, `src/main/java/frc/robot/subsystems/Shooter.java:258`.
- `povPresetSet` is never cleared and `distanceSource` is not updated when a preset is staged, so the UI can show "Default" until a command calls `resolveShooterDistance`. Add a clear method and update `distanceSource` in `setDistancePreset`. See `src/main/java/frc/robot/subsystems/Shooter.java:62`, `src/main/java/frc/robot/subsystems/Shooter.java:166`.
- `DISTANCES_FEET` and `DISTANCE_RPM_MAP` are assumed to be the same length and sorted. Add a static validation to avoid silent interpolation errors if they diverge. See `src/main/java/frc/robot/subsystems/Shooter.java:137`, `src/main/java/frc/robot/Constants.java:95`.
- "NO IMPERIAL UNITS" contradicts the shooter distance map in feet. Either convert to meters or update the comment to avoid confusion. See `src/main/java/frc/robot/Constants.java:26`, `src/main/java/frc/robot/Constants.java:95`.
- Comments contain garbled characters (likely encoding issues), which makes documentation harder to read. Consider re-encoding to UTF-8 and cleaning comments. See `src/main/java/frc/robot/subsystems/Shooter.java:37`, `src/main/java/frc/robot/RobotContainer.java:42`, `src/main/java/frc/robot/Constants.java:64`.

## Testing Suggestions
- Verify jam clear behavior with actual current limits and stall conditions after adjusting `FEEDER_SPIKE_THRESHOLD_AMPS`.
- Confirm `initializePose` handles null autos or add a default auto and test with chooser set to "None".
- Test zone gating with intentionally noisy vision to ensure odometry fallback behavior matches expectations.
