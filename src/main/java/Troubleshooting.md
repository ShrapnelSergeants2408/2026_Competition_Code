# Troubleshooting Log — 2026 Competition Code
**Date:** 2026-03-16 / 2026-03-17
**Branch:** hieb-trial

---

## Issue: Trigger Motor (CAN 32) Not Running After Certain Operations

### Subsystems Involved
- `Feeder.java` — trigger motor (SparkMAX CAN 32), intake motor (SparkMAX CAN 31)
- `RobotContainer.java` — `stopAllCommand()`, button bindings
- `Shooter.java` — shooter flywheel (TalonFX CAN 30), `spinUpCommand()`

---

### Symptom Timeline

#### Phase 1 — Initial Report
- Inhale (LB) and exhale (RB) worked correctly (both motors).
- Shoot (RT) worked the **first time** after enable.
- After pressing **B button** (`stopAllCommand`), the trigger motor no longer ran for shoot. Inhale and exhale continued to work.

#### Phase 2 — Direction-Specific Failure Identified
- On repeated testing (without B pressed), inhale worked (trigger +1.0), but exhale and shoot failed (trigger −1.0).
- The intake motor ran correctly in all cases.
- Pattern: **positive direction worked, negative direction did not.**

#### Phase 3 — Sticky Fault Found
- REV Hardware Client revealed a **`kHasReset` sticky fault** (reset warning) on CAN 32.
- After clearing the fault, all three functions worked on the first run.
- After disable/re-enable, all three still worked.
- After running the **shooter (Y button / `spinUpCommand`)**, the trigger motor stopped working again.

#### Phase 4 — PDH Slot Investigation
- TalonFX (CAN 30) was moved to the **opposite side of the PDH**.
- Result: **no change** — same failure pattern persisted.
- First trial: inhale worked, exhale/shoot trigger failed.
- Second trial: all three worked.
- Third trial: shooter (Y) run → trigger motor failed again.

#### Phase 5 — Deterministic Pattern Identified
- Discovered a repeatable pattern **without pressing B and without running the shooter**:
  1. Press inhale → exhale and shoot **work** immediately after.
  2. Press exhale or shoot a second time (without inhale first) → trigger motor **does not run**.
  3. Press inhale again → exhale and shoot **work** again.

#### Phase 6 — Brownout Identified as Root Cause
- Reducing intake/trigger motors from 100% to 50% stopped brownouts.
- Switching between inhale and exhale caused **30–40A current spikes** (0A draw during trigger failure = brownout had already occurred).
- During shooter spin-up, PDH bus voltage **drops to ~9.6V** before recovering.
- At 9.6V on a 12V system, SparkMAX may reboot → `kHasReset` sticky fault.
- Physical assist on the trigger motor shaft during failure produced no response → motor was in post-brownout lockout state, not a commutation dead zone.

#### Phase 7 — Ramp Rates Added
- `openLoopRampRate(0.15)` added to both SparkMAX motors (intake CAN 31 and trigger CAN 32).
- `VoltageClosedLoopRampPeriod(0.25)` and `SupplyCurrentLimit(40A)` added to TalonFX (shooter CAN 30).
- Result: brownout symptoms reduced but **CAN 32 continued failing intermittently** even after clearing `kHasReset`.

#### Phase 8 — Firmware Fault Confirmed Hardware Damage
- Simultaneous **LB + RT press** (inhale + shoot) caused near-instantaneous direction reversal on trigger motor.
- REV Hardware Client reported **firmware fault + kHasReset** on CAN 32.
- Firmware fault = internal SparkMAX controller damage from repeated brownout events.
- **Conclusion: CAN 32 SparkMAX is damaged hardware. Replace the unit.**

---

### Investigated and Eliminated Causes

| Candidate | Why Eliminated |
|---|---|
| `stopAllCommand` cancellation breaking `whileTrue` re-arming | `whileTrue` won't reschedule without a false→true edge; real issue was elsewhere |
| Double binding of `shootCommand` to RT and A button | Two command instances caused scheduler state confusion; A binding was commented out |
| Missing `finallyDo` on feeder commands | Motors kept running after button release; `finallyDo(() -> stopAll())` reinstated on all three commands |
| SparkMAX soft/hard limits | Confirmed not set via REV Hardware Client |
| CAN bus congestion from Phoenix 6 (TalonFX) | Consistent with some symptoms but ruled out by deterministic reproduce without shooter |
| Brownout / PDH slot proximity | Moving TalonFX to opposite PDH side had no effect on direction-failure pattern |
| Brushless motor commutation dead zone (brake mode) | Coast mode showed no improvement; physical assist during failure also failed → not rotor position |
| Battery voltage sag from SparkMAX alone | Pattern was reproducible; sag from TalonFX during spin-up was the main contributor |

---

### Root Cause

**CAN 32 SparkMAX hardware damage from repeated brownout resets.**

The trigger motor SparkMAX (CAN 32) has been browned out multiple times. SparkMAX brownout occurs when VIN drops below ~5.5V for >1ms — this causes the controller to reboot and record a `kHasReset` sticky fault. Multiple brownout resets cause cumulative stress on the controller's internal power management circuitry.

The damaged controller intermittently fails during normal operation (random single-direction failures after the `kHasReset` fault is set) and confirmed failure mode under a firmware fault triggered by simultaneous LB+RT (direction reversal under load).

Contributing cause: **excessive current spikes** from zero `openLoopRampRate` on SparkMAX motors and zero `VoltageClosedLoopRampPeriod` on TalonFX, combined with a **30–40A instantaneous draw** on direction changes and **9.6V bus voltage** during shooter spin-up. All of these lowered the margin for CAN 32 to survive without brownout.

---

### Code Changes Made This Session

1. **`RobotContainer.java`** — Commented out the A-button binding for `shootCommand()`.
   Previously both RT and A called `feeder.shootCommand()`, creating two separate command instances that caused WPILib scheduler state confusion when one was externally cancelled.

2. **`Feeder.java`** — Reinstated `finallyDo(() -> stopAll())` on `intakeCommand()`, `ejectCommand()`, and `shootCommand()`.
   Without this, motors kept running at their last set speed after a button was released, causing inconsistent state between commands.

3. **`Feeder.java`** — Added `openLoopRampRate(0.15)` to both `intakeConfig` and `triggerConfig`.
   Limits the rate of duty cycle change to reduce 30–40A instantaneous current spikes on direction reversal, reducing PDH bus voltage sag.

4. **`Shooter.java`** — Added `withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.25))`.
   Limits the rate of voltage output change during TalonFX velocity PID to reduce the 9.6V bus voltage sag during shooter spin-up.

5. **`Shooter.java`** — Added `withSupplyCurrentLimit(40)` and `withSupplyCurrentLimitEnable(true)` to `CurrentLimitsConfigs`.
   Supply current limit restricts battery draw from the TalonFX, reducing worst-case PDH bus voltage sag.

6. **`Feeder.java`** — Added constructor return value checking for both `configure()` calls.
   `SparkMax.configure()` returns `REVLibError`; previously ignored. Now calls `DriverStation.reportWarning()` on failure so config errors appear in the DS log.

7. **`Shooter.java`** — Added constructor return value checking for `getConfigurator().apply()`.
   `TalonFX.getConfigurator().apply()` returns `StatusCode`; previously ignored. Now calls `DriverStation.reportWarning()` on failure.

---

### Remaining Work

#### Hardware (Required Before Competition)
- **Replace CAN 32 SparkMAX** — firmware fault confirmed the unit is damaged. A replacement will eliminate the intermittent trigger motor failures.
- **Verify `TRIGGER_MOTOR_INVERTED`** — this constant has a `TODO: verify on bench; CW must be positive` comment and has never been confirmed correct. Wrong inversion means the motor works against a hard stop in one direction, causing additional current spikes.

#### Software
- After CAN 32 replacement, **re-run full test sequence**: inhale → Y (shooter) → rapid RT presses → no `kHasReset` should appear.
- **Investigate roboRIO RAM (3MB free)** — `CameraServer.startAutomaticCapture` at 320×240/30fps is the primary suspect. Camera was not connected during most of this troubleshooting session, so it could not be confirmed as a culprit. Camera was removed from testing by physical disconnection. RAM should be measured with camera connected before competition.

#### Behavior Notes
- `stopAllCommand()` has no subsystem requirements by design (so it can always be scheduled). After B is pressed and a `whileTrue`-bound command is externally cancelled, the user must **fully release and re-press** the button to restart — `whileTrue` only reschedules on a false→true rising edge.
- SparkMAX `configure()` with `ResetMode.kResetSafeParameters` and `PersistMode.kPersistParameters` re-applies config on every enable (parameters are persisted to flash). This means a brownout reset will correctly restore the ramp rate and current limit settings when the controller reboots.
