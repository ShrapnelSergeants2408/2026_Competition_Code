# Troubleshooting Log — 2026 Competition Code
**Date:** 2026-03-16
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

#### Phase 5 — Deterministic Pattern Identified (Root Cause)
- Discovered a perfectly repeatable pattern **without pressing B and without running the shooter**:
  1. Press inhale → exhale and shoot **work** immediately after.
  2. Press exhale or shoot a second time (without inhale first) → trigger motor **does not run**.
  3. Press inhale again → exhale and shoot **work** again.
- This deterministic behavior ruled out all electrical/brownout/CAN theories.

---

### Investigated and Eliminated Causes

| Candidate | Why Eliminated |
|---|---|
| `stopAllCommand` cancellation breaking `whileTrue` re-arming | `whileTrue` won't reschedule without a false→true edge; real issue was elsewhere |
| Double binding of `shootCommand` to RT and A button | Two command instances caused scheduler state confusion; A binding was commented out |
| Missing `finallyDo` on feeder commands | Motors kept running after button release; `finallyDo(() -> stopAll())` reinstated on all three commands |
| SparkMAX soft/hard limits | Confirmed not set via REV Hardware Client |
| CAN bus congestion from Phoenix 6 (TalonFX) | Consistent with some symptoms but ruled out by deterministic reproduce without shooter |
| Brownout / PDH slot proximity | Moving TalonFX to opposite PDH side had no effect; PDH-level cause eliminated |
| Battery voltage sag | Possible contributing factor for the `kHasReset` fault but not the root cause of direction failure |

---

### Root Cause

**Brushless motor commutation dead zone combined with `IdleMode.kBrake` on the trigger motor.**

The trigger motor (NEO + SparkMAX CAN 32) uses hall effect sensors and trapezoidal commutation (6 sectors per electrical revolution). At certain rotor positions — particularly near commutation sector boundaries — the electromagnetic starting torque in one direction is below the mechanism's static friction threshold.

`IdleMode.kBrake` actively shorts the motor phases when output is zero, holding the rotor at a stable magnetic equilibrium. After any negative-direction run (exhale or shoot), the motor **reliably parks at the same equilibrium position** — which happens to be a dead zone for restarting in the negative direction.

From this parked position:
- **Positive direction (inhale):** sufficient torque — motor starts ✓
- **Negative direction (exhale/shoot):** insufficient torque at this rotor angle — motor does not move ✗

Running inhale first moves the rotor to a different position where negative-direction torque is sufficient, restoring exhale and shoot.

This also explains why the `kHasReset` sticky fault appeared to "fix" the issue when cleared via REV Hardware Client — the tool briefly releases the brake, allowing the rotor to drift off the dead zone position. The shooter's vibration (TalonFX spinning) similarly shook the assembly enough to settle the rotor at the dead zone position before the next feeder command.

---

### Code Changes Made This Session

1. **`RobotContainer.java`** — Commented out the A-button binding for `shootCommand()`.
   Previously both RT and A called `feeder.shootCommand()`, creating two separate command instances that caused WPILib scheduler state confusion when one was externally cancelled.

2. **`Feeder.java`** — Reinstated `finallyDo(() -> stopAll())` on `intakeCommand()`, `ejectCommand()`, and `shootCommand()`.
   Without this, motors kept running at their last set speed after a button was released, causing inconsistent state between commands.

3. **`Feeder.java`** — Changed trigger motor idle mode from `IdleMode.kBrake` to `IdleMode.kCoast`.
   In coast mode the rotor is not actively held between commands and drifts to different angular positions, preventing the deterministic dead zone parking behavior.

---

### Remaining Notes

- The `stopAllCommand()` in `RobotContainer` has no subsystem requirements by design (so it can always be scheduled). This means after B is pressed and a `whileTrue`-bound command is externally cancelled, the user must **fully release and re-press** the button to restart — `whileTrue` only reschedules on a false→true rising edge.
- The `kHasReset` sticky fault on CAN 32 should be monitored. If it continues to appear after the coast mode fix, a genuine brownout (battery voltage under TalonFX load) may still be present and warrants electrical inspection of the trigger motor's power wiring.
