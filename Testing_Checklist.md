# 2026 Competition Robot — First Power-Up Testing Checklist

**Team:** FRC 2026 Season | **Branch:** hieb-trial
**Use this checklist sequentially. Do not skip sections. Mark each item ✅ when complete.**

---

## SAFETY FIRST

Before any power-up:
- Robot is on a stand with all four wheels clear of the ground
- All personnel are clear of the drivetrain wheels and shooter
- Emergency stop (DS enable/disable) is manned at all times
- Fire extinguisher is present in the pit
- Battery is fully charged (≥ 12.5 V before enabling)

---

## SECTION 1 — Firmware Updates (Pre-Power-Up)

> Complete all firmware updates before deploying robot code. These must be done with the robot powered but the **DS not enabled**. Follow 2026 official FRC update procedures.

---

### **1. Update roboRIO Firmware**

**Update the roboRIO 2.0 operating system image to the latest 2026 FRC image.**

1. Download the latest 2026 FRC roboRIO image from NI (National Instruments) via the FRC Game Tools installer.
2. Connect to the roboRIO via USB-B cable or Ethernet.
3. Open **roboRIO Imaging Tool** (installed with FRC Game Tools).
4. Select your roboRIO in the device list.
5. Select **Format Target** and choose the latest 2026 image file.
6. Enter your team number and click **Reformat**.
7. Wait for imaging to complete (~3–5 minutes). The RIO will reboot.
8. Verify the firmware version displayed matches the 2026 release version.
9. Confirm the RIO is accessible via the Driver Station after reboot.

- [ ] roboRIO reimaged successfully
- [ ] RIO visible in Driver Station (green communication light)
- [ ] Team number confirmed correct on RIO

---

### **2. Update Radio Firmware**

**Program the 2026 FRC game radio with the team's configuration using the FRC Radio Configuration Utility.**

1. Download the **FRC Radio Configuration Utility** from the WPILib documentation site (2026 version).
2. Connect the radio to the programming port via Ethernet.
3. Power the radio via PoE injector or the robot's radio power port.
4. Open the Radio Configuration Utility.
5. Select your radio model (OpenMesh OM5P-AC or as specified by FIRST for 2026).
6. Enter your team number and select the event SSID if applicable.
7. Click **Configure** and wait for programming to complete.
8. Reconnect the radio to the roboRIO on the robot network port.
9. Verify the Driver Station connects to the robot over the radio (not just USB).

- [ ] Radio firmware programmed successfully
- [ ] Robot communicates wirelessly with Driver Station
- [ ] Ping to 10.TE.AM.2 succeeds (replace TE.AM with your team number)

---

### **3. Update SparkMAX Firmware and CAN IDs (REV Hardware Client)**

**Flash all four SparkMAX motor controllers to the latest 2026 REV firmware and set their CAN IDs.**

Required CAN IDs from `Constants.DriveTrainConstants` and `Constants.ShooterConstants`:

| Motor                  | CAN ID | Location          |
|------------------------|--------|-------------------|
| Left Drive Lead        | **20** | DriveTrain        |
| Left Drive Follow      | **21** | DriveTrain        |
| Right Drive Lead       | **22** | DriveTrain        |
| Right Drive Follow     | **23** | DriveTrain        |
| Feeder/Intake Motor    | **40** | Shooter subsystem |

**Steps for each SparkMAX:**
1. Install **REV Hardware Client** (latest 2026 version) on the pit laptop.
2. Connect to each SparkMAX one at a time via USB-C cable.
3. In REV Hardware Client, go to **Firmware** → click **Update Firmware** to flash the latest release.
4. After flashing, go to the **Basic** tab and set the **CAN ID** per the table above.
5. Click **Burn Flash** to persist the CAN ID.
6. Verify the CAN ID is saved by disconnecting and reconnecting.

- [ ] Left Drive Lead (CAN 20) firmware updated and CAN ID set
- [ ] Left Drive Follow (CAN 21) firmware updated and CAN ID set
- [ ] Right Drive Lead (CAN 22) firmware updated and CAN ID set
- [ ] Right Drive Follow (CAN 23) firmware updated and CAN ID set
- [ ] Feeder/Intake SparkMAX (CAN 40) firmware updated and CAN ID set

---

### **4. Update PDH Firmware and CAN ID (REV Hardware Client)**

**Flash the Power Distribution Hub to the latest 2026 REV firmware and confirm its CAN ID.**

Required CAN ID: **1** (REV PDH default; WPILib `PowerDistribution` constructor expects CAN ID 1 for PDH unless overridden)

1. Connect to the PDH via USB-C cable (dedicated USB port on the PDH).
2. Open **REV Hardware Client** and select the PDH from the device list.
3. Go to **Firmware** → click **Update Firmware** and flash the latest 2026 release.
4. Confirm the CAN ID is set to **1** in the Basic tab.
5. Click **Burn Flash** to persist.
6. Open the **Power** tab and verify all channels report 0 A with the robot disabled.
7. Verify the PDH is visible on the CAN bus (green in REV Hardware Client's CAN bus view).

- [ ] PDH firmware updated to latest 2026 release
- [ ] PDH CAN ID confirmed as **1**
- [ ] All PDH channels visible and reporting in REV Hardware Client
- [ ] PDH total current reads ~0 A when robot is disabled

---

### **5. Update TalonFX Firmware and CAN ID (Tuner X)**

**Flash the TalonFX shooter motor controller to the latest 2026 Phoenix 6 firmware and set its CAN ID.**

Required CAN ID from `Constants.ShooterConstants`:

| Motor          | CAN ID | Subsystem |
|----------------|--------|-----------|
| Shooter Wheel  | **30** | Shooter   |

1. Install **Phoenix Tuner X** (latest 2026 version) on the pit laptop.
2. Connect to the roboRIO via USB or over the robot network.
3. In Tuner X, navigate to **CANivore / roboRIO** and select the TalonFX device.
4. Click **Update** and flash the latest Phoenix 6 firmware.
5. After flashing, click on the device and set the **Device ID** to **30**.
6. Click **Set ID** and confirm the change.
7. Verify the device reappears with CAN ID 30 in the device list.
8. Run the **Self-Test** (lightning bolt icon) — confirm no faults are reported.
9. Confirm the device shows on the CAN bus in both Tuner X and the Driver Station CAN diagnostics.

- [ ] TalonFX (CAN 30) firmware updated to latest Phoenix 6 2026 release
- [ ] TalonFX CAN ID confirmed as **30**
- [ ] TalonFX self-test passes with no faults
- [ ] TalonFX visible on CAN bus in Tuner X

---

## SECTION 2 — CAN Bus Verification

**Confirm the full CAN bus is healthy before deploying code.**

1. Power on the robot fully (main breaker on).
2. Connect via USB to the roboRIO (do not enable yet).
3. Open **REV Hardware Client** — verify the following devices appear:
   - SparkMAX CAN 20, 21, 22, 23 (green/connected)
   - SparkMAX CAN 40 (green/connected)
   - PDH CAN 1 (green/connected)
4. Open **Phoenix Tuner X** — verify:
   - TalonFX CAN 30 (connected, no faults)
5. Open the **Driver Station** → **Messages** tab — confirm no CAN bus errors appear at startup.
6. Open **SmartDashboard** or **Glass** — after deploying code, verify no "Device Not Found" errors in the console.

- [ ] All 5 SparkMAX controllers visible on CAN bus
- [ ] PDH visible on CAN bus
- [ ] TalonFX visible on CAN bus
- [ ] No CAN bus utilization errors (utilization < 90%)
- [ ] No "Device Not Found" or CAN timeout errors in DS console

---

## SECTION 3 — Code Deployment and Boot

**Deploy the robot code and verify clean startup.**

1. Open the project in VS Code with WPILib extension (2026).
2. Click **WPILib: Deploy Robot Code** (or `Ctrl+Shift+P` → Deploy).
3. Wait for the build and deploy to succeed — watch for build errors.
4. Watch the Driver Station console for startup exceptions.
5. Confirm the DS shows:
   - Robot Code: **green**
   - Communications: **green**
   - No warnings in the console about missing cameras or CAN devices.

- [ ] Code deploys without build errors
- [ ] Driver Station shows Robot Code green
- [ ] No Java exceptions or WPILib errors on startup
- [ ] PhotonVision connection warning is acceptable if cameras not yet running (note any)

---

## SECTION 4 — DriveTrain Tests

> **IMPORTANT:** Keep robot on the stand (wheels in air) for all motor direction tests. Lower to the ground only for driving tests.

### 4A — Motor Direction Verification (Wheels in Air)

**Confirm each SparkMAX drives the correct wheel in the correct direction.**

1. Enable the robot in **Teleop** mode.
2. Push the **left joystick forward** on Driver Controller (port 0) — in default Field-Oriented Tank mode, the left Y axis controls left wheel speed.
   - [ ] Left front wheel (CAN 20 lead) spins **forward**
   - [ ] Left rear wheel (CAN 21 follow) spins **forward** and matches left front
3. Push the **right joystick forward** — the right Y axis controls right wheel speed.
   - [ ] Right front wheel (CAN 22 lead) spins **forward**
   - [ ] Right rear wheel (CAN 23 follow) spins **forward** and matches right front
4. If any wheel spins backward, adjust `Constants.DriveTrainConstants` inversion or SparkMAX config.
5. Verify follow motors match lead motors at all times (no independent motion).

- [ ] Left side (CAN 20/21) drives forward when left stick pushed forward
- [ ] Right side (CAN 22/23) drives forward when right stick pushed forward
- [ ] Follow motors mirror lead motors precisely
- [ ] No motor controller faults (LEDs solid, not blinking fault codes)

### 4B — Drive Mode Toggle

**Verify Tank ↔ Arcade toggle and Field-Oriented ↔ Robot-Relative toggle.**

Default mode: **Field-Oriented Tank** (left Y = left wheel, right Y = right wheel).

| Button                | Controller | Action                                    |
|-----------------------|------------|-------------------------------------------|
| **Back** button       | Driver (0) | Toggle Tank / Arcade drive mode           |
| **Start** button      | Driver (0) | Toggle Field-Oriented / Robot-Relative    |

1. Enable Teleop. Confirm SmartDashboard `DriveTrain/DriveState` reads **"Field-Oriented Tank"**.
2. Press **Back** — verify `DriveTrain/DriveState` changes to **"Field-Oriented Arcade"**. Test that left stick controls speed and right stick controls rotation.
3. Press **Back** again — verify returns to **"Field-Oriented Tank"**.
4. Press **Start** — verify `DriveTrain/OrientationMode` changes to **"ROBOT_RELATIVE"**. Confirm robot drives robot-relative (heading-independent).
5. Press **Start** again — verify returns to **"FIELD_ORIENTED"**.

- [ ] Back button toggles Tank/Arcade mode (confirmed on SmartDashboard)
- [ ] Start button toggles Field-Oriented/Robot-Relative (confirmed on SmartDashboard)
- [ ] Arcade mode: left Y = forward, right Y = rotation ✓
- [ ] Tank mode: left Y = left wheels, right Y = right wheels ✓

### 4C — Joystick Deadband

**Verify joystick deadband prevents creep.**

1. Release both joysticks to center.
2. Confirm wheels do not spin (deadband of 0.05 is set in `JOYSTICK_DEADBAND`).
3. Slightly deflect each joystick below the deadband — wheels should remain still.

- [ ] Wheels stay still with joysticks centered
- [ ] Deadband functions — slight deflection does not cause movement

### 4D — Driving Test (Robot on Ground)

**Lower robot to ground and verify drivability.**

1. Lower robot to the floor.
2. Enable Teleop in Tank mode (default).
3. Drive forward ~10 ft — robot should track straight.
4. Drive backward ~10 ft.
5. Turn left and right in place (push sticks in opposite directions).
6. Verify SmartDashboard `DriveTrain/LeftDistMeters` and `RightDistMeters` increase/decrease as expected.
7. Verify `DriveTrain/HeadingDeg` updates as robot turns.

- [ ] Robot drives straight forward and backward
- [ ] Robot turns left and right without binding
- [ ] Encoder distance readings update correctly on SmartDashboard
- [ ] Heading reading updates on SmartDashboard

---

## SECTION 5 — Gyro (NavX) Tests

**Verify the NavX MXP gyro initializes and reports heading correctly.**

1. Power on robot and deploy code.
2. Open SmartDashboard — check `DriveTrain/HeadingDeg`.
3. Confirm it reads approximately **0.0°** after startup (gyro zeroes on boot).
4. Rotate the robot 90° clockwise — verify `HeadingDeg` reads approximately **-90°** (negative because CCW positive convention is used in code).
5. Rotate robot back — verify heading returns to near 0°.
6. In **Test** mode, verify no NavX errors appear in Driver Station console.

- [ ] NavX initializes without errors on startup
- [ ] Heading reads 0° at startup
- [ ] Heading changes correctly with robot rotation
- [ ] Heading returns to ~0° after rotating back to start position

---

## SECTION 6 — Shooter System Tests

> **SAFETY:** The shooter wheel will spin at high RPM. Ensure all personnel are clear of the shooter exit path before enabling. No game pieces should be loaded during initial spin-up tests.
>
> **NOTE:** In **Teleop** mode, the shooter is **zone-locked** — it will not spin if the robot pose is outside the offensive zone. Use **Test** mode to bypass the zone lock for bench/pit testing.

### 6A — Enable Test Mode for Shooter Testing

**Test mode bypasses the zone lock so the shooter can spin anywhere on the bench.**

1. In Driver Station, switch to **Test** mode (not Teleop).
2. Enable the robot.
3. SmartDashboard `Shooter/In Offensive Zone` may read **false** — this is expected in a pit; spin-up is still allowed in Test mode.

- [ ] Test mode enabled in Driver Station

### 6B — Shooter Spin-Up (Operator Y Button)

**Verify shooter motor spins up and reports RPM to SmartDashboard.**

| Button  | Controller   | Action                              |
|---------|--------------|-------------------------------------|
| **Y**   | Operator (1) | Toggle shooter spin-up (zone-locked)|

1. Enable robot in **Test** mode.
2. Press **Y** on Operator Controller (port 1) — toggle spin-up ON.
3. Verify on SmartDashboard:
   - `Shooter/State` changes to **"SPIN_UP"**
   - `Shooter/Current RPM` climbs from 0
   - `Shooter/Target RPM` shows the distance-resolved target (~2825 RPM at default 10 ft)
   - `Shooter/RPM At Speed` turns **true** when within ±50 RPM of target
4. Press **Y** again to toggle spin-up OFF.
5. Verify `Shooter/State` returns to **"IDLE"** and RPM coasts to 0.

- [ ] Shooter spins up on Y press
- [ ] Current RPM climbs and stabilizes on SmartDashboard
- [ ] "RPM At Speed" indicator goes true when within tolerance
- [ ] Shooter stops/coasts on second Y press
- [ ] No TalonFX faults during spin-up

### 6C — Emergency Stop (Operator B Button)

**Verify B button immediately stops all shooter and feeder activity.**

| Button  | Controller   | Action                                     |
|---------|--------------|--------------------------------------------|
| **B**   | Operator (1) | Emergency stop — stop shooter and feeder   |

1. Spin up shooter with Y button.
2. Press **B** — verify shooter immediately stops (coasts to zero).
3. Verify `Shooter/State` returns to **"IDLE"**.
4. Confirm feeder also stops (if running).

- [ ] B button stops shooter immediately
- [ ] State returns to IDLE after B press

### 6D — Full Shoot Sequence (Operator X Button)

**Verify X button initiates spin-up then auto-feeds when at speed. No game piece for this test.**

| Button  | Controller   | Action                                            |
|---------|--------------|---------------------------------------------------|
| **X**   | Operator (1) | Full shoot: spin up → auto-feed when at speed (hold) |

1. Enable in Test mode.
2. Hold **X** on Operator Controller.
3. Verify SmartDashboard:
   - `Shooter/State` = **"SPIN_UP"** initially
   - Once RPM is at target, `Shooter/State` transitions to **"FEED"** and `Feeder/Motor Current` increases slightly (feeder motor running)
   - `Shooter/Can Shoot` turns **true** when ready
4. Release X — shooter and feeder both stop, state returns to IDLE.

- [ ] X button triggers spin-up then auto-feeds when at speed
- [ ] Feeder motor activates when shooter reaches target RPM
- [ ] Releasing X stops both motors and returns to IDLE

### 6E — Feed-Only Command (Operator RB)

**Verify RB only runs feeder when shooter is already at target speed.**

| Button  | Controller   | Action                                      |
|---------|--------------|---------------------------------------------|
| **RB**  | Operator (1) | Feed only if shooter is at target speed (hold) |

1. Spin up shooter with Y button first.
2. Once `Shooter/RPM At Speed` is **true**, hold **RB**.
3. Verify `Feeder/Motor Current` increases (feeder running).
4. Release RB — feeder stops, shooter continues spinning.
5. Press Y again to stop shooter.

**Also test feeder protection:** Hold RB before spinning up — feeder should NOT run (shooter not at speed).

- [ ] RB starts feeder only when shooter is at target speed
- [ ] RB does not run feeder if shooter is below target speed
- [ ] Feeder stops when RB released; shooter continues

### 6G — Distance Preset (Operator POV / D-Pad)

**Verify POV buttons stage distance presets and update the target RPM.**

| Button        | Controller   | Distance Preset |
|---------------|--------------|-----------------|
| **POV Up**    | Operator (1) | 15.0 ft         |
| **POV Down**  | Operator (1) | 10.0 ft         |
| **POV Left**  | Operator (1) | 7.5 ft          |
| **POV Right** | Operator (1) | 12.5 ft         |

RPM targets (from `Constants.ShooterConstants.DISTANCE_RPM_MAP`):

| Distance | Expected RPM |
|----------|-------------|
| 7.5 ft   | 2700 RPM    |
| 10.0 ft  | ~2825 RPM (interpolated) |
| 12.5 ft  | 2950 RPM    |
| 15.0 ft  | 3100 RPM    |

1. Press **POV Up** — verify `Shooter/POV Preset Distance (ft)` = **15.0** and `Shooter/Distance Source` = **"POV Preset"**.
2. Spin up with Y — verify `Shooter/Target RPM` ≈ **3100**.
3. Stop shooter (B button), press **POV Left** — verify preset = **7.5 ft**.
4. Spin up with Y — verify target RPM ≈ **2700**.
5. Press **A** to clear preset — verify `Shooter/Distance Source` returns to **"Default"** or **"Odometry"**.

- [ ] POV Up sets 15.0 ft preset (3100 RPM target)
- [ ] POV Down sets 10.0 ft preset (~2825 RPM target)
- [ ] POV Left sets 7.5 ft preset (2700 RPM target)
- [ ] POV Right sets 12.5 ft preset (2950 RPM target)
- [ ] A button clears preset and returns to automatic distance resolution

---

## SECTION 7 — Feeder / Intake Tests

> **SAFETY:** Keep fingers clear of the feeder/intake mechanism. The feeder motor (SparkMAX CAN 40) runs open-loop at duty cycles up to 60%.

### 7A — Intake Toggle (Operator LB)

**Verify LB toggles intake on and off.**

| Button  | Controller   | Action                              |
|---------|--------------|-------------------------------------|
| **LB**  | Operator (1) | Toggle intake — draws ball in       |

1. Enable in Test mode.
2. Press **LB** — feeder motor should run **forward** at 50% duty cycle (INTAKE_SPEED = 0.5).
3. Verify `Shooter/State` = **"INTAKE"** and `Shooter/Intake Active` = **true** on SmartDashboard.
4. Press **LB** again — feeder stops, `Shooter/Intake Active` = **false**, state = **"IDLE"**.

- [ ] LB starts intake (feeder forward at 50%)
- [ ] SmartDashboard "Intake Active" turns true
- [ ] Second LB press stops intake
- [ ] State returns to IDLE after stop

### 7B — Eject Toggle (Operator LT)

**Verify LT toggles eject on and off.**

| Button  | Controller   | Action                              |
|---------|--------------|-------------------------------------|
| **LT**  | Operator (1) | Toggle eject — reverses feeder      |

1. Enable in Test mode.
2. Press **LT** (Left Trigger) past the activation threshold — feeder motor should run **reverse** at 50% (EJECT_SPEED = -0.5).
3. Verify `Shooter/State` = **"EJECT"** and `Shooter/Eject Active` = **true** on SmartDashboard.
4. Press **LT** again — feeder stops, state = **"IDLE"**.

- [ ] LT starts eject (feeder reverse at 50%)
- [ ] SmartDashboard "Eject Active" turns true
- [ ] Second LT press stops eject
- [ ] State returns to IDLE

### 7C — Jam Detection Test

**Verify automatic jam detection triggers a brief reverse to clear the feeder.**

> Jam clears automatically when feeder current exceeds **30A** for more than 100 ms. The feeder reverses at 50% for 0.25 seconds then resumes.

1. Enable intake with LB.
2. Manually stall the feeder mechanism (hold a game piece firmly against it to spike current).
3. Observe SmartDashboard — when current exceeds 30A for 100 ms, `Feeder/Jam Clearing` should turn **true** briefly (~0.25 sec).
4. After 0.25 seconds, jam clear ends and intake resumes.
5. Release game piece.

> **Note:** If testing without a game piece, this step may be skipped and verified in game-piece integration testing.

- [ ] Jam clear triggers when feeder current spikes above 30A
- [ ] "Feeder/Jam Clearing" indicator activates on SmartDashboard
- [ ] Feeder reverses briefly then resumes intake automatically
- [ ] Feeder Motor Current visible and plausible on SmartDashboard

---

## SECTION 8 — Shooter PID Tuning Verification (Test Mode Only)

**Verify SmartDashboard PID tuning is active in Test mode.**

The shooter reads PID gains from SmartDashboard in Test mode only (`Shooter/Tuning/kP`, `kI`, `kD`, `kV`). These are applied to TalonFX slot 0 every loop.

1. Enable in **Test** mode.
2. On SmartDashboard, confirm these entries exist with initial values from constants:
   - `Shooter/Tuning/kP` = 0.0
   - `Shooter/Tuning/kI` = 0.0
   - `Shooter/Tuning/kD` = 0.0
   - `Shooter/Tuning/kV` = 0.12
3. Spin up shooter (Y button). Note whether RPM converges to target or oscillates.
4. **Initial kV tuning:** The kV = 0.12 is the starting feedforward. If shooter doesn't reach target RPM, increase kV in small increments (0.01 at a time) until RPM converges.
5. After tuning, update `Constants.ShooterConstants` with the verified kV value.

- [ ] SmartDashboard tuning entries are present in Test mode
- [ ] Changing kV on SmartDashboard affects TalonFX slot 0 immediately
- [ ] Shooter RPM converges to target within 3–5 seconds of spin-up
- [ ] kV value recorded for update to Constants: **kV = ______**

---

## SECTION 9 — Photo Sensor Test

**Photo sensor is currently DISABLED in code (`PHOTO_SENSOR_ENABLED = false`). This section applies when the sensor is physically installed.**

> When installed, the sensor is on **DIO Port 1** (roboRIO Digital I/O pin 1).

1. With `PHOTO_SENSOR_ENABLED = true` in `Constants.SensorConstants`, enable the robot.
2. Check `Shooter/Ball Detected` on SmartDashboard — should read **false** with no ball.
3. Place a game piece at the sensor location — `Shooter/Ball Detected` should turn **true**.
4. Remove the ball — should return to **false**.
5. If polarity is inverted, set `PHOTO_SENSOR_INVERTED = true` in Constants.

- [ ] Sensor enabled in Constants (when hardware installed)
- [ ] `Shooter/Ball Detected` reads false with no ball
- [ ] `Shooter/Ball Detected` reads true when ball is detected
- [ ] Polarity verified (set INVERTED if needed)

> **Current status:** `PHOTO_SENSOR_ENABLED = false` — sensor not yet installed. `canShoot()` currently returns true when shooter is at speed (sensor bypassed).

---

## SECTION 10 — Vision System Tests

**Verify PhotonVision cameras initialize, connect, and produce valid pose estimates.**

Camera configuration:
| Camera          | Name (in code)   | Hardware            | Mount Position              |
|-----------------|------------------|---------------------|-----------------------------|
| Front Camera    | `"Front_Camera"` | Pi4 + PiCam v2      | ~1" from front edge, 20" up |
| Rear Camera     | `"Rear_Camera"`  | Pi5 + OV9281        | ~1" from rear edge, 20" up  |
| Driver Camera   | `"Driver_Camera"`| USB webcam          | USB port 0 on RIO           |

### 10A — PhotonVision Coprocessor Startup

1. Power on the robot fully.
2. On a laptop connected to the robot network, navigate to `http://photonvision.local:5800` (or `http://10.TE.AM.11:5800`).
3. Verify PhotonVision dashboard loads and shows both cameras: **Front_Camera** and **Rear_Camera**.
4. Confirm camera names exactly match `VisionConstants.FRONT_CAMERA_NAME` and `REAR_CAMERA_NAME` in code.
5. Verify camera feeds are live (not frozen).
6. Confirm the **2026 AprilTag field layout** (`k2026RebuiltWelded`) is selected in PhotonVision.
7. Set pipeline mode to **AprilTag** for each camera.

- [ ] PhotonVision dashboard accessible
- [ ] Front_Camera feed is live
- [ ] Rear_Camera feed is live
- [ ] Camera names match exactly what's in Constants.java
- [ ] 2026 AprilTag field layout loaded in PhotonVision
- [ ] AprilTag pipeline active on both cameras

### 10B — Camera Connection Status (SmartDashboard)

1. Deploy code and enable in any mode.
2. On SmartDashboard, verify:
   - `Vision/FrontCamConnected` = **true**
   - `Vision/RearCamConnected` = **true**
3. If either shows false, check camera USB/CSI connections and PhotonVision configuration.

- [ ] `Vision/FrontCamConnected` = true
- [ ] `Vision/RearCamConnected` = true

### 10C — AprilTag Detection

1. Place the robot where it can see at least one 2026 field AprilTag.
2. Verify on SmartDashboard:
   - `Vision/NumTagsVisible` ≥ 1
   - `Vision/VisibleTags` lists tag IDs matching the tags in view
3. Move closer to a tag (within 4 meters — `MAX_TAG_DISTANCE_METERS`).
4. Verify `Vision/BestPoseX`, `Vision/BestPoseY`, `Vision/BestPoseTheta` update to plausible field coordinates.
5. Verify `Vision/NumTagsUsed` ≥ 1.

- [ ] At least 1 AprilTag detected when in view
- [ ] Pose estimate appears on SmartDashboard
- [ ] Pose coordinates are plausible for the robot's physical location
- [ ] Tags beyond 4 meters are rejected (no pose update)

### 10D — Driver Camera (USB)

1. Open SmartDashboard or Shuffleboard.
2. Add the `Driver_Camera` stream (check `CameraServer` streams).
3. Verify the driver camera video feed is visible and live.
4. If camera is not found, the DS console will show a warning — this is non-fatal but must be resolved before competition.

- [ ] Driver camera feed visible in Shuffleboard/SmartDashboard
- [ ] No camera-related exception in DS console

---

## SECTION 11 — Odometry and Pose Estimation Tests

**Verify the DifferentialDrivePoseEstimator fuses encoder and gyro data correctly.**

1. Deploy code. Note initial `Field2d` robot position on SmartDashboard (`DriveTrain/Field`).
2. Drive the robot forward exactly **1 meter** (measure with tape).
3. Verify `DriveTrain/LeftDistMeters` and `DriveTrain/RightDistMeters` both read approximately **1.0 m**.
4. Verify the `Field2d` widget shows the robot moved ~1 m from starting position.
5. If vision cameras can see AprilTags, verify vision measurements are fused (pose stays stable rather than drifting).
6. Drive a 1-meter square and return to start — verify pose returns close to (0,0).

- [ ] Encoder distance readings are accurate to within ±5%
- [ ] Field2d pose updates as robot moves
- [ ] Vision fuses with odometry (pose corrects when tags visible)
- [ ] Pose reasonably accurate after simple driving path

---

## SECTION 12 — Zone Lock Verification

**Verify shooter zone lock is enforced in Teleop but bypassed in Test mode.**

The zone lock restricts shooter spin-up to the offensive zone:
- **Blue** offensive zone: X ≤ 5.2 m (blue side, between DS and hub)
- **Red** offensive zone: X ≥ 11.3 m

### 12A — Zone Lock Blocks Shooter Outside Zone (Teleop)

1. Switch to **Teleop** mode.
2. With robot outside the offensive zone (e.g., on a bench with no AprilTags visible — pose defaults to origin which may be outside zone), press **Y** on Operator controller.
3. Verify `Shooter/In Offensive Zone` = **false** on SmartDashboard.
4. Verify shooter does NOT spin up (`Shooter/Current RPM` remains 0).
5. Verify `Shooter/State` remains **"IDLE"**.

- [ ] Shooter does not spin in Teleop when outside offensive zone
- [ ] `Shooter/In Offensive Zone` correctly reflects zone status

### 12B — Test Mode Bypasses Zone Lock

1. Switch to **Test** mode.
2. Press **Y** on Operator controller.
3. Verify shooter spins up regardless of zone status.
4. Verify `Shooter/State` = **"SPIN_UP"** even if `Shooter/In Offensive Zone` = **false**.

- [ ] Shooter spins freely in Test mode regardless of zone

---

## SECTION 13 — Distance Resolution Priority Test

**Verify the 4-tier distance resolution priority (Vision > Odometry > POV > Default).**

1. Enable in **Test** mode.
2. With **no** AprilTags visible and **no** POV preset:
   - Spin up (Y button) — verify `Shooter/Distance Source` = **"Odometry"** (or **"Default"** if odometry is at origin)
3. Set a POV preset (e.g., POV Left = 7.5 ft):
   - Verify `Shooter/Distance Source` = **"POV Preset"** when no vision/odometry in zone
4. Move robot to where it can see AprilTags:
   - Verify `Shooter/Distance Source` = **"Vision"** and `Shooter/Vision Distance (ft)` is populated
5. Press **A** to clear preset — verify automatic distance resolution resumes.

- [ ] Distance source priority: Vision > Odometry > POV Preset > Default
- [ ] `Shooter/Distance Source` correctly shows active tier
- [ ] A button clears POV preset and returns to automatic resolution
- [ ] Vision distance and odometry distance both reported on SmartDashboard

---

## SECTION 14 — SmartDashboard / Telemetry Verification

**Verify all critical telemetry values are present and updating.**

Open SmartDashboard and confirm all keys below are visible and updating during operation:

**DriveTrain:**
- [ ] `DriveTrain/LeftDistMeters`
- [ ] `DriveTrain/RightDistMeters`
- [ ] `DriveTrain/HeadingDeg`
- [ ] `DriveTrain/DriveMode`
- [ ] `DriveTrain/OrientationMode`
- [ ] `DriveTrain/DriveState`
- [ ] `DriveTrain/IsFieldOriented`
- [ ] `DriveTrain/IsTankDrive`
- [ ] `DriveTrain/Field` (Field2d widget)
- [ ] `DriveTrain/PPLTV/Qx` and related LTV tuning keys

**Shooter:**
- [ ] `Shooter/Current RPM`
- [ ] `Shooter/Target RPM`
- [ ] `Shooter/RPM At Speed`
- [ ] `Shooter/Can Shoot`
- [ ] `Shooter/Motor Current (A)`
- [ ] `Shooter/Active Distance (ft)`
- [ ] `Shooter/POV Preset Distance (ft)`
- [ ] `Shooter/Vision Distance (ft)`
- [ ] `Shooter/Odometry Distance (ft)`
- [ ] `Shooter/Distance Source`
- [ ] `Shooter/In Offensive Zone`
- [ ] `Shooter/Intake Active`
- [ ] `Shooter/Eject Active`
- [ ] `Shooter/Ball Detected`
- [ ] `Shooter/State`
- [ ] `Feeder/Motor Current (A)`
- [ ] `Feeder/Jam Clearing`
- [ ] `Shooter/Tuning/kP`, `kI`, `kD`, `kV`

**Vision:**
- [ ] `Vision/FrontCamConnected`
- [ ] `Vision/RearCamConnected`
- [ ] `Vision/NumTagsVisible`
- [ ] `Vision/VisibleTags`
- [ ] `Vision/BestPoseX`, `Y`, `Theta`
- [ ] `Vision/NumTagsUsed`
- [ ] `Vision/AvgTagDistance`
- [ ] `Vision/HubDistance`
- [ ] `Vision/HubAligned`

---

## SECTION 15 — Autonomous Test

**Verify autonomous mode initializes correctly and the auto chooser works.**

1. Open SmartDashboard and locate the **Auto Chooser** widget.
2. Verify "Do Nothing" is listed as an option.
3. Verify any PathPlanner autos appear in the chooser (if paths are configured).
4. Select "Do Nothing" auto.
5. Enable in **Autonomous** mode — robot should remain stationary.
6. Verify no auto errors in DS console.
7. If a PathPlanner auto path is available:
   - Place robot at the path's starting pose.
   - Enable auto and verify robot follows the path.
   - Check `DriveTrain/Field` widget for path visualization.

- [ ] Auto Chooser visible on SmartDashboard
- [ ] "Do Nothing" auto works (robot stationary in auto)
- [ ] PathPlanner path visualization appears in Field2d widget when auto runs
- [ ] No PathPlanner errors in DS console during auto

---

## SECTION 16 — Full Integration Test (Game Piece)

**Perform a complete ball-intake-shoot cycle with a game piece.**

> Only perform this test after all individual system tests above have passed.

1. Enable in **Test** mode (zone lock bypassed).
2. Manually place a game piece at the intake.
3. Press **LB** (Operator) to begin intake — verify ball is drawn in.
4. Release **LB** when ball is seated (or use photo sensor if installed).
5. Set a distance preset with **POV Down** (10 ft = ~2825 RPM target).
6. Hold **X** (Operator) to begin shoot sequence:
   - Shooter spins up to ~2825 RPM
   - When at speed, feeder auto-activates and launches ball
7. Verify ball exits the shooter at a reasonable velocity.
8. Press **B** to reset to IDLE.
9. Repeat with different distance presets (POV Left = 7.5 ft, POV Up = 15 ft).

- [ ] Ball intakes successfully with LB
- [ ] Shooter spins to correct RPM for preset distance
- [ ] Feeder activates automatically when RPM is at target
- [ ] Ball is launched successfully
- [ ] Integration sequence works for 3+ consecutive shots
- [ ] No jams during integration test (or jam cleared automatically if detected)

---

## SECTION 17 — Pre-Competition Final Check

**Run this section immediately before leaving for competition.**

- [ ] All firmware versions confirmed (RIO, Radio, SparkMAX, PDH, TalonFX)
- [ ] Battery voltage ≥ 12.5 V, battery connector secure
- [ ] All CAN IDs verified on CAN bus (no missing devices)
- [ ] Robot code deployed from latest committed branch
- [ ] No Java exceptions or CAN errors in DS console on startup
- [ ] SmartDashboard / Shuffleboard layout saved and loads correctly
- [ ] PhotonVision camera names and pipelines confirmed
- [ ] Auto chooser populated with competition autos
- [ ] Driver and Operator controllers connected on ports 0 and 1
- [ ] All button bindings tested and confirmed functional
- [ ] SparkMAX kV value updated in Constants after tuning: **kV = ______**
- [ ] `PHOTO_SENSOR_ENABLED` set correctly for installed hardware
- [ ] `SHOOTER_INVERTED` and `FEEDER_INVERTED` verified on bench

---

## QUICK REFERENCE — Button Bindings

### Driver Controller (Port 0)
| Input             | Action                                       |
|-------------------|----------------------------------------------|
| Left Y Axis       | Left wheel speed (Tank) / Forward (Arcade)   |
| Right Y Axis      | Right wheel speed (Tank) / Rotation (Arcade) |
| **Back** button   | Toggle Tank ↔ Arcade drive mode              |
| **Start** button  | Toggle Field-Oriented ↔ Robot-Relative       |

### Operator Controller (Port 1)
| Input           | Action                                                |
|-----------------|-------------------------------------------------------|
| **Y** button    | Toggle shooter spin-up to distance RPM (zone-locked)  |
| **X** button    | Full shoot: spin-up + auto-feed when at speed (hold)  |
| **RB**          | Feed only if shooter is at target speed (hold)        |
| **B** button    | Emergency stop — immediately stop shooter and feeder  |
| **A** button    | Clear POV distance preset → automatic distance        |
| **LB**          | Toggle intake — draw ball in                          |
| **LT**          | Toggle eject — reverse feeder to expel ball           |
| **POV Up**      | Stage 15.0 ft distance preset (3100 RPM target)       |
| **POV Down**    | Stage 10.0 ft distance preset (~2825 RPM target)      |
| **POV Left**    | Stage 7.5 ft distance preset (2700 RPM target)        |
| **POV Right**   | Stage 12.5 ft distance preset (2950 RPM target)       |

---

## QUICK REFERENCE — CAN IDs

| Device                  | CAN ID | Type     | Tool          |
|-------------------------|--------|----------|---------------|
| Power Distribution Hub  | 1      | REV PDH  | REV Hardware Client |
| Left Drive Lead         | 20     | SparkMAX | REV Hardware Client |
| Left Drive Follow       | 21     | SparkMAX | REV Hardware Client |
| Right Drive Lead        | 22     | SparkMAX | REV Hardware Client |
| Right Drive Follow      | 23     | SparkMAX | REV Hardware Client |
| Shooter Wheel           | 30     | TalonFX  | Phoenix Tuner X |
| Feeder / Intake         | 40     | SparkMAX | REV Hardware Client |

---

## QUICK REFERENCE — Key Constants

| Constant                  | Value         | Notes                                |
|---------------------------|---------------|--------------------------------------|
| Shooter current limit     | 40 A          | TalonFX stator limit                 |
| Feeder current limit      | 30 A          | SparkMAX smart current limit         |
| Drive current limit       | 40 A          | SparkMAX smart current limit each    |
| Joystick deadband         | 0.05          |                                      |
| Intake speed              | 0.50 (50%)    | Feeder motor forward duty cycle      |
| Feeder speed              | 0.60 (60%)    | Feeder motor pushing ball to shooter |
| Eject speed               | -0.50 (-50%)  | Feeder motor reverse                 |
| Jam threshold             | 30 A          | Feeder current spike triggers clear  |
| Jam reverse duration      | 0.25 sec      |                                      |
| RPM tolerance             | ±50 RPM       | "At speed" window                    |
| Photo sensor DIO port     | 1             | Currently disabled                   |
| Blue offensive max X      | 5.2 m         | Zone lock boundary                   |
| Red offensive min X       | 11.3 m        | Zone lock boundary                   |
| Max vision tag distance   | 4.0 m         | Tags beyond this are rejected        |
| Max tag ambiguity         | 0.30          | Single-tag quality gate              |

---

*Document generated 2026-02-24 | Update after each test session with findings and tuning values.*
