# Intake Implementation Plan
## Kitbot Intake with Neo Motor and SparkMax Controller

---

## Executive Summary

This document outlines the complete implementation plan for the intake subsystem using the kitbot intake mechanism with a single Neo motor controlled via SparkMax (CAN). The implementation includes motor control, optional ball detection sensing, telemetry, and command integration with the shooter subsystem for coordinated operation.

---

## Current Code Analysis

### Strengths of Existing Implementation

1. **Clean Architecture**: The subsystem class exists and extends SubsystemBase
2. **Proper Structure**: Has periodic() and simulationPeriodic() methods
3. **Import Ready**: Package structure is correct

### Weaknesses and Issues

1. **Empty Implementation**: Constructor and methods are empty
2. **No Motor**: No motor controller instantiated
3. **No Constants**: IntakeConstants class is empty
4. **No Methods**: No intake control methods exist
5. **No Telemetry**: No SmartDashboard integration
6. **No Sensor**: No ball detection capability
7. **No Commands**: No command factories or bindings
8. **No Integration**: Not connected to RobotContainer

### What Can Stay

- Class structure (extends SubsystemBase)
- Package location
- periodic() and simulationPeriodic() methods

### What Must Change

- Add SparkMax motor controller
- Add motor configuration (current limits, idle mode)
- Implement control methods (start, stop, reverse)
- Add constants to IntakeConstants
- Add telemetry
- Consider ball detection sensor
- Create command factories
- Integrate with RobotContainer

---

## Requirements Summary

1. **Motor Control**: Single Neo motor via SparkMax (CAN)
   - Variable speed control
   - Forward (intake) and reverse (eject) operation
   - Configurable speeds via constants
2. **Current Limiting**: Protect motor, detect stalls
3. **Ball Detection**: Optional sensor for automatic operation
4. **Telemetry**: Display speed, current, ball status
5. **Commands**: Button-triggered intake/eject operations
6. **Shooter Coordination**: Coordinate with shooter for optimal game piece handling

---

## Hardware Specifications

### Neo Motor Specs
- **Free Speed**: 5676 RPM
- **Stall Torque**: 2.6 N⋅m
- **Stall Current**: 105 A
- **Free Current**: 1.8 A
- **Voltage**: 12V nominal

### Kitbot Intake Specs
- **Gearing**: Kitbot standard (varies by configuration)
- **Mechanism**: Roller-based intake
- **Game Piece**: 2026 game balls/cargo
- **Operation**: Pull game pieces into robot

### Current Limiting Strategy
- **Stall Limit**: 30A (detect game piece contact)
- **Free Limit**: 20A (normal operation)
- **Spike Detection**: Current spike indicates game piece captured

---

## Detailed Implementation Plan

---

## Phase 1: Constants and Configuration

### 1.1 Update IntakeConstants Class

**File**: `src/main/java/frc/robot/Constants.java`

**Add the following constants**:

```java
public static class IntakeConstants {
    // CAN ID for SparkMax controller
    public static final int INTAKE_MOTOR_ID = 5;  // Adjust to match robot wiring

    // Motor configuration
    public static final int STALL_LIMIT = 30;     // Amps - stall detection
    public static final int FREE_LIMIT = 20;      // Amps - normal running
    public static final double NOMINAL_VOLTAGE = 12.0;

    // Motor speeds
    public static final double INTAKE_SPEED = 0.7;    // Forward (intake game pieces)
    public static final double REVERSE_SPEED = -0.5;  // Reverse (eject game pieces)
    public static final double SLOW_INTAKE_SPEED = 0.4; // Gentle intake for positioning

    // Sensor configuration (if using ball detection)
    public static final int BALL_SENSOR_DIO_PORT = 1;  // Digital input port
    public static final boolean BALL_SENSOR_INVERTED = true; // Beam-break inverted

    // Current spike detection
    public static final double CURRENT_SPIKE_THRESHOLD = 25.0; // Amps
    public static final double CURRENT_SPIKE_TIME = 0.1;       // Seconds
}
```

---

## Phase 2: Motor Setup

### 2.1 Implement Intake Subsystem

**File**: `src/main/java/frc/robot/subsystems/Intake.java`

**Components to Instantiate**:
- `SparkMax intakeMotor` - Intake motor controller
- `SparkMaxConfig config` - Motor configuration
- `DigitalInput ballSensor` - Optional ball detection sensor

**Configuration Requirements**:
- Voltage compensation (12V)
- Smart current limiting (stall detection)
- Brake mode (hold game piece when stopped)
- Proper motor direction

**Methods Needed**:

| Method | Parameters | Return Type | Description |
|--------|------------|-------------|-------------|
| `Intake()` | constructor | | Initialize motor and sensor |
| `runIntake()` | `double speed` | `void` | Run intake at specified speed |
| `startIntake()` | none | `void` | Run at INTAKE_SPEED |
| `reverseIntake()` | none | `void` | Run at REVERSE_SPEED |
| `stopIntake()` | none | `void` | Stop motor |
| `slowIntake()` | none | `void` | Run at SLOW_INTAKE_SPEED |
| `hasBall()` | none | `boolean` | Check ball sensor (if installed) |
| `getCurrentDraw()` | none | `double` | Get motor current for spike detection |
| `isStalled()` | none | `boolean` | Detect if motor is stalled |
| `periodic()` | none | `void` | Update telemetry |

---

## Phase 3: Ball Detection (Optional)

### 3.1 Sensor Integration

**Options for Ball Detection**:
1. **Beam Break Sensor**: IR beam across intake path
2. **Limit Switch**: Physical contact detection
3. **Current Spike Detection**: Monitor motor current (no additional hardware)
4. **Color Sensor**: Detect ball color (advanced)

**Current Spike Method** (No Extra Hardware):
- Monitor motor output current
- Spike above threshold indicates game piece contact
- Debounce to avoid false positives

---

## Phase 4: Telemetry

### 4.1 SmartDashboard Integration

**Telemetry to Display**:
- Motor speed (percent output)
- Motor current (amps)
- Ball detected (boolean)
- Intake state (RUNNING/STOPPED/REVERSING)
- Motor temperature
- Is stalled (boolean)

---

## Phase 5: Commands and Integration

### 5.1 Command Factories

**Add to Intake.java**:

| Factory Method | Return Type | Description |
|----------------|-------------|-------------|
| `intakeCommand()` | `Command` | Runs intake while button held, stops when released |
| `ejectCommand()` | `Command` | Reverses intake while button held |
| `intakeUntilBallCommand()` | `Command` | Runs until ball detected, then stops |
| `feedToShooterCommand()` | `Command` | Coordinates with shooter for feeding |

### 5.2 RobotContainer Integration

**Button Bindings**:
- Left Trigger: Run intake while held
- Left Bumper: Reverse intake while held
- (Optional) Auto-stop when ball detected

---

## Incremental Implementation Plan (For Beginning Programmers)

**Philosophy**: Build one small, testable feature at a time. Each milestone adds ONE new concept and can be demonstrated independently. Don't move to the next milestone until the current one works!

---

### Prerequisites (Do Once at Start)
- [ ] Install REVLib via VS Code (Ctrl+Shift+P -> "WPILib: Manage Vendor Libraries")
- [ ] Verify intake motor CAN ID in constants matches actual wiring
- [ ] Identify correct motor direction (intake should pull game pieces IN)

---

### Milestone 1: Basic Motor Control
**Goal**: Get the intake motor to spin at a fixed speed when a button is pressed.

**What You'll Learn**: SparkMax basics, motor configuration, button bindings

**Tasks**:
- [ ] Add IntakeConstants with INTAKE_MOTOR_ID = 5 (or your actual ID)
- [ ] Add STALL_LIMIT = 30 and NOMINAL_VOLTAGE = 12.0
- [ ] In Intake.java:
  - Import SparkMax and related classes
  - Create `SparkMax intakeMotor` field
  - In constructor, instantiate motor with CAN ID
  - Create SparkMaxConfig with voltage compensation and current limit
  - Set idle mode to brake
  - Apply config to motor
- [ ] Add method `runIntake(double speed)`:
  - Sets motor output to given speed
- [ ] Add method `stopIntake()`:
  - Calls `intakeMotor.stopMotor()`
- [ ] In RobotContainer:
  - Create `Intake m_intake = new Intake()`
  - Bind left trigger to run intake at 0.7 while held, stop when released

**Test**: Press left trigger, intake motor spins. Release, it stops.

**Expected Time**: 1 hour

---

### Milestone 2: Add Preset Speeds
**Goal**: Add dedicated methods for common intake operations.

**What You'll Learn**: Method organization, constants usage

**Tasks**:
- [ ] Add constants to IntakeConstants:
  - INTAKE_SPEED = 0.7
  - REVERSE_SPEED = -0.5
  - SLOW_INTAKE_SPEED = 0.4
- [ ] Add methods to Intake.java:
  - `startIntake()` - runs at INTAKE_SPEED
  - `reverseIntake()` - runs at REVERSE_SPEED
  - `slowIntake()` - runs at SLOW_INTAKE_SPEED
- [ ] Update RobotContainer bindings:
  - Left trigger: `m_intake.startIntake()` while held
  - Left bumper: `m_intake.reverseIntake()` while held

**Test**: Left trigger intakes, left bumper reverses, both stop when released.

**Expected Time**: 30 minutes

---

### Milestone 3: Add Basic Telemetry
**Goal**: Display intake status on SmartDashboard.

**What You'll Learn**: SmartDashboard, debugging with telemetry

**Tasks**:
- [ ] Add method `logTelemetry()` to Intake
- [ ] Call `logTelemetry()` in `periodic()`
- [ ] Display to SmartDashboard:
  - Intake motor speed (applied output)
  - Intake motor current (amps)
- [ ] Open SmartDashboard/Shuffleboard and verify data

**Test**: Run intake, watch speed and current on dashboard.

**Expected Time**: 20 minutes

---

### Milestone 4: Current Spike Detection (Ball Detection Without Sensor)
**Goal**: Detect when a ball is captured using motor current spike.

**What You'll Learn**: Sensor-less detection, current monitoring

**Tasks**:
- [ ] Add constants:
  - CURRENT_SPIKE_THRESHOLD = 25.0
- [ ] Add method `getCurrentDraw()`:
  - Returns `intakeMotor.getOutputCurrent()`
- [ ] Add method `isCurrentSpiking()`:
  - Returns true if current > CURRENT_SPIKE_THRESHOLD
- [ ] Add current spike status to telemetry
- [ ] Add state tracking for debouncing (optional)

**Test**: Run intake, push game piece in, watch current spike on dashboard.

**Expected Time**: 30 minutes

---

### Milestone 5: Ball Detection Sensor (Optional)
**Goal**: Add beam break or limit switch for reliable ball detection.

**What You'll Learn**: Digital input sensors, boolean logic

**Tasks**:
- [ ] Add constant BALL_SENSOR_DIO_PORT = 1 (your port)
- [ ] Add constant BALL_SENSOR_INVERTED = true (for beam-break)
- [ ] In Intake constructor:
  - Import `DigitalInput`
  - Create `DigitalInput ballSensor`
- [ ] Add method `hasBall()`:
  - Returns sensor state (inverted if beam-break)
- [ ] Add "Ball Detected" to telemetry
- [ ] Handle case where sensor not installed (optional)

**Test**: Block sensor, dashboard shows ball detected.

**Expected Time**: 30 minutes

---

### Milestone 6: Command Factories
**Goal**: Create command methods for cleaner RobotContainer code.

**What You'll Learn**: Command factories, inline commands

**Tasks**:
- [ ] Add method `intakeCommand()`:
```java
public Command intakeCommand() {
    return runEnd(
        () -> startIntake(),   // While running
        () -> stopIntake()     // When ended
    );
}
```
- [ ] Add method `ejectCommand()`:
```java
public Command ejectCommand() {
    return runEnd(
        () -> reverseIntake(),
        () -> stopIntake()
    );
}
```
- [ ] Update RobotContainer to use command factories:
  - `m_driverController.leftTrigger().whileTrue(m_intake.intakeCommand())`
  - `m_driverController.leftBumper().whileTrue(m_intake.ejectCommand())`

**Test**: Same behavior as before, but cleaner code.

**Expected Time**: 30 minutes

---

### Milestone 7: Smart Intake (Auto-Stop)
**Goal**: Automatically stop intake when ball is detected.

**What You'll Learn**: Conditional commands, state-based control

**Tasks**:
- [ ] Add method `intakeUntilBallCommand()`:
```java
public Command intakeUntilBallCommand() {
    return run(() -> startIntake())
        .until(() -> hasBall() || isCurrentSpiking())
        .andThen(runOnce(() -> stopIntake()));
}
```
- [ ] Add alternate button binding for auto-stop mode
- [ ] Add option to continue in slow mode after detection

**Test**: Press button, intake runs until ball detected, then stops automatically.

**Expected Time**: 45 minutes

---

### Milestone 8: Shooter Coordination
**Goal**: Coordinate intake with shooter for smooth game piece transfer.

**What You'll Learn**: Subsystem coordination, multi-subsystem commands

**Tasks**:
- [ ] Decide: Should intake stop before shooting? (usually yes)
- [ ] Create method `feedToShooterCommand(Shooter shooter)`:
  - Only runs if shooter is ready (at target RPM)
  - Runs intake to feed game piece to shooter
  - Stops after configurable time
- [ ] In RobotContainer, bind feed button:
  - Same button as shooter, or dedicated feed button
- [ ] Add safety: don't feed if shooter not running

**Test**: Shooter spins up, press feed, ball transfers smoothly.

**Expected Time**: 1 hour

---

### Milestone 9: State Machine (Optional)
**Goal**: Add proper state tracking for complex operations.

**What You'll Learn**: State machines, enums

**Tasks**:
- [ ] Create enum `IntakeState`:
```java
public enum IntakeState {
    STOPPED,
    INTAKING,
    REVERSING,
    HOLDING  // Game piece detected, motor stopped
}
```
- [ ] Add `currentState` field
- [ ] Update methods to set state
- [ ] Add state to telemetry
- [ ] Use state for smarter control logic

**Test**: Watch state change on dashboard as you operate intake.

**Expected Time**: 30 minutes

---

### Milestone 10: Safety and Polish
**Goal**: Add safety features and finalize for competition.

**What You'll Learn**: Error handling, safety systems

**Tasks**:
- [ ] Add motor temperature monitoring
- [ ] Add stall detection with auto-reverse:
  - If current high but motor not spinning, reverse briefly
- [ ] Add motor fault detection
- [ ] Add emergency stop handling
- [ ] Document final speed values
- [ ] Test under competition-like conditions

**Test**: Verify all safety features work, intake handles jams gracefully.

**Expected Time**: 1 hour

---

## Total Incremental Timeline

| Milestone | Time Estimate | Cumulative | What Works After This Milestone |
|-----------|---------------|------------|----------------------------------|
| 1. Basic Motor | 1 hour | 1 hour | Motor spins with button |
| 2. Preset Speeds | 0.5 hours | 1.5 hours | Intake and eject work |
| 3. Telemetry | 0.3 hours | 1.8 hours | Can see motor data |
| 4. Current Spike | 0.5 hours | 2.3 hours | Detect ball with no sensor |
| 5. Ball Sensor | 0.5 hours | 2.8 hours | Reliable ball detection |
| 6. Commands | 0.5 hours | 3.3 hours | Clean command structure |
| 7. Smart Intake | 0.75 hours | 4 hours | Auto-stop on capture |
| 8. Shooter Coord | 1 hour | 5 hours | Smooth shooting cycle |
| 9. State Machine | 0.5 hours | 5.5 hours | Clear state tracking |
| 10. Safety | 1 hour | 6.5 hours | Competition ready |

**Total**: ~6.5 hours of focused work

---

## Key Principles for Success

1. **One Thing at a Time**: Complete each milestone fully before moving on
2. **Test Immediately**: After every change, test it
3. **Check Motor Direction First**: Intake should pull game pieces IN
4. **Current Limits Matter**: Prevent brownouts and motor damage
5. **Commit Often**: Git commit after each working milestone
6. **Keep It Simple**: Basic intake/eject is usually all you need

---

## Common Beginner Mistakes to Avoid

- Wrong motor direction -> Intake ejects instead of intakes
- No current limit -> Motor overheats or causes brownout
- Wrong CAN ID -> Motor doesn't respond
- Ball sensor wired wrong -> Always shows ball or never shows ball
- Not testing with actual game pieces -> Speeds might be wrong
- Over-engineering -> Too many features, not enough testing

---

## Risk Mitigation

### Technical Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Motor jams | Game piece stuck | Add stall detection with auto-reverse |
| Ball sensor fails | Can't detect game piece | Use current spike as backup |
| Wrong speed | Drops or damages game piece | Test with actual game pieces |
| CAN issues | Motor unresponsive | Use REV Hardware Client to diagnose |

### Safety Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Finger entrapment | Injury | Guards on mechanism, careful testing |
| Game piece projectile | Hit someone | Test in safe area |
| Motor overheating | Fire risk | Current limits, temperature monitoring |

---

## Performance Targets

### Acceptance Criteria

- [ ] Intake captures game piece in < 0.5 seconds
- [ ] Game piece held securely when motor stopped
- [ ] Eject clears game piece completely
- [ ] Ball detection accurate > 95% of time
- [ ] No motor faults during normal operation
- [ ] No brownouts during rapid cycling
- [ ] Shooter coordination seamless

---

## Future Enhancements

### Post-Initial Implementation

1. **Multi-Ball Handling**: Index multiple game pieces
2. **Smart Speed**: Adjust speed based on game piece position
3. **LED Indicators**: Show ball status with LEDs
4. **Auto-Collect**: Drive toward game piece and auto-intake
5. **Jam Recovery**: Automatic jam detection and clearing
6. **Game Piece Counting**: Track how many collected

---

## Summary

This implementation plan provides a straightforward roadmap for implementing the intake subsystem. The intake is one of the simpler subsystems, making it an excellent first project for new programmers. The modular approach allows for incremental implementation while maintaining working functionality at each stage.

**Estimated Total Implementation Time**: 6.5 hours

**Key Success Factors**:
1. Correct motor direction
2. Appropriate speed settings
3. Reliable ball detection (sensor or current)
4. Proper coordination with shooter
5. Safety features (current limits, stall detection)

**Critical Path**:
1. Basic motor control
2. Speed constants tuning
3. Ball detection
4. Shooter coordination
5. Testing with actual game pieces
