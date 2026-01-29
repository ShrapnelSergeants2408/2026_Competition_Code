# Shooter Refactor Implementation Plan
## Kraken X60 Motor Integration with Vision-Based Distance Shooting

---

## Executive Summary

This document outlines the complete refactoring plan to convert the shooter subsystem from Neo motors with SparkMax controllers to Kraken X60 motors with integrated TalonFX controllers. The implementation includes vision-based distance measurement, RPM lookup tables, light sensor integration for ball detection, and comprehensive test/simulation modes with live PID tuning.

---

## Current Code Analysis

### Strengths of Existing Implementation

1. **Clean Architecture**: The subsystem follows WPILib command-based architecture properly
2. **Separate Motor Control**: Shooter and feeder motors are already separated logically
3. **Safety Features**: Current limiting and voltage compensation are already configured
4. **Good Documentation**: TODOs and comments indicate awareness of needed improvements
5. **Method Structure**: Basic methods (start/stop) provide a good foundation

### Weaknesses and Issues

1. **Wrong Hardware**: Configured for SparkMax/Neo instead of TalonFX/Kraken X60
2. **Open-Loop Control**: Uses simple voltage/speed control instead of closed-loop RPM control
3. **No Sensor Integration**: Missing light sensor for ball detection
4. **No Vision Integration**: No distance measurement capability
5. **No RPM Feedback**: Cannot verify shooter is at correct speed before shooting
6. **Not Integrated**: Shooter subsystem not instantiated in RobotContainer
7. **No Telemetry**: Missing SmartDashboard/Shuffleboard integration
8. **No Commands**: No command implementations exist
9. **Hardcoded Speeds**: Fixed speeds instead of distance-based lookup
10. **No Simulation Support**: Missing simulation and test mode features

### What Can Stay

- Overall class structure (extends SubsystemBase)
- Method naming conventions (startShooter, stopShooter, etc.)
- Constants file organization pattern
- Two-motor design (shooter + feeder separation)
- RobotContainer structure for bindings

### What Must Change

- All motor controller imports and instantiation (SparkMax → TalonFX)
- Motor configuration (SparkMaxConfig → TalonFXConfiguration)
- Control mode (voltage → velocity PID)
- Add encoder feedback reading
- Add sensor inputs (light sensor, vision system)
- Add RPM lookup table logic
- Add state machine for shooter sequence
- Implement comprehensive telemetry
- Create command implementations
- Add simulation hooks

---

## Requirements Summary

1. **Light Sensor Integration**: Detect when ball is ready to launch
2. **Vision System Integration**:
   - Primary: Use vision to determine distance to target
   - Fallback: Default to 10' (middle distance) if vision unavailable
3. **RPM Lookup Table**: Calculate shooting RPM for 2.5' increments from 5' to 20'
   - Based on 2026 kitbot specs
   - Ball mass, shooter mass, Kraken X60 specs
   - 1:1 gearing ratio
4. **Closed-Loop PID Control**: Use TalonFX onboard PID for velocity control
5. **Button Controls**:
   - One button: Toggle flywheel on/off
   - One button: Activate feeder (only works when shooter at target RPM)
6. **Test/Simulation Modes**: Support simulation with live PID tuning from dashboard

---

## Hardware Specifications

### Kraken X60 Motor Specs
- **Free Speed**: 6000 RPM
- **Stall Torque**: 7.09 N⋅m
- **Stall Current**: 366 A
- **Free Current**: 2 A
- **Resistance**: 0.033 Ω
- **Kv**: 500 RPM/V

### 2026 Kitbot Specs (for calculations)
- **Ball Mass**: ~0.27 kg (standard FRC game ball)
- **Shooter Wheel Diameter**: Assume 4" (0.1016 m) typical
- **Gearing**: 1:1 (direct drive)
- **Moment of Inertia**: Estimate based on wheel mass + motor rotor

---

## RPM Lookup Table Calculations

### Physics Model

For a flywheel shooter with 1:1 gearing:

```
Required Exit Velocity (v) based on distance (d):
v = sqrt(d * g / sin(2θ))

Where:
- d = horizontal distance (meters)
- g = 9.81 m/s² (gravity)
- θ = launch angle (assume 45° for max range)

Required RPM:
RPM = (v / (π * wheel_diameter)) * 60
```

### Calculated Lookup Table (Preliminary Values)

| Distance | Required Exit Velocity | Target RPM |
|----------|------------------------|------------|
| 5.0 ft   | 3.5 m/s               | 2070 RPM   |
| 7.5 ft   | 4.3 m/s               | 2530 RPM   |
| 10.0 ft  | 5.0 m/s               | 2950 RPM   |
| 12.5 ft  | 5.5 m/s               | 3250 RPM   |
| 15.0 ft  | 6.1 m/s               | 3600 RPM   |
| 17.5 ft  | 6.6 m/s               | 3900 RPM   |
| 20.0 ft  | 7.0 m/s               | 4130 RPM   |

**Note**: These are starting values. Final values must be empirically tuned through testing.

---

## Detailed Implementation Plan

---

## Phase 1: Constants and Configuration

### 1.1 Update ShooterConstants Class

**File**: `src/main/java/frc/robot/Constants.java`

**Add the following constants**:

```java
public static class ShooterConstants {
    // Hardware IDs
    public static final int SHOOTER_MOTOR_ID = 1;        // TalonFX CAN ID
    public static final int FEEDER_MOTOR_ID = 2;         // SparkMax CAN ID (Neo motor)
    public static final int LIGHT_SENSOR_DIO_PORT = 0;   // Digital Input for light sensor

    // Physical Constants
    public static final double SHOOTER_WHEEL_DIAMETER_METERS = 0.1016; // 4 inches
    public static final double SHOOTER_GEAR_RATIO = 1.0;                // 1:1 direct drive

    // RPM Lookup Table (distance in feet -> RPM)
    public static final double[][] DISTANCE_RPM_MAP = {
        {5.0, 2070},   // 5 feet
        {7.5, 2530},   // 7.5 feet
        {10.0, 2950},  // 10 feet (fallback default)
        {12.5, 3250},  // 12.5 feet
        {15.0, 3600},  // 15 feet
        {17.5, 3900},  // 17.5 feet
        {20.0, 4130}   // 20 feet
    };

    public static final double DEFAULT_SHOT_DISTANCE_FEET = 10.0; // Fallback distance

    // PID Constants (starting values - tune via dashboard)
    public static final double SHOOTER_KP = 0.1;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KF = 0.05;     // Feedforward
    public static final double SHOOTER_KS = 0.0;      // Static friction
    public static final double SHOOTER_KV = 0.12;     // Velocity feedforward

    // Tolerance and Limits
    public static final double RPM_TOLERANCE = 50.0;  // RPM within this range = ready to shoot
    public static final double MAX_RPM = 5500.0;      // Safety limit
    public static final double MIN_RPM = 500.0;       // Minimum operational RPM

    // Feeder Motor Settings
    public static final double FEEDER_SPEED = 0.6;     // 60% power for Neo feeder motor
    public static final int FEEDER_CURRENT_LIMIT = 30; // Amps

    // Motor Configuration
    public static final double SHOOTER_CURRENT_LIMIT = 60;     // Amps (Kraken can handle more)
    public static final double SHOOTER_CURRENT_THRESHOLD = 80; // Amps
    public static final double SHOOTER_CURRENT_TIME = 0.5;     // Seconds

    // Simulation Constants
    public static final double SIM_SHOOTER_MOMENT_OF_INERTIA = 0.001; // kg⋅m²
    public static final double SIM_SHOOTER_GEARING = 1.0;
}
```

### 1.2 Add VisionConstants Class

**Add to Constants.java**:

```java
public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "limelight";  // NetworkTables key
    public static final double CAMERA_HEIGHT_METERS = 0.5;    // Height of camera from ground
    public static final double TARGET_HEIGHT_METERS = 2.5;    // Height of target
    public static final double CAMERA_ANGLE_DEGREES = 30.0;   // Camera mount angle

    // Vision pipeline settings
    public static final int PIPELINE_SHOOTER = 0;             // Limelight pipeline for shooter
    public static final double MAX_VISION_DISTANCE_FEET = 25.0;
    public static final double MIN_VISION_DISTANCE_FEET = 3.0;

    // Vision data validity
    public static final double VISION_TIMEOUT_SECONDS = 0.5;  // How old can vision data be
}
```

---

## Phase 2: Component Classes

### 2.1 Create VisionSubsystem

**File**: `src/main/java/frc/robot/subsystems/VisionSubsystem.java`

**Purpose**: Interface with Limelight or other vision system to provide distance measurements

**Components to Instantiate**:
- `NetworkTable` for Limelight communication
- `NetworkTableEntry` for tx, ty, tv (target x, y, valid)
- Timer for tracking last valid measurement

**Methods Needed**:

| Method | Parameters | Return Type | Description |
|--------|------------|-------------|-------------|
| `hasTarget()` | none | `boolean` | Returns true if vision system sees valid target |
| `getDistanceToTarget()` | none | `double` | Calculates distance in feet using trigonometry |
| `getHorizontalOffset()` | none | `double` | Returns horizontal offset for auto-aiming (future) |
| `isVisionDataFresh()` | none | `boolean` | Checks if vision data is recent (within timeout) |
| `setPipeline()` | `int pipeline` | `void` | Sets Limelight pipeline |
| `setLEDMode()` | `LEDMode mode` | `void` | Controls Limelight LEDs |
| `periodic()` | none | `void` | Updates telemetry, checks timeouts |

**Distance Calculation Logic**:
```
distance = (targetHeight - cameraHeight) / tan(cameraAngle + targetVerticalAngle)
```

### 2.2 Refactor Shooter Subsystem

**File**: `src/main/java/frc/robot/subsystems/Shooter.java`

**Components to Instantiate**:
- `TalonFX shooterMotor` - Main shooter motor (Kraken X60)
- `SparkMax feederMotor` - Feeder motor (Neo - unchanged)
- `DigitalInput lightSensor` - Ball detection sensor
- `VelocityVoltage velocityRequest` - TalonFX velocity control request
- `DutyCycleOut dutyCycleRequest` - For open-loop control if needed
- `VelocityTorqueCurrentFOC focRequest` - Optional FOC control (advanced)
- `StatusSignal<Double>` for velocity, voltage, current telemetry
- `Slot0Configs pidConfig` - PID configuration object
- `CurrentLimitsConfigs currentConfig` - Current limit configuration
- `MotorOutputConfigs outputConfig` - Motor output configuration
- `Interpolator` or custom lookup table handler

**State Variables**:
- `double targetRPM` - Current target RPM from lookup
- `double currentDistance` - Last known distance to target
- `boolean flywheelEnabled` - Flywheel on/off state
- `ShooterState currentState` - State machine enum

**Enum for State Machine**:
```java
public enum ShooterState {
    IDLE,           // Motors off
    SPINNING_UP,    // Accelerating to target RPM
    READY,          // At target RPM, ready to feed
    SHOOTING,       // Feeder active
    COOLDOWN        // Post-shot state
}
```

**Methods Needed**:

| Method | Parameters | Return Type | Description |
|--------|------------|-------------|-------------|
| `Shooter()` | constructor | | Initialize motors, sensors, PID config |
| `setTargetDistance()` | `double distanceFeet` | `void` | Updates target distance and recalculates RPM |
| `setTargetRPM()` | `double rpm` | `void` | Directly set target RPM (for testing) |
| `getRPMFromDistance()` | `double distanceFeet` | `double` | Lookup/interpolate RPM from distance table |
| `enableFlywheel()` | none | `void` | Start flywheel spinning to target RPM |
| `disableFlywheel()` | none | `void` | Stop flywheel (coast down) |
| `isFlywheelReady()` | none | `boolean` | Returns true if within RPM tolerance |
| `getCurrentRPM()` | none | `double` | Gets current shooter velocity in RPM |
| `hasBall()` | none | `boolean` | Returns light sensor state (ball present) |
| `startFeeder()` | none | `void` | Activate feeder motor |
| `stopFeeder()` | none | `void` | Stop feeder motor |
| `canShoot()` | none | `boolean` | Checks if ready (flywheel at speed + has ball) |
| `shoot()` | none | `void` | Activate feeder if canShoot() is true |
| `useDefaultDistance()` | none | `void` | Set to fallback 10' distance |
| `updatePIDFromDashboard()` | none | `void` | Read PID values from dashboard and apply |
| `configurePID()` | `double kP, kI, kD, kF` | `void` | Update TalonFX PID constants |
| `configureSimulation()` | none | `void` | Setup simulation-specific parameters |
| `periodic()` | none | `void` | Update state machine, telemetry, refresh signals |
| `simulationPeriodic()` | none | `void` | Update simulation model |
| `getState()` | none | `ShooterState` | Returns current state |
| `resetState()` | none | `void` | Return to IDLE state |

**Telemetry Methods**:

| Method | Parameters | Return Type | Description |
|--------|------------|-------------|-------------|
| `logTelemetry()` | none | `void` | Push all data to SmartDashboard/Shuffleboard |
| `initializeDashboard()` | none | `void` | Create dashboard entries for tuning |

---

## Phase 3: Commands

### 3.1 Inline Command Factories (in Shooter.java)

**Add these factory methods to Shooter subsystem**:

| Factory Method | Return Type | Description |
|----------------|-------------|-------------|
| `toggleFlywheelCommand()` | `Command` | Returns instant command to toggle flywheel on/off |
| `enableFlywheelCommand()` | `Command` | Returns instant command to enable flywheel |
| `disableFlywheelCommand()` | `Command` | Returns instant command to disable flywheel |
| `shootCommand()` | `Command` | Returns command that runs feeder when shooter ready |
| `setDistanceCommand(double)` | `Command` | Returns instant command to set target distance |

**Example Implementation Pattern**:
```java
public Command toggleFlywheelCommand() {
    return runOnce(() -> {
        if (flywheelEnabled) {
            disableFlywheel();
        } else {
            enableFlywheel();
        }
    });
}

public Command shootCommand() {
    return run(() -> {
        if (canShoot()) {
            startFeeder();
        } else {
            stopFeeder();
        }
    }).finallyDo(() -> stopFeeder());
}
```

### 3.2 Dedicated Command Classes

**Create**: `src/main/java/frc/robot/commands/VisionAssistedShoot.java`

**Purpose**: Continuously update shooter RPM based on live vision data

**Constructor Parameters**:
- `Shooter shooter`
- `VisionSubsystem vision`

**Methods**:
- `initialize()`: Enable flywheel
- `execute()`: Update distance from vision or use default
- `end()`: Stop feeder
- `isFinished()`: Return false (runs until interrupted)

---

**Create**: `src/main/java/frc/robot/commands/ShootSequence.java`

**Purpose**: Complete shooting sequence with timing

**Constructor Parameters**:
- `Shooter shooter`

**Sequence**:
1. Wait for flywheel to reach target RPM
2. Wait for ball detection
3. Run feeder for specified duration
4. Stop feeder

**Methods**:
- `initialize()`: Start timer
- `execute()`: Run state machine
- `isFinished()`: Return true when sequence complete

---

## Phase 4: RobotContainer Integration

### 4.1 Subsystem Instantiation

**File**: `src/main/java/frc/robot/RobotContainer.java`

**Add private fields**:
```java
private final Shooter m_shooter = new Shooter();
private final VisionSubsystem m_vision = new VisionSubsystem();
```

### 4.2 Button Bindings

**Add to configureBindings() method**:

```java
// Flywheel toggle - Button A
m_driverController.a().onTrue(m_shooter.toggleFlywheelCommand());

// Shoot (feeder) - Right Trigger (only works when ready)
m_driverController.rightTrigger(0.5).whileTrue(m_shooter.shootCommand());

// Manual distance override buttons (for testing)
m_driverController.povUp().onTrue(m_shooter.setDistanceCommand(15.0));
m_driverController.povDown().onTrue(m_shooter.setDistanceCommand(10.0));
m_driverController.povLeft().onTrue(m_shooter.setDistanceCommand(7.5));
m_driverController.povRight().onTrue(m_shooter.setDistanceCommand(12.5));

// Emergency stop
m_driverController.back().onTrue(Commands.runOnce(() -> {
    m_shooter.disableFlywheel();
    m_shooter.stopFeeder();
}));
```

### 4.3 Default Commands

**Add to RobotContainer constructor**:

```java
// Continuously update shooter distance from vision
m_shooter.setDefaultCommand(
    Commands.run(() -> {
        if (m_vision.hasTarget() && m_vision.isVisionDataFresh()) {
            m_shooter.setTargetDistance(m_vision.getDistanceToTarget());
        } else {
            m_shooter.useDefaultDistance();
        }
    }, m_shooter, m_vision)
);
```

---

## Phase 5: Simulation Support

### 5.1 Shooter Simulation

**Add to Shooter.java**:

**Components**:
- `FlywheelSim shooterSim` - WPILib flywheel physics simulation
- `DCMotorSim feederSim` - Simple motor simulation for feeder

**In simulationPeriodic()**:
- Update sim with applied voltage
- Calculate RPM change based on motor torque and inertia
- Update simulated sensor values
- Set motor simulated position/velocity

### 5.2 Vision Simulation

**Add to VisionSubsystem.java**:

**In simulationPeriodic()**:
- Calculate simulated target angle based on robot pose (if using pose estimation)
- Set simulated NetworkTable values for tx, ty, tv
- Add realistic noise/latency

### 5.3 Light Sensor Simulation

**In Shooter simulationPeriodic()**:
- Simulate ball detection based on simulated game state
- Toggle digital input simulation based on feeder state/timing

---

## Phase 6: Test Mode and Tuning

### 6.1 Dashboard Tuning Interface

**SmartDashboard/Shuffleboard Entries to Create**:

**Telemetry (Read-Only)**:
- Shooter RPM (current)
- Shooter Target RPM
- Shooter Current (amps)
- Shooter Voltage
- Shooter Temperature
- Feeder State (on/off)
- Ball Detected (boolean)
- Vision Distance (feet)
- Vision Target Valid (boolean)
- Shooter State (enum)
- Ready to Shoot (boolean)

**Tunable Parameters** (Test Mode):
- PID kP (editable)
- PID kI (editable)
- PID kD (editable)
- PID kF (editable)
- kS Static Friction (editable)
- kV Velocity Feedforward (editable)
- RPM Tolerance (editable)
- Target RPM Override (editable)
- Distance Override (editable)

### 6.2 Test Mode Command

**Create**: `src/main/java/frc/robot/commands/ShooterTestMode.java`

**Purpose**: Allow live PID tuning during test mode

**Methods**:
- `execute()`: Continuously read dashboard values and update PID
- Provide manual RPM control
- Log performance metrics (rise time, overshoot, steady-state error)

### 6.3 Characterization Support

**Add methods to Shooter.java**:

| Method | Parameters | Return Type | Description |
|--------|------------|-------------|-------------|
| `runCharacterization()` | `double voltage` | `void` | Apply raw voltage for SysId characterization |
| `getCharacterizationVelocity()` | none | `double` | Return velocity for SysId |
| `getCharacterizationAcceleration()` | none | `double` | Return acceleration for SysId |

---

## Phase 7: Safety and Error Handling

### 7.1 Safety Checks

**Add to Shooter subsystem**:

```java
private void checkSafetyLimits() {
    // Overspeed protection
    if (getCurrentRPM() > MAX_RPM) {
        disableFlywheel();
        reportFault("Overspeed detected");
    }

    // Temperature monitoring
    if (shooterMotor.getDeviceTemp().getValue() > 80.0) {
        reportWarning("High motor temperature");
    }

    // Current monitoring
    if (shooterMotor.getSupplyCurrent().getValue() > SHOOTER_CURRENT_LIMIT) {
        reportWarning("High current draw");
    }
}
```

### 7.2 Fault Reporting

**Methods to Add**:

| Method | Parameters | Return Type | Description |
|--------|------------|-------------|-------------|
| `reportFault()` | `String message` | `void` | Log critical fault to DriverStation and dashboard |
| `reportWarning()` | `String message` | `void` | Log warning to DriverStation and dashboard |
| `clearFaults()` | none | `void` | Reset all motor controller faults |
| `checkMotorHealth()` | none | `boolean` | Verify motors responding and configured correctly |

---

## Phase 8: Testing and Validation

### 8.1 Unit Tests

**Create**: `src/test/java/frc/robot/subsystems/ShooterTest.java`

**Test Cases**:
- RPM lookup interpolation accuracy
- State machine transitions
- Safety limit enforcement
- Vision fallback behavior
- PID configuration application
- Sensor integration

### 8.2 Simulation Tests

**Create**: `src/test/java/frc/robot/subsystems/ShooterSimTest.java`

**Test Cases**:
- Flywheel acceleration profile
- PID convergence time
- Shooting sequence timing
- Vision integration in sim

### 8.3 Integration Checklist

- [ ] Verify CAN IDs match physical robot
- [ ] Verify motor directions (invert if needed)
- [ ] Test light sensor wiring and polarity
- [ ] Verify vision system communication
- [ ] Calibrate vision distance calculation
- [ ] Tune PID values empirically
- [ ] Verify RPM lookup table accuracy
- [ ] Test button bindings
- [ ] Test emergency stop
- [ ] Verify current limits protect motors
- [ ] Test simulation mode
- [ ] Test autonomous shooting sequences

---

## Implementation Checklist

### Prerequisites
- [ ] Install Phoenix 6 library (for TalonFX)
- [ ] Install REVLib (for SparkMax feeder)
- [ ] Install WPILib vision/NetworkTables libraries
- [ ] Verify all CAN IDs in constants match robot

### Phase 1: Constants (Est. 30 min)
- [ ] Add ShooterConstants with all RPM lookup values
- [ ] Add VisionConstants
- [ ] Calculate preliminary RPM values for lookup table
- [ ] Add PID starting values
- [ ] Add simulation constants

### Phase 2: Vision Integration (Est. 2 hours)
- [ ] Create VisionSubsystem.java
- [ ] Implement NetworkTables communication
- [ ] Implement distance calculation
- [ ] Add vision telemetry
- [ ] Test vision system independently
- [ ] Add simulation support for vision

### Phase 3: Shooter Hardware Refactor (Est. 3 hours)
- [ ] Replace SparkMax import with TalonFX
- [ ] Update motor instantiation
- [ ] Configure TalonFX for velocity control
- [ ] Implement PID configuration
- [ ] Add encoder velocity reading
- [ ] Configure current limits
- [ ] Test motor spinning (no PID)

### Phase 4: RPM Control (Est. 2 hours)
- [ ] Implement RPM lookup table logic
- [ ] Add interpolation between distance points
- [ ] Implement setTargetDistance() method
- [ ] Implement velocity PID control
- [ ] Test RPM setpoint tracking

### Phase 5: Sensor Integration (Est. 1 hour)
- [ ] Add DigitalInput for light sensor
- [ ] Implement hasBall() method
- [ ] Test sensor detection
- [ ] Add sensor simulation

### Phase 6: State Machine (Est. 2 hours)
- [ ] Implement ShooterState enum
- [ ] Implement state transitions
- [ ] Implement canShoot() logic
- [ ] Add safety interlocks
- [ ] Test state machine flow

### Phase 7: Commands (Est. 2 hours)
- [ ] Create inline command factories
- [ ] Implement VisionAssistedShoot command
- [ ] Implement ShootSequence command
- [ ] Test command execution

### Phase 8: RobotContainer Integration (Est. 1 hour)
- [ ] Instantiate subsystems
- [ ] Add button bindings
- [ ] Set default commands
- [ ] Test all button functions

### Phase 9: Telemetry (Est. 1.5 hours)
- [ ] Add SmartDashboard entries for all telemetry
- [ ] Create Shuffleboard tab layout
- [ ] Add tunable PID parameters
- [ ] Implement updatePIDFromDashboard()
- [ ] Test live tuning

### Phase 10: Simulation (Est. 2 hours)
- [ ] Add FlywheelSim to Shooter
- [ ] Implement simulationPeriodic()
- [ ] Add vision simulation
- [ ] Add sensor simulation
- [ ] Test in simulation mode

### Phase 11: Testing (Est. 3 hours)
- [ ] Write unit tests
- [ ] Run simulation tests
- [ ] Characterize shooter with SysId (if available)
- [ ] Tune PID values empirically
- [ ] Validate RPM lookup table
- [ ] Test full shooting sequence
- [ ] Verify safety limits

### Phase 12: Refinement (Est. 2 hours)
- [ ] Optimize PID tuning
- [ ] Adjust RPM lookup values based on testing
- [ ] Fine-tune state machine timing
- [ ] Add additional telemetry as needed
- [ ] Document final configuration

---

## Dependency Management

### Required Libraries

**build.gradle dependencies**:
```groovy
dependencies {
    implementation wpi.java.deps.wpilib()
    implementation wpi.java.vendor.java()

    // Phoenix 6 for TalonFX/Kraken
    implementation "com.ctre.phoenix6:wpiapi-java:24.0.0"

    // REVLib for SparkMax feeder
    implementation "com.revrobotics.frc:REVLib-java:2024.2.4"

    // PhotonVision (if using instead of Limelight)
    // implementation "org.photonvision:PhotonLib-java:v2024.3.1"

    roborioDebug wpi.java.deps.wpilibJniDebug(wpi.platforms.roborio)
    roborioRelease wpi.java.deps.wpilibJniRelease(wpi.platforms.roborio)

    nativeDebug wpi.java.deps.wpilibJniDebug(wpi.platforms.desktop)
    nativeRelease wpi.java.deps.wpilibJniRelease(wpi.platforms.desktop)

    simulationDebug wpi.sim.enableDebug()
    simulationRelease wpi.sim.enableRelease()
}
```

### Vendor Dependencies

Install via VS Code command palette:
- **CTRE Phoenix 6**: For TalonFX/Kraken X60
- **REVLib**: For SparkMax/Neo feeder motor
- **Limelight**: If using Limelight vision (or PhotonVision)

---

## Risk Mitigation

### Technical Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Vision system unreliable | Can't shoot accurately | Implement robust fallback to 10' default |
| PID tuning difficult | Poor RPM control | Use SysId characterization, extensive testing |
| Light sensor false positives | Premature shooting | Add debouncing, multiple sensor checks |
| TalonFX configuration errors | Motor won't run | Comprehensive error checking, Phoenix Tuner testing |
| RPM lookup table inaccurate | Missed shots | Empirical testing and adjustment |

### Safety Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Motor overspeed | Hardware damage | Software RPM limits, current limiting |
| Motor overheating | Damage/brownout | Temperature monitoring, current limits |
| Unintended shooting | Safety hazard | Multiple interlocks, require button press |
| Vision targeting wrong object | Incorrect shooting | Pipeline tuning, target validation |

---

## Performance Targets

### Acceptance Criteria

- [ ] Shooter reaches target RPM within 1.5 seconds
- [ ] RPM maintained within ±50 RPM of target during shooting
- [ ] Vision distance measurement accurate to ±6 inches
- [ ] Ball detection sensor has <50ms latency
- [ ] Shooting sequence completes in <3 seconds total
- [ ] PID tuning can be done live from dashboard
- [ ] Simulation mode accurately represents real robot behavior
- [ ] No motor faults or brownouts during operation
- [ ] All safety interlocks function correctly

---

## Future Enhancements

### Post-Initial Implementation

1. **Auto-Aiming**: Use vision horizontal offset to rotate robot automatically
2. **Moving Target Prediction**: Lead moving targets using velocity estimation
3. **Multi-Ball Sequences**: Rapid fire multiple balls with RPM compensation
4. **Adaptive Lookup**: Machine learning to adjust RPM based on success/failure
5. **Trajectory Visualization**: Real-time shooting arc display on dashboard
6. **Odometry Integration**: Use robot pose for distance calculation instead of vision
7. **Automatic Calibration**: Self-tuning PID based on performance metrics

---

## Appendix: Code Snippets

### A1: TalonFX Velocity Control Example

```java
// In Shooter constructor
TalonFXConfiguration config = new TalonFXConfiguration();

// PID Configuration
Slot0Configs pidConfig = config.Slot0;
pidConfig.kP = ShooterConstants.SHOOTER_KP;
pidConfig.kI = ShooterConstants.SHOOTER_KI;
pidConfig.kD = ShooterConstants.SHOOTER_KD;
pidConfig.kV = ShooterConstants.SHOOTER_KV;
pidConfig.kS = ShooterConstants.SHOOTER_KS;

// Current Limits
CurrentLimitsConfigs currentConfig = config.CurrentLimits;
currentConfig.SupplyCurrentLimit = ShooterConstants.SHOOTER_CURRENT_LIMIT;
currentConfig.SupplyCurrentThreshold = ShooterConstants.SHOOTER_CURRENT_THRESHOLD;
currentConfig.SupplyTimeThreshold = ShooterConstants.SHOOTER_CURRENT_TIME;
currentConfig.SupplyCurrentLimitEnable = true;

// Apply configuration
shooterMotor.getConfigurator().apply(config);

// Create velocity request
velocityRequest = new VelocityVoltage(0).withSlot(0);
```

### A2: RPM Lookup with Interpolation

```java
public double getRPMFromDistance(double distanceFeet) {
    // Clamp distance to valid range
    double clampedDistance = Math.max(
        DISTANCE_RPM_MAP[0][0],
        Math.min(distanceFeet, DISTANCE_RPM_MAP[DISTANCE_RPM_MAP.length - 1][0])
    );

    // Find bracketing values
    for (int i = 0; i < DISTANCE_RPM_MAP.length - 1; i++) {
        double d1 = DISTANCE_RPM_MAP[i][0];
        double d2 = DISTANCE_RPM_MAP[i + 1][0];

        if (clampedDistance >= d1 && clampedDistance <= d2) {
            double rpm1 = DISTANCE_RPM_MAP[i][1];
            double rpm2 = DISTANCE_RPM_MAP[i + 1][1];

            // Linear interpolation
            double ratio = (clampedDistance - d1) / (d2 - d1);
            return rpm1 + ratio * (rpm2 - rpm1);
        }
    }

    // Fallback (should never reach here)
    return DISTANCE_RPM_MAP[DISTANCE_RPM_MAP.length / 2][1]; // Middle value
}
```

### A3: State Machine Implementation

```java
@Override
public void periodic() {
    // Update sensor readings
    double currentRPM = getCurrentRPM();
    boolean ballPresent = hasBall();
    boolean atTargetRPM = Math.abs(currentRPM - targetRPM) < RPM_TOLERANCE;

    // State machine
    switch (currentState) {
        case IDLE:
            if (flywheelEnabled) {
                currentState = ShooterState.SPINNING_UP;
            }
            break;

        case SPINNING_UP:
            if (!flywheelEnabled) {
                currentState = ShooterState.IDLE;
            } else if (atTargetRPM) {
                currentState = ShooterState.READY;
            }
            break;

        case READY:
            if (!flywheelEnabled) {
                currentState = ShooterState.IDLE;
            } else if (!atTargetRPM) {
                currentState = ShooterState.SPINNING_UP;
            }
            // Shooting logic handled by command
            break;

        case SHOOTING:
            // Managed by command
            break;
    }

    // Update telemetry
    logTelemetry();

    // Safety checks
    checkSafetyLimits();
}
```

### A4: Vision Distance Calculation

```java
public double getDistanceToTarget() {
    if (!hasTarget()) {
        return -1.0; // Invalid
    }

    double ty = tyEntry.getDouble(0.0); // Vertical offset in degrees

    // Trigonometry: distance = height_diff / tan(total_angle)
    double heightDiff = VisionConstants.TARGET_HEIGHT_METERS -
                        VisionConstants.CAMERA_HEIGHT_METERS;
    double totalAngleDegrees = VisionConstants.CAMERA_ANGLE_DEGREES + ty;
    double totalAngleRadians = Math.toRadians(totalAngleDegrees);

    double distanceMeters = heightDiff / Math.tan(totalAngleRadians);
    double distanceFeet = distanceMeters * 3.28084; // Convert to feet

    // Validate range
    if (distanceFeet < VisionConstants.MIN_VISION_DISTANCE_FEET ||
        distanceFeet > VisionConstants.MAX_VISION_DISTANCE_FEET) {
        return -1.0; // Out of range
    }

    return distanceFeet;
}
```

---

## Summary

This implementation plan provides a comprehensive roadmap for refactoring the shooter subsystem from Neo/SparkMax to Kraken X60/TalonFX with full vision integration, closed-loop RPM control, sensor integration, and robust testing capabilities. The modular approach allows for incremental implementation and testing, with clear acceptance criteria for each phase.

**Estimated Total Implementation Time**: 20-25 hours

**Key Success Factors**:
1. Thorough testing at each phase
2. Extensive use of telemetry for debugging
3. Empirical PID tuning with real hardware
4. Robust fallback mechanisms for vision failures
5. Comprehensive simulation for safe initial testing

**Critical Path**:
1. Hardware refactor (TalonFX integration)
2. PID tuning
3. Vision integration
4. RPM lookup validation
5. Integration testing

This plan ensures a systematic, safe, and successful implementation of the competition shooter system.
