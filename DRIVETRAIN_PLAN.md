# Drivetrain Implementation Plan
## KoP Chassis 6-Wheel Drop Center Differential Drive with Neo Motors

---

## Executive Summary

This document outlines the complete implementation plan for the drivetrain subsystem using the Kit of Parts (KoP) chassis in square configuration with 6-wheel drop center differential drive. The implementation includes converting from PWM to CAN-based SparkMax controllers, adding encoder feedback and odometry, implementing closed-loop velocity control, and integrating with PathPlanner for autonomous navigation.

---

## Current Code Analysis

### Strengths of Existing Implementation

1. **Clean Architecture**: The subsystem follows WPILib command-based architecture properly
2. **Drive Mode Support**: Already implements arcade and tank drive modes with toggle
3. **DifferentialDrive**: Uses WPILib's DifferentialDrive class correctly
4. **Method Structure**: Basic drive methods provide a good foundation
5. **Command Factory**: Has driveCommand() factory method pattern

### Weaknesses and Issues

1. **PWM Control**: Uses PWMSparkMax instead of CAN-based SparkMax
2. **Single Motor Per Side**: Only one motor per side, need follower motors
3. **No Encoder Feedback**: Cannot track position or velocity
4. **No Odometry**: Cannot determine robot position on field
5. **No Current Limiting**: Motors have no protection
6. **No Closed-Loop Control**: Open-loop only, no velocity PID
7. **Limited Telemetry**: No SmartDashboard integration
8. **No Autonomous Support**: Cannot follow paths or trajectories
9. **No Characterization**: Unknown physical constants (gear ratio, wheel size)

### What Can Stay

- Overall class structure (extends SubsystemBase)
- DriveMode enum and toggle logic
- Method naming conventions (arcadeDrive, tankDrive)
- driveCommand() factory pattern
- DifferentialDrive wrapper usage

### What Must Change

- Motor controllers (PWMSparkMax -> SparkMax CAN)
- Add follower motors for each side
- Add SparkMaxConfig for current limits and settings
- Add encoder access and position/velocity tracking
- Implement odometry for field position
- Add SmartDashboard telemetry
- Add closed-loop velocity control
- Integrate with PathPlanner for autonomous

---

## Requirements Summary

1. **Motor Configuration**: 2x Neo motors per side with SparkMax (CAN)
   - Left leader + left follower
   - Right leader + right follower
2. **Current Limiting**: Protect motors and prevent brownouts
3. **Encoder Feedback**: Track wheel position and velocity
4. **Odometry**: Maintain robot pose (x, y, heading) on field
5. **Telemetry**: Display position, velocity, pose on dashboard
6. **Closed-Loop Control**: Velocity PID for consistent autonomous driving
7. **PathPlanner Integration**: Support trajectory following for autonomous
8. **Drive Modes**: Maintain arcade/tank toggle for driver preference

---

## Hardware Specifications

### Neo Motor Specs
- **Free Speed**: 5676 RPM
- **Stall Torque**: 2.6 N⋅m
- **Stall Current**: 105 A
- **Free Current**: 1.8 A
- **Voltage**: 12V nominal

### KoP Chassis Specs (Square Configuration)
- **Wheel Diameter**: 6 inches (0.1524 m)
- **Gear Ratio**: 10.71:1 (kitbot standard)
- **Track Width**: ~21.5 inches (0.546 m) - measure actual robot
- **Wheelbase**: ~21.5 inches (square configuration)
- **Drop Center**: Middle wheels dropped for turning

### Calculated Values
- **Wheel Circumference**: 0.479 m (π × 0.1524)
- **Position Conversion**: 0.0447 m per motor rotation (circumference / gear ratio)
- **Velocity Conversion**: 0.000745 m/s per RPM
- **Max Theoretical Speed**: ~4.2 m/s (5676 RPM / 10.71 × circumference / 60)

---

## Detailed Implementation Plan

---

## Phase 1: Constants and Configuration

### 1.1 Update DriveTrainConstants Class

**File**: `src/main/java/frc/robot/Constants.java`

**Add the following constants**:

```java
public static class DriveTrainConstants {
    // CAN IDs for SparkMax controllers
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // Motor configuration
    public static final int STALL_LIMIT = 40;        // Amps - stall current limit
    public static final int FREE_LIMIT = 30;         // Amps - free running limit
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double RAMP_RATE = 0.1;      // Seconds from 0 to full

    // Physical constants - KoP Chassis
    public static final double WHEEL_DIAMETER_METERS = 0.1524;  // 6 inches
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double GEAR_RATIO = 10.71;              // Kitbot standard
    public static final double TRACK_WIDTH_METERS = 0.546;      // Measure actual robot

    // Encoder conversion factors
    public static final double POSITION_CONVERSION_FACTOR =
        WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO;  // Motor rotations to meters
    public static final double VELOCITY_CONVERSION_FACTOR =
        POSITION_CONVERSION_FACTOR / 60.0;        // RPM to meters per second

    // PID constants for velocity control (tune on robot)
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.22;  // Feedforward (1 / max velocity in m/s)

    // Max speeds
    public static final double MAX_VELOCITY_MPS = 3.5;          // Conservative estimate
    public static final double MAX_ACCELERATION_MPS2 = 2.0;     // For path planning
    public static final double MAX_ANGULAR_VELOCITY_RPS = 2.0;  // Radians per second

    // Drive modifiers
    public static final double SLOW_MODE_MULTIPLIER = 0.5;
    public static final double TURN_SENSITIVITY = 0.8;
}
```

---

## Phase 2: Motor Controller Conversion

### 2.1 Refactor DriveTrain Subsystem

**File**: `src/main/java/frc/robot/subsystems/DriveTrain.java`

**Components to Instantiate**:
- `SparkMax leftLeader` - Left side leader motor
- `SparkMax leftFollower` - Left side follower motor
- `SparkMax rightLeader` - Right side leader motor
- `SparkMax rightFollower` - Right side follower motor
- `RelativeEncoder leftEncoder` - Left side encoder from leader
- `RelativeEncoder rightEncoder` - Right side encoder from leader
- `SparkMaxConfig config` - Motor configuration object
- `DifferentialDrive driver` - Drive helper (unchanged)

**Configuration Requirements**:
- Voltage compensation for consistent behavior
- Smart current limiting to prevent brownouts
- Follower mode for secondary motors
- Proper motor inversion for robot orientation
- Encoder conversion factors for meters

---

## Phase 3: Odometry Implementation

### 3.1 Add Odometry Components

**Additional Components**:
- `DifferentialDriveOdometry odometry` - WPILib odometry tracker
- `Gyro/AHRS gyro` - NavX or Pigeon for heading (optional, can use encoders only)
- `Pose2d currentPose` - Current robot position

**Methods Needed**:

| Method | Parameters | Return Type | Description |
|--------|------------|-------------|-------------|
| `getLeftPosition()` | none | `double` | Returns left encoder position in meters |
| `getRightPosition()` | none | `double` | Returns right encoder position in meters |
| `getLeftVelocity()` | none | `double` | Returns left velocity in m/s |
| `getRightVelocity()` | none | `double` | Returns right velocity in m/s |
| `getPose()` | none | `Pose2d` | Returns current robot pose |
| `resetOdometry()` | `Pose2d pose` | `void` | Resets odometry to given pose |
| `resetEncoders()` | none | `void` | Zeros encoder positions |
| `getHeading()` | none | `Rotation2d` | Returns robot heading |

---

## Phase 4: Telemetry

### 4.1 SmartDashboard Integration

**Telemetry to Display**:
- Left encoder position (meters)
- Right encoder position (meters)
- Left velocity (m/s)
- Right velocity (m/s)
- Robot pose (x, y, rotation)
- Current drive mode (ARCADE/TANK)
- Left motor current (amps)
- Right motor current (amps)
- Left motor temperature
- Right motor temperature

**Field2d Widget**:
- Add Field2d for visualizing robot position
- Update in periodic()

---

## Phase 5: Closed-Loop Control

### 5.1 Velocity Control

**Methods to Add**:

| Method | Parameters | Return Type | Description |
|--------|------------|-------------|-------------|
| `setVelocity()` | `double leftMPS, double rightMPS` | `void` | Set wheel velocities |
| `tankDriveVolts()` | `double leftVolts, double rightVolts` | `void` | Direct voltage control for paths |
| `getWheelSpeeds()` | none | `DifferentialDriveWheelSpeeds` | For kinematics |

---

## Phase 6: PathPlanner Integration

### 6.1 Autonomous Requirements

**Methods for PathPlanner**:
- Pose supplier: `getPose()`
- Wheel speeds supplier: `getWheelSpeeds()`
- Voltage consumer: `tankDriveVolts()`
- Odometry reset: `resetOdometry()`
- Kinematics object for trajectory generation

---

## Incremental Implementation Plan (For Beginning Programmers)

**Philosophy**: Build one small, testable feature at a time. Each milestone adds ONE new concept and can be demonstrated independently. Don't move to the next milestone until the current one works!

---

### Prerequisites (Do Once at Start)
- [ ] Install REVLib via VS Code (Ctrl+Shift+P -> "WPILib: Manage Vendor Libraries" -> "Install new library (online)" -> REVLib)
- [ ] Verify CAN IDs in constants match your actual robot wiring
- [ ] Identify which motor should be inverted for your robot orientation

---

### Milestone 1: Convert to CAN SparkMax (Single Motor Per Side)
**Goal**: Replace PWMSparkMax with CAN-based SparkMax for the two leader motors only.

**What You'll Learn**: SparkMax basics, CAN communication, motor configuration

**Tasks**:
- [ ] Add DriveTrainConstants with LEFT_LEADER_ID and RIGHT_LEADER_ID
- [ ] Import `com.revrobotics.spark.SparkMax` and related classes
- [ ] Replace `PWMSparkMax` instantiation with `SparkMax` using CAN IDs
- [ ] Create SparkMaxConfig with:
  - Voltage compensation (12V)
  - Smart current limit (40A stall, 30A free)
  - Idle mode (brake)
- [ ] Apply config to both motors
- [ ] Test motor inversion - right side usually inverted
- [ ] Verify DifferentialDrive still works with new motors

**Test**: Drive robot with joystick. Should work exactly as before but now using CAN.

**Expected Time**: 1.5 hours

---

### Milestone 2: Add Follower Motors
**Goal**: Add second motor per side that follows the leader automatically.

**What You'll Learn**: Leader/follower configuration, motor grouping

**Tasks**:
- [ ] Add LEFT_FOLLOWER_ID and RIGHT_FOLLOWER_ID to constants
- [ ] Create leftFollower and rightFollower SparkMax objects
- [ ] Configure followers with same settings as leaders
- [ ] Use `follow()` method to make followers track leaders
- [ ] Set follower inversion if needed (usually not for same-side motors)
- [ ] Verify both motors on each side spin together

**Test**: Lift robot, run motors, verify both left motors spin same direction and both right motors spin same direction.

**Expected Time**: 1 hour

---

### Milestone 3: Add Basic Telemetry
**Goal**: Display motor information on SmartDashboard.

**What You'll Learn**: SmartDashboard, debugging with telemetry

**Tasks**:
- [ ] Add method `logTelemetry()` to DriveTrain
- [ ] Call `logTelemetry()` in `periodic()`
- [ ] Display to SmartDashboard:
  - Current drive mode (ARCADE/TANK)
  - Left leader output current
  - Right leader output current
  - Left leader applied output
  - Right leader applied output
- [ ] Open SmartDashboard/Shuffleboard and verify data updates

**Test**: Drive robot, watch current draw change on dashboard.

**Expected Time**: 30 minutes

---

### Milestone 4: Add Encoder Reading
**Goal**: Read wheel positions and velocities from motor encoders.

**What You'll Learn**: Encoder access, unit conversion, position tracking

**Tasks**:
- [ ] Add physical constants to DriveTrainConstants:
  - WHEEL_DIAMETER_METERS
  - GEAR_RATIO
  - POSITION_CONVERSION_FACTOR
  - VELOCITY_CONVERSION_FACTOR
- [ ] Get encoder objects from leader motors: `motor.getEncoder()`
- [ ] Configure encoder conversion factors:
  - `encoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR)`
  - `encoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR)`
- [ ] Add methods:
  - `getLeftPosition()` - returns encoder position
  - `getRightPosition()` - returns encoder position
  - `getLeftVelocity()` - returns encoder velocity
  - `getRightVelocity()` - returns encoder velocity
  - `resetEncoders()` - zeros both encoders
- [ ] Add encoder values to telemetry

**Test**: Push robot by hand, watch position values change on dashboard. Reset encoders, values go to zero.

**Expected Time**: 1 hour

---

### Milestone 5: Implement Odometry (Encoder-Only)
**Goal**: Track robot position on field using wheel encoders (no gyro yet).

**What You'll Learn**: Odometry, coordinate frames, pose estimation

**Tasks**:
- [ ] Add TRACK_WIDTH_METERS to constants (measure your robot!)
- [ ] Import `DifferentialDriveOdometry` and `Pose2d`
- [ ] Create odometry object in constructor:
  - `new DifferentialDriveOdometry(new Rotation2d(), 0, 0)`
- [ ] Add state variable: `Pose2d currentPose`
- [ ] In periodic(), update odometry:
  - `odometry.update(getHeading(), getLeftPosition(), getRightPosition())`
- [ ] Add methods:
  - `getPose()` - returns current pose
  - `resetOdometry(Pose2d pose)` - resets to given pose
  - `getHeading()` - for now, calculate from encoders or return 0
- [ ] Add pose (x, y, rotation) to telemetry
- [ ] Add Field2d widget for visualization

**Test**: Reset odometry, drive robot forward 1 meter, check X position on dashboard shows ~1.0.

**Expected Time**: 1.5 hours

---

### Milestone 6: Add Gyro for Heading (Optional but Recommended)
**Goal**: Use NavX or Pigeon IMU for accurate heading measurement.

**What You'll Learn**: IMU integration, sensor fusion

**Tasks**:
- [ ] If using NavX: Install navX-MXP library via vendor deps
- [ ] Import gyro class (AHRS for NavX, Pigeon2 for CTRE)
- [ ] Create gyro object in constructor
- [ ] Update `getHeading()` to return gyro angle as Rotation2d
- [ ] Add method `zeroHeading()` to reset gyro
- [ ] Update odometry.update() to use gyro heading
- [ ] Add heading to telemetry (degrees)

**Test**: Rotate robot in place, watch heading change on dashboard. Zero heading, it resets to 0.

**Expected Time**: 1 hour

---

### Milestone 7: Closed-Loop Velocity Control
**Goal**: Drive at specific wheel velocities using PID control.

**What You'll Learn**: Velocity PID, closed-loop control, feedforward

**Tasks**:
- [ ] Add PID constants to DriveTrainConstants: kP, kI, kD, kFF
- [ ] Configure SparkMax PID controller:
  - Get PIDController from SparkMax
  - Set P, I, D, FF gains
- [ ] Add method `setVelocity(double leftMPS, double rightMPS)`:
  - Use PID controller setReference with velocity setpoint
- [ ] Add method `tankDriveVolts(double leftVolts, double rightVolts)`:
  - Direct voltage control for trajectory following
- [ ] Add target vs actual velocity to telemetry

**Test**: Call setVelocity(1.0, 1.0), robot should drive at 1 m/s. Check telemetry shows velocities match.

**Expected Time**: 1.5 hours

---

### Milestone 8: PathPlanner Integration
**Goal**: Enable autonomous path following using PathPlanner.

**What You'll Learn**: Trajectory following, autonomous routines

**Tasks**:
- [ ] Install PathPlannerLib vendor dependency
- [ ] Create DifferentialDriveKinematics object with track width
- [ ] Add method `getWheelSpeeds()`:
  - Returns DifferentialDriveWheelSpeeds from encoder velocities
- [ ] Configure PathPlanner in RobotContainer:
  - Provide pose supplier: `drivetrain::getPose`
  - Provide reset consumer: `drivetrain::resetOdometry`
  - Provide wheel speeds: `drivetrain::getWheelSpeeds`
  - Provide output consumer: `drivetrain::tankDriveVolts`
  - Provide kinematics object
- [ ] Create simple test path in PathPlanner GUI
- [ ] Test path following command

**Test**: Deploy, run auto routine, robot follows path. Check pose on Field2d matches expected.

**Expected Time**: 2 hours

---

### Milestone 9: PID Tuning and Optimization
**Goal**: Tune PID values for smooth, accurate driving.

**What You'll Learn**: PID tuning, system characterization

**Tasks**:
- [ ] Add dashboard controls for live PID tuning:
  - Editable kP, kI, kD, kFF values
  - Button to apply changes
- [ ] Create method `updatePIDFromDashboard()`
- [ ] Tune velocity PID:
  - Start with kFF only (calculate from max speed)
  - Add kP until response is fast but not oscillating
  - Add kD if overshoot occurs
  - kI usually not needed for velocity
- [ ] Document final PID values
- [ ] Add ramp rate limiting for smooth acceleration

**Test**: Set velocity, watch response on graph. Should reach target quickly without overshoot.

**Expected Time**: 2 hours

---

### Milestone 10: Driver Features
**Goal**: Add quality-of-life features for driver control.

**What You'll Learn**: User experience, input processing

**Tasks**:
- [ ] Add slow mode toggle:
  - Button to reduce max speed to 50%
  - Add SLOW_MODE_MULTIPLIER to constants
- [ ] Add input deadband:
  - Ignore small joystick inputs
  - Prevents drift when hands off controller
- [ ] Add input curves (optional):
  - Square inputs for finer control at low speed
- [ ] Add turn sensitivity adjustment
- [ ] Verify arcade/tank toggle still works
- [ ] Add current drive mode to driver dashboard

**Test**: Test slow mode, test deadband, verify all drive modes feel good.

**Expected Time**: 1 hour

---

### Milestone 11: Safety and Diagnostics
**Goal**: Add safety features and health monitoring.

**What You'll Learn**: Error handling, safety systems

**Tasks**:
- [ ] Add motor temperature monitoring
- [ ] Add warning if temperature too high
- [ ] Add motor fault detection
- [ ] Add brownout protection (reduce speed if voltage low)
- [ ] Add method `checkMotorHealth()`:
  - Returns false if any motor has faults
- [ ] Add diagnostics to dashboard:
  - Motor temperatures
  - Fault indicators
  - Battery voltage

**Test**: Drive robot hard, watch temperatures. Simulate fault, verify warning appears.

**Expected Time**: 1 hour

---

## Total Incremental Timeline

| Milestone | Time Estimate | Cumulative | What Works After This Milestone |
|-----------|---------------|------------|----------------------------------|
| 1. CAN Conversion | 1.5 hours | 1.5 hours | Robot drives with CAN motors |
| 2. Follower Motors | 1 hour | 2.5 hours | Full power with 4 motors |
| 3. Telemetry | 0.5 hours | 3 hours | Can see motor data |
| 4. Encoders | 1 hour | 4 hours | Know wheel positions |
| 5. Odometry | 1.5 hours | 5.5 hours | Know robot position on field |
| 6. Gyro | 1 hour | 6.5 hours | Accurate heading |
| 7. Velocity PID | 1.5 hours | 8 hours | Consistent speed control |
| 8. PathPlanner | 2 hours | 10 hours | Autonomous paths work |
| 9. PID Tuning | 2 hours | 12 hours | Smooth, accurate driving |
| 10. Driver Features | 1 hour | 13 hours | Great driver experience |
| 11. Safety | 1 hour | 14 hours | Protected and monitored |

**Total**: ~14 hours of focused work, spread across multiple sessions

---

## Key Principles for Success

1. **One Thing at a Time**: Complete each milestone fully before moving on
2. **Test Immediately**: After every change, test it. Don't write lots of code before testing
3. **Measure Track Width**: Accurate track width is critical for odometry - measure carefully!
4. **Commit Often**: Git commit after each working milestone
5. **Use REV Hardware Client**: Monitor SparkMax status and configure CAN IDs
6. **Keep It Simple**: Don't add features until basics work perfectly
7. **Verify Motor Direction**: Wrong direction = robot drives in circles

---

## Common Beginner Mistakes to Avoid

- Wrong motor inversion -> Robot drives in circles or backwards
- Wrong CAN IDs -> Motor doesn't respond
- Forgot follower.follow() -> One motor per side doesn't run
- Wrong gear ratio -> Position/velocity values way off
- Track width inaccurate -> Robot turns wrong amount in auto
- No current limits -> Brownouts during competition
- Not resetting encoders -> Odometry starts at wrong position

---

## Risk Mitigation

### Technical Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| CAN bus issues | Motors don't respond | Use REV Hardware Client to diagnose |
| Encoder drift | Odometry inaccurate | Add gyro, periodic reset at known positions |
| PID tuning difficult | Jerky or slow response | Start with feedforward only, tune systematically |
| PathPlanner paths wrong | Robot goes off course | Test with simple paths first, verify odometry |
| Motor overheating | Reduced performance | Monitor temperature, add cooldown logic |

### Safety Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Uncontrolled movement | Safety hazard | Always have e-stop ready, test in open area |
| Brownouts | Robot disabled | Current limiting, monitor battery voltage |
| Motor burnout | Hardware damage | Temperature monitoring, current limits |

---

## Performance Targets

### Acceptance Criteria

- [ ] Robot drives straight when commanded straight
- [ ] Odometry accurate to within 10% over 5 meters
- [ ] Velocity control within 10% of setpoint
- [ ] PathPlanner paths followed within 15cm accuracy
- [ ] No motor faults during normal operation
- [ ] No brownouts during aggressive driving
- [ ] All telemetry updates at 50Hz minimum
- [ ] Driver reports controls feel responsive

---

## Future Enhancements

### Post-Initial Implementation

1. **Slip Detection**: Detect wheel slip using encoder comparison
2. **Traction Control**: Reduce power on slipping wheel
3. **Heading Correction**: Auto-correct for drift during straight driving
4. **Vision-Assisted Odometry**: Fuse vision pose with encoder odometry
5. **Motion Profiling**: Smoother acceleration curves
6. **Adaptive PID**: Adjust gains based on battery voltage
7. **Replay Logging**: AdvantageKit integration for post-match analysis

---

## Summary

This implementation plan provides a comprehensive roadmap for upgrading the drivetrain from basic PWM control to a fully-featured system with CAN-based motor control, accurate odometry, closed-loop velocity control, and autonomous path following. The modular approach allows for incremental implementation and testing, ensuring each feature works before moving to the next.

**Estimated Total Implementation Time**: 14 hours

**Key Success Factors**:
1. Accurate physical measurements (track width, wheel diameter)
2. Systematic PID tuning
3. Thorough testing at each milestone
4. Proper motor direction configuration
5. Current limiting for safety

**Critical Path**:
1. CAN motor conversion
2. Encoder integration
3. Odometry implementation
4. PathPlanner integration
5. PID tuning and validation
