# TP-Autonomous Branch Code Review

**Branch:** `TP-Autonomous`
**Reviewed by:** Hiebert
**Date:** 2026-02-06
**WPILib Season:** 2026

---

## Overview

The `TP-Autonomous` branch focuses on autonomous path planning and navigation using PathPlanner 2025.0. This branch introduces multiple autonomous routines designed for different starting positions and game strategies. The implementation leverages the existing differential drivetrain (tank/arcade drive) and establishes the foundation for autonomous movement during the 15-second autonomous period.

### Key Components
- **PathPlanner Integration:** Path planning and trajectory following
- **Autonomous Routines:** Pre-defined paths for various starting positions
- **Differential Drivetrain:** Tank drive robot with PWM SparkMax controllers
- **Robot Characterization:** Physical specifications for path generation

### Branch Changes
The TP-Autonomous branch adds the following files:
- **Autonomous Routines (3):**
  - `src/main/deploy/pathplanner/autos/s1 to hp auto.auto`
  - `src/main/deploy/pathplanner/autos/s2 to center auto.auto`
  - `src/main/deploy/pathplanner/autos/s2 to depo auto.auto`
  - `src/main/deploy/pathplanner/autos/s3 to center auto.auto`

- **Path Definitions (14):**
  - Test paths: `(P)Square (goofy).path`, `(P)arc.path`, `(P)fwd 10ft rvs 10ft (fix).path`, etc.
  - Game paths: `start 1.path`, `start 2.path`, `start 3.path`
  - Navigation paths: `s2 center.path`, `s3 center.path`, `human play f s1.path`, `depo f s2.path`
  - Example template: `Example Path.path`

- **Configuration:**
  - `src/main/deploy/pathplanner/settings.json` (PathPlanner robot configuration)

**Note:** This branch does NOT modify any Java code - all changes are PathPlanner JSON configuration files.

---

## Physical Robot Characteristics

### Robot Specifications (from PathPlanner settings.json)

#### Dimensions
- **Robot Width:** 0.9271 m (36.5 inches) - measured with bumpers
- **Robot Length:** 0.9271 m (36.5 inches) - measured with bumpers
- **Robot Shape:** Square footprint (typical FRC robot with bumpers)
- **Bumper Offset X:** 0.0 m (centered)
- **Bumper Offset Y:** 0.0 m (centered)

#### Mass & Inertia
- **Robot Mass:** 61.235 kg (135 lbs) - typical FRC weight limit compliance
- **Moment of Inertia (MOI):** 6.883 kg⋅m² - rotational inertia about center

#### Drivetrain Configuration
- **Drive Type:** Differential (tank) drive
- **Holonomic Mode:** `false` (cannot strafe sideways)
- **Trackwidth:** 0.546 m (21.5 inches) - distance between left/right wheels
- **Wheel Radius:** 0.0508 m (2 inches) - typical 4-inch diameter wheels
- **Drive Gearing:** 8.46:1 gear ratio (motor revolutions to wheel revolutions)
- **Motor Type:** NEO (REV Robotics brushless)
- **Max Drive Speed:** 3.0 m/s (~9.8 ft/s) - theoretical maximum
- **Drive Current Limit:** 40A per motor

#### Wheel Properties
- **Coefficient of Friction (COF):** 1.2 (high-traction wheels, e.g., blue nitrile tread)

#### Movement Constraints
- **Default Max Velocity:** 2.0 m/s (~6.6 ft/s)
- **Default Max Acceleration:** 2.0 m/s²
- **Default Max Angular Velocity:** 540°/s (1.5 rotations per second)
- **Default Max Angular Acceleration:** 720°/s²
- **Nominal Voltage:** 12.0V (standard FRC battery voltage)

### Drivetrain Hardware (from DriveTrain.java - baseline)

#### Motors
1. **Left Drive Motor**
   - **Type:** PWM SparkMax (REV Robotics)
   - **PWM Port:** 0
   - **Inverted:** False (default)

2. **Right Drive Motor**
   - **Type:** PWM SparkMax (REV Robotics)
   - **PWM Port:** 1
   - **Inverted:** True (common for differential drive)

#### Control System
- **Drive Modes:** Arcade (default) and Tank drive
- **Toggle Support:** Can switch between drive modes during operation
- **Control Input:** Xbox controller joystick axes

**⚠️ Important Note:** The drivetrain uses PWM control, not CAN. PathPlanner settings specify NEO motors with CAN-like configuration, but the actual Java implementation uses `PWMSparkMax`, which means:
- No encoder feedback available through motor controllers
- No current limiting configured in code (only in settings.json)
- Open-loop control only (no PID velocity control)

**🔴 CRITICAL:** To follow PathPlanner paths, the drivetrain will need:
1. Encoders for position feedback (currently missing)
2. Gyroscope for heading control (not present in code)
3. RamseteController or similar trajectory follower (not implemented)

---

## Expected Actions & Autonomous Routines

### Autonomous Strategy Overview

The TP-Autonomous branch defines **4 main autonomous routines** designed for different starting positions on the field:

| Routine Name | Starting Position | Strategy | Complexity |
|--------------|-------------------|----------|------------|
| **s1 to hp auto** | Starting Position 1 (left) | Navigate from start to Human Player station | Sequential (2 paths) |
| **s2 to center auto** | Starting Position 2 (center) | Navigate from start to center field | Sequential (2 paths) |
| **s2 to depo auto** | Starting Position 2 (center) | Navigate from start to Depot area | Sequential (2 paths) |
| **s3 to center auto** | Starting Position 3 (right) | Navigate from start to center field | Sequential (2 paths) |

### Autonomous Routine Breakdown

#### 1. s1 to hp auto (Start 1 → Human Player)
**Purpose:** Score pre-loaded game piece, then navigate to Human Player station for additional pieces

**Path Sequence:**
1. **start 1** - Initial movement from starting position 1
2. **human play f s1** - Navigate to Human Player (HP) station

**Expected Behavior:**
- Robot begins at leftmost starting position
- Moves forward/rotates as needed
- Navigates to Human Player zone for game piece pickup
- **Intended Use:** Autonomous routine when starting on driver station left side

**Estimated Duration:** ~5-8 seconds (depends on path complexity)

---

#### 2. s2 to center auto (Start 2 → Center Field)
**Purpose:** Score pre-loaded piece, then navigate to center field for additional game pieces

**Path Sequence:**
1. **start 2** - Initial movement from starting position 2 (center)
2. **s2 center** - Navigate to center field (middle of game area)

**Path Details (from s2 center.path):**
- **Starting Point:** (2.476, 6.0) meters
- **Waypoints:**
  - (4.0, 7.25) - intermediate position
  - (5.5, 7.25) - second intermediate
  - (6.5, 6.0) - curve point
  - (6.2, 4.0) - approach center
- **End Point:** (7.0, 4.0) meters
- **Direction:** Reversed path (drives backward)
- **Constraints:** 2.0 m/s max, 2.0 m/s² accel
- **End State:** 0 m/s velocity, 0° rotation

**Expected Behavior:**
- Robot starts in center starting position
- Navigates complex curved path to center field
- Likely intended to collect additional game pieces at center
- Ends in controlled stop (0 velocity) facing forward

**Estimated Duration:** ~8-10 seconds

---

#### 3. s2 to depo auto (Start 2 → Depot)
**Purpose:** Score pre-loaded piece, then navigate to Depot area

**Path Sequence:**
1. **start 2** - Initial movement from starting position 2
2. **depo f s2** - Navigate to Depot area from start 2

**Expected Behavior:**
- Robot starts in center starting position
- Navigates to Depot (likely scoring/storage area)
- Alternative strategy to "s2 to center"
- **Intended Use:** When center field is contested or depot strategy preferred

**Estimated Duration:** ~6-9 seconds

---

#### 4. s3 to center auto (Start 3 → Center Field)
**Purpose:** Score pre-loaded piece from rightmost start, then navigate to center

**Path Sequence:**
1. **start 3** - Initial movement from starting position 3 (right)
2. **s3 center** - Navigate to center field from start 3

**Expected Behavior:**
- Robot begins at rightmost starting position
- Mirrors similar strategy to "s2 to center" but from different angle
- Navigates to center field for game piece collection
- **Intended Use:** When starting on driver station right side

**Estimated Duration:** ~8-10 seconds

---

### Test Paths (Practice/Tuning)

The branch includes several test paths prefixed with `(P)` for practice and tuning:

| Test Path | Purpose | Description |
|-----------|---------|-------------|
| **(P)fwd 10ft** | Simple forward test | Drive 10 feet forward from start position |
| **(P)fwd 10ft rvs 10ft (fix)** | Forward/reverse test | Drive 10 ft forward, then 10 ft backward |
| **(P)arc** | Curve testing | Test smooth arc/curved path following |
| **(P)Square (goofy)** | Complex navigation | Drive in square pattern to test turning |
| **(P)rotate 90 left** | Rotation test | Pure rotation (90° counterclockwise) |
| **Example Path** | Template | PathPlanner default example template |

**Intended Use:** These paths are for:
1. **Characterization:** Tune PID constants and feedforward
2. **Validation:** Verify path following accuracy
3. **Debugging:** Isolate specific movement issues
4. **Driver Practice:** Allow drivers to see autonomous behavior

---

## Methods & Implementation Details

### PathPlanner Configuration (settings.json)

#### ✅ Completed Configuration

1. **Robot Physical Properties** ✅
   - All dimensions, mass, and inertia specified
   - Trackwidth and wheel radius defined
   - Accurate bumper measurements

2. **Drive Constraints** ✅
   - Max velocity, acceleration limits set
   - Angular velocity and acceleration defined
   - Nominal voltage specified (12V)

3. **Autonomous Routines** ✅ (4 complete)
   - s1 to hp auto (HP station strategy)
   - s2 to center auto (center field strategy)
   - s2 to depo auto (depot strategy)
   - s3 to center auto (alternate center strategy)

4. **Path Organization** ✅
   - Folders: "Test" and "actual autos"
   - 14 individual path files created
   - Naming convention: descriptive and consistent

#### PathPlanner Settings Summary
```json
{
  "holonomicMode": false,              // Differential drive (tank)
  "defaultMaxVel": 2.0,                // m/s (conservative for reliability)
  "defaultMaxAccel": 2.0,              // m/s²
  "defaultMaxAngVel": 540.0,           // degrees/s
  "defaultMaxAngAccel": 720.0,         // degrees/s²
  "robotMass": 61.235,                 // kg (~135 lbs)
  "robotTrackwidth": 0.546,            // m (21.5 inches)
  "driveWheelRadius": 0.0508,          // m (4-inch diameter wheels)
  "driveGearing": 8.46,                // gear ratio
  "driveMotorType": "NEO"              // REV brushless motor
}
```

---

### Drivetrain Implementation (DriveTrain.java - baseline)

**Note:** The TP-Autonomous branch does NOT modify DriveTrain.java. This is the baseline implementation from the `main` branch.

#### ✅ Completed Methods

##### Constructor: `DriveTrain()`
- **Status:** ✅ Implemented
- **Function:** Initializes left and right PWM motors
- **Configuration:** Right motor inverted for differential drive

##### Drive Control Methods

1. **`drive(double x, double y)`** ✅
   - **Function:** Abstract drive method, routes to arcade or tank
   - **Inputs:** x (left/speed), y (right/turn)
   - **Behavior:** Switches based on current `driveMode` enum

2. **`arcadeDrive(double x, double y)`** ✅ (private)
   - **Function:** Single-joystick driving (speed + turn)
   - **Control:** `-x` for speed (inverted), `y` for turn
   - **2026 API:** ✅ Uses `DifferentialDrive.arcadeDrive()`

3. **`tankDrive(double x, double y)`** ✅ (private)
   - **Function:** Dual-joystick driving (left stick, right stick)
   - **Control:** `-x` for left side, `-y` for right side (both inverted)
   - **2026 API:** ✅ Uses `DifferentialDrive.tankDrive()`

##### Drive Mode Management

1. **`toggleDriveMode()`** ✅
   - **Function:** Switch between ARCADE and TANK modes
   - **Implementation:** Modern switch expression syntax

2. **`setDriveMode(DriveMode mode)`** ✅
   - **Function:** Directly set drive mode (ARCADE or TANK)

3. **`getDriveMode()`** ✅
   - **Returns:** Current drive mode enum

##### Command Factories

1. **`driveCommand(DoubleSupplier fnX, DoubleSupplier fnY)`** ✅
   - **Returns:** `RunCommand` for continuous driving
   - **Inputs:** Joystick axis suppliers
   - **2026 API:** ✅ Uses modern Command-based patterns

##### Testing

1. **`testDriveTrain()`** ✅ (static)
   - **Returns:** List of test results
   - **Current Implementation:** Returns dummy passing test
   - **Purpose:** Framework for unit testing

---

### 🔴 Missing Components for Autonomous

The TP-Autonomous branch provides **path planning only**. To execute these paths, the following components are **NOT IMPLEMENTED** but **REQUIRED**:

#### 🔴 CRITICAL MISSING COMPONENTS

1. **Odometry System** ❌ NOT IMPLEMENTED
   - **Required:** Track robot position on field (x, y, heading)
   - **Needs:** Encoders on drive motors OR external tracking
   - **2026 API:** `DifferentialDriveOdometry` class
   - **Status:** No encoders configured (PWM controllers don't provide feedback)

2. **Gyroscope/IMU** ❌ NOT IMPLEMENTED
   - **Required:** Track robot heading (rotation angle)
   - **Options:** NavX, Pigeon 2, ADIS16470 IMU
   - **2026 API:** `AHRS` (NavX) or `Pigeon2` (CTRE)
   - **Status:** No gyro present in code

3. **Drive Encoders** ❌ NOT IMPLEMENTED
   - **Required:** Measure wheel rotations for distance traveled
   - **Options:**
     - Switch to CAN SparkMax (built-in NEO encoders)
     - Add external encoders to PWM motors
   - **2026 API:** `Encoder` class or `RelativeEncoder` (SparkMax)
   - **Status:** PWM controllers have no encoder feedback

4. **Path Following Controller** ❌ NOT IMPLEMENTED
   - **Required:** Convert PathPlanner trajectories to motor commands
   - **Options:**
     - `RamseteController` (for differential drive)
     - PathPlanner's `PPRamseteController`
   - **2026 API:** `RamseteCommand` or PathPlanner auto builder
   - **Status:** No trajectory following code

5. **Autonomous Command Integration** ❌ NOT IMPLEMENTED
   - **Required:** Load and execute autonomous routines
   - **Location:** `RobotContainer.getAutonomousCommand()`
   - **2026 API:** PathPlanner's `AutoBuilder.buildAuto("s1 to hp auto")`
   - **Status:** Method commented out in RobotContainer

6. **Pose Estimation** ❌ NOT IMPLEMENTED
   - **Required:** Estimate robot position during match
   - **2026 API:** `DifferentialDrivePoseEstimator` (combines odometry + vision)
   - **Status:** No pose estimation configured

---

## Planned Features & Implementation Roadmap

### 🔴 HIGH PRIORITY (Required for Autonomous)

#### 1. Hardware Upgrades ⚠️ REQUIRED
**Decision Point: PWM vs CAN Motors**

**Option A: Switch to CAN SparkMax (RECOMMENDED)**
- **Pros:**
  - Built-in encoders (no additional wiring)
  - Current limiting in firmware
  - Better diagnostics and telemetry
  - Easier integration with PathPlanner
- **Cons:**
  - Higher cost (~$70/motor vs $30 for PWM)
  - Requires CAN IDs assignment
- **Implementation:**
  ```java
  // Replace PWMSparkMax with CANSparkMax
  private final CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  ```

**Option B: Keep PWM + Add External Encoders**
- **Pros:**
  - Lower cost (keep existing motors)
  - Minimal hardware changes
- **Cons:**
  - Additional wiring complexity
  - Separate encoder channels (4 DIO ports total)
  - More points of failure
- **Implementation:**
  ```java
  private final Encoder leftEncoder = new Encoder(0, 1); // DIO 0, 1
  private final Encoder rightEncoder = new Encoder(2, 3); // DIO 2, 3
  leftEncoder.setDistancePerPulse(2 * Math.PI * 0.0508 / 2048); // wheel circumference / pulses
  ```

#### 2. Gyroscope Integration ⚠️ REQUIRED
**Recommended: NavX2 or Pigeon 2**

**NavX2 Implementation:**
```java
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

private final AHRS gyro = new AHRS(SPI.Port.kMXP);

public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle()); // Inverted for CCW positive
}

public void resetGyro() {
    gyro.reset();
}
```

**Pigeon 2 Implementation (if using CTRE ecosystem):**
```java
import com.ctre.phoenix6.hardware.Pigeon2;

private final Pigeon2 gyro = new Pigeon2(0); // CAN ID 0

public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
}
```

#### 3. Odometry System ⚠️ REQUIRED
**Implement DifferentialDriveOdometry**

```java
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

private final DifferentialDriveOdometry odometry;

public DriveTrain() {
    // ... existing motor setup ...
    odometry = new DifferentialDriveOdometry(
        getHeading(),              // Initial heading from gyro
        0.0,                       // Left encoder initial position
        0.0,                       // Right encoder initial position
        new Pose2d()               // Initial pose (0, 0, 0°)
    );
}

@Override
public void periodic() {
    // Update odometry every 20ms
    odometry.update(
        getHeading(),
        leftEncoder.getDistance(),   // meters
        rightEncoder.getDistance()   // meters
    );

    // Log to SmartDashboard
    Pose2d pose = odometry.getPoseMeters();
    SmartDashboard.putNumber("Robot X", pose.getX());
    SmartDashboard.putNumber("Robot Y", pose.getY());
    SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
}

public Pose2d getPose() {
    return odometry.getPoseMeters();
}

public void resetOdometry(Pose2d pose) {
    leftEncoder.reset();
    rightEncoder.reset();
    odometry.resetPosition(getHeading(), 0.0, 0.0, pose);
}
```

#### 4. PathPlanner Integration ⚠️ REQUIRED
**Implement Auto Builder**

**Add to `build.gradle` (if not present):**
```gradle
dependencies {
    implementation 'com.pathplanner.lib:PathplannerLib:2025.0.0'
}
```

**In DriveTrain.java:**
```java
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// Add kinematics
private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(0.546); // trackwidth in meters

public void configureAutoBuilder() {
    AutoBuilder.configure(
        this::getPose,                // Pose supplier
        this::resetOdometry,          // Pose reset consumer
        this::getChassisSpeeds,       // ChassisSpeeds supplier
        this::drive,                  // ChassisSpeeds consumer (drive method)
        new ReplanningConfig(),       // Replanning config
        this                          // Subsystem requirement
    );
}

public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
        leftEncoder.getRate(),   // left wheel velocity (m/s)
        rightEncoder.getRate()   // right wheel velocity (m/s)
    );
}

public void drive(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    // Convert to voltage outputs or velocity setpoints
    // This requires velocity PID on motors
}
```

**In RobotContainer.java:**
```java
import com.pathplanner.lib.auto.AutoBuilder;

public RobotContainer() {
    configureBindings();

    // Configure PathPlanner
    m_drivetrain.configureAutoBuilder();
}

public Command getAutonomousCommand() {
    // Load autonomous routine from PathPlanner
    return AutoBuilder.buildAuto("s2 to center auto");
}
```

#### 5. PID Velocity Control ⚠️ REQUIRED
**Implement closed-loop velocity control for path following**

**For CAN SparkMax:**
```java
import com.revrobotics.spark.SparkMaxPID;
import com.revrobotics.spark.SparkMaxConfig;

private final SparkMaxPID leftPID;
private final SparkMaxPID rightPID;

public DriveTrain() {
    // ... motor setup ...

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(0.1, 0.0, 0.0); // kP, kI, kD - tune these!
    config.closedLoop.outputRange(-1.0, 1.0);

    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftPID = leftMotor.getClosedLoopController();
    rightPID = rightMotor.getClosedLoopController();
}

public void setVelocity(double leftVelocityMPS, double rightVelocityMPS) {
    // Convert m/s to RPM
    double leftRPM = (leftVelocityMPS / (2 * Math.PI * 0.0508)) * 60.0;
    double rightRPM = (rightVelocityMPS / (2 * Math.PI * 0.0508)) * 60.0;

    leftPID.setReference(leftRPM, SparkMax.ControlType.kVelocity);
    rightPID.setReference(rightRPM, SparkMax.ControlType.kVelocity);
}
```

---

### 🟡 MEDIUM PRIORITY (Improves Performance)

#### 6. Drive Characterization 📋 Planned
**Use SysID to characterize drivetrain**
- Measure kS (static friction), kV (velocity), kA (acceleration)
- Improves path following accuracy
- Tools: WPILib SysID application

#### 7. Vision Integration (AprilTags) 📋 Planned
**Enhance odometry with vision**
- PhotonVision already in vendordeps
- Add `DifferentialDrivePoseEstimator` (replaces `DifferentialDriveOdometry`)
- Fuse encoder + gyro + vision data
- Improves position accuracy during auto

#### 8. Advanced Auto Routines 📋 Planned
**Expand autonomous options**
- Add more starting positions
- Create defensive/offensive variants
- Implement game piece manipulation commands
- Add event markers in paths (e.g., intake on, shooter spin-up)

---

### 🟢 LOW PRIORITY (Nice to Have)

#### 9. Autonomous Selector 📋 Future
**Allow drivers to choose auto routine**
- SendableChooser on SmartDashboard/Shuffleboard
- List all available autos from PathPlanner
- Default to preferred strategy

#### 10. Path Visualization 📋 Future
**Real-time path display**
- Use Glass or AdvantageScope
- Show planned path vs actual robot position
- Easier debugging and tuning

#### 11. Simulation Testing 📋 Future
**Test autos in simulation**
- WPILib simulation support
- Verify paths before deploying to robot
- Safer testing without hardware

---

## 2026 WPILib API Compliance

### ✅ Currently Used 2026 APIs (baseline DriveTrain)

#### WPILib Drivetrain
- ✅ `DifferentialDrive` - standard tank/arcade drive control
- ✅ `MotorController` interface
- ✅ `PWMSparkMax` - PWM motor controllers
- ✅ `SubsystemBase` - modern subsystem base class

#### Command Framework
- ✅ `Command` and `RunCommand` - command-based programming
- ✅ `DoubleSupplier` - functional programming for joystick inputs

### ⚠️ REQUIRED 2026 APIs (not yet implemented)

#### Odometry & Kinematics
- ⚠️ `DifferentialDriveOdometry` - position tracking
- ⚠️ `DifferentialDriveKinematics` - velocity calculations
- ⚠️ `Pose2d`, `Rotation2d` - geometry classes
- ⚠️ `ChassisSpeeds` - robot velocity representation

#### Sensors
- ⚠️ `Encoder` (WPILib) OR `RelativeEncoder` (SparkMax)
- ⚠️ `AHRS` (NavX) OR `Pigeon2` (CTRE gyro)

#### Path Following
- ⚠️ `RamseteCommand` - path following for differential drive
- ⚠️ PathPlanner library (`AutoBuilder`, `PathPlannerAuto`)

#### Motor Control (if upgrading to CAN)
- ⚠️ `CANSparkMax` - CAN motor controller (replaces PWM)
- ⚠️ `SparkMaxConfig` - 2026 configuration API
- ⚠️ `SparkMaxPID` - closed-loop velocity control

### PathPlanner 2025.0 Compliance

- ✅ **PathPlanner Version:** 2025.0 (latest for 2026 season)
- ✅ **JSON Format:** All path files use correct schema
- ✅ **Settings Configuration:** Robot specs properly defined
- ✅ **Folder Organization:** "Test" and "actual autos" folders
- ⚠️ **AutoBuilder Integration:** Not yet implemented in Java code

---

## Code Quality & Best Practices

### ✅ Strengths (PathPlanner Configuration)
- Clean, organized folder structure (Test vs actual autos)
- Descriptive path/auto names (easy to understand intent)
- Realistic robot specifications (accurate dimensions and mass)
- Conservative velocity limits (2.0 m/s for reliability)
- Multiple autonomous options for strategy flexibility

### ✅ Strengths (DriveTrain.java baseline)
- Clean code with clear method separation
- Flexible drive mode switching (arcade/tank)
- Modern Java syntax (switch expressions)
- Command-based architecture properly used

### ⚠️ Critical Gaps
1. **No encoder feedback** - cannot measure distance traveled
2. **No gyroscope** - cannot measure rotation accurately
3. **No odometry** - cannot track robot position
4. **No path following** - cannot execute PathPlanner trajectories
5. **Open-loop control** - no velocity PID for accurate movement
6. **PWM motors** - limited functionality compared to CAN

### 🔧 Recommended Implementation Order

**Phase 1: Hardware & Basic Feedback (Week 1-2)**
1. Upgrade to CAN SparkMax motors OR add external encoders
2. Install and configure gyroscope (NavX2 recommended)
3. Test encoder readings and gyro heading
4. Verify wheel distance calibration

**Phase 2: Odometry & Basic Auto (Week 3-4)**
5. Implement `DifferentialDriveOdometry`
6. Test odometry accuracy (drive known distances/angles)
7. Add test path execution (start with "(P)fwd 10ft")
8. Tune encoder distance per pulse

**Phase 3: Path Following (Week 5-6)**
9. Implement PID velocity control on motors
10. Configure PathPlanner `AutoBuilder`
11. Test simple path execution
12. Tune Ramsete PID constants

**Phase 4: Competition Autos (Week 7-8)**
13. Test all 4 autonomous routines
14. Add event markers for actions (intake, shooter, etc.)
15. Create autonomous selector for driver station
16. Practice autonomous routines at competition field dimensions

---

## Testing Recommendations

### Phase 1: Sensor Validation
1. **Encoder Testing**
   - Push robot by hand, verify encoder counts
   - Drive 10 feet, measure actual distance vs. encoder distance
   - Verify both left and right encoders count correctly
   - Check for encoder direction (positive = forward)

2. **Gyro Calibration**
   - Robot must remain stationary during gyro initialization
   - Rotate robot 360°, verify gyro returns to ~0° (within 2-3°)
   - Test CW and CCW rotation
   - Verify CCW = positive angle

3. **Odometry Validation**
   - Drive straight 10 feet → odometry should show (3.048m, 0, 0°)
   - Rotate 90° in place → odometry should show heading = 90°
   - Drive in square → odometry should return to start (within 10cm)

### Phase 2: Path Following
4. **Simple Path Testing**
   - Start with "(P)fwd 10ft" test path
   - Measure actual distance traveled vs. expected
   - Verify robot drives straight (no drift)
   - Test "(P)rotate 90 left" for pure rotation

5. **Complex Path Testing**
   - Test "(P)arc" for smooth curve following
   - Verify robot follows waypoints accurately (within 10cm)
   - Test "(P)Square (goofy)" for combined translation + rotation

### Phase 3: Competition Autos
6. **Autonomous Routine Testing**
   - Test each of the 4 main autos on competition-sized field
   - Measure final position accuracy (should be within 15cm of target)
   - Test with different battery voltages (full charge vs 11V)
   - Practice with alliance partners (avoid collisions)

7. **Edge Case Testing**
   - Test starting from all 3 starting positions
   - Verify paths don't collide with field elements
   - Test autonomous with pre-loaded game piece
   - Emergency stop during autonomous (verify safe behavior)

---

## Summary

The TP-Autonomous branch provides **comprehensive path planning** for autonomous operation, but **lacks all necessary code implementation** to execute these paths. The PathPlanner configuration is well-designed with realistic robot specifications and multiple strategic autonomous routines.

**Key Strengths:**
- ✅ Professional PathPlanner configuration (4 competition autos + test paths)
- ✅ Accurate robot physical specifications
- ✅ Multiple autonomous strategies for different starting positions
- ✅ Well-organized folder structure and naming

**Critical Gaps (Must Address):**
1. 🔴 **BLOCKING:** No encoders (cannot measure distance)
2. 🔴 **BLOCKING:** No gyroscope (cannot measure rotation)
3. 🔴 **BLOCKING:** No odometry (cannot track position)
4. 🔴 **BLOCKING:** No path following controller (cannot execute paths)
5. 🔴 **BLOCKING:** No velocity PID control (cannot follow trajectories accurately)

**Implementation Roadmap:**
1. 🔴 **Week 1-2:** Hardware upgrades (CAN motors OR encoders + gyro)
2. 🔴 **Week 3-4:** Odometry implementation and testing
3. 🟡 **Week 5-6:** PathPlanner integration and path following
4. 🟢 **Week 7-8:** Competition auto testing and tuning

**Overall Assessment:**
- **Path Planning:** Competition-ready (8/10)
- **Code Implementation:** Not started (0/10)
- **Hardware Readiness:** Missing critical sensors (2/10)

**Estimated Effort:** 6-8 weeks of focused development to reach competition-ready autonomous capability.

---

**Reviewer Notes:**
The autonomous paths demonstrate solid game strategy and field awareness. The team has clearly planned out multiple autonomous options for different scenarios. However, **this branch cannot execute autonomously in its current state** - it requires significant drivetrain upgrades (sensors + software) before any path can be followed. Prioritize encoder/gyro installation immediately, as all subsequent work depends on these foundational components.

The PathPlanner settings indicate the team intends to use NEO motors with CAN communication, but the actual drivetrain code uses PWM. This discrepancy must be resolved before autonomous can work. **Recommendation: Upgrade to CAN SparkMax motors** for best PathPlanner integration and reliability.
