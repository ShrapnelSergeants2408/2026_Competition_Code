# Drivetrain Architecture Documentation

## Overview
This document outlines the complete architecture for the 2026 FRC robot drivetrain system, including:
- **Pure drivetrain** with differential drive (tank/arcade modes, robot/field oriented)
- **PathPlanner autonomous** using LTV (Linear Time-Varying) controller
- **Vision processing** with PhotonVision for AprilTag detection and odometry fusion

## System Components

### 1. Hardware Configuration
- **Motors**: 4x REV Robotics NEO motors (SparkMax controllers) in leader/follower configuration
  - Left side:
    - Leader: CAN ID 20
    - Follower: CAN ID 21
  - Right side:
    - Leader: CAN ID 22 (inverted)
    - Follower: CAN ID 23 (inverted)
- **Encoders**: Integrated NEO encoders (via SparkMax on leader controllers)
- **Gyro**: Studica AHRS NavX (MXP SPI connection)
- **Cameras**: 2x Raspberry Pi cameras with PhotonVision
  - Front camera (Pi4 with PiCam v2):
    - AprilTag detection for field position
    - Ball (fuel) detection and intake alignment
    - Trench and tower alignment
  - Rear camera (Pi5 with OV9281):
    - AprilTag detection for field position
    - Shot targeting (hub center) - shooter is rear-facing
    - Trench, tower, and outpost alignment
- **Robot Dimensions** (from PathPlanner settings):
  - Track width: 0.546 m (21.5 inches)
  - Wheel radius: 0.0508 m (2 inches)
  - Drive gearing: 8.46:1
  - Robot mass: 61.235 kg
  - Robot MOI: 6.883 kg⋅m²

---

## Section 1: Pure Drivetrain (No Odometry)

### 1.1 Purpose
Basic differential drive control for teleoperated mode with flexible driving styles.

### 1.2 Required Declarations

#### Hardware
```java
// Motor controllers (leader/follower configuration)
private final SparkMax leftLeader;
private final SparkMax leftFollower;
private final SparkMax rightLeader;
private final SparkMax rightFollower;

// Differential drive helper (uses leaders only)
private final DifferentialDrive differentialDrive;

// Drive mode state
public enum DriveMode { ARCADE, TANK }
public enum OrientationMode { ROBOT_ORIENTED, FIELD_ORIENTED }
private DriveMode driveMode = DriveMode.ARCADE;
private OrientationMode orientationMode = OrientationMode.ROBOT_ORIENTED;
```

#### Constants
```java
public static final class DrivetrainConstants {
    // CAN IDs (leader/follower configuration)
    public static final int LEFT_LEADER_CAN_ID = 20;
    public static final int LEFT_FOLLOWER_CAN_ID = 21;
    public static final int RIGHT_LEADER_CAN_ID = 22;
    public static final int RIGHT_FOLLOWER_CAN_ID = 23;

    // Current limits
    public static final int DRIVE_CURRENT_LIMIT = 40; // Amps

    // Ramping
    public static final double OPEN_LOOP_RAMP = 0.25; // seconds
    public static final double CLOSED_LOOP_RAMP = 0.0; // seconds

    // Inversion (right side inverted)
    public static final boolean LEFT_MOTORS_INVERTED = false;
    public static final boolean RIGHT_MOTORS_INVERTED = true;

    // Deadband
    public static final double JOYSTICK_DEADBAND = 0.05;
}
```

### 1.3 Required Methods

#### Basic Drive Control
```java
/**
 * Main drive method that routes to appropriate drive mode
 */
public void drive(double x, double y);

/**
 * Arcade drive: one stick for forward/back, one for rotation
 * @param speed Forward speed [-1.0, 1.0]
 * @param rotation Rotation rate [-1.0, 1.0]
 */
private void arcadeDrive(double speed, double rotation);

/**
 * Tank drive: independent left/right control
 * @param leftSpeed Left side speed [-1.0, 1.0]
 * @param rightSpeed Right side speed [-1.0, 1.0]
 */
private void tankDrive(double leftSpeed, double rightSpeed);

/**
 * Apply deadband to joystick input
 */
private double applyDeadband(double value, double deadband);

/**
 * Stop all motors
 */
public void stop();
```

#### Mode Management
```java
/**
 * Set drive mode (ARCADE or TANK)
 */
public void setDriveMode(DriveMode mode);

/**
 * Get current drive mode
 */
public DriveMode getDriveMode();

/**
 * Toggle between ARCADE and TANK modes
 */
public void toggleDriveMode();

/**
 * Set orientation mode (ROBOT_ORIENTED or FIELD_ORIENTED)
 */
public void setOrientationMode(OrientationMode mode);

/**
 * Get current orientation mode
 */
public OrientationMode getOrientationMode();
```

#### Command Factories
```java
/**
 * Create arcade drive command for teleop
 */
public Command teleopArcadeCommand(DoubleSupplier fwd, DoubleSupplier rot);

/**
 * Create tank drive command for teleop
 */
public Command teleopTankCommand(DoubleSupplier left, DoubleSupplier right);

/**
 * Create field-oriented arcade drive command
 */
public Command fieldOrientedArcadeCommand(DoubleSupplier fwd, DoubleSupplier rot);
```

### 1.4 Implementation Notes
- Use `DifferentialDrive` class from WPILib for safety features and squaring inputs
- Configure leader/follower relationships:
  - Follower motors should use `.follow(leader)` method
  - Followers automatically mirror leader's output including inversion
  - Only leaders need to be passed to `DifferentialDrive`
- Configure motor idle mode (brake vs coast) based on competition needs
- Implement voltage compensation for consistent performance across battery levels
- Consider implementing slew rate limiters for smoother acceleration

---

## Section 2: PathPlanner Autonomous with LTV Controller

### 2.1 Purpose
Enable autonomous path following using PathPlanner trajectories with Linear Time-Varying (LTV) control for robust trajectory tracking.

### 2.2 What is LTV (Linear Time-Varying) Control?

#### Overview
LTV is a **state-space control technique** that computes optimal control outputs by solving the Linear Quadratic Regulator (LQR) problem at each timestep along the trajectory.

#### How LTV Works
1. **Linearization**: At each point along the path, the controller linearizes the robot's dynamics around the current trajectory state
2. **Time-Varying**: Unlike standard LQR, LTV adapts the controller gains as the robot moves along the trajectory
3. **Optimal Control**: Minimizes a cost function that balances:
   - **Tracking error** (Q matrix): How much you care about position/heading errors
   - **Control effort** (R matrix): How much you care about limiting actuator usage

#### Why Use LTV for Differential Drive?
- **Better trajectory tracking** than pure pursuit or ramsete at varying speeds
- **Handles velocity changes** smoothly (acceleration/deceleration)
- **Robust to modeling errors** when properly tuned
- **Built into PathPlanner** via `PPLTVController`

#### LTV Tuning Parameters

**Q Matrix (State Weights)**
- Penalizes errors in state variables: [x_error, y_error, heading_error]
- Higher values = controller works harder to minimize that error
- Typical differential drive values:
  - `qx = 0.0625` (6.25 cm tolerance in X)
  - `qy = 0.125` (12.5 cm tolerance in Y)
  - `qθ = 2.0` (2 radian tolerance in heading)

**R Matrix (Control Effort Weights)**
- Penalizes control outputs: [linear_velocity, angular_velocity]
- Higher values = smoother but slower response
- Typical values:
  - `rv = 1.0` (linear velocity penalty)
  - `rω = 2.0` (angular velocity penalty)

**dt (Control Loop Period)**
- Must match your robot's update frequency
- FRC standard: `0.02` seconds (20 ms / 50 Hz)

**Max Velocity**
- Maximum expected robot velocity in m/s
- Used for controller linearization
- Should match or exceed your PathPlanner max velocity setting
- From settings: `3.0 m/s`

### 2.3 Required Declarations

#### Kinematics and Odometry
```java
// Kinematics
private final DifferentialDriveKinematics kinematics;

// Odometry/Pose Estimation
private final DifferentialDrivePoseEstimator poseEstimator;

// Encoders (from leader motors)
private final RelativeEncoder leftEncoder;  // From leftLeader
private final RelativeEncoder rightEncoder; // From rightLeader

// Gyro
private final AHRS gyro;

// Field visualization
private final Field2d field;

// AprilTag field layout
private final AprilTagFieldLayout aprilTagFieldLayout;
```

#### PathPlanner Configuration
```java
// Robot configuration (loaded from PathPlanner GUI)
private RobotConfig robotConfig;

// LTV Controller
private PPLTVController ltvController;
```

#### Constants
```java
public static final class AutoConstants {
    // LTV Tuning Parameters
    // Q Matrix: State cost weights [x, y, heading]
    public static final double LTV_Q_X = 0.0625;      // 6.25 cm tolerance
    public static final double LTV_Q_Y = 0.125;       // 12.5 cm tolerance
    public static final double LTV_Q_THETA = 2.0;     // 2 radian tolerance

    // R Matrix: Control effort weights [v, omega]
    public static final double LTV_R_V = 1.0;         // Linear velocity
    public static final double LTV_R_OMEGA = 2.0;     // Angular velocity

    // Control loop period (20ms)
    public static final double LTV_DT = 0.02;

    // Max velocity for linearization (m/s)
    public static final double MAX_VELOCITY = 3.0;

    // Encoder conversion factors
    public static final double WHEEL_DIAMETER_METERS = 0.1016; // 4 inches
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_GEAR_RATIO = 8.46; // Motor rotations per wheel rotation

    // Position conversion: rotations -> meters
    // (motor rotations) / (gear ratio) * (wheel circumference)
    public static final double POSITION_CONVERSION_FACTOR =
        WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;

    // Velocity conversion: RPM -> meters/second
    // (motor RPM) / (gear ratio) * (wheel circumference) / 60
    public static final double VELOCITY_CONVERSION_FACTOR =
        POSITION_CONVERSION_FACTOR / 60.0;

    // Track width (distance between left and right wheels)
    public static final double TRACK_WIDTH_METERS = 0.546;

    // PathPlanner settings (should match GUI settings.json)
    public static final double MAX_MODULE_SPEED = 3.0; // m/s
    public static final double MAX_ACCEL = 2.0; // m/s²
    public static final double MAX_ANG_VEL = Math.toRadians(540); // rad/s
    public static final double MAX_ANG_ACCEL = Math.toRadians(720); // rad/s²
}
```

### 2.4 Required Methods

#### Odometry and Pose Management
```java
/**
 * Get current robot pose from pose estimator
 */
public Pose2d getPose();

/**
 * Reset pose to a known position
 * @param pose New pose to set
 */
public void resetPose(Pose2d pose);

/**
 * Get current robot heading from gyro
 */
public Rotation2d getHeading();

/**
 * Reset gyro to zero
 */
public void resetGyro();

/**
 * Get left encoder position in meters
 */
public double getLeftDistanceMeters();

/**
 * Get right encoder position in meters
 */
public double getRightDistanceMeters();

/**
 * Get current wheel speeds
 */
public DifferentialDriveWheelSpeeds getWheelSpeeds();

/**
 * Get current chassis speeds (robot-relative)
 */
public ChassisSpeeds getRobotRelativeSpeeds();
```

#### PathPlanner Integration
```java
/**
 * Drive robot using robot-relative chassis speeds
 * This is the main callback for PathPlanner AutoBuilder
 * @param speeds Robot-relative chassis speeds
 */
public void driveRobotRelative(ChassisSpeeds speeds);

/**
 * Drive robot using field-relative chassis speeds
 * @param speeds Field-relative chassis speeds
 */
public void driveFieldRelative(ChassisSpeeds speeds);

/**
 * Configure PathPlanner AutoBuilder
 * Should be called once during subsystem initialization
 */
private void configureAutoBuilder();

/**
 * Get an autonomous command from PathPlanner
 * @param autoName Name of the auto in PathPlanner GUI
 */
public Command getAutoCommand(String autoName);
```

#### Conversion Methods
```java
/**
 * Convert chassis speeds to wheel voltages using feedforward
 */
private void setSpeedsVoltage(DifferentialDriveWheelSpeeds speeds);

/**
 * Convert wheel speeds to left/right motor percent outputs
 * Simple open-loop conversion: speed / maxSpeed
 */
private void setSpeedsOpenLoop(DifferentialDriveWheelSpeeds speeds);
```

#### Telemetry
```java
/**
 * Update telemetry on SmartDashboard
 * - Current pose
 * - Heading
 * - Wheel speeds
 * - Encoder positions
 */
private void updateTelemetry();
```

### 2.5 Implementation Flow

#### Initialization (in constructor)
```java
public Drivetrain() {
    // 1. Configure motor controllers (leaders and followers)
    configureMotors();

    // 2. Set up leader/follower relationships
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // 3. Get encoders from leader motors
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    // 4. Set encoder conversion factors
    leftEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
    leftEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    rightEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
    rightEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

    // 5. Reset encoders and gyro
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    gyro.reset();

    // 6. Initialize kinematics
    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    // 7. Initialize pose estimator
    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        new Pose2d() // Start at origin
    );

    // 8. Load AprilTag field layout
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2026RebuiltWelded
    );

    // 9. Configure PathPlanner AutoBuilder
    configureAutoBuilder();

    // 10. Add field to dashboard
    SmartDashboard.putData("Field", field);
}
```

#### Configure AutoBuilder
```java
private void configureAutoBuilder() {
    try {
        // Load robot config from PathPlanner GUI settings
        robotConfig = RobotConfig.fromGUISettings();

        // Create LTV controller with tuning parameters
        ltvController = new PPLTVController(
            LTV_DT,  // Control loop period (0.02s)
            MAX_VELOCITY  // Max velocity for linearization
        );

        // Configure AutoBuilder with all required callbacks
        AutoBuilder.configure(
            this::getPose,                    // Pose supplier
            this::resetPose,                  // Pose reset consumer
            this::getRobotRelativeSpeeds,     // Speeds supplier
            this::driveRobotRelative,         // Robot-relative drive consumer
            ltvController,                    // Path following controller
            robotConfig,                      // Robot configuration
            () -> {
                // Mirror paths for red alliance
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() &&
                       alliance.get() == DriverStation.Alliance.Red;
            },
            this  // Subsystem requirement
        );

        // Set up path logging for visualization
        PathPlannerLogging.setLogActivePathCallback(
            (poses) -> field.getObject("path").setPoses(poses)
        );

    } catch (Exception e) {
        DriverStation.reportError("Failed to load PathPlanner config",
                                 e.getStackTrace());
    }
}
```

#### Periodic Update (in periodic())
```java
@Override
public void periodic() {
    // Update pose estimator with encoder and gyro data
    poseEstimator.update(
        getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters()
    );

    // Update field widget with current pose
    field.setRobotPose(getPose());

    // Update dashboard telemetry
    updateTelemetry();

    // Vision updates will be added in Section 3
}
```

### 2.6 LTV Tuning Guide

#### Step 1: Start with Conservative Values
```java
// More tolerant (larger values in Q)
qx = 0.125, qy = 0.25, qtheta = 4.0
rv = 2.0, romega = 4.0
```

#### Step 2: Observe Behavior
- **Sluggish response**: Decrease R values (less penalty on control effort)
- **Oscillation/overshoot**: Increase R values or decrease Q values
- **Poor X tracking**: Increase qx
- **Poor Y tracking**: Increase qy
- **Poor heading tracking**: Increase qtheta

#### Step 3: Iterate
- Test on simple paths first (straight lines, simple turns)
- Gradually make Q values smaller (tighter tracking) as system stabilizes
- Balance between tracking accuracy and control smoothness

#### Common Issues
| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| Oscillates along path | Q too high or R too low | Increase R or decrease Q |
| Doesn't follow path closely | Q too low or R too high | Decrease R or increase Q |
| Jerky motion | R too low | Increase R for smoother control |
| Slow to correct errors | Q too low | Increase Q |
| Heading drifts | qtheta too low | Increase qtheta |

---

## Section 3: AprilTag Vision Processing

### 3.1 Purpose
Use PhotonVision with AprilTag detection to:
1. **Update odometry** with vision measurements for accurate pose estimation
2. **Provide driver feedback** for game piece targeting
3. **Enable alignment** with field elements (hub, human player station)

### 3.2 Vision System Architecture

We use a **separate VisionSubsystem** to maintain clean separation of concerns:
- **VisionSubsystem**: Owns cameras, processes detections, provides filtered pose estimates
- **Drivetrain**: Consumes vision measurements and fuses them into pose estimator

#### Dual-Camera Field Position System
Both cameras contribute to field position determination through AprilTag detection:
- **Front Camera (Pi4 + PiCam v2)**:
  - Primary: Ball (fuel) detection and intake alignment
  - Secondary: AprilTag-based field localization
  - Alignment targets: Trench and tower
- **Rear Camera (Pi5 + OV9281)**:
  - Primary: Shot targeting to hub center (shooter is rear-facing)
  - Secondary: AprilTag-based field localization
  - Alignment targets: Trench, tower, and outpost

The VisionSubsystem fuses measurements from both cameras to provide robust field position estimates, using the best available data based on tag visibility, distance, and ambiguity.

### 3.3 Required Declarations (VisionSubsystem)

#### Hardware
```java
// PhotonVision cameras
private final PhotonCamera frontCamera;
private final PhotonCamera rearCamera;

// Pose estimators (one per camera)
private final PhotonPoseEstimator frontEstimator;
private final PhotonPoseEstimator rearEstimator;

// Field layout
private final AprilTagFieldLayout fieldLayout;

// Camera transforms (robot-to-camera)
private final Transform3d robotToFrontCam;
private final Transform3d robotToRearCam;
```

#### Vision Measurement Data Class
```java
public record VisionMeasurement(
    Pose2d estimatedPose,
    double timestampSeconds,
    Matrix<N3, N1> standardDeviations,
    int numTagsUsed,
    double averageDistance
) {}
```

#### Constants
```java
public static final class VisionConstants {
    // Camera names (must match PhotonVision configuration)
    public static final String FRONT_CAMERA_NAME = "Front_Camera"; // Pi4 with PiCam v2
    public static final String REAR_CAMERA_NAME = "Rear_Camera";   // Pi5 with OV9281

    // Camera hardware specifications
    // Front: Raspberry Pi 4 with PiCam v2
    //   - Used for: AprilTag field position, ball/fuel detection, intake alignment
    //   - Alignment: trench and tower
    // Rear: Raspberry Pi 5 with OV9281
    //   - Used for: AprilTag field position, shot targeting (hub center)
    //   - Shooter is rear-facing
    //   - Alignment: trench, tower, and outpost

    // Camera positions relative to robot center (meters)
    // Format: Translation3d(forward, left, up)
    // Rotation3d(roll, pitch, yaw) - all in radians

    // Front camera: mounted on front bumper, 0.3m forward, 0.0m left, 0.5m up
    // Tilted 30 degrees down
    public static final Transform3d ROBOT_TO_FRONT_CAM = new Transform3d(
        new Translation3d(0.3, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(-30), 0)
    );

    // Rear camera: mounted on rear bumper, -0.3m forward, 0.0m left, 0.5m up
    // Tilted 30 degrees down, facing backwards (180 deg yaw)
    // NOTE: Shooter is rear-facing, so this camera is used for shot targeting
    public static final Transform3d ROBOT_TO_REAR_CAM = new Transform3d(
        new Translation3d(-0.3, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(-30), Math.toRadians(180))
    );

    // Vision measurement quality gating
    public static final double MAX_TAG_DISTANCE_METERS = 4.0; // Ignore tags beyond 4m
    public static final double MAX_AMBIGUITY = 0.3; // Reject ambiguous detections
    public static final int MIN_TAGS_FOR_MULTI_TAG = 2; // Require 2+ tags for fusion

    // Standard deviations for pose estimation
    // [x_stddev, y_stddev, theta_stddev]
    // Lower = trust more, higher = trust less

    // Single tag close (< 2m): fairly trustworthy
    public static final Matrix<N3, N1> SINGLE_TAG_CLOSE_STDDEVS =
        VecBuilder.fill(0.5, 0.5, Math.toRadians(10));

    // Single tag far (> 2m): less trustworthy
    public static final Matrix<N3, N1> SINGLE_TAG_FAR_STDDEVS =
        VecBuilder.fill(1.0, 1.0, Math.toRadians(20));

    // Multi-tag: very trustworthy
    public static final Matrix<N3, N1> MULTI_TAG_STDDEVS =
        VecBuilder.fill(0.2, 0.2, Math.toRadians(5));

    // Known field element poses (from 2026 field layout)
    // Update these based on actual 2026 game elements
    public static final Pose2d HUB_POSE = new Pose2d(8.27, 4.1, new Rotation2d());
    public static final Pose2d HP_STATION_POSE = new Pose2d(1.5, 7.5, Rotation2d.fromDegrees(180));
}
```

### 3.4 Required Methods (VisionSubsystem)

#### Core Vision Processing
```java
/**
 * Get best available vision measurement from all cameras
 * Returns empty if no good measurements available
 */
public Optional<VisionMeasurement> getBestVisionMeasurement();

/**
 * Process results from a single camera
 */
private Optional<VisionMeasurement> processCameraResult(
    PhotonPoseEstimator estimator,
    PhotonCamera camera,
    String cameraName
);

/**
 * Determine if a vision measurement should be trusted
 * Gates on: ambiguity, distance, number of tags
 */
private boolean shouldUseMeasurement(
    EstimatedRobotPose estimate,
    PhotonPipelineResult result
);

/**
 * Calculate standard deviations based on measurement quality
 */
private Matrix<N3, N1> calculateStandardDeviations(
    EstimatedRobotPose estimate,
    double averageDistance
);

/**
 * Calculate average distance to all detected tags
 */
private double calculateAverageTagDistance(
    List<PhotonTrackedTarget> targets,
    Pose2d robotPose
);
```

#### Target Detection and Alignment
```java
/**
 * Get the best visible AprilTag target
 * @return Target with lowest ambiguity and distance
 */
public Optional<PhotonTrackedTarget> getBestTarget();

/**
 * Check if a specific tag ID is visible
 */
public boolean isTagVisible(int tagId);

/**
 * Get distance to a field element using robot pose
 */
public Optional<Double> getDistanceToPose(Pose2d targetPose);

/**
 * Get yaw angle to a field element
 * @return Rotation needed to face target
 */
public Optional<Rotation2d> getYawToPose(Pose2d targetPose);

/**
 * Check if robot is aligned with target within tolerance
 */
public boolean isAlignedWithTarget(Pose2d targetPose, double yawToleranceDeg);

/**
 * Get distance to hub for shooting
 */
public Optional<Double> getDistanceToHub();

/**
 * Check if aligned with hub for shooting
 */
public boolean isAlignedWithHub(double toleranceDeg);

/**
 * Get distance to human player station
 */
public Optional<Double> getDistanceToHPStation();

/**
 * Check if aligned with human player station
 */
public boolean isAlignedWithHPStation(double toleranceDeg);
```

#### Telemetry
```java
/**
 * Update vision telemetry on SmartDashboard
 * - Tag detections
 * - Pose estimates
 * - Ambiguity values
 * - Camera connection status
 */
private void updateTelemetry();

/**
 * Log camera connection status
 */
private boolean isCameraConnected(PhotonCamera camera);
```

### 3.5 Required Methods (Drivetrain - Vision Integration)

```java
/**
 * Add vision measurement to pose estimator
 * Called from periodic() when vision data available
 */
private void updateVisionMeasurements();

/**
 * Check if we should seed initial pose from vision
 * Useful at match start if positioned near known tags
 */
public Optional<Pose2d> getVisionSeededPose();

/**
 * Enable/disable vision fusion
 */
public void setVisionEnabled(boolean enabled);
```

### 3.6 Implementation Flow

#### VisionSubsystem Initialization
```java
public VisionSubsystem() {
    // Load field layout
    fieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2026RebuiltWelded
    );

    // Create cameras
    frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
    rearCamera = new PhotonCamera(REAR_CAMERA_NAME);

    // Create pose estimators
    frontEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCamera,
        ROBOT_TO_FRONT_CAM
    );
    frontEstimator.setMultiTagFallbackStrategy(
        PoseStrategy.LOWEST_AMBIGUITY
    );

    rearEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        rearCamera,
        ROBOT_TO_REAR_CAM
    );
    rearEstimator.setMultiTagFallbackStrategy(
        PoseStrategy.LOWEST_AMBIGUITY
    );
}
```

#### VisionSubsystem Periodic
```java
@Override
public void periodic() {
    // Update telemetry
    updateTelemetry();
}
```

#### Processing Camera Results
```java
private Optional<VisionMeasurement> processCameraResult(
    PhotonPoseEstimator estimator,
    PhotonCamera camera,
    String cameraName
) {
    // Get latest result
    PhotonPipelineResult result = camera.getLatestResult();

    // Check if we have targets
    if (!result.hasTargets()) {
        return Optional.empty();
    }

    // Update estimator with current robot pose hint (improves multi-tag)
    // This would come from the drivetrain via dependency injection

    // Get estimated pose
    Optional<EstimatedRobotPose> estimateOpt = estimator.update(result);
    if (estimateOpt.isEmpty()) {
        return Optional.empty();
    }

    EstimatedRobotPose estimate = estimateOpt.get();

    // Gate measurement quality
    if (!shouldUseMeasurement(estimate, result)) {
        return Optional.empty();
    }

    // Calculate average tag distance
    double avgDistance = calculateAverageTagDistance(
        result.getTargets(),
        estimate.estimatedPose.toPose2d()
    );

    // Calculate standard deviations
    Matrix<N3, N1> stdDevs = calculateStandardDeviations(estimate, avgDistance);

    // Create measurement
    return Optional.of(new VisionMeasurement(
        estimate.estimatedPose.toPose2d(),
        estimate.timestampSeconds,
        stdDevs,
        estimate.targetsUsed.size(),
        avgDistance
    ));
}
```

#### Quality Gating
```java
private boolean shouldUseMeasurement(
    EstimatedRobotPose estimate,
    PhotonPipelineResult result
) {
    // Reject if using single tag with high ambiguity
    if (estimate.targetsUsed.size() == 1) {
        PhotonTrackedTarget target = result.getBestTarget();
        if (target.getPoseAmbiguity() > MAX_AMBIGUITY) {
            return false;
        }
    }

    // Reject if tags are too far away
    double avgDistance = calculateAverageTagDistance(
        result.getTargets(),
        estimate.estimatedPose.toPose2d()
    );
    if (avgDistance > MAX_TAG_DISTANCE_METERS) {
        return false;
    }

    // Reject if pose seems unreasonable (off field)
    Pose2d pose = estimate.estimatedPose.toPose2d();
    if (pose.getX() < -1 || pose.getX() > 17.5 ||
        pose.getY() < -1 || pose.getY() > 9.0) {
        return false;
    }

    return true;
}
```

#### Drivetrain Vision Integration
```java
// In Drivetrain periodic()
@Override
public void periodic() {
    // Standard odometry update
    poseEstimator.update(
        getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters()
    );

    // Vision fusion
    if (visionEnabled) {
        updateVisionMeasurements();
    }

    // Telemetry
    field.setRobotPose(getPose());
    updateTelemetry();
}

private void updateVisionMeasurements() {
    Optional<VisionMeasurement> measurement = visionSubsystem.getBestVisionMeasurement();

    measurement.ifPresent(m -> {
        poseEstimator.addVisionMeasurement(
            m.estimatedPose(),
            m.timestampSeconds(),
            m.standardDeviations()
        );

        // Log vision updates
        SmartDashboard.putString("Vision/LastUpdate",
            String.format("Tags: %d, Dist: %.2fm",
                m.numTagsUsed(), m.averageDistance()));
    });
}
```

### 3.7 Use Cases

#### Use Case 1: Hub Targeting for Shooting
```java
// In RobotContainer or a command
public Command alignToHubCommand() {
    return Commands.run(() -> {
        Optional<Rotation2d> yawOpt = vision.getYawToPose(VisionConstants.HUB_POSE);

        if (yawOpt.isPresent()) {
            double rotationSpeed = yawOpt.get().getRadians() * 2.0; // P controller
            drivetrain.arcadeDrive(0, rotationSpeed);
        }
    }, drivetrain)
    .until(() -> vision.isAlignedWithHub(2.0)) // 2 degree tolerance
    .andThen(() -> drivetrain.stop());
}
```

#### Use Case 2: Human Player Station Alignment
```java
public Command alignToHPStationCommand() {
    return Commands.run(() -> {
        Optional<Rotation2d> yawOpt = vision.getYawToPose(VisionConstants.HP_STATION_POSE);

        if (yawOpt.isPresent()) {
            double rotationSpeed = yawOpt.get().getRadians() * 2.0;
            drivetrain.arcadeDrive(0, rotationSpeed);
        }
    }, drivetrain)
    .until(() -> vision.isAlignedWithHPStation(2.0))
    .andThen(() -> drivetrain.stop());
}
```

#### Use Case 3: Driver Feedback
```java
// In VisionSubsystem periodic()
private void updateTelemetry() {
    // Distance to hub
    Optional<Double> hubDist = getDistanceToHub();
    SmartDashboard.putNumber("Vision/HubDistance",
        hubDist.orElse(-1.0));
    SmartDashboard.putBoolean("Vision/HubAligned",
        isAlignedWithHub(2.0));

    // Human player station status
    SmartDashboard.putBoolean("Vision/HPAligned",
        isAlignedWithHPStation(2.0));

    // Tag visibility
    SmartDashboard.putBoolean("Vision/FrontCamConnected",
        isCameraConnected(frontCamera));
    SmartDashboard.putBoolean("Vision/RearCamConnected",
        isCameraConnected(rearCamera));
}
```

#### Use Case 4: Initial Pose Seeding
```java
// In Robot.java autonomousInit()
@Override
public void autonomousInit() {
    // Try to seed pose from vision if we're near tags
    Optional<Pose2d> visionPose = drivetrain.getVisionSeededPose();

    if (visionPose.isPresent()) {
        drivetrain.resetPose(visionPose.get());
        System.out.println("Seeded initial pose from vision: " + visionPose.get());
    } else {
        // Fall back to known starting position
        drivetrain.resetPose(autoStartPose);
    }

    // Get and schedule autonomous command
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
        autonomousCommand.schedule();
    }
}
```

### 3.8 Troubleshooting Vision

| Issue | Possible Causes | Solutions |
|-------|----------------|-----------|
| No pose estimates | Camera not connected, wrong name | Check PhotonVision dashboard, verify camera names |
| Jumping poses | Bad measurements not filtered | Tighten ambiguity/distance thresholds |
| Odometry drifts despite tags | Standard deviations too high | Lower stddev values to trust vision more |
| Odometry jumps with tags | Standard deviations too low | Raise stddev values to trust vision less |
| Single tag unreliable | Ambiguity too high | Use multi-tag strategy, filter high ambiguity |
| Multi-tag never triggers | Tags too far apart | Adjust camera FoV or robot positioning |

---

## Summary Table: All Required Components

### Declarations

| Component | Type | Purpose |
|-----------|------|---------|
| **leftLeader** | SparkMax | Left drivetrain leader motor (CAN 20) |
| **leftFollower** | SparkMax | Left drivetrain follower motor (CAN 21) |
| **rightLeader** | SparkMax | Right drivetrain leader motor (CAN 22) |
| **rightFollower** | SparkMax | Right drivetrain follower motor (CAN 23) |
| **leftEncoder** | RelativeEncoder | Left wheel position/velocity (from leftLeader) |
| **rightEncoder** | RelativeEncoder | Right wheel position/velocity (from rightLeader) |
| **gyro** | AHRS | Robot heading |
| **differentialDrive** | DifferentialDrive | Basic drive control |
| **kinematics** | DifferentialDriveKinematics | Convert chassis ↔ wheel speeds |
| **poseEstimator** | DifferentialDrivePoseEstimator | Fuse odometry + vision |
| **field** | Field2d | Visualize robot on field |
| **robotConfig** | RobotConfig | PathPlanner robot parameters |
| **ltvController** | PPLTVController | LTV trajectory follower |
| **aprilTagFieldLayout** | AprilTagFieldLayout | Field tag positions |
| **frontCamera** | PhotonCamera | Front camera (Pi4 + PiCam v2) for AprilTags, fuel detection, alignment |
| **rearCamera** | PhotonCamera | Rear camera (Pi5 + OV9281) for AprilTags, shot targeting (rear-facing shooter) |
| **frontEstimator** | PhotonPoseEstimator | Front camera pose solver |
| **rearEstimator** | PhotonPoseEstimator | Rear camera pose solver |

### Methods by Category

#### Basic Drive (Section 1)
- `drive(x, y)`
- `arcadeDrive(speed, rotation)`
- `tankDrive(leftSpeed, rightSpeed)`
- `stop()`
- `setDriveMode(mode)` / `getDriveMode()`
- `setOrientationMode(mode)` / `getOrientationMode()`
- `teleopArcadeCommand()` / `teleopTankCommand()`

#### Odometry & Kinematics (Section 2)
- `getPose()` / `resetPose(pose)`
- `getHeading()` / `resetGyro()`
- `getLeftDistanceMeters()` / `getRightDistanceMeters()`
- `getWheelSpeeds()`
- `getRobotRelativeSpeeds()`
- `driveRobotRelative(speeds)` / `driveFieldRelative(speeds)`

#### PathPlanner (Section 2)
- `configureAutoBuilder()`
- `getAutoCommand(autoName)`

#### Vision (Section 3)
- `getBestVisionMeasurement()`
- `processCameraResult()`
- `shouldUseMeasurement()`
- `calculateStandardDeviations()`
- `getBestTarget()` / `isTagVisible(tagId)`
- `getDistanceToPose()` / `getYawToPose()`
- `isAlignedWithTarget()`
- `getDistanceToHub()` / `isAlignedWithHub()`
- `getDistanceToHPStation()` / `isAlignedWithHPStation()`
- `updateVisionMeasurements()`
- `getVisionSeededPose()`

---

## Implementation Checklist

### Phase 1: Basic Drivetrain
- [ ] Configure all 4 motor controllers (CAN IDs 20-23, inversion, current limits)
- [ ] Set up leader/follower relationships (leftFollower follows leftLeader, rightFollower follows rightLeader)
- [ ] Configure encoder conversion factors on leader motors
- [ ] Implement arcade and tank drive methods
- [ ] Create teleop drive commands
- [ ] Test basic driving on robot with all 4 motors

### Phase 2: Odometry
- [ ] Configure gyro and reset in initialization
- [ ] Create `DifferentialDriveKinematics` with track width
- [ ] Create `DifferentialDrivePoseEstimator`
- [ ] Implement pose/heading getter methods
- [ ] Update pose estimator in `periodic()`
- [ ] Add `Field2d` to SmartDashboard
- [ ] Test odometry accuracy with simple movements

### Phase 3: PathPlanner
- [ ] Verify PathPlanner GUI settings (track width, wheel radius, gear ratio)
- [ ] Implement `getRobotRelativeSpeeds()` method
- [ ] Implement `driveRobotRelative()` method
- [ ] Configure AutoBuilder with LTV controller
- [ ] Test with simple straight-line path
- [ ] Tune LTV parameters (Q and R matrices)
- [ ] Test with complex paths

### Phase 4: Vision
- [ ] Configure PhotonVision on Raspberry Pis (Pi4 front, Pi5 rear)
- [ ] Set up front camera (Pi4 with PiCam v2) for AprilTag and fuel detection
- [ ] Set up rear camera (Pi5 with OV9281) for AprilTag and shot targeting
- [ ] Measure and define camera transforms
- [ ] Create VisionSubsystem
- [ ] Implement camera result processing (fusing both cameras for field position)
- [ ] Implement quality gating
- [ ] Test pose estimates from both cameras
- [ ] Integrate vision into drivetrain pose estimator
- [ ] Tune vision standard deviations
- [ ] Create alignment commands (trench, tower, outpost, hub)
- [ ] Add driver feedback telemetry

### Testing & Tuning
- [ ] Verify unit conversions (encoder counts → meters)
- [ ] Verify gyro sign convention (CW vs CCW)
- [ ] Test field-oriented drive
- [ ] Characterize robot for feedforward
- [ ] Test vision fusion in various lighting
- [ ] Test all autonomous paths
- [ ] Verify alliance mirroring works correctly
- [ ] Document final tuning parameters

---

## Common Pitfalls and Solutions

### 1. Unit Conversion Errors
**Problem**: Encoder values not converting correctly to meters
**Solution**: Double-check conversion factors match physical robot
```java
// Verify: (motor rotations) / (gear ratio) * (wheel circumference)
double measuredDistance = 1.0; // Move robot 1 meter
double encoderReading = leftEncoder.getPosition();
double calculatedFactor = measuredDistance / (encoderReading * WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO);
System.out.println("Calculated factor: " + calculatedFactor);
```

### 2. Gyro Inversion
**Problem**: Robot drives in wrong direction with field-oriented
**Solution**: Invert gyro reading if necessary
```java
public Rotation2d getHeading() {
    // Try both ways and see which matches field convention
    return Rotation2d.fromDegrees(-gyro.getAngle()); // or +gyro.getAngle()
}
```

### 3. PathPlanner Not Following
**Problem**: Robot doesn't follow autonomous paths
**Solution**: Verify all AutoBuilder callbacks are wired correctly
- Check `driveRobotRelative()` actually moves the robot
- Verify `getRobotRelativeSpeeds()` returns correct values
- Ensure `getPose()` is updating each cycle

### 4. Vision Measurements Ignored
**Problem**: Vision not improving odometry
**Solution**: Check standard deviations and quality gating
- Lower stddev = trust vision more
- Check ambiguity thresholds not too strict
- Verify timestamps are in seconds, not milliseconds

### 5. Oscillation During Autonomous
**Problem**: Robot shakes or oscillates along path
**Solution**: Adjust LTV tuning parameters
- Increase R values (reduce control aggressiveness)
- Decrease Q values (tolerate more error)
- Verify max velocity setting matches robot capabilities

---

## References and Resources

### WPILib Documentation
- [Differential Drive Kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html)
- [Differential Drive Odometry](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html)
- [Pose Estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)

### PathPlanner Documentation
- [PathPlanner Wiki](https://pathplanner.dev/home.html)
- [Differential Drive Setup](https://pathplanner.dev/pplib-build-an-auto.html)
- [LTV Controller](https://pathplanner.dev/pplib-follow-a-path.html#lqr-based-controller-unicycle-model)

### PhotonVision Documentation
- [PhotonVision Docs](https://docs.photonvision.org/)
- [AprilTag Pipeline Setup](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/index.html)
- [Pose Estimation](https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html)

### Example Code
- [WPILib Example Projects](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples)
- [PathPlanner Example Projects](https://github.com/mjansen4857/pathplanner/tree/main/examples)
- [PhotonVision Examples](https://github.com/PhotonVision/photonvision/tree/master/photonlib-java-examples)
