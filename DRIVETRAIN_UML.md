# Drivetrain System UML Diagrams

This document provides UML class diagrams and sequence diagrams for the complete drivetrain system.

---

## 1. Class Diagram: Complete System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Robot                                          │
├─────────────────────────────────────────────────────────────────────────────┤
│ - robotContainer: RobotContainer                                            │
├─────────────────────────────────────────────────────────────────────────────┤
│ + robotInit()                                                               │
│ + autonomousInit()                                                          │
│ + teleopInit()                                                              │
│ + robotPeriodic()                                                           │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ creates
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          RobotContainer                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│ - drivetrain: Drivetrain                                                    │
│ - vision: VisionSubsystem                                                   │
│ - driverController: CommandXboxController                                   │
│ - autoChooser: SendableChooser<Command>                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│ + RobotContainer()                                                          │
│ - configureDefaultCommands()                                                │
│ - configureBindings()                                                       │
│ - configureAutos()                                                          │
│ + getAutonomousCommand(): Command                                           │
└─────────────────────────────────────────────────────────────────────────────┘
                    │                                │
                    │ creates                        │ creates
                    ▼                                ▼
    ┌───────────────────────────┐    ┌──────────────────────────────┐
    │   Drivetrain              │    │   VisionSubsystem            │
    │   extends SubsystemBase   │    │   extends SubsystemBase      │
    └───────────────────────────┘    └──────────────────────────────┘
```

---

## 2. Class Diagram: Drivetrain Subsystem

```
┌────────────────────────────────────────────────────────────────────────────────────┐
│                               Drivetrain                                           │
│                          extends SubsystemBase                                     │
├────────────────────────────────────────────────────────────────────────────────────┤
│ ENUMS:                                                                             │
│   + DriveMode { ARCADE, TANK }                                                     │
│   + OrientationMode { ROBOT_ORIENTED, FIELD_ORIENTED }                             │
│                                                                                    │
│ HARDWARE:                                                                          │
│   - leftMotor: SparkMax                                                            │
│   - rightMotor: SparkMax                                                           │
│   - leftEncoder: RelativeEncoder                                                   │
│   - rightEncoder: RelativeEncoder                                                  │
│   - gyro: AHRS                                                                     │
│   - differentialDrive: DifferentialDrive                                           │
│                                                                                    │
│ MATH/KINEMATICS:                                                                   │
│   - kinematics: DifferentialDriveKinematics                                        │
│   - poseEstimator: DifferentialDrivePoseEstimator                                  │
│   - field: Field2d                                                                 │
│                                                                                    │
│ PATHPLANNER:                                                                       │
│   - robotConfig: RobotConfig                                                       │
│   - ltvController: PPLTVController                                                 │
│   - aprilTagFieldLayout: AprilTagFieldLayout                                       │
│                                                                                    │
│ STATE:                                                                             │
│   - driveMode: DriveMode                                                           │
│   - orientationMode: OrientationMode                                               │
│   - visionEnabled: boolean                                                         │
│                                                                                    │
│ DEPENDENCIES:                                                                      │
│   - visionSubsystem: VisionSubsystem                                               │
├────────────────────────────────────────────────────────────────────────────────────┤
│ CONSTRUCTOR:                                                                       │
│   + Drivetrain(visionSubsystem: VisionSubsystem)                                   │
│                                                                                    │
│ BASIC DRIVE METHODS (Section 1):                                                   │
│   + drive(x: double, y: double)                                                    │
│   - arcadeDrive(speed: double, rotation: double)                                   │
│   - tankDrive(leftSpeed: double, rightSpeed: double)                               │
│   + stop()                                                                         │
│   - applyDeadband(value: double, deadband: double): double                         │
│   + setDriveMode(mode: DriveMode)                                                  │
│   + getDriveMode(): DriveMode                                                      │
│   + toggleDriveMode()                                                              │
│   + setOrientationMode(mode: OrientationMode)                                      │
│   + getOrientationMode(): OrientationMode                                          │
│                                                                                    │
│ COMMAND FACTORIES:                                                                 │
│   + teleopArcadeCommand(fwd: DoubleSupplier, rot: DoubleSupplier): Command        │
│   + teleopTankCommand(left: DoubleSupplier, right: DoubleSupplier): Command       │
│   + fieldOrientedArcadeCommand(fwd: DoubleSupplier, rot: DoubleSupplier): Command │
│                                                                                    │
│ ODOMETRY METHODS (Section 2):                                                      │
│   + getPose(): Pose2d                                                              │
│   + resetPose(pose: Pose2d)                                                        │
│   + getHeading(): Rotation2d                                                       │
│   + resetGyro()                                                                    │
│   + getLeftDistanceMeters(): double                                                │
│   + getRightDistanceMeters(): double                                               │
│   + getWheelSpeeds(): DifferentialDriveWheelSpeeds                                 │
│   + getRobotRelativeSpeeds(): ChassisSpeeds                                        │
│   + driveRobotRelative(speeds: ChassisSpeeds)                                      │
│   + driveFieldRelative(speeds: ChassisSpeeds)                                      │
│                                                                                    │
│ PATHPLANNER METHODS (Section 2):                                                   │
│   - configureAutoBuilder()                                                         │
│   + getAutoCommand(autoName: String): Command                                      │
│   - setSpeedsVoltage(speeds: DifferentialDriveWheelSpeeds)                         │
│   - setSpeedsOpenLoop(speeds: DifferentialDriveWheelSpeeds)                        │
│                                                                                    │
│ VISION INTEGRATION (Section 3):                                                    │
│   - updateVisionMeasurements()                                                     │
│   + getVisionSeededPose(): Optional<Pose2d>                                        │
│   + setVisionEnabled(enabled: boolean)                                             │
│                                                                                    │
│ TELEMETRY:                                                                         │
│   - updateTelemetry()                                                              │
│                                                                                    │
│ LIFECYCLE:                                                                         │
│   + periodic()                                                                     │
│   - configureMotors()                                                              │
└────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Class Diagram: VisionSubsystem

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                          VisionSubsystem                                       │
│                       extends SubsystemBase                                    │
├────────────────────────────────────────────────────────────────────────────────┤
│ HARDWARE:                                                                      │
│   - frontCamera: PhotonCamera                                                  │
│   - rearCamera: PhotonCamera                                                   │
│   - fieldLayout: AprilTagFieldLayout                                           │
│                                                                                │
│ POSE ESTIMATORS:                                                               │
│   - frontEstimator: PhotonPoseEstimator                                        │
│   - rearEstimator: PhotonPoseEstimator                                         │
│                                                                                │
│ TRANSFORMS:                                                                    │
│   - robotToFrontCam: Transform3d                                               │
│   - robotToRearCam: Transform3d                                                │
│                                                                                │
│ STATE:                                                                         │
│   - lastValidPose: Optional<Pose2d>                                            │
│   - lastUpdateTime: double                                                     │
├────────────────────────────────────────────────────────────────────────────────┤
│ CONSTRUCTOR:                                                                   │
│   + VisionSubsystem()                                                          │
│                                                                                │
│ CORE VISION PROCESSING:                                                        │
│   + getBestVisionMeasurement(): Optional<VisionMeasurement>                    │
│   - processCameraResult(estimator: PhotonPoseEstimator,                        │
│                         camera: PhotonCamera,                                  │
│                         name: String): Optional<VisionMeasurement>             │
│   - shouldUseMeasurement(estimate: EstimatedRobotPose,                         │
│                          result: PhotonPipelineResult): boolean                │
│   - calculateStandardDeviations(estimate: EstimatedRobotPose,                  │
│                                 avgDistance: double): Matrix<N3, N1>           │
│   - calculateAverageTagDistance(targets: List<PhotonTrackedTarget>,            │
│                                 robotPose: Pose2d): double                     │
│                                                                                │
│ TARGET DETECTION:                                                              │
│   + getBestTarget(): Optional<PhotonTrackedTarget>                             │
│   + isTagVisible(tagId: int): boolean                                          │
│   + getVisibleTags(): List<Integer>                                            │
│                                                                                │
│ ALIGNMENT HELPERS:                                                             │
│   + getDistanceToPose(targetPose: Pose2d): Optional<Double>                    │
│   + getYawToPose(targetPose: Pose2d): Optional<Rotation2d>                     │
│   + isAlignedWithTarget(targetPose: Pose2d, toleranceDeg: double): boolean    │
│   + getDistanceToHub(): Optional<Double>                                       │
│   + isAlignedWithHub(toleranceDeg: double): boolean                            │
│   + getDistanceToHPStation(): Optional<Double>                                 │
│   + isAlignedWithHPStation(toleranceDeg: double): boolean                      │
│                                                                                │
│ TELEMETRY:                                                                     │
│   - updateTelemetry()                                                          │
│   - isCameraConnected(camera: PhotonCamera): boolean                           │
│                                                                                │
│ LIFECYCLE:                                                                     │
│   + periodic()                                                                 │
└────────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────────┐
│                        VisionMeasurement (record)                              │
├────────────────────────────────────────────────────────────────────────────────┤
│   + estimatedPose: Pose2d                                                      │
│   + timestampSeconds: double                                                   │
│   + standardDeviations: Matrix<N3, N1>                                         │
│   + numTagsUsed: int                                                           │
│   + averageDistance: double                                                    │
└────────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Class Diagram: Constants Organization

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Constants                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│ INNER CLASSES:                                                              │
│   + DrivetrainConstants { }                                                 │
│   + AutoConstants { }                                                       │
│   + VisionConstants { }                                                     │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                ┌───────────────────┼───────────────────┐
                │                   │                   │
                ▼                   ▼                   ▼
┌───────────────────────┐ ┌───────────────────┐ ┌────────────────────────┐
│ DrivetrainConstants   │ │  AutoConstants    │ │  VisionConstants       │
├───────────────────────┤ ├───────────────────┤ ├────────────────────────┤
│ CAN IDs:              │ │ LTV Parameters:   │ │ Camera Names:          │
│ + LEFT_MOTOR_CAN_ID   │ │ + LTV_Q_X         │ │ + FRONT_CAMERA_NAME    │
│ + RIGHT_MOTOR_CAN_ID  │ │ + LTV_Q_Y         │ │ + REAR_CAMERA_NAME     │
│                       │ │ + LTV_Q_THETA     │ │                        │
│ Motor Config:         │ │ + LTV_R_V         │ │ Transforms:            │
│ + CURRENT_LIMIT       │ │ + LTV_R_OMEGA     │ │ + ROBOT_TO_FRONT_CAM   │
│ + OPEN_LOOP_RAMP      │ │ + LTV_DT          │ │ + ROBOT_TO_REAR_CAM    │
│ + CLOSED_LOOP_RAMP    │ │ + MAX_VELOCITY    │ │                        │
│ + LEFT_INVERTED       │ │                   │ │ Quality Gating:        │
│ + RIGHT_INVERTED      │ │ Conversions:      │ │ + MAX_TAG_DISTANCE     │
│                       │ │ + WHEEL_DIAMETER  │ │ + MAX_AMBIGUITY        │
│ Driving:              │ │ + WHEEL_CIRCUM    │ │ + MIN_TAGS_MULTI_TAG   │
│ + JOYSTICK_DEADBAND   │ │ + DRIVE_GEAR_RATIO│ │                        │
│                       │ │ + POSITION_FACTOR │ │ Std Deviations:        │
│                       │ │ + VELOCITY_FACTOR │ │ + SINGLE_TAG_CLOSE     │
│                       │ │                   │ │ + SINGLE_TAG_FAR       │
│                       │ │ Kinematics:       │ │ + MULTI_TAG_STDDEVS    │
│                       │ │ + TRACK_WIDTH     │ │                        │
│                       │ │                   │ │ Field Poses:           │
│                       │ │ PathPlanner:      │ │ + HUB_POSE             │
│                       │ │ + MAX_MODULE_SPEED│ │ + HP_STATION_POSE      │
│                       │ │ + MAX_ACCEL       │ │                        │
│                       │ │ + MAX_ANG_VEL     │ │                        │
│                       │ │ + MAX_ANG_ACCEL   │ │                        │
└───────────────────────┘ └───────────────────┘ └────────────────────────┘
```

---

## 5. Sequence Diagram: Teleop Tank Drive

```
Driver      Controller    RobotContainer    Drivetrain       Motors
  │              │               │              │              │
  │ Push sticks  │               │              │              │
  │─────────────>│               │              │              │
  │              │               │              │              │
  │              │  [20ms tick]  │              │              │
  │              │──────────────>│              │              │
  │              │               │              │              │
  │              │               │ periodic()   │              │
  │              │               │─────────────>│              │
  │              │               │              │              │
  │              │               │ [default cmd]│              │
  │              │<─ getLeftY() ─┤ executes     │              │
  │              │               │              │              │
  │              │               │              │              │
  │              │<─ getRightY()─┤              │              │
  │              │               │              │              │
  │              │               │              │              │
  │              │               │ tankDrive()  │              │
  │              │               │─────────────>│              │
  │              │               │              │              │
  │              │               │              │ set(-0.5)    │
  │              │               │              │─────────────>│
  │              │               │              │              │
  │              │               │              │ set(-0.5)    │
  │              │               │              │─────────────>│
  │              │               │              │              │
  │              │               │              │ [Robot moves]│
  │              │               │              │              │
  │              │               │  update      │              │
  │              │               │  odometry    │              │
  │              │               │  (encoders,  │              │
  │              │               │   gyro)      │              │
  │              │               │              │              │
```

---

## 6. Sequence Diagram: Autonomous Path Following with LTV

```
Robot      Auto       Drivetrain    LTV          Pose        Motors
Init       Builder                  Controller   Estimator
  │           │           │             │            │           │
  │ getAuto() │           │             │            │           │
  │──────────>│           │             │            │           │
  │           │           │             │            │           │
  │           │ buildAuto()            │            │           │
  │           │──────────>│             │            │           │
  │           │           │             │            │           │
  │           │<─ Command ┤             │            │           │
  │           │           │             │            │           │
  │<─ Command─┤           │             │            │           │
  │           │           │             │            │           │
  │ schedule()│           │             │            │           │
  │───────────────────────────────────>│            │           │
  │           │           │             │            │           │
  │           │  [every 20ms]          │            │           │
  │           │           │             │            │           │
  │           │  execute()│             │            │           │
  │           │──────────>│             │            │           │
  │           │           │             │            │           │
  │           │           │ getPose()   │            │           │
  │           │           │─────────────────────────>│           │
  │           │           │             │            │           │
  │           │           │<─ current pose ──────────┤           │
  │           │           │             │            │           │
  │           │           │ calculate() │            │           │
  │           │           │────────────>│            │           │
  │           │           │             │            │           │
  │           │           │ [Linearize  │            │           │
  │           │           │  dynamics,  │            │           │
  │           │           │  solve LQR, │            │           │
  │           │           │  compute    │            │           │
  │           │           │  control]   │            │           │
  │           │           │             │            │           │
  │           │           │<─ chassis speeds ────────┤           │
  │           │           │             │            │           │
  │           │           │ driveRobot  │            │           │
  │           │           │ Relative()  │            │           │
  │           │           │─────────────────────────────────────>│
  │           │           │             │            │           │
  │           │           │             │            │  [Robot   │
  │           │           │             │            │   follows │
  │           │           │             │            │   path]   │
  │           │           │             │            │           │
  │           │  periodic()            │            │           │
  │           │──────────>│             │            │           │
  │           │           │             │            │           │
  │           │           │ update pose │            │           │
  │           │           │─────────────────────────>│           │
  │           │           │ (encoders,  │            │           │
  │           │           │  gyro)      │            │           │
  │           │           │             │            │           │
```

---

## 7. Sequence Diagram: Vision Fusion

```
Drivetrain   Vision      PhotonCam   Pose       Field      Pose
             Subsystem               Estimator  Layout     Estimator
  │              │           │           │          │          │
  │ periodic()   │           │           │          │          │
  │──────────────────────────────────────────────────────────>│
  │              │           │           │          │          │
  │ update(gyro, │           │           │          │          │
  │ encoders)    │           │          │          │          │
  │─────────────────────────────────────────────────────────>│
  │              │           │           │          │          │
  │ getBest      │           │           │          │          │
  │ Vision       │           │           │          │          │
  │ Measurement()│           │           │          │          │
  │─────────────>│           │           │          │          │
  │              │           │           │          │          │
  │              │ getLatest │           │          │          │
  │              │ Result()  │           │          │          │
  │              │──────────>│           │          │          │
  │              │           │           │          │          │
  │              │<─ targets─┤           │          │          │
  │              │           │           │          │          │
  │              │ [process  │           │          │          │
  │              │  camera   │           │          │          │
  │              │  result]  │           │          │          │
  │              │           │           │          │          │
  │              │           │ update()  │          │          │
  │              │           │──────────>│          │          │
  │              │           │           │          │          │
  │              │           │           │ getTag   │          │
  │              │           │           │ Pose()   │          │
  │              │           │           │─────────>│          │
  │              │           │           │          │          │
  │              │           │           │<─ 3D pose──────────┤
  │              │           │           │          │          │
  │              │           │<─ estimated pose ────┤          │
  │              │           │           │          │          │
  │              │ [quality  │           │          │          │
  │              │  gating:  │           │          │          │
  │              │  ambiguity│           │          │          │
  │              │  distance]│           │          │          │
  │              │           │           │          │          │
  │              │ [calculate│           │          │          │
  │              │  std devs]│           │          │          │
  │              │           │           │          │          │
  │<─ Vision     │           │           │          │          │
  │   Measurement┤           │           │          │          │
  │              │           │           │          │          │
  │ addVision    │           │           │          │          │
  │ Measurement()│           │           │          │          │
  │─────────────────────────────────────────────────────────>│
  │              │           │           │          │          │
  │              │           │           │          │  [Fuse   │
  │              │           │           │          │   with   │
  │              │           │           │          │   Kalman │
  │              │           │           │          │   filter]│
  │              │           │           │          │          │
```

---

## 8. Sequence Diagram: Target Alignment Command

```
Driver    Button    Command         Drivetrain    Vision       Motors
  │          │         │                │             │            │
  │ Press A  │         │                │             │            │
  │─────────>│         │                │             │            │
  │          │         │                │             │            │
  │          │ onTrue()│                │             │            │
  │          │────────>│                │             │            │
  │          │         │                │             │            │
  │          │         │ initialize()   │             │            │
  │          │         │────────────────>             │            │
  │          │         │                │             │            │
  │          │  [every 20ms while active]            │            │
  │          │         │                │             │            │
  │          │         │ execute()      │             │            │
  │          │         │────────────────>             │            │
  │          │         │                │             │            │
  │          │         │                │ getYawTo    │            │
  │          │         │                │ Pose()      │            │
  │          │         │                │────────────>│            │
  │          │         │                │             │            │
  │          │         │                │<─ yaw error─┤            │
  │          │         │                │             │            │
  │          │         │                │ [P control] │            │
  │          │         │                │ rotation =  │            │
  │          │         │                │ error * kP  │            │
  │          │         │                │             │            │
  │          │         │                │ arcade      │            │
  │          │         │                │ Drive(0,rot)│            │
  │          │         │                │────────────────────────>│
  │          │         │                │             │            │
  │          │         │                │             │  [Robot    │
  │          │         │                │             │   rotates] │
  │          │         │                │             │            │
  │          │         │ isFinished()   │             │            │
  │          │         │────────────────>             │            │
  │          │         │                │             │            │
  │          │         │                │ isAligned   │            │
  │          │         │                │ WithHub()   │            │
  │          │         │                │────────────>│            │
  │          │         │                │             │            │
  │          │         │                │<─ true ─────┤            │
  │          │         │                │             │            │
  │          │         │<─ true ────────┤             │            │
  │          │         │                │             │            │
  │          │         │ end()          │             │            │
  │          │         │────────────────>             │            │
  │          │         │                │             │            │
  │          │         │                │ stop()      │            │
  │          │         │                │────────────────────────>│
  │          │         │                │             │            │
```

---

## 9. State Machine Diagram: Drive Modes

```
                    ┌──────────────────────────┐
                    │  Robot Initialized       │
                    └────────────┬─────────────┘
                                 │
                                 │ setDefaultCommand()
                                 │
                                 ▼
                    ┌──────────────────────────┐
            ┌──────▶│   ARCADE Mode            │◀─────┐
            │       │   ROBOT_ORIENTED         │      │
            │       └────────────┬─────────────┘      │
            │                    │                    │
            │                    │ toggleMode()       │
            │                    │                    │
            │                    ▼                    │
            │       ┌──────────────────────────┐      │
            │       │   TANK Mode              │      │
            │       │   ROBOT_ORIENTED         │      │
            │       └────────────┬─────────────┘      │
            │                    │                    │
toggleMode()│                    │ toggleMode()       │ toggleOrientation()
            │                    │                    │
            │                    ▼                    │
            │       ┌──────────────────────────┐      │
            │       │   ARCADE Mode            │      │
            └───────┤   FIELD_ORIENTED         │──────┘
                    └────────────┬─────────────┘
                                 │
                                 │ toggleMode()
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │   TANK Mode              │
                    │   FIELD_ORIENTED         │
                    └──────────────────────────┘

States:
• ARCADE + ROBOT_ORIENTED: Single stick forward/turn, relative to robot
• TANK + ROBOT_ORIENTED: Independent left/right sticks, relative to robot
• ARCADE + FIELD_ORIENTED: Forward always goes "upfield", turn is robot-relative
• TANK + FIELD_ORIENTED: Left/right adjusted by gyro heading
```

---

## 10. Component Diagram: System Dependencies

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Hardware Layer                                  │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐ │
│  │ SparkMax │  │ SparkMax │  │  NavX    │  │  RPi +   │  │  RPi +   │ │
│  │  Left    │  │  Right   │  │  Gyro    │  │ PhotonV  │  │ PhotonV  │ │
│  │  Motor   │  │  Motor   │  │          │  │  (Front) │  │  (Rear)  │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘ │
│       │             │             │             │             │       │
└───────┼─────────────┼─────────────┼─────────────┼─────────────┼───────┘
        │             │             │             │             │
        │             │             │             │             │
┌───────┼─────────────┼─────────────┼─────────────┼─────────────┼───────┐
│       │             │             │             │             │       │
│       ▼             ▼             ▼             ▼             ▼       │
│  ┌────────────┐  ┌────────────┐  ┌────────────────────────────┐      │
│  │ Drivetrain │  │ Drivetrain │  │   VisionSubsystem          │      │
│  │  Motors    │  │  Sensors   │  │                            │      │
│  └─────┬──────┘  └─────┬──────┘  └──────────┬─────────────────┘      │
│        │               │                    │                        │
│        │               │                    │                        │
│        │       ┌───────┴───────────────┬────┘                        │
│        │       │                       │                             │
│        ▼       ▼                       ▼                             │
│  ┌──────────────────────────────────────────────────────┐            │
│  │           Drivetrain Subsystem                       │            │
│  │  ┌──────────────┐  ┌──────────────┐  ┌────────────┐ │            │
│  │  │ Differential │  │ Differential │  │   Pose     │ │            │
│  │  │    Drive     │  │    Drive     │  │ Estimator  │ │            │
│  │  │              │  │  Kinematics  │  │ (Odometry  │ │            │
│  │  │              │  │              │  │ + Vision)  │ │            │
│  │  └──────────────┘  └──────────────┘  └────────────┘ │            │
│  └─────────────────────────┬────────────────────────────┘            │
│                            │                                         │
│                     Application Layer                                │
└────────────────────────────┼─────────────────────────────────────────┘
                             │
                             │
┌────────────────────────────┼─────────────────────────────────────────┐
│                            │                                         │
│                            ▼                                         │
│                  ┌──────────────────┐                                │
│                  │  RobotContainer  │                                │
│                  └────────┬─────────┘                                │
│                           │                                          │
│           ┌───────────────┼────────────────┐                         │
│           │               │                │                         │
│           ▼               ▼                ▼                         │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────┐                 │
│  │  Commands   │  │ PathPlanner  │  │  Dashboard  │                 │
│  │  (Teleop &  │  │  AutoBuilder │  │  Telemetry  │                 │
│  │   Vision)   │  │              │  │             │                 │
│  └─────────────┘  └──────────────┘  └─────────────┘                 │
│                                                                      │
│                     Command Layer                                    │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 11. Data Flow Diagram: Pose Estimation Pipeline

```
                          ┌──────────────────┐
                          │   Encoders       │
                          │  (Left, Right)   │
                          └────────┬─────────┘
                                   │
                                   │ Position (meters)
                                   │ Velocity (m/s)
                                   │
                                   ▼
                          ┌──────────────────┐
                          │   Gyro (AHRS)    │
                          │                  │
                          └────────┬─────────┘
                                   │
                                   │ Heading (Rotation2d)
                                   │
                                   ▼
┌─────────────────┐      ┌──────────────────────────┐
│  Vision         │─────▶│ DifferentialDrivePose   │
│  Measurements   │      │      Estimator           │
│ (w/ timestamps  │      │  (Extended Kalman Filter)│
│  & std devs)    │      └──────────┬───────────────┘
└─────────────────┘                 │
                                    │ Robot Pose (Pose2d)
                                    │
                                    ▼
                          ┌──────────────────┐
                          │   Field2d        │
                          │  (Visualization) │
                          └──────────────────┘
                                    │
                                    ▼
                          ┌──────────────────┐
                          │  SmartDashboard  │
                          │  Driver Station  │
                          └──────────────────┘

Data Sources:
• Encoders: Continuous, 50 Hz, dead reckoning
• Gyro: Continuous, 50 Hz, heading only
• Vision: Intermittent, when tags visible, full pose

Kalman Filter weights each source by:
• Encoder: Medium trust (drifts over time)
• Gyro: High trust for heading
• Vision: Variable trust based on standard deviations
```

---

## 12. Deployment Diagram: Physical Hardware Layout

```
                        ┌─────────────────────────────┐
                        │      RoboRIO 2.0            │
                        │  (Main Robot Controller)    │
                        │                             │
                        │  • Java Code                │
                        │  • WPILib                   │
                        │  • Command Scheduler        │
                        └──────────┬──────────────────┘
                                   │
                        CAN Bus    │    NetworkTables
           ┌───────────────────────┼─────────────────────────┐
           │                       │                         │
           ▼                       ▼                         ▼
┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  SparkMax CAN:0  │    │  NavX (MXP SPI)  │    │ Raspberry Pi #1  │
│  NEO Motor       │    │  Gyro/IMU        │    │ (Front Camera)   │
│  (Left Drive)    │    │                  │    │                  │
└──────────────────┘    └──────────────────┘    │ • PhotonVision   │
                                                 │ • AprilTag Detect│
                                                 │ • Coprocessor    │
┌──────────────────┐                             └──────────────────┘
│  SparkMax CAN:1  │
│  NEO Motor       │
│  (Right Drive)   │                             ┌──────────────────┐
└──────────────────┘                             │ Raspberry Pi #2  │
                                                 │ (Rear Camera)    │
                                                 │                  │
┌──────────────────┐                             │ • PhotonVision   │
│ Driver Station   │                             │ • AprilTag Detect│
│                  │                             │ • Coprocessor    │
│ • Xbox Controller│◀─────── USB ───────────────▶└──────────────────┘
│ • SmartDashboard │
│ • Shuffleboard   │
└──────────────────┘

Network: 10.TE.AM.XX (Team-specific IP subnet)
• RoboRIO: 10.TE.AM.2
• Raspberry Pi #1: 10.TE.AM.11
• Raspberry Pi #2: 10.TE.AM.12
• Driver Station: 10.TE.AM.5

Protocols:
• CAN: Motor control & sensor data
• SPI: High-speed gyro data
• NetworkTables: Vision pose estimates
• USB: Driver input
```

---

## Summary

These UML diagrams illustrate:

1. **Class Structure**: Complete hierarchy of subsystems and their relationships
2. **Method Organization**: Grouped by functionality (basic drive, odometry, PathPlanner, vision)
3. **Data Flow**: How sensor data flows through odometry and vision fusion
4. **Sequence of Operations**: Step-by-step execution during teleop and autonomous
5. **State Management**: Drive mode and orientation mode transitions
6. **System Architecture**: Layered design separating hardware, subsystems, and commands
7. **Physical Deployment**: Hardware connections and network topology

Use these diagrams as reference when implementing the system described in `DRIVETRAIN_ARCHITECTURE.md`.
