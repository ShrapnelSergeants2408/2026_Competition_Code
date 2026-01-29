# Vision System Implementation Plan
## Multi-Camera AprilTag Detection and Driver Feedback

---

## Executive Summary

This document outlines the complete implementation plan for the vision subsystem using a multi-camera setup for AprilTag detection and driver feedback. The system uses Raspberry Pi coprocessors running PhotonVision for AprilTag detection (front and rear cameras) and a USB webcam for driver video feed. The implementation includes pose estimation, odometry fusion, and comprehensive telemetry.

---

## Hardware Configuration

### Camera Systems

| Camera | Coprocessor | Purpose | Location |
|--------|-------------|---------|----------|
| Inno-maker OV7251 (mipiov7251-trigger) | Raspberry Pi 5 | AprilTag Detection | Rear |
| Pi Camera v2 | Raspberry Pi 4 | AprilTag Detection + Ball Detection | Front |
| Generic USB Webcam | roboRIO or Driver Station | Driver Feedback | TBD (Front or Back) |

### Raspberry Pi 5 Specs (Rear Camera)
- **Camera**: Inno-maker OV7251 (global shutter, 640x480 @ 90fps capable)
- **Interface**: MIPI CSI
- **Processing**: PhotonVision for AprilTag detection
- **Network**: Ethernet to robot radio (static IP)
- **Advantages**: Global shutter = no motion blur, fast frame rate

### Raspberry Pi 4 Specs (Front Camera)
- **Camera**: Pi Camera v2 (rolling shutter, 1080p capable)
- **Interface**: MIPI CSI
- **Processing**: PhotonVision for AprilTag + optional ball detection
- **Network**: Ethernet to robot radio (static IP)
- **Advantages**: Higher resolution for longer range detection

### USB Webcam Specs (Driver Feedback)
- **Connection**: USB to roboRIO or direct to Driver Station
- **Processing**: CameraServer (WPILib) or direct DS feed
- **Purpose**: Driver situational awareness
- **Resolution**: 320x240 recommended for bandwidth

---

## Network Configuration

### IP Address Scheme

| Device | IP Address | Hostname |
|--------|------------|----------|
| roboRIO | 10.TE.AM.2 | roborio-TEAM-frc.local |
| Raspberry Pi 5 (Rear) | 10.TE.AM.11 | photonvision-rear.local |
| Raspberry Pi 4 (Front) | 10.TE.AM.12 | photonvision-front.local |
| Radio | 10.TE.AM.1 | - |

### Port Usage

| Service | Port |
|---------|------|
| PhotonVision Web UI | 5800 |
| PhotonVision NT | 5800-5810 |
| CameraServer (driver cam) | 1181-1190 |

---

## Current Code Analysis

### What Needs to Be Created

1. **Vision Subsystem**: New subsystem class for PhotonVision integration
2. **VisionConstants**: Camera positions, pipeline settings
3. **Pose Estimation**: Multi-camera pose fusion
4. **Driver Camera**: CameraServer setup
5. **Odometry Fusion**: Vision pose correction for drivetrain

### Dependencies Required

- PhotonLib (vendor dependency)
- WPILib CameraServer
- Optional: AdvantageKit for logging

---

## Requirements Summary

1. **AprilTag Detection**: Detect AprilTags from both front and rear cameras
2. **Pose Estimation**: Estimate robot pose on field from AprilTag observations
3. **Multi-Camera Fusion**: Combine estimates from both cameras
4. **Distance Measurement**: Calculate distance to AprilTags for shooter
5. **Driver Feedback**: Low-latency video stream for driver
6. **Ball Detection**: Optional ball tracking via front camera
7. **Telemetry**: Display all vision data on dashboard
8. **Robustness**: Graceful handling of camera failures

---

## Detailed Implementation Plan

---

## Phase 1: PhotonVision Setup (Off-Robot)

### 1.1 Raspberry Pi Setup

**For Each Pi**:
1. Flash PhotonVision image to SD card
2. Configure static IP address
3. Connect camera
4. Access web UI at http://IP:5800
5. Configure camera settings (exposure, resolution)
6. Calibrate camera for AprilTag detection
7. Set up AprilTag pipeline

### 1.2 Camera Calibration

**Required for Accurate Pose Estimation**:
- Use PhotonVision's built-in calibration tool
- Print calibration checkerboard pattern
- Take 15-20 images from different angles
- Verify reprojection error < 0.5 pixels
- Save calibration to PhotonVision

### 1.3 Pipeline Configuration

**AprilTag Pipeline Settings**:
- Family: 36h11 (2024+ FRC standard)
- Decimate: 2 (balance speed vs accuracy)
- Threads: 4
- Refine edges: On
- Multi-tag: Enabled (for better pose estimation)

---

## Phase 2: Constants and Configuration

### 2.1 Add VisionConstants Class

**File**: `src/main/java/frc/robot/Constants.java`

```java
public static class VisionConstants {
    // Camera names (must match PhotonVision config)
    public static final String FRONT_CAMERA_NAME = "front-camera";
    public static final String REAR_CAMERA_NAME = "rear-camera";
    public static final String DRIVER_CAMERA_NAME = "driver-cam";

    // Driver camera settings
    public static final int DRIVER_CAMERA_PORT = 0;  // USB port
    public static final int DRIVER_CAMERA_WIDTH = 320;
    public static final int DRIVER_CAMERA_HEIGHT = 240;
    public static final int DRIVER_CAMERA_FPS = 15;

    // Camera positions relative to robot center (meters)
    // IMPORTANT: Measure these carefully on your robot!
    // X = forward, Y = left, Z = up, rotation = yaw

    // Front camera (Pi Camera v2 on Pi 4)
    public static final double FRONT_CAMERA_X = 0.25;      // 25cm forward of center
    public static final double FRONT_CAMERA_Y = 0.0;       // Centered
    public static final double FRONT_CAMERA_Z = 0.30;      // 30cm above ground
    public static final double FRONT_CAMERA_PITCH = -15.0; // Tilted down 15 degrees
    public static final double FRONT_CAMERA_YAW = 0.0;     // Facing forward

    // Rear camera (OV7251 on Pi 5)
    public static final double REAR_CAMERA_X = -0.20;      // 20cm behind center
    public static final double REAR_CAMERA_Y = 0.0;        // Centered
    public static final double REAR_CAMERA_Z = 0.35;       // 35cm above ground
    public static final double REAR_CAMERA_PITCH = -10.0;  // Tilted down 10 degrees
    public static final double REAR_CAMERA_YAW = 180.0;    // Facing backward

    // Pose estimation settings
    public static final double MAX_POSE_AMBIGUITY = 0.2;   // Reject high ambiguity
    public static final double MAX_TAG_DISTANCE = 5.0;     // Meters - ignore far tags
    public static final double VISION_MEASUREMENT_STD_DEV_X = 0.5;  // Trust odometry more
    public static final double VISION_MEASUREMENT_STD_DEV_Y = 0.5;
    public static final double VISION_MEASUREMENT_STD_DEV_THETA = 0.9; // Radians

    // Standard deviations scale with distance
    public static final double STD_DEV_DISTANCE_FACTOR = 0.3;  // STD scales with distance

    // Field layout
    public static final String APRILTAG_FIELD_LAYOUT = "2026-crescendo.json";  // Update for actual game
}
```

---

## Phase 3: Vision Subsystem Implementation

### 3.1 Create Vision Subsystem

**File**: `src/main/java/frc/robot/subsystems/Vision.java`

**Components to Instantiate**:
- `PhotonCamera frontCamera` - Front AprilTag camera
- `PhotonCamera rearCamera` - Rear AprilTag camera
- `PhotonPoseEstimator frontPoseEstimator` - Front pose calculator
- `PhotonPoseEstimator rearPoseEstimator` - Rear pose calculator
- `AprilTagFieldLayout fieldLayout` - 2026 field tag positions
- `UsbCamera driverCamera` - Driver feedback camera
- `Transform3d frontCameraToRobot` - Front camera transform
- `Transform3d rearCameraToRobot` - Rear camera transform

**Methods Needed**:

| Method | Parameters | Return Type | Description |
|--------|------------|-------------|-------------|
| `Vision()` | constructor | | Initialize cameras and pose estimators |
| `getFrontCameraResult()` | none | `PhotonPipelineResult` | Latest front camera data |
| `getRearCameraResult()` | none | `PhotonPipelineResult` | Latest rear camera data |
| `hasAprilTagTargets()` | none | `boolean` | True if any camera sees tags |
| `getEstimatedGlobalPose()` | none | `Optional<EstimatedRobotPose>` | Best pose estimate |
| `getFrontPoseEstimate()` | none | `Optional<EstimatedRobotPose>` | Front camera pose |
| `getRearPoseEstimate()` | none | `Optional<EstimatedRobotPose>` | Rear camera pose |
| `getDistanceToTag()` | `int tagID` | `Optional<Double>` | Distance to specific tag |
| `getAngleToTag()` | `int tagID` | `Optional<Double>` | Angle to specific tag |
| `getVisibleTagIDs()` | none | `List<Integer>` | All currently visible tag IDs |
| `getLatencySeconds()` | none | `double` | Pipeline latency |
| `setDriverCameraExposure()` | `int exposure` | `void` | Adjust driver cam |
| `periodic()` | none | `void` | Update telemetry |

---

## Phase 4: Pose Estimation and Fusion

### 4.1 Multi-Camera Pose Fusion Strategy

**Fusion Approach**:
1. Get pose estimates from both cameras
2. Filter based on ambiguity and distance
3. Weight by number of tags seen and distance
4. Return weighted average or best estimate

**Selection Criteria**:
- Reject if ambiguity > MAX_POSE_AMBIGUITY
- Reject if average tag distance > MAX_TAG_DISTANCE
- Prefer estimate with more tags visible
- Prefer lower ambiguity when tag count equal

### 4.2 Standard Deviation Scaling

**Dynamic STD Based on Distance**:
```java
public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimate) {
    double avgDistance = getAverageTagDistance(estimate);
    double xyStdDev = VISION_MEASUREMENT_STD_DEV_X * (1 + avgDistance * STD_DEV_DISTANCE_FACTOR);
    double thetaStdDev = VISION_MEASUREMENT_STD_DEV_THETA * (1 + avgDistance * STD_DEV_DISTANCE_FACTOR);

    // Reduce stddev if multiple tags seen
    int tagCount = estimate.targetsUsed.size();
    if (tagCount > 1) {
        xyStdDev /= tagCount;
        thetaStdDev /= tagCount;
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
}
```

---

## Phase 5: Drivetrain Integration

### 5.1 Odometry Fusion

**In DriveTrain subsystem**, add method to accept vision measurements:

```java
public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
}
```

**In RobotContainer or periodic command**, update drivetrain with vision:

```java
m_vision.getEstimatedGlobalPose().ifPresent(estimate -> {
    m_drivetrain.addVisionMeasurement(
        estimate.estimatedPose.toPose2d(),
        estimate.timestampSeconds,
        m_vision.getEstimationStdDevs(estimate)
    );
});
```

---

## Phase 6: Driver Feedback Camera

### 6.1 CameraServer Setup

**In Vision constructor**:
```java
try {
    driverCamera = CameraServer.startAutomaticCapture(DRIVER_CAMERA_NAME, DRIVER_CAMERA_PORT);
    driverCamera.setResolution(DRIVER_CAMERA_WIDTH, DRIVER_CAMERA_HEIGHT);
    driverCamera.setFPS(DRIVER_CAMERA_FPS);
    driverCamera.setExposureManual(50);  // Adjust for lighting
} catch (Exception e) {
    DriverStation.reportWarning("Driver camera not found", false);
}
```

**Bandwidth Considerations**:
- Keep resolution low (320x240 or 160x120)
- Keep FPS low (10-15)
- MJPEG compression helps
- Consider using only in teleop

---

## Phase 7: Ball Detection (Optional)

### 7.1 Color-Based Ball Detection

**Create Separate Pipeline in PhotonVision**:
- Pipeline 1: AprilTags
- Pipeline 2: Ball/Game piece detection

**Ball Detection Methods**:

| Method | Parameters | Return Type | Description |
|--------|------------|-------------|-------------|
| `setBallDetectionPipeline()` | none | `void` | Switch front camera to ball pipeline |
| `setAprilTagPipeline()` | none | `void` | Switch front camera to AprilTag pipeline |
| `hasBallTarget()` | none | `boolean` | True if ball visible |
| `getBallYaw()` | none | `double` | Horizontal angle to ball |
| `getBallArea()` | none | `double` | Ball size (proxy for distance) |

---

## Incremental Implementation Plan (For Beginning Programmers)

**Philosophy**: Vision is complex. Build and test ONE camera at a time. Don't try to do everything at once!

---

### Prerequisites (Do Once at Start)
- [ ] Flash PhotonVision to both Raspberry Pis
- [ ] Set static IP addresses (10.TE.AM.11 and 10.TE.AM.12)
- [ ] Connect cameras and verify video in PhotonVision web UI
- [ ] Install PhotonLib vendor dependency in VS Code
- [ ] Calibrate cameras using PhotonVision calibration wizard

---

### Milestone 1: Single Camera AprilTag Detection (No Pose)
**Goal**: Detect AprilTags and display what we see. No pose estimation yet.

**What You'll Learn**: PhotonVision basics, NetworkTables, camera data

**Tasks**:
- [ ] Add PhotonLib vendor dependency
- [ ] Add VisionConstants with FRONT_CAMERA_NAME
- [ ] Create Vision.java subsystem:
  - Import `org.photonvision.PhotonCamera`
  - Create `PhotonCamera frontCamera`
  - Add method `hasAprilTagTargets()`:
    - `return frontCamera.getLatestResult().hasTargets()`
  - Add method `getVisibleTagIDs()`:
    - Return list of target fiducial IDs
- [ ] In periodic(), display to SmartDashboard:
  - Has targets (boolean)
  - Number of targets
  - List of visible tag IDs
- [ ] Instantiate Vision in RobotContainer

**Test**: Point camera at AprilTag, see detection on dashboard.

**Expected Time**: 1 hour

---

### Milestone 2: Basic Pose Estimation (Single Camera)
**Goal**: Get robot pose estimate from AprilTag observations.

**What You'll Learn**: Pose estimation, coordinate transforms, field layout

**Tasks**:
- [ ] Download/create 2026 AprilTag field layout JSON
- [ ] Add camera position constants (measure your robot!):
  - FRONT_CAMERA_X, Y, Z
  - FRONT_CAMERA_PITCH, YAW
- [ ] Create Transform3d for camera position:
  - `frontCameraToRobot = new Transform3d(...)`
- [ ] Create PhotonPoseEstimator:
  - Load AprilTagFieldLayout
  - Set strategy to MULTI_TAG_PNP_ON_COPROCESSOR (preferred)
  - Fallback to LOWEST_AMBIGUITY
- [ ] Add method `getFrontPoseEstimate()`:
  - Call `poseEstimator.update(frontCamera.getLatestResult())`
  - Return Optional<EstimatedRobotPose>
- [ ] Display estimated pose (x, y, rotation) on dashboard

**Test**: Point at known AprilTag, verify pose estimate is reasonable.

**Expected Time**: 1.5 hours

---

### Milestone 3: Add Second Camera
**Goal**: Add rear camera and get pose estimates from both.

**What You'll Learn**: Multi-camera systems, camera transforms

**Tasks**:
- [ ] Add REAR_CAMERA_NAME to constants
- [ ] Add rear camera position constants
- [ ] Create `PhotonCamera rearCamera`
- [ ] Create Transform3d for rear camera (remember: 180° yaw for backward!)
- [ ] Create second PhotonPoseEstimator for rear camera
- [ ] Add method `getRearPoseEstimate()`
- [ ] Display both pose estimates on dashboard
- [ ] Verify both cameras work independently

**Test**: AprilTags visible from rear, pose estimate works.

**Expected Time**: 1 hour

---

### Milestone 4: Pose Fusion
**Goal**: Combine both camera estimates into single best estimate.

**What You'll Learn**: Sensor fusion, filtering, confidence weighting

**Tasks**:
- [ ] Add method `getEstimatedGlobalPose()`:
  - Get both camera estimates
  - Filter by ambiguity (reject > MAX_POSE_AMBIGUITY)
  - Filter by distance (reject > MAX_TAG_DISTANCE)
  - Select best estimate based on tag count and ambiguity
- [ ] Add method `getEstimationStdDevs()`:
  - Calculate standard deviations based on distance
  - Lower stddev when more tags visible
- [ ] Display fused pose and confidence on dashboard

**Test**: Both cameras see tags, get unified pose estimate.

**Expected Time**: 1 hour

---

### Milestone 5: Drivetrain Integration
**Goal**: Feed vision pose into drivetrain odometry.

**What You'll Learn**: Pose estimator fusion, timestamp handling

**Tasks**:
- [ ] In DriveTrain, switch to SwerveDrivePoseEstimator or DifferentialDrivePoseEstimator
  (instead of plain Odometry)
- [ ] Add method `addVisionMeasurement(pose, timestamp, stdDevs)`
- [ ] In RobotContainer (or default command), periodically call:
```java
m_vision.getEstimatedGlobalPose().ifPresent(estimate -> {
    m_drivetrain.addVisionMeasurement(
        estimate.estimatedPose.toPose2d(),
        estimate.timestampSeconds,
        m_vision.getEstimationStdDevs(estimate)
    );
});
```
- [ ] Add Field2d showing both odometry and vision poses

**Test**: Drive around, watch Field2d - vision should correct drift.

**Expected Time**: 1.5 hours

---

### Milestone 6: Driver Feedback Camera
**Goal**: Add USB webcam for driver video feed.

**What You'll Learn**: CameraServer, bandwidth management

**Tasks**:
- [ ] Connect USB webcam to roboRIO (or directly to DS laptop)
- [ ] Add driver camera constants (port, resolution, FPS)
- [ ] In Vision constructor, initialize CameraServer:
```java
driverCamera = CameraServer.startAutomaticCapture(DRIVER_CAMERA_NAME, DRIVER_CAMERA_PORT);
driverCamera.setResolution(320, 240);
driverCamera.setFPS(15);
```
- [ ] Open Shuffleboard, add CameraServer widget
- [ ] Verify video appears with acceptable latency
- [ ] Adjust resolution/FPS if bandwidth issues

**Test**: Driver can see video feed with < 200ms latency.

**Expected Time**: 45 minutes

---

### Milestone 7: Distance to Target
**Goal**: Calculate distance to specific AprilTags for shooter.

**What You'll Learn**: 3D geometry, target selection

**Tasks**:
- [ ] Add method `getDistanceToTag(int tagID)`:
  - Search results for target with matching fiducial ID
  - Calculate distance from camera to target
  - Transform to distance from robot center
- [ ] Add method `getAngleToTag(int tagID)`:
  - Return horizontal angle (yaw) to tag
- [ ] Add method to find nearest tag of specific type (e.g., speaker, amp)
- [ ] Display distance/angle to relevant game targets

**Test**: Point at tag, distance readout matches tape measure.

**Expected Time**: 1 hour

---

### Milestone 8: Comprehensive Telemetry
**Goal**: Display all vision information for debugging.

**What You'll Learn**: Telemetry design, debugging strategies

**Tasks**:
- [ ] Create detailed telemetry display:
  - Front camera: connected, has targets, tag count, latency
  - Rear camera: connected, has targets, tag count, latency
  - Fused pose: x, y, rotation, confidence
  - Individual tag distances and angles
  - Driver camera: connected, FPS
- [ ] Add Field2d with:
  - Robot pose
  - Visible AprilTag positions
  - Camera field-of-view visualization (optional)
- [ ] Add alerts for camera disconnection

**Test**: All telemetry displays correctly in all scenarios.

**Expected Time**: 1 hour

---

### Milestone 9: Ball Detection (Optional)
**Goal**: Detect game pieces using front camera.

**What You'll Learn**: Color detection, pipeline switching

**Tasks**:
- [ ] In PhotonVision, create second pipeline for ball detection
- [ ] Configure color thresholds for game piece color
- [ ] Add method `setBallDetectionPipeline()`:
  - `frontCamera.setPipelineIndex(1)`
- [ ] Add method `setAprilTagPipeline()`:
  - `frontCamera.setPipelineIndex(0)`
- [ ] Add methods: `hasBallTarget()`, `getBallYaw()`, `getBallArea()`
- [ ] Decide when to use each pipeline (teleop vs auto?)

**Test**: Switch to ball pipeline, detect game pieces reliably.

**Expected Time**: 1 hour

---

### Milestone 10: Robustness and Polish
**Goal**: Handle failures gracefully, finalize for competition.

**What You'll Learn**: Error handling, graceful degradation

**Tasks**:
- [ ] Add camera connection status checking
- [ ] Handle camera disconnection gracefully (don't crash)
- [ ] Add fallback behavior when vision unavailable
- [ ] Test with network issues (disconnect cable, verify recovery)
- [ ] Optimize latency (use latest timestamp handling)
- [ ] Document camera positions and calibration data
- [ ] Backup calibration files!

**Test**: Disconnect camera, robot continues. Reconnect, vision resumes.

**Expected Time**: 1 hour

---

## Total Incremental Timeline

| Milestone | Time Estimate | Cumulative | What Works After This Milestone |
|-----------|---------------|------------|----------------------------------|
| 1. Single Camera Detection | 1 hour | 1 hour | See AprilTags on dashboard |
| 2. Basic Pose Estimation | 1.5 hours | 2.5 hours | Know robot position from tags |
| 3. Second Camera | 1 hour | 3.5 hours | Front and rear detection |
| 4. Pose Fusion | 1 hour | 4.5 hours | Best combined estimate |
| 5. Drivetrain Integration | 1.5 hours | 6 hours | Vision corrects odometry |
| 6. Driver Camera | 0.75 hours | 6.75 hours | Driver video feed |
| 7. Distance to Target | 1 hour | 7.75 hours | Shooter distance info |
| 8. Telemetry | 1 hour | 8.75 hours | Full debugging display |
| 9. Ball Detection | 1 hour | 9.75 hours | Game piece tracking |
| 10. Robustness | 1 hour | 10.75 hours | Competition ready |

**Total**: ~11 hours of focused work

---

## Key Principles for Success

1. **Measure Camera Position Accurately**: Small errors = big pose errors
2. **Calibrate Cameras Well**: Bad calibration = bad pose estimation
3. **Test One Camera at a Time**: Debug completely before adding complexity
4. **Use Static IPs**: Dynamic IPs cause endless problems
5. **Monitor Bandwidth**: Low resolution for driver cam
6. **Commit Often**: Git commit after each working milestone
7. **Backup Calibrations**: Save calibration data off the Pi

---

## Common Beginner Mistakes to Avoid

- Wrong camera transform -> Pose is mirrored or offset
- Camera not calibrated -> Wildly wrong pose estimates
- Wrong AprilTag family -> Doesn't detect any tags
- Dynamic IP addresses -> Can't connect to PhotonVision
- Too high resolution driver cam -> Network lag
- Not checking ambiguity -> Jumping pose estimates
- Camera mounted crooked -> Systematic pose error

---

## Risk Mitigation

### Technical Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Camera disconnects | No pose updates | Graceful degradation, encoder-only fallback |
| Bad calibration | Wrong poses | Careful calibration, verify with tape measure |
| Network bandwidth | Lag/dropped frames | Low resolution driver cam, optimize settings |
| Lighting changes | Detection failures | Auto-exposure, test in various lighting |
| Ambiguous poses | Jumping estimates | Filter by ambiguity, prefer multi-tag |

### Hardware Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Pi overheats | Shuts down | Proper ventilation, heat sinks |
| Camera loose | Wrong poses | Secure mounting, lock-tight |
| Ethernet disconnects | Camera offline | Secure cable routing, strain relief |

---

## Performance Targets

### Acceptance Criteria

- [ ] Pose estimation latency < 100ms
- [ ] Pose accuracy within 10cm at 2m distance
- [ ] Pose accuracy within 20cm at 4m distance
- [ ] Driver camera latency < 200ms
- [ ] Both cameras detect tags simultaneously
- [ ] Graceful degradation when camera offline
- [ ] Vision pose fuses smoothly with odometry
- [ ] No pose jumping during normal operation

---

## Future Enhancements

### Post-Initial Implementation

1. **Machine Learning Object Detection**: Neural network for game pieces
2. **SLAM**: Simultaneous localization and mapping
3. **Motion Blur Compensation**: Better detection while moving
4. **Multi-Robot Coordination**: Share vision data between robots
5. **Replay Logging**: AdvantageKit vision logging
6. **Auto-Calibration**: Automatic extrinsic calibration
7. **LED Ring Lighting**: Consistent AprilTag illumination

---

## Appendix: PhotonVision Installation

### Step-by-Step Pi Setup

1. Download PhotonVision image from photonvision.org
2. Flash to microSD using balenaEtcher
3. Insert SD, connect Ethernet, power on
4. Find Pi on network (look for photonvision.local)
5. Access web UI at http://photonvision.local:5800
6. Configure static IP:
   - Settings -> Networking
   - Set static IP (10.TE.AM.11 or .12)
   - Apply and reboot
7. Connect camera, verify video stream
8. Run calibration wizard
9. Configure AprilTag pipeline

### Camera Calibration Checklist

- [ ] Print checkerboard pattern (actual size!)
- [ ] Good lighting, no glare
- [ ] 15-20 images from different angles
- [ ] Include angled views (not just straight-on)
- [ ] Verify reprojection error < 0.5 pixels
- [ ] Save calibration
- [ ] Export calibration file as backup

---

## Summary

This implementation plan provides a comprehensive roadmap for setting up a multi-camera vision system using PhotonVision on Raspberry Pi coprocessors. The system enables accurate AprilTag-based pose estimation from both front and rear cameras, provides driver feedback video, and integrates with drivetrain odometry for field-relative positioning.

**Estimated Total Implementation Time**: 11 hours (plus off-robot Pi setup time)

**Key Success Factors**:
1. Accurate camera calibration
2. Precise camera position measurements
3. Proper network configuration
4. Systematic testing approach
5. Graceful failure handling

**Critical Path**:
1. PhotonVision setup on Pis
2. Camera calibration
3. Single camera pose estimation
4. Multi-camera fusion
5. Drivetrain integration
6. Competition testing and tuning
