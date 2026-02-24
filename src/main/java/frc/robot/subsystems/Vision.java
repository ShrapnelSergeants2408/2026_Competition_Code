// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.VisionMeasurement;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    // PhotonVision cameras
    private final PhotonCamera frontCamera;
    private final PhotonCamera rearCamera;

    // Pose estimators
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator rearPoseEstimator;

    // AprilTag field layout
    private final AprilTagFieldLayout fieldLayout;

    // Class fields
    private double lastFrontTimestamp = -1.0;
    private double lastRearTimestamp  = -1.0;
    private double lastFrontFpgaTs    = -1.0; // BUG-06: FPGA-time seconds, not wall-clock ms
    private double lastRearFpgaTs     = -1.0;
    private static final double CAMERA_STALE_TIMEOUT_SECONDS = 0.5;
    private int telemetryLoopCounter = 0;

    // Per-loop caches — computed once in periodic(), read by all callers this cycle.
    // INEFF-01/04/07: prevents cameras from being processed more than once per 20 ms loop.
    private Optional<VisionMeasurement> cachedMeasurement = Optional.empty();
    // INEFF-03: boolean array indexed by tag ID (1–30). O(1) lookup with no boxing.
    private boolean[] cachedTagsVisible = new boolean[31];
    private Optional<PhotonPipelineResult> cachedFrontResult = Optional.empty();
    private Optional<PhotonPipelineResult> cachedRearResult = Optional.empty();

    public Vision() {



        // Load AprilTag field layout
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Initialize cameras
        frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
        rearCamera = new PhotonCamera(VisionConstants.REAR_CAMERA_NAME);

        // Initialize pose estimators with MULTI_TAG_PNP_ON_COPROCESSOR strategy
        // In PhotonLib 2026, PhotonCamera is not passed to constructor
        frontPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.ROBOT_TO_FRONT_CAM
        );

        rearPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.ROBOT_TO_REAR_CAM
        );
    }

    /**
     * Returns the best vision measurement cached this loop by periodic().
     * INEFF-01/07: O(1) — no camera processing; safe to call from commands every cycle.
     */
    public Optional<VisionMeasurement> getBestVisionMeasurement() {
        return cachedMeasurement;
    }

    /**
     * Returns the best cached vision measurement if it is recent enough.
     */
    public Optional<VisionMeasurement> getBestVisionMeasurementIfFresh() {
        return cachedMeasurement.filter(this::isMeasurementFresh);
    }

    private boolean isMeasurementFresh(VisionMeasurement measurement) {
        return (Timer.getFPGATimestamp() - measurement.timestampSeconds())
            <= VisionConstants.MAX_VISION_AGE_SECONDS;
    }

    /**
     * Computes the best vision measurement from all cameras.
     * Called exactly once per loop from periodic(); result stored in cachedMeasurement.
     */
    private Optional<VisionMeasurement> computeBestVisionMeasurement() {
        PhotonPipelineResult frontResult = getFrontResult();
        PhotonPipelineResult rearResult = getRearResult();

        Optional<VisionMeasurement> frontMeasurement = processCameraResult(
            frontPoseEstimator,
            frontCamera,
            frontResult,
            VisionConstants.FRONT_CAMERA_NAME
        );
        Optional<VisionMeasurement> rearMeasurement = processCameraResult(
            rearPoseEstimator,
            rearCamera,
            rearResult,
            VisionConstants.REAR_CAMERA_NAME
        );

        // If both cameras have measurements, choose the best one
        if (frontMeasurement.isPresent() && rearMeasurement.isPresent()) {
            VisionMeasurement front = frontMeasurement.get();
            VisionMeasurement rear = rearMeasurement.get();

            // Prefer multi-tag over single-tag
            if (front.numTagsUsed() >= VisionConstants.MIN_TAGS_FOR_MULTI_TAG
                && rear.numTagsUsed() < VisionConstants.MIN_TAGS_FOR_MULTI_TAG) {
                return frontMeasurement;
            } else if (rear.numTagsUsed() >= VisionConstants.MIN_TAGS_FOR_MULTI_TAG
                && front.numTagsUsed() < VisionConstants.MIN_TAGS_FOR_MULTI_TAG) {
                return rearMeasurement;
            }

            // Both multi-tag or both single-tag: prefer closer distance
            if (front.averageDistance() < rear.averageDistance()) {
                return frontMeasurement;
            } else {
                return rearMeasurement;
            }
        }

        // Return whichever is present
        return frontMeasurement.or(() -> rearMeasurement);
    }

    /**
     * Process a single camera's result and create a VisionMeasurement if valid.
     */

    private Optional<VisionMeasurement> processCameraResult(
        PhotonPoseEstimator poseEstimator,
        PhotonCamera camera,
        PhotonPipelineResult result,
        String cameraName
    ) {
        updateCameraFreshness(camera, result);


        if (!result.hasTargets()) {
            return Optional.empty();
        }

        var estimatedPose = poseEstimator.update(result);

        if (estimatedPose.isEmpty()) {
            return Optional.empty();
        }

        var pose = estimatedPose.get();
        var pose2d = pose.estimatedPose.toPose2d();

        // BUG-04/05: Compute average distance exactly once; reuse for quality gate,
        // std-dev selection, and the VisionMeasurement record.  This also ensures
        // POSITIVE_INFINITY (unknown tag IDs) is caught by the quality gate before
        // it can reach calculateStandardDeviations().
        double avgDistance = calculateAverageTagDistance(result.getTargets(), pose2d);

        // Quality gating
        if (!shouldUseMeasurement(pose, result, avgDistance)) {
            SmartDashboard.putString("Vision/" + cameraName + "CamStatus", "Rejected (quality gate)");
            return Optional.empty();
        }

        Matrix<N3, N1> stdDevs = calculateStandardDeviations(pose, avgDistance);
        int numTags = pose.targetsUsed.size();

        return Optional.of(new VisionMeasurement(
            pose2d,
            pose.timestampSeconds,
            stdDevs,
            numTags,
            avgDistance
        ));
    }

    /**
     * Track camera frame freshness based on result timestamp
     * @param camera
     * @param result
     * @return
     */
    // BUG-06: Use FPGA time throughout — same domain as PhotonVision timestamps.
    private void updateCameraFreshness(PhotonCamera camera, PhotonPipelineResult result) {
        double timestampSeconds = result.getTimestampSeconds();
        double nowFpga = Timer.getFPGATimestamp();

        if (camera == frontCamera && timestampSeconds > lastFrontTimestamp) {
            lastFrontTimestamp = timestampSeconds;
            lastFrontFpgaTs    = nowFpga;
        } else if (camera == rearCamera && timestampSeconds > lastRearTimestamp) {
            lastRearTimestamp = timestampSeconds;
            lastRearFpgaTs    = nowFpga;
        }
    }


    /**
     * Determine if a measurement should be used based on quality criteria.
     */

    // BUG-04: avgDistance is pre-computed by the caller — no redundant recalculation here.
    // BUG-05: if avgDistance == POSITIVE_INFINITY (unknown tag IDs), the distance check
    //         below rejects the measurement before it can propagate to std-dev selection.
    private boolean shouldUseMeasurement(
        EstimatedRobotPose pose,
        PhotonPipelineResult result,
        double avgDistance
    ) {
        var pose2d = pose.estimatedPose.toPose2d();

        double fieldLength = fieldLayout.getFieldLength();
        double fieldWidth  = fieldLayout.getFieldWidth();

        if (pose2d.getX() < 0 || pose2d.getX() > fieldLength
                || pose2d.getY() < 0 || pose2d.getY() > fieldWidth) {
            return false;
        }

        // For single-tag detections, check ambiguity
        if (result.getTargets().size() == 1) {
            double ambiguity = result.getBestTarget().getPoseAmbiguity();
            if (ambiguity > VisionConstants.MAX_AMBIGUITY) {
                return false;
            }
        }

        // Reject if beyond max distance; also catches POSITIVE_INFINITY (BUG-05)
        if (avgDistance > VisionConstants.MAX_TAG_DISTANCE_METERS) {
            return false;
        }

        return true;
    }

    /**
     * Calculate standard deviations based on measurement quality.
     * Multi-tag detections and closer distances get higher trust (lower std dev).
     */
    private Matrix<N3, N1> calculateStandardDeviations(
        EstimatedRobotPose pose,
        double averageDistance
    ) {
        int numTags = pose.targetsUsed.size();

        // Multi-tag: highest trust
        if (numTags >= VisionConstants.MIN_TAGS_FOR_MULTI_TAG) {
            return VisionConstants.MULTI_TAG_STDDEVS;
        }

        // Single tag: trust depends on distance
        if (averageDistance < 2.0) {
            return VisionConstants.SINGLE_TAG_CLOSE_STDDEVS;
        } else {
            return VisionConstants.SINGLE_TAG_FAR_STDDEVS;
        }
    }

    /**
     * Calculate average distance from robot to all detected tags.
     */
    private double calculateAverageTagDistance(List<PhotonTrackedTarget> targets, Pose2d robotPose) {
        if (targets.isEmpty()) {
            return 0.0;
        }

        double totalDistance = 0.0;
        int validTagCount = 0;

        for (var target : targets) {
            var tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                totalDistance += robotPose.getTranslation()
                    .getDistance(tagPose.get().toPose2d().getTranslation());
                validTagCount++;
            }
        }

        //If no valid tag IDs were in the layout, treat as invalid/very poor measurement 
        if (validTagCount == 0){
            return Double.POSITIVE_INFINITY;
        }


        return totalDistance / validTagCount;
    }

    private PhotonPipelineResult getFrontResult() {
        return cachedFrontResult.orElseGet(frontCamera::getLatestResult);
    }

    private PhotonPipelineResult getRearResult() {
        return cachedRearResult.orElseGet(rearCamera::getLatestResult);
    }

    /**
     * Get the best target from all cameras (highest confidence/lowest ambiguity).
     */
    
    public Optional<PhotonTrackedTarget> getBestTarget() {
        var frontResult = getFrontResult();
        var rearResult = getRearResult();

        PhotonTrackedTarget bestTarget = null;
        double lowestAmbiguity = Double.MAX_VALUE;

        if (frontResult.hasTargets()) {
            var frontTarget = frontResult.getBestTarget();
            if (frontTarget.getPoseAmbiguity() < lowestAmbiguity) {
                bestTarget = frontTarget;
                lowestAmbiguity = frontTarget.getPoseAmbiguity();
            }
        }

        if (rearResult.hasTargets()) {
            var rearTarget = rearResult.getBestTarget();
            if (rearTarget.getPoseAmbiguity() < lowestAmbiguity) {
                bestTarget = rearTarget;
                lowestAmbiguity = rearTarget.getPoseAmbiguity();
            }
        }

        return Optional.ofNullable(bestTarget);
    }

    /**
     * Check if a specific AprilTag is visible in any camera.
     * INEFF-03: O(1) bounds-checked array access; no boxing, no linear scan.
     */
    public boolean isTagVisible(int tagId) {
        return tagId >= 1 && tagId <= 30 && cachedTagsVisible[tagId];
    }

    /**
     * Returns a list of all currently visible AprilTag IDs from all cameras.
     * Built on demand from the boolean array; allocation only happens when a caller
     * actually needs the list (typically telemetry, not hot-path code).
     */
    public List<Integer> getVisibleTags() {
        List<Integer> tags = new ArrayList<>();
        for (int i = 1; i <= 30; i++) {
            if (cachedTagsVisible[i]) tags.add(i);
        }
        return tags;
    }

    /**
     * Builds the visible-tag boolean array from both cameras.
     * Called once per loop from periodic() when a new frame has arrived.
     * INEFF-03: single-pass fill; writing true to an already-set slot is a no-op,
     * so the former O(n²) rear-camera deduplication loop is eliminated entirely.
     */
    private boolean[] computeVisibleTags() {
        boolean[] visible = new boolean[31];

        var frontResult = getFrontResult();
        if (frontResult.hasTargets()) {
            for (var target : frontResult.getTargets()) {
                int id = target.getFiducialId();
                if (id >= 1 && id <= 30) visible[id] = true;
            }
        }

        var rearResult = getRearResult();
        if (rearResult.hasTargets()) {
            for (var target : rearResult.getTargets()) {
                int id = target.getFiducialId();
                if (id >= 1 && id <= 30) visible[id] = true;
            }
        }

        return visible;
    }

    /**
     * Get distance from current best pose estimate to a target pose.
     */
    public Optional<Double> getDistanceToPose(Pose2d targetPose) {
        // BUG-08: use fresh-gated measurement so commands don't act on data up to
        // MAX_VISION_AGE_SECONDS old.
        return getBestVisionMeasurementIfFresh()
            .map(measurement ->
                measurement.estimatedPose().getTranslation().getDistance(targetPose.getTranslation())
            );
    }

    /**
     * Get yaw angle from current best pose estimate to a target pose.
     */
    public Optional<Rotation2d> getYawToPose(Pose2d targetPose) {
        // BUG-08: same freshness gate as getDistanceToPose.
        return getBestVisionMeasurementIfFresh()
            .map(measurement -> {
                var currentPose = measurement.estimatedPose();
                var translation = targetPose.getTranslation().minus(currentPose.getTranslation());
                return new Rotation2d(translation.getX(), translation.getY())
                    .minus(currentPose.getRotation());
            });
    }

    /**
     * Check if robot is aligned with a target pose within tolerance.
     */
    public boolean isAlignedWithTarget(Pose2d targetPose, double toleranceDeg) {
        return getYawToPose(targetPose)
            .map(yaw -> Math.abs(yaw.getDegrees()) < toleranceDeg)
            .orElse(false);
    }

    /**
     * Get distance to hub.
     */
    public Optional<Double> getDistanceToHub() {
        return getDistanceToPose(VisionConstants.HUB_POSE);
    }

    /**
     * Check if aligned with hub.
     */
    public boolean isAlignedWithHub(double toleranceDeg) {
        return isAlignedWithTarget(VisionConstants.HUB_POSE, toleranceDeg);
    }

    /**
     * Get distance to human player station.
     */
    public Optional<Double> getDistanceToHPStation() {
        return getDistanceToPose(VisionConstants.HP_STATION_POSE);
    }

    /**
     * Check if aligned with human player station.
     */
    public boolean isAlignedWithHPStation(double toleranceDeg) {
        return isAlignedWithTarget(VisionConstants.HP_STATION_POSE, toleranceDeg);
    }

    /**
     * Get distance to trench.
     */
    public Optional<Double> getDistanceToTrench() {
        return getDistanceToPose(VisionConstants.TRENCH_POSE);
    }

    /**
     * Check if aligned with trench.
     */
    public boolean isAlignedWithTrench(double toleranceDeg) {
        return isAlignedWithTarget(VisionConstants.TRENCH_POSE, toleranceDeg);
    }

    /**
     * Get distance to depot.
     */
    public Optional<Double> getDistanceToDepot() {
        return getDistanceToPose(VisionConstants.DEPOT_POSE);
    }

    /**
     * Check if aligned with depot.
     */
    public boolean isAlignedWithDepot(double toleranceDeg) {
        return isAlignedWithTarget(VisionConstants.DEPOT_POSE, toleranceDeg);
    }

    /**
     * Get distance to outpost.
     */
    public Optional<Double> getDistanceToOutpost() {
        return getDistanceToPose(VisionConstants.OUTPOST_POSE);
    }

    /**
     * Check if aligned with outpost.
     */
    public boolean isAlignedWithOutpost(double toleranceDeg) {
        return isAlignedWithTarget(VisionConstants.OUTPOST_POSE, toleranceDeg);
    }

    /**
     * Get distance to tower.
     */
    public Optional<Double> getDistanceToTower() {
        return getDistanceToPose(VisionConstants.TOWER_POSE);
    }

    /**
     * Check if aligned with tower.
     */
    public boolean isAlignedWithTower(double toleranceDeg) {
        return isAlignedWithTarget(VisionConstants.TOWER_POSE, toleranceDeg);
    }

    /**
     * Update telemetry to SmartDashboard.
     */
    private void updateTelemetry() {
        // Camera connection status
        SmartDashboard.putBoolean("Vision/FrontCamConnected", isCameraConnected(frontCamera));
        SmartDashboard.putBoolean("Vision/RearCamConnected", isCameraConnected(rearCamera));

        // Visible tags — built on demand from the boolean array (INEFF-03)
        List<Integer> visibleTagList = getVisibleTags();
        SmartDashboard.putNumber("Vision/NumTagsVisible", visibleTagList.size());
        SmartDashboard.putString("Vision/VisibleTags", visibleTagList.toString());

        // Best measurement — read from per-loop cache (INEFF-01)
        Optional<VisionMeasurement> bestMeasurement = cachedMeasurement;

        if (bestMeasurement.isPresent()) {
            var measurement = bestMeasurement.get();
            var pose = measurement.estimatedPose();

            SmartDashboard.putNumber("Vision/BestPoseX", pose.getX());
            SmartDashboard.putNumber("Vision/BestPoseY", pose.getY());
            SmartDashboard.putNumber("Vision/BestPoseTheta", pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/NumTagsUsed", measurement.numTagsUsed());
            SmartDashboard.putNumber("Vision/AvgTagDistance", measurement.averageDistance());

            getDistanceToPose(VisionConstants.HUB_POSE, bestMeasurement)
                .ifPresent(distance -> SmartDashboard.putNumber("Vision/HubDistance", distance));
            SmartDashboard.putBoolean("Vision/HubAligned",
                isAlignedWithTarget(VisionConstants.HUB_POSE, 2.0, bestMeasurement));

            getDistanceToPose(VisionConstants.HP_STATION_POSE, bestMeasurement)
                .ifPresent(distance -> SmartDashboard.putNumber("Vision/HPStationDistance", distance));
            SmartDashboard.putBoolean("Vision/HPStationAligned",
                isAlignedWithTarget(VisionConstants.HP_STATION_POSE, 2.0, bestMeasurement));

            getDistanceToPose(VisionConstants.TRENCH_POSE, bestMeasurement)
                .ifPresent(distance -> SmartDashboard.putNumber("Vision/TrenchDistance", distance));
            SmartDashboard.putBoolean("Vision/TrenchAligned",
                isAlignedWithTarget(VisionConstants.TRENCH_POSE, 2.0, bestMeasurement));

            getDistanceToPose(VisionConstants.DEPOT_POSE, bestMeasurement)
                .ifPresent(distance -> SmartDashboard.putNumber("Vision/DepotDistance", distance));
            SmartDashboard.putBoolean("Vision/DepotAligned",
                isAlignedWithTarget(VisionConstants.DEPOT_POSE, 2.0, bestMeasurement));

            getDistanceToPose(VisionConstants.OUTPOST_POSE, bestMeasurement)
                .ifPresent(distance -> SmartDashboard.putNumber("Vision/OutpostDistance", distance));
            SmartDashboard.putBoolean("Vision/OutpostAligned",
                isAlignedWithTarget(VisionConstants.OUTPOST_POSE, 2.0, bestMeasurement));

            getDistanceToPose(VisionConstants.TOWER_POSE, bestMeasurement)
                .ifPresent(distance -> SmartDashboard.putNumber("Vision/TowerDistance", distance));
            SmartDashboard.putBoolean("Vision/TowerAligned",
                isAlignedWithTarget(VisionConstants.TOWER_POSE, 2.0, bestMeasurement));
        } else {
            SmartDashboard.putNumber("Vision/BestPoseX", 0.0);
            SmartDashboard.putNumber("Vision/BestPoseY", 0.0);
            SmartDashboard.putNumber("Vision/BestPoseTheta", 0.0);
            SmartDashboard.putNumber("Vision/NumTagsUsed", 0);
            SmartDashboard.putNumber("Vision/AvgTagDistance", 0.0);

            SmartDashboard.putNumber("Vision/HubDistance", Double.NaN);
            SmartDashboard.putBoolean("Vision/HubAligned", false);
            SmartDashboard.putNumber("Vision/HPStationDistance", Double.NaN);
            SmartDashboard.putBoolean("Vision/HPStationAligned", false);
            SmartDashboard.putNumber("Vision/TrenchDistance", Double.NaN);
            SmartDashboard.putBoolean("Vision/TrenchAligned", false);
            SmartDashboard.putNumber("Vision/DepotDistance", Double.NaN);
            SmartDashboard.putBoolean("Vision/DepotAligned", false);
            SmartDashboard.putNumber("Vision/OutpostDistance", Double.NaN);
            SmartDashboard.putBoolean("Vision/OutpostAligned", false);
            SmartDashboard.putNumber("Vision/TowerDistance", Double.NaN);
            SmartDashboard.putBoolean("Vision/TowerAligned", false);
        }

    }

    private Optional<Double> getDistanceToPose(Pose2d targetPose, Optional<VisionMeasurement> measurement) {
            return measurement.map(m -> m.estimatedPose().getTranslation().getDistance(targetPose.getTranslation()));
        }

    private Optional<Rotation2d> getYawToPose(Pose2d targetPose, Optional<VisionMeasurement> measurement) {
            return measurement.map(m -> {
                var currentPose = m.estimatedPose();
                var translation = targetPose.getTranslation().minus(currentPose.getTranslation());
                return new Rotation2d(translation.getX(), translation.getY()).minus(currentPose.getRotation());
            });
        }

    private boolean isAlignedWithTarget(Pose2d targetPose, double toleranceDeg, Optional<VisionMeasurement> measurement) {
        return getYawToPose(targetPose, measurement)
            .map(yaw -> Math.abs(yaw.getDegrees()) < toleranceDeg)
            .orElse(false);
    }
    /**
     * Check if a camera is connected by verifying fresh frames are still arriving.
     * BUG-06: Uses FPGA time (same domain as PhotonVision timestamps) instead of wall-clock.
     * The lastXFpgaTs == -1.0 guard ensures cameras are not reported as connected
     * before the first frame arrives.
     */
    private boolean isCameraConnected(PhotonCamera camera) {
        double nowFpga = Timer.getFPGATimestamp();

        if (camera == frontCamera) {
            return lastFrontFpgaTs >= 0
                && (nowFpga - lastFrontFpgaTs) <= CAMERA_STALE_TIMEOUT_SECONDS;
        } else if (camera == rearCamera) {
            return lastRearFpgaTs >= 0
                && (nowFpga - lastRearFpgaTs) <= CAMERA_STALE_TIMEOUT_SECONDS;
        }

        return false;
    }

    /**
     * Returns true if at least one camera is producing fresh frames.
     * Used by DriveTrain to auto-enable vision pose fusion (BUG-01).
     */
    public boolean isAnyVisionAvailable() {
        return isCameraConnected(frontCamera) || isCameraConnected(rearCamera);
    }

    @Override
    public void periodic() {
        cachedFrontResult = Optional.of(frontCamera.getLatestResult());
        cachedRearResult = Optional.of(rearCamera.getLatestResult());

        // INEFF-04: skip heavy recomputation when neither camera has produced a new frame.
        // lastFrontTimestamp / lastRearTimestamp are updated inside updateCameraFreshness(),
        // which runs as part of computeBestVisionMeasurement(), so they always hold the
        // timestamp of the last frame that was actually processed.
        double frontTs = cachedFrontResult.get().getTimestampSeconds();
        double rearTs  = cachedRearResult.get().getTimestampSeconds();
        if (frontTs > lastFrontTimestamp || rearTs > lastRearTimestamp) {
            cachedMeasurement = computeBestVisionMeasurement();
            cachedTagsVisible = computeVisibleTags();
        }

        if (++telemetryLoopCounter >= VisionConstants.TELEMETRY_PERIOD_LOOPS) {
            telemetryLoopCounter = 0;
            updateTelemetry();
        }
    }
}
