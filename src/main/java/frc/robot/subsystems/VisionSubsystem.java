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
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
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
    private double lastRearTimestamp = -1.0;
    private long lastFrontUpdateMs = 0;
    private long lastRearUpdateMs = 0;
    private static final long CAMERA_STALE_TIMEOUT_MS = 500;

    public VisionSubsystem() {



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
     * Get the best vision measurement from all cameras.
     * Prioritizes multi-tag detections and closer, lower-ambiguity measurements.
     *
     * @return Optional VisionMeasurement containing the best pose estimate, or empty if none available
     */

    public Optional<VisionMeasurement> getBestVisionMeasurement() {
        Optional<VisionMeasurement> frontMeasurement =
            processCameraResult(frontPoseEstimator, frontCamera, VisionConstants.FRONT_CAMERA_NAME);
        Optional<VisionMeasurement> rearMeasurement =
            processCameraResult(rearPoseEstimator, rearCamera, VisionConstants.REAR_CAMERA_NAME);

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
        String cameraName
    ) {
        var result = camera.getLatestResult();
        updateCameraFreshness(camera, result);


        if (!result.hasTargets()) {
            return Optional.empty();
        }

        var estimatedPose = poseEstimator.update(result);

        if (estimatedPose.isEmpty()) {
            return Optional.empty();
        }

        var pose = estimatedPose.get();

        // Quality gating
        if (!shouldUseMeasurement(pose, result)) {
            SmartDashboard.putString("Vision/" + cameraName + "CamStatus", "Rejected (quality gate)");
            return Optional.empty();
        }

        // Calculate standard deviations based on measurement quality
        Matrix<N3, N1> stdDevs = calculateStandardDeviations(
            pose,
            calculateAverageTagDistance(result.getTargets(), pose.estimatedPose.toPose2d())
        );

        // Count tags used
        int numTags = pose.targetsUsed.size();

        // Calculate average distance
        double avgDistance = calculateAverageTagDistance(result.getTargets(), pose.estimatedPose.toPose2d());

        return Optional.of(new VisionMeasurement(
            pose.estimatedPose.toPose2d(),
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
    private void updateCameraFreshness(PhotonCamera camera, PhotonPipelineResult result) {
        double timestampSeconds = result.getTimestampSeconds();
        long nowMs = System.currentTimeMillis();

        if (camera == frontCamera && timestampSeconds > lastFrontTimestamp) {
            lastFrontTimestamp = timestampSeconds;
            lastFrontUpdateMs = nowMs;
        } else if (camera == rearCamera && timestampSeconds > lastRearTimestamp) {
            lastRearTimestamp = timestampSeconds;
            lastRearUpdateMs = nowMs;
        }

    }


    /**
     * Determine if a measurement should be used based on quality criteria.
     */

    private boolean shouldUseMeasurement(
        EstimatedRobotPose pose,
        PhotonPipelineResult result
    ) {
        // Check if pose is within field boundaries (assuming standard FRC field ~16.5m x 8.2m)
        var pose2d = pose.estimatedPose.toPose2d();

        double fieldLength = fieldLayout.getFieldLength();
        double fieldWidth = fieldLayout.getFieldWidth();

        if (pose2d.getX() < 0 || pose2d.getX() > fieldLength || pose2d.getY() < 0 || pose2d.getY() > fieldWidth) {
            return false;
        }

        // For single-tag detections, check ambiguity
        if (result.getTargets().size() == 1) {
            double ambiguity = result.getBestTarget().getPoseAmbiguity();
            if (ambiguity > VisionConstants.MAX_AMBIGUITY) {
                return false;
            }
        }

        // Check if any tag is beyond max distance
        double avgDistance = calculateAverageTagDistance(result.getTargets(), pose2d);
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

    /**
     * Get the best target from all cameras (highest confidence/lowest ambiguity).
     */
    
    public Optional<PhotonTrackedTarget> getBestTarget() {
        var frontResult = frontCamera.getLatestResult();
        var rearResult = rearCamera.getLatestResult();

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
     */
    public boolean isTagVisible(int tagId) {
        return getVisibleTags().contains(tagId);
    }

    /**
     * Get list of all visible AprilTag IDs from all cameras.
     */
    public List<Integer> getVisibleTags() {
        List<Integer> visibleTags = new ArrayList<>();

        var frontResult = frontCamera.getLatestResult();
        if (frontResult.hasTargets()) {
            frontResult.getTargets().forEach(target -> visibleTags.add(target.getFiducialId()));
        }

        var rearResult = rearCamera.getLatestResult();
        if (rearResult.hasTargets()) {
            rearResult.getTargets().forEach(target -> {
                if (!visibleTags.contains(target.getFiducialId())) {
                    visibleTags.add(target.getFiducialId());
                }
            });
        }

        return visibleTags;
    }

    /**
     * Get distance from current best pose estimate to a target pose.
     */
    public Optional<Double> getDistanceToPose(Pose2d targetPose) {
        return getBestVisionMeasurement()
            .map(measurement ->
                measurement.estimatedPose().getTranslation().getDistance(targetPose.getTranslation())
            );
    }

    /**
     * Get yaw angle from current best pose estimate to a target pose.
     */
    public Optional<Rotation2d> getYawToPose(Pose2d targetPose) {
        return getBestVisionMeasurement()
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

        // Visible tags
        List<Integer> visibleTags = getVisibleTags();
        SmartDashboard.putNumber("Vision/NumTagsVisible", visibleTags.size());
        SmartDashboard.putString("Vision/VisibleTags", visibleTags.toString());

        // Cache best pose estimate once for this cycle
        Optional<VisionMeasurement> bestMeasurement = getBestVisionMeasurement();

        if (bestMeasurement.isPresent()) {
            var measurement = bestMeasurement.get();
            var pose = measurement.estimatedPose();

            SmartDashboard.putNumber("Vision/BestPoseX", pose.getX());
            SmartDashboard.putNumber("Vision/BestPoseY", pose.getY());
            SmartDashboard.putNumber("Vision/BestPoseTheta", pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/NumTagsUsed", measurement.numTagsUsed());
            SmartDashboard.putNumber("Vision/AvgTagDistance", measurement.averageDistance());
        } else {
            SmartDashboard.putNumber("Vision/BestPoseX", 0.0);
            SmartDashboard.putNumber("Vision/BestPoseY", 0.0);
            SmartDashboard.putNumber("Vision/BestPoseTheta", 0.0);
            SmartDashboard.putNumber("Vision/NumTagsUsed", 0);
            SmartDashboard.putNumber("Vision/AvgTagDistance", 0.0);
        }

        // Hub distance and alignment
        getDistanceToPose(VisionConstants.HUB_POSE, bestMeasurement)
            .ifPresent(distance -> SmartDashboard.putNumber("Vision/HubDistance", distance)
            );
        SmartDashboard.putBoolean("Vision/HubAligned", 
            isAlignedWithTarget(VisionConstants.HUB_POSE, 2.0, bestMeasurement)
            );


        // HP Station distance and alignment
        getDistanceToPose(VisionConstants.HP_STATION_POSE, bestMeasurement)
            .ifPresent(distance -> SmartDashboard.putNumber("Vision/HPStationDistance", distance)
            );
        SmartDashboard.putBoolean("Vision/HPStationAligned", 
            isAlignedWithTarget(VisionConstants.HP_STATION_POSE, 2.0, bestMeasurement)
            );

        // Trench distance and alignment
        getDistanceToPose(VisionConstants.TRENCH_POSE, bestMeasurement)
            .ifPresent(distance -> SmartDashboard.putNumber("Vision/TrenchDistance", distance)
            );
        SmartDashboard.putBoolean("Vision/TrenchAligned", 
            isAlignedWithTarget(VisionConstants.TRENCH_POSE, 2.0, bestMeasurement)
            );


        // Depot distance and alignment
        getDistanceToPose(VisionConstants.DEPOT_POSE, bestMeasurement)
            .ifPresent(distance -> SmartDashboard.putNumber("Vision/DepotDistance", distance)
            );
        SmartDashboard.putBoolean("Vision/DepotAligned", 
            isAlignedWithTarget(VisionConstants.DEPOT_POSE, 2.0, bestMeasurement)
            );

        // Outpost distance and alignment
        getDistanceToPose(VisionConstants.OUTPOST_POSE, bestMeasurement)
            .ifPresent(distance -> SmartDashboard.putNumber("Vision/OutpostDistance", distance)
            );
        SmartDashboard.putBoolean("Vision/OutpostAligned", 
            isAlignedWithTarget(VisionConstants.OUTPOST_POSE, 2.0, bestMeasurement)
            );

        // Tower distance and alignment
        getDistanceToPose(VisionConstants.TOWER_POSE, bestMeasurement)
            .ifPresent(distance -> SmartDashboard.putNumber("Vision/TowerDistance", distance)
            );
        SmartDashboard.putBoolean("Vision/TowerAligned", 
            isAlignedWithTarget(VisionConstants.TOWER_POSE, 2.0, bestMeasurement)
            );


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
     * Check if a camera is connected by verifying fresh frames are still arriving
     */
    private boolean isCameraConnected(PhotonCamera cameraName) {
        long nowMs = System.currentTimeMillis();

        if (cameraName == frontCamera) {
            return nowMs - lastFrontUpdateMs <= CAMERA_STALE_TIMEOUT_MS;
        } else if (cameraName == rearCamera) {
            return nowMs - lastRearUpdateMs <= CAMERA_STALE_TIMEOUT_MS;
        }

        return false;
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }
}
