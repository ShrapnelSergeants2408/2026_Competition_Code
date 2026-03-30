// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Immutable snapshot of a single vision-based pose estimate from PhotonVision.
 *
 * <p>Produced once per loop by {@code Vision.periodic()} and cached. Consumed
 * by {@code DriveTrain} to fuse vision into the pose estimator and by
 * {@code Shooter} to resolve shooting distance.
 *
 * <p>Fields are accessed via auto-generated accessor methods, e.g.
 * {@code measurement.estimatedPose()}, {@code measurement.numTagsUsed()}.
 *
 * @param estimatedPose       robot position on the field derived from AprilTag detections
 * @param timestampSeconds    FPGA timestamp (seconds) when the camera frame was captured;
 *                            used to check measurement freshness and for latency compensation
 * @param standardDeviations  Kalman filter trust weights for x, y, and heading (lower = more trusted);
 *                            set based on number of tags seen and distance to tags
 * @param bestTargetAmbiguity pose ambiguity of the best detected tag (0 = unambiguous, 1 = completely ambiguous);
 *                            measurements with ambiguity above {@code VisionConstants.MAX_AMBIGUITY} are rejected
 * @param numTagsUsed         number of AprilTags that contributed to this pose estimate;
 *                            multi-tag estimates (≥ 2) are significantly more accurate than single-tag
 * @param averageDistance     average distance in meters from the robot to all detected tags;
 *                            used for quality gating and standard deviation selection
 */
public record VisionMeasurement(
    Pose2d estimatedPose,
    double timestampSeconds,
    Matrix<N3, N1> standardDeviations,
    double bestTargetAmbiguity,
    int numTagsUsed,
    double averageDistance
) {}
