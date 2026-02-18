// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Container for vision-based pose measurements from PhotonVision.
 * Used to pass vision data from VisionSubsystem to drivetrain pose estimator.
 */
public record VisionMeasurement(
    Pose2d estimatedPose,
    double timestampSeconds,
    Matrix<N3, N1> standardDeviations,
    int numTagsUsed,
    double averageDistance
) {}
