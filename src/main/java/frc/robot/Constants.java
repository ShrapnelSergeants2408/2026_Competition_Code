// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

// NO IMPERIAL UNITS
public final class Constants {

    public static class OperatorConstants {

        public static final int kDriverControllerPort = 0;
    }

    public static class ClimberConstants {}

    public static class DriveTrainConstants {

        // // distance from center of robot to center of wheel (cm)
        // public static final double WHEEL_DISTANCE = 0.0; // cm
        public static final int LEFT_MOTOR_ID_LEAD = 20;
        public static final int LEFT_MOTOR_ID_FOLLOW = 21;
        public static final int RIGHT_MOTOR_ID_LEAD = 22;
        public static final int RIGHT_MOTOR_ID_FOLLOW = 23;        
    }

    public static class IntakeConstants {}

    public static class ShooterConstants {

        public static final int STALL_LIMIT = 30;
        // CAN IDs
        public static final int SHOOTER_MOTOR_ID = 30;
        public static final int FEEDER_MOTOR_ID = 31;

        // Motor speeds
        public static final double SHOOTER_SPEED = 0.9; // Shoots the ball
        public static final double FEEDER_SPEED = 0.6; // Feeds ball into shooter
        public static final double NOMINAL_VOLTAGE = 12; // i dunno, it was a hardcoded value i moved it
    }

    public static class VisionConstants {
        // Camera names (must match PhotonVision configuration)
        public static final String FRONT_CAMERA_NAME = "Front_Camera";
        public static final String REAR_CAMERA_NAME = "Rear_Camera";
        public static final String DRIVER_CAMERA_NAME = "Driver_Camera";

        // Camera transforms (robot-to-camera)
        // Estimated placement: centerline, 20" (~0.508m) above ground, 1" (~0.0254m) from edge
        // Assumptions: Robot is ~28" (0.71m) bumper-to-bumper, cameras tilted 30° down

        // Front PhotonVision camera (Pi4 + PiCam v2)
        // Position: 1" from front edge = 0.355m forward from center (0.71/2 - 0.0254)
        // TODO: Measure actual robot dimensions and camera mounting position
        public static final Transform3d ROBOT_TO_FRONT_CAM = new Transform3d(
            new Translation3d(0.355, 0.0, 0.508),  // forward, left, up (meters) // TODO: Verify measurements
            new Rotation3d(0, Math.toRadians(-30), 0) // roll, pitch, yaw // TODO: Measure actual camera angles
        );

        // Rear PhotonVision camera (Pi5 + OV9281)
        // Position: 1" from rear edge = -0.355m from center
        // Rotated 180° (facing backwards)
        // TODO: Measure actual robot dimensions and camera mounting position
        public static final Transform3d ROBOT_TO_REAR_CAM = new Transform3d(
            new Translation3d(-0.355, 0.0, 0.508), // TODO: Verify measurements
            new Rotation3d(0, Math.toRadians(-30), Math.toRadians(180)) // TODO: Measure actual camera angles
        );

        // Vision measurement quality gating
        public static final double MAX_TAG_DISTANCE_METERS = 4.0; // TODO: Tune based on camera performance
        public static final double MAX_AMBIGUITY = 0.3; // TODO: Tune based on field testing
        public static final int MIN_TAGS_FOR_MULTI_TAG = 2;

        // Standard deviations for pose estimation (meters and radians)
        // Single tag close (< 2m)
        public static final Matrix<N3, N1> SINGLE_TAG_CLOSE_STDDEVS =
            VecBuilder.fill(0.5, 0.5, Math.toRadians(10)); // TODO: Tune based on testing

        // Single tag far (> 2m)
        public static final Matrix<N3, N1> SINGLE_TAG_FAR_STDDEVS =
            VecBuilder.fill(1.0, 1.0, Math.toRadians(20)); // TODO: Tune based on testing

        // Multi-tag (2+ tags)
        public static final Matrix<N3, N1> MULTI_TAG_STDDEVS =
            VecBuilder.fill(0.2, 0.2, Math.toRadians(5)); // TODO: Tune based on testing

        // Known field element poses (2026 field - update based on actual game)
        public static final Pose2d HUB_POSE = new Pose2d(8.27, 4.1, new Rotation2d()); // TODO: Update with actual 2026 game positions
        public static final Pose2d HP_STATION_POSE = new Pose2d(1.5, 7.5, Rotation2d.fromDegrees(180)); // TODO: Update with actual 2026 game positions
    }
}
