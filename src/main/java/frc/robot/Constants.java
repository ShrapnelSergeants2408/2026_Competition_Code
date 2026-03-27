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

public final class Constants {

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class ClimberConstants {}

    public static class DriveTrainConstants {

        // CAN IDs
        public static final int LEFT_LEAD_CAN_ID = 20;
        public static final int RIGHT_LEAD_CAN_ID = 22;
        public static final int LEFT_FOLLOW_CAN_ID = 21;
        public static final int RIGHT_FOLLOW_CAN_ID = 23;

        // Motor config
        public static final int CURRENT_LIMIT = 60; //60   // amps

        // Pose initialization — vision measurements up to this many seconds old are
        // accepted when seeding the pose at auto/teleop init. More generous than the
        // in-match freshness window because any recent fix beats defaulting to field origin.
        public static final double POSE_INIT_MAX_VISION_AGE_SECONDS = 5.0;

        // Driving
        public static final double JOYSTICK_DEADBAND = 0.05;
        public static final int TELEMETRY_PERIOD_LOOPS = 5;

        // Physical dimensions
        public static final double GEAR_RATIO = 8.46;
        public static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 inches
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double TRACK_WIDTH_METERS = 0.546;

        public static final double SPEED_SCALE = 0.7; //0.5
    }

    public static class IntakeConstants {}

    public static class ShooterConstants {

        // CAN IDs
        public static final int SHOOTER_MOTOR_ID = 30;  // TalonFX (Phoenix 6) — shooter wheel
        public static final int INTAKE_MOTOR_ID  = 31;  // SparkMAX — intake roller (primary + secondary linked, CCW only)
        public static final int TRIGGER_MOTOR_ID = 32;  // SparkMAX — trigger/hopper (bidirectional)

        // Motor inversion — verify polarity on bench, flip here if wrong
        // Intake roller:  positive set() should = CCW (into robot)
        // Trigger motor:  positive set() should = CW  (hopper → shooter)
        public static final boolean SHOOTER_INVERTED       = false; // TODO: verify on bench
        public static final boolean INTAKE_MOTOR_INVERTED  = false; // TODO: verify on bench; CCW must be positive
        public static final boolean TRIGGER_MOTOR_INVERTED = false; // TODO: verify on bench; CW must be positive

        // Intake roller (CAN 31) speeds — this motor runs CCW only
        // Positive duty cycle = CCW (into robot) when INTAKE_MOTOR_INVERTED is set correctly
        public static final double INTAKE_SPEED       =  1.0; // 100% clockwise — pulls ball from ground into robot
        public static final double INTAKE_EJECT_SPEED = -1.0; // 100% counterclockwise — reverses to push ball back out

        // Shooter pulley sizes (belt drive between Kraken and shooter wheel shaft)
        public static final double SHOOTER_MOTOR_PULLEY_TEETH = 26.0; // 15T pulley on Kraken shaft
        public static final double SHOOTER_SHAFT_PULLEY_TEETH = 32.0; // 30T pulley on shooter wheel shaft
        public static final double SHOOTER_GEAR_RATIO = SHOOTER_MOTOR_PULLEY_TEETH / SHOOTER_SHAFT_PULLEY_TEETH;
        // Trigger/hopper motor (CAN 32) speeds — bidirectional
        // Positive duty cycle = CW (into shooter) when TRIGGER_MOTOR_INVERTED is set correctly
        public static final double TRIGGER_FEED_SPEED   = -0.5;  // shooting: trigger runs opposite intake (exhale)
        public static final double TRIGGER_INTAKE_SPEED =  0.5;  // 100% clockwise — both motors same direction during intake
        public static final double TRIGGER_EJECT_SPEED  = -0.5;  // same direction as intake during exhale — both motors reverse together
        public static final double JAM_REVERSE_SPEED    = -0.5;  // CCW — trigger jam-clear reverse

        public static final double NOMINAL_VOLTAGE = 12;
        public static final int SHOOTER_TELEMETRY_PERIOD_LOOPS = 5;

        // Current limits
        public static final int SHOOTER_CURRENT_LIMIT      = 60; // TalonFX stator limit (amps)
        public static final int INTAKE_MOTOR_CURRENT_LIMIT = 50; // SparkMAX smart current limit (amps)
        public static final int TRIGGER_MOTOR_CURRENT_LIMIT= 50; // SparkMAX smart current limit (amps)

        // Jam detection (trigger motor — most likely jam point)
        public static final double TRIGGER_SPIKE_THRESHOLD_AMPS = 50.0; // current spike triggers jam clear
        public static final double JAM_REVERSE_TIME_SEC = 0.25;          // duration of jam-clear reverse

        // Shooter PID / feedforward — Phoenix 6 on-controller slot 0
        public static final double TARGET_RPM_10_FEET = 3150.0; // interpolated mechanism RPM at 10 ft
        public static final double SHOOTER_KP = 1.0;       // proportional (starting value)
        public static final double SHOOTER_KI = 0.0;       // integral
        public static final double SHOOTER_KD = 0.0;       // derivative
        public static final double SHOOTER_KV = 0.12;      // feedforward (tune first)
        public static final double RPM_TOLERANCE = 50.0;   // within ±50 RPM is considered ready

        // Distance to mechanism RPM mapping (distance in feet -> mechanism RPM).
        // Values calculated from projectile physics (70° launch, 18" launcher height,
        // 72" target height, 0.556 slip factor, 4" wheel diameter, calibrated from test data).
        // Update SHOOTER_MOTOR_PULLEY_TEETH/SHOOTER_SHAFT_PULLEY_TEETH when gearing changes.
        public static final double[] DISTANCES_FEET    = {5,    7.5,  10, 12.5, 15,   17.5, 18.75};
        public static final double[] DISTANCE_RPM_MAP  = {2150, 2400, 2600, 2850, 3050, 3200, 3300};
    }

    public static class SensorConstants {
        // Photo sensor (ball detection) — DIO port 1
        // Set PHOTO_SENSOR_ENABLED = true once the sensor is physically installed
        public static final int PHOTO_SENSOR_DIO_PORT = 1;
        public static final boolean PHOTO_SENSOR_ENABLED = false;   // disabled until installed
        public static final boolean PHOTO_SENSOR_INVERTED = false;  // TODO: verify polarity on bench
    }

    public static class Auto {
        // Single authoritative max robot velocity.
        // 15 ft/s converted to m/s. Used for both PathPlanner speed limiting
        // and the PPLTVController gain-table upper bound so they stay in sync.
        public static final double MAX_ROBOT_VELOCITY_MPS = 15.0 * 0.3048; // 4.572 m/s

        // PathPlanner motion limits
        public static final double MAX_MODULE_SPEED = MAX_ROBOT_VELOCITY_MPS; // m/s
        public static final double MAX_ACCELERATION = 2.0; // m/s^2
        public static final double MAX_ANGULAR_VELOCITY = 540.0; // deg/s
        public static final double MAX_ANGULAR_ACCELERATION = 720.0; // deg/s^2

        // PPLTVController tuning defaults (state tolerances and control effort limits).
        // Live adjustment is available on SmartDashboard during test mode only.
        public static final double PPLTV_DT = 0.02;
        public static final double PPLTV_MAX_VELOCITY = MAX_ROBOT_VELOCITY_MPS; // matches MAX_MODULE_SPEED
        public static final double PPLTV_Q_X = 0.0625;
        public static final double PPLTV_Q_Y = 0.125;
        public static final double PPLTV_Q_THETA = 0.75;
        public static final double PPLTV_R_VEL = 1.0;
        public static final double PPLTV_R_OMEGA = 2.0;
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
            //new Translation3d(0.368, -0.181, 0.362),  // forward, left, up (meters) // TODO: Verify measurements
            new Translation3d(0.102, -0.181, 0.089),  // forward, left, up (meters) // TODO: Verify measurements
            new Rotation3d(0, Math.toRadians(-30), 0) // roll, pitch, yaw // TODO: Measure actual camera angles
        );

        // Rear PhotonVision camera (Pi5 + OV9281)
        // Position: 1" from rear edge = -0.355m from center
        // Rotated 180° (facing backwards)
        // TODO: Measure actual robot dimensions and camera mounting position
        public static final Transform3d ROBOT_TO_REAR_CAM = new Transform3d(
            new Translation3d(-0.305, 0.0, 0.318), // TODO: Verify measurements
            new Rotation3d(0, Math.toRadians(-30), Math.toRadians(180)) // TODO: Measure actual camera angles
        );

        // Vision measurement quality gating
        public static final double MAX_TAG_DISTANCE_METERS = 4.0; // TODO: Tune based on camera performance
        public static final double MAX_AMBIGUITY = 0.3; // TODO: Tune based on field testing
        public static final int MIN_TAGS_FOR_MULTI_TAG = 2;
        public static final double MAX_VISION_AGE_SECONDS = 0.5;
        public static final int TELEMETRY_PERIOD_LOOPS = 5;

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

        // Hub positions derived from the 2026-rebuilt-welded AprilTag layout.
        // Blue hub core tags (18-21, 24-27) span X=4.02–5.23, Y=3.43–4.64 → center ≈ (4.63, 4.03)
        // Red  hub core tags ( 2- 5,  8-11) span X=11.31–12.52, Y=3.43–4.64 → center ≈ (11.92, 4.03)
        public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.63, 4.03, new Rotation2d());
        public static final Pose2d RED_HUB_POSE  = new Pose2d(11.92, 4.03, new Rotation2d());

        // Field zone boundaries (X-axis, meters).
        // Field runs X=0 (blue DS wall) → X=16.541 (red DS wall).
        // Offensive zone for each alliance = between their driver station and their hub.
        //   Blue offensive:  X ≤ BLUE_OFFENSIVE_MAX_X  (hub far edge from blue DS ≈ 5.2 m)
        //   Red  offensive:  X ≥ RED_OFFENSIVE_MIN_X   (hub far edge from red  DS ≈ 11.3 m)
        // Outside these bounds = neutral zone or opponent defensive zone — shooter must not spin.
        public static final double BLUE_OFFENSIVE_MAX_X = 5.2;  // m — blue hub outer edge toward center
        public static final double RED_OFFENSIVE_MIN_X  = 11.3; // m — red  hub outer edge toward center

        public static final Pose2d HP_STATION_POSE = new Pose2d(1.5, 7.5, Rotation2d.fromDegrees(180)); // TODO: Update with actual 2026 game positions
        public static final Pose2d TRENCH_POSE = new Pose2d(2.5, 2.0, Rotation2d.fromDegrees(0)); // TODO: Update with actual 2026 game positions
        public static final Pose2d DEPOT_POSE = new Pose2d(14.0, 2.0, Rotation2d.fromDegrees(180)); // TODO: Update with actual 2026 game positions
        public static final Pose2d OUTPOST_POSE = new Pose2d(8.27, 0.5, Rotation2d.fromDegrees(90)); // TODO: Update with actual 2026 game positions
        public static final Pose2d TOWER_POSE = new Pose2d(8.27, 7.5, Rotation2d.fromDegrees(270)); // TODO: Update with actual 2026 game positions
    }
}
