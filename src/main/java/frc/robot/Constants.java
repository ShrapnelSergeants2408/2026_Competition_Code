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

    /** Controller USB port assignments. */
    public static class OperatorConstants {
        /** Xbox controller port for the driver (drive motions only). */
        public static final int DRIVER_CONTROLLER_PORT = 0;
        /** Xbox controller port for the operator (intake, shooting, distance overrides). */
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    /** Reserved for future climber implementation. */
    public static class ClimberConstants {}

    /** Constants for the differential drive subsystem. */
    public static class DriveTrainConstants {

        // CAN IDs
        /** CAN ID of the left-side lead SparkMax motor controller. */
        public static final int LEFT_LEAD_CAN_ID = 20;
        /** CAN ID of the right-side lead SparkMax motor controller. */
        public static final int RIGHT_LEAD_CAN_ID = 22;
        /** CAN ID of the left-side follower SparkMax motor controller. */
        public static final int LEFT_FOLLOW_CAN_ID = 21;
        /** CAN ID of the right-side follower SparkMax motor controller. */
        public static final int RIGHT_FOLLOW_CAN_ID = 23;

        // Motor config
        /** Smart current limit (amps) applied to all four drive SparkMax controllers. */
        public static final int CURRENT_LIMIT = 60; //60   // amps

        // Pose initialization — vision measurements up to this many seconds old are
        // accepted when seeding the pose at auto/teleop init. More generous than the
        // in-match freshness window because any recent fix beats defaulting to field origin.
        /**
         * Maximum age (seconds) of a vision measurement accepted when seeding pose at
         * auto/teleop init. More generous than the in-match freshness window
         * ({@link VisionConstants#MAX_VISION_AGE_SECONDS}) because any recent fix is better
         * than defaulting to field origin.
         */
        public static final double POSE_INIT_MAX_VISION_AGE_SECONDS = 5.0;

        // Driving
        /** Joystick input below this magnitude is treated as zero (prevents drift from controller dead zones). */
        public static final double JOYSTICK_DEADBAND = 0.05;
        /** Publish telemetry to SmartDashboard every N scheduler loops (reduces dashboard update overhead). */
        public static final int TELEMETRY_PERIOD_LOOPS = 5;

        // Physical dimensions
        /** Drive motor-to-wheel gear reduction ratio (motor rotations per wheel rotation). */
        public static final double GEAR_RATIO = 8.46;
        /** Wheel diameter in meters (6-inch wheels). Used to compute encoder conversion factors. */
        public static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 inches
        /** Wheel circumference in meters — derived from {@link #WHEEL_DIAMETER_METERS}. */
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        /** Distance between left and right wheel contact patches (meters). Used by kinematics. */
        public static final double TRACK_WIDTH_METERS = 0.546;

        /** Default drive speed scale (0.0–1.0). Right trigger boost overrides up to 1.0. */
        public static final double SPEED_SCALE = 0.7; //0.5
    }

    /** Reserved for future intake implementation. */
    public static class IntakeConstants {}

    /** Constants for the Shooter and Feeder subsystems (flywheel, intake roller, trigger/hopper). */
    public static class ShooterConstants {

        // CAN IDs
        /** CAN ID of the TalonFX (Kraken X60) shooter flywheel motor controller. */
        public static final int SHOOTER_MOTOR_ID = 30;  // TalonFX (Phoenix 6) — shooter wheel
        /** CAN ID of the SparkMax intake roller motor controller. */
        public static final int INTAKE_MOTOR_ID  = 31;  // SparkMAX — intake roller (primary + secondary linked, CCW only)
        /** CAN ID of the SparkMax trigger/hopper motor controller. */
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

    /** Constants for ball-detection sensors. */
    public static class SensorConstants {
        /** roboRIO DIO port number for the ball-presence photo sensor. */
        public static final int PHOTO_SENSOR_DIO_PORT = 1;
        /**
         * Set to {@code true} once the photo sensor is physically installed on the robot.
         * While {@code false}, {@code Feeder.hasBall()} always returns {@code false} and
         * no DIO port is allocated.
         */
        public static final boolean PHOTO_SENSOR_ENABLED = false;   // disabled until installed
        /** Set to {@code true} if the sensor output is inverted (beam-break vs reflective). Verify on bench. */
        public static final boolean PHOTO_SENSOR_INVERTED = false;  // TODO: verify polarity on bench
    }

    /** Constants for PathPlanner autonomous path following and the PPLTVController. */
    public static class Auto {
        /**
         * Maximum robot translation speed (m/s). 15 ft/s converted to SI.
         * Used as the upper bound for both PathPlanner motion constraints and the
         * PPLTVController gain-table so they stay synchronized.
         */
        public static final double MAX_ROBOT_VELOCITY_MPS = 15.0 * 0.3048; // 4.572 m/s

        /** PathPlanner maximum wheel speed (m/s) — matches {@link #MAX_ROBOT_VELOCITY_MPS}. */
        public static final double MAX_MODULE_SPEED = MAX_ROBOT_VELOCITY_MPS; // m/s
        /** PathPlanner maximum translational acceleration (m/s²). */
        public static final double MAX_ACCELERATION = 2.0; // m/s^2
        /** PathPlanner maximum rotational velocity (degrees/s). */
        public static final double MAX_ANGULAR_VELOCITY = 540.0; // deg/s
        /** PathPlanner maximum rotational acceleration (degrees/s²). */
        public static final double MAX_ANGULAR_ACCELERATION = 720.0; // deg/s^2

        // PPLTVController tuning defaults (state tolerances and control effort limits).
        // Live adjustment is available on SmartDashboard during test mode only.
        /** Control loop period (seconds) passed to the PPLTVController. Must match robot loop rate (20 ms). */
        public static final double PPLTV_DT = 0.02;
        /** Maximum velocity (m/s) used to pre-compute the LTV gain table. Match {@link #MAX_ROBOT_VELOCITY_MPS}. */
        public static final double PPLTV_MAX_VELOCITY = MAX_ROBOT_VELOCITY_MPS;
        /** Q matrix x-position state weight. Lower = less aggressive x correction. */
        public static final double PPLTV_Q_X = 0.0625;
        /** Q matrix y-position state weight. Lower = less aggressive y correction. */
        public static final double PPLTV_Q_Y = 0.125;
        /** Q matrix heading state weight. Higher = more aggressive heading correction. */
        public static final double PPLTV_Q_THETA = 0.75;
        /** R matrix velocity control effort weight. Higher = softer velocity commands. */
        public static final double PPLTV_R_VEL = 1.0;
        /** R matrix angular velocity control effort weight. Higher = softer turning commands. */
        public static final double PPLTV_R_OMEGA = 2.0;
    }

    /** Constants for the Vision subsystem — camera names, transforms, and pose estimation quality gates. */
    public static class VisionConstants {
        // Camera names (must match PhotonVision configuration)
        /** Camera name as configured in PhotonVision web UI (case-sensitive). Front-facing Pi4 + PiCam v2. */
        public static final String FRONT_CAMERA_NAME = "Front_Camera";
        /** Camera name as configured in PhotonVision web UI (case-sensitive). Rear-facing Pi5 + OV9281. */
        public static final String REAR_CAMERA_NAME = "Rear_Camera";
        /** USB driver camera name — passed to CameraServer for driver feed only; not used for pose estimation. */
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
