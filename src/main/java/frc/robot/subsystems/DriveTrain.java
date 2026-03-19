package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Result;
import frc.robot.VisionMeasurement;


import com.studica.frc.AHRS;

import static frc.robot.Constants.DriveTrainConstants.*;
import static frc.robot.Constants.Auto.*;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class DriveTrain extends SubsystemBase {

    // Hardware
    private final SparkMax leftMotorLead   = new SparkMax(LEFT_LEAD_CAN_ID,   MotorType.kBrushless);
    private final SparkMax rightMotorLead  = new SparkMax(RIGHT_LEAD_CAN_ID,  MotorType.kBrushless);
    private final SparkMax leftMotorFollow = new SparkMax(LEFT_FOLLOW_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightMotorFollow= new SparkMax(RIGHT_FOLLOW_CAN_ID,MotorType.kBrushless);

    private final RelativeEncoder leftEncoder  = leftMotorLead.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotorLead.getEncoder();

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private final DifferentialDrive driver = new DifferentialDrive(leftMotorLead, rightMotorLead);

    // Kinematics
    private final DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    // Pose estimator — fuses encoder + gyro odometry with vision measurements.
    private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics, getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        new Pose2d()
    );

    private final Field2d field = new Field2d();

    // PathPlanner
    private RobotConfig robotConfig;
    private PPLTVController ltvController;
    private static final String PPLTV_PREFIX = "DriveTrain/PPLTV/";
    private double lastQx = PPLTV_Q_X;
    private double lastQy = PPLTV_Q_Y;
    private double lastQtheta = PPLTV_Q_THETA;
    private double lastRvel = PPLTV_R_VEL;
    private double lastRomega = PPLTV_R_OMEGA;
    private double lastDt = PPLTV_DT;
    private double lastMaxVelocity = PPLTV_MAX_VELOCITY;

    // Dependencies
    private final Vision visionSubsystem;

    private int telemetryLoopCounter = 0;

    public DriveTrain(Vision visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        configureMotors();
        initLtvTuning();
        configurePathPlanner();
        SmartDashboard.putData("Field", field); // INEFF-03: register once, not every loop
    }

    @Override
    public void periodic() {
        poseEstimator.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());

        if (visionSubsystem != null && visionSubsystem.isAnyVisionAvailable()) {
            updateVisionMeasurements();
        }
        refreshLtvControllerFromDashboard();
        if (++telemetryLoopCounter >= TELEMETRY_PERIOD_LOOPS) {
            telemetryLoopCounter = 0;
            updateTelemetry();
        }
    }

    // ---- Motor Configuration ----

    private void configureMotors() {
        // Right lead — inverted so positive output = forward on both sides.
        // Encoder conversion factors are set on the primary (built-in hall) encoder so that
        // getPosition() returns meters and getVelocity() returns m/s directly.
        SparkMaxConfig rightLeadConfig = new  SparkMaxConfig();
            rightLeadConfig.inverted(true);
            rightLeadConfig.idleMode(IdleMode.kCoast);
            rightLeadConfig.smartCurrentLimit(CURRENT_LIMIT);
            rightLeadConfig.openLoopRampRate(0.25); //0.15
            rightLeadConfig.encoder.positionConversionFactor(WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO);
            rightLeadConfig.encoder.velocityConversionFactor(WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO / 60.0);
            rightMotorLead.configure(   rightLeadConfig,
                                        ResetMode.kResetSafeParameters,
                                        PersistMode.kPersistParameters);

        // Left lead
        SparkMaxConfig leftLeadConfig = new SparkMaxConfig();
            leftLeadConfig.inverted(false);
            leftLeadConfig.idleMode(IdleMode.kCoast);
            leftLeadConfig.smartCurrentLimit(CURRENT_LIMIT);
            leftLeadConfig.openLoopRampRate(0.25); //0.15
            leftLeadConfig.encoder.positionConversionFactor(WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO);
            leftLeadConfig.encoder.velocityConversionFactor(WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO / 60.0);
            leftMotorLead.configure(leftLeadConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follow motors mirror their respective leads.
        // .follow() only sets which motor to mirror — each follower enforces its own
        // current limit and idle mode independently.
        // Status frame periods are slowed significantly since no data is ever read
        // from followers, reducing unnecessary CAN bus traffic.
        SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
            leftFollowConfig.follow(leftMotorLead);
            leftFollowConfig.inverted(false);
            leftFollowConfig.idleMode(IdleMode.kCoast);
            leftFollowConfig.smartCurrentLimit(CURRENT_LIMIT);
            leftFollowConfig.openLoopRampRate(0.25); //0.15
            leftFollowConfig.signals
                .appliedOutputPeriodMs(500)
                .primaryEncoderPositionPeriodMs(500)
                .primaryEncoderVelocityPeriodMs(500)
                .outputCurrentPeriodMs(500);
            leftMotorFollow.configure(  leftFollowConfig,
                                        ResetMode.kResetSafeParameters,
                                        PersistMode.kPersistParameters);

        SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
            rightFollowConfig.follow(rightMotorLead);
            rightFollowConfig.idleMode(IdleMode.kCoast);
            rightFollowConfig.inverted(true);
            rightFollowConfig.smartCurrentLimit(CURRENT_LIMIT);
            rightFollowConfig.openLoopRampRate(0.25); //0.15
            rightFollowConfig.signals
                .appliedOutputPeriodMs(500)
                .primaryEncoderPositionPeriodMs(500)
                .primaryEncoderVelocityPeriodMs(500)
                .outputCurrentPeriodMs(500);
            rightMotorFollow.configure( rightFollowConfig,
                                        ResetMode.kResetSafeParameters,
                                        PersistMode.kPersistParameters);

        // Reset encoder positions to zero
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    // ---- PathPlanner Configuration ----

    private void configurePathPlanner() {
        PathPlannerLogging.setLogActivePathCallback(
            poses -> field.getObject("path").setPoses(poses)
        );

        try {
            robotConfig = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                ltvController,
                robotConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(),
                e.getStackTrace());
        }
    }

    private void initLtvTuning() {
        SmartDashboard.putNumber(PPLTV_PREFIX + "Qx", PPLTV_Q_X);
        SmartDashboard.putNumber(PPLTV_PREFIX + "Qy", PPLTV_Q_Y);
        SmartDashboard.putNumber(PPLTV_PREFIX + "Qtheta", PPLTV_Q_THETA);
        SmartDashboard.putNumber(PPLTV_PREFIX + "Rvel", PPLTV_R_VEL);
        SmartDashboard.putNumber(PPLTV_PREFIX + "Romega", PPLTV_R_OMEGA);
        SmartDashboard.putNumber(PPLTV_PREFIX + "Dt", PPLTV_DT);
        SmartDashboard.putNumber(PPLTV_PREFIX + "MaxVelocity", PPLTV_MAX_VELOCITY);
        rebuildLtvControllerFromDashboard();
    }

    private void refreshLtvControllerFromDashboard() {
        if (!DriverStation.isTest()) {
            return;
        }

        double qx = SmartDashboard.getNumber(PPLTV_PREFIX + "Qx", PPLTV_Q_X);
        double qy = SmartDashboard.getNumber(PPLTV_PREFIX + "Qy", PPLTV_Q_Y);
        double qtheta = SmartDashboard.getNumber(PPLTV_PREFIX + "Qtheta", PPLTV_Q_THETA);
        double rvel = SmartDashboard.getNumber(PPLTV_PREFIX + "Rvel", PPLTV_R_VEL);
        double romega = SmartDashboard.getNumber(PPLTV_PREFIX + "Romega", PPLTV_R_OMEGA);
        double dt = SmartDashboard.getNumber(PPLTV_PREFIX + "Dt", PPLTV_DT);
        double maxVel = SmartDashboard.getNumber(PPLTV_PREFIX + "MaxVelocity", PPLTV_MAX_VELOCITY);

        if (qx != lastQx || qy != lastQy || qtheta != lastQtheta
            || rvel != lastRvel || romega != lastRomega
            || dt != lastDt || maxVel != lastMaxVelocity) {
            // BUG-07: Only rebuild the controller object. AutoBuilder.configure() was
            // called once in the constructor and must not be called again — repeated
            // calls register duplicate command factories. PathPlanner reads ltvController
            // by field reference on each path invocation, so reassigning the field here
            // is sufficient for the new gains to take effect on the next auto run.
            rebuildLtvController(qx, qy, qtheta, rvel, romega, dt, maxVel);
        }
    }

    private void rebuildLtvControllerFromDashboard() {
        rebuildLtvController(
            SmartDashboard.getNumber(PPLTV_PREFIX + "Qx", PPLTV_Q_X),
            SmartDashboard.getNumber(PPLTV_PREFIX + "Qy", PPLTV_Q_Y),
            SmartDashboard.getNumber(PPLTV_PREFIX + "Qtheta", PPLTV_Q_THETA),
            SmartDashboard.getNumber(PPLTV_PREFIX + "Rvel", PPLTV_R_VEL),
            SmartDashboard.getNumber(PPLTV_PREFIX + "Romega", PPLTV_R_OMEGA),
            SmartDashboard.getNumber(PPLTV_PREFIX + "Dt", PPLTV_DT),
            SmartDashboard.getNumber(PPLTV_PREFIX + "MaxVelocity", PPLTV_MAX_VELOCITY)
        );
    }

    private void rebuildLtvController(
        double qx,
        double qy,
        double qtheta,
        double rvel,
        double romega,
        double dt,
        double maxVelocity
    ) {
        if (dt <= 0 || maxVelocity <= 0) {
            DriverStation.reportError("Invalid PPLTV tuning values (dt/maxVelocity)", false);
            return;
        }
        ltvController = new PPLTVController(
            VecBuilder.fill(qx, qy, qtheta),
            VecBuilder.fill(rvel, romega),
            dt,
            maxVelocity
        );
        lastQx = qx;
        lastQy = qy;
        lastQtheta = qtheta;
        lastRvel = rvel;
        lastRomega = romega;
        lastDt = dt;
        lastMaxVelocity = maxVelocity;
    }

    // ---- Pose Initialization ----

    /**
     * Seeds the pose estimator with the best available pose.
     * Priority: (1) vision AprilTag fix, (2) PathPlanner auto starting pose, (3) field origin.
     *
     * Call this at auto init (pass the selected auto command) and at teleop init (pass null).
     *
     * Vision staleness: uses a generous 5 s window at init time — a slightly old fix is far
     * better than defaulting to field origin. The in-match window (0.5 s) is stricter because
     * stale data can confuse active path following.
     *
     * Red alliance: PathPlanner stores paths in blue-alliance coordinates. getStartingPose()
     * returns those raw coordinates, so we flip manually when the DS reports red alliance.
     */
    public void initializePose(Command autoCommand) {
        Pose2d initialPose = null;

        // Priority 1: vision — accept measurements up to POSE_INIT_MAX_VISION_AGE_SECONDS old.
        if (visionSubsystem != null) {
            Optional<VisionMeasurement> visionMeasurement = visionSubsystem.getBestVisionMeasurement();
            if (visionMeasurement.isPresent()) {
                double ageSec = Timer.getFPGATimestamp() - visionMeasurement.get().timestampSeconds();
                if (ageSec <= POSE_INIT_MAX_VISION_AGE_SECONDS) {
                    initialPose = visionMeasurement.get().estimatedPose();
                }
            }
        }

        // Priority 2: PathPlanner auto starting pose (carries position + heading).
        // Flip X and heading for red alliance — getStartingPose() is always blue-side coords.
        if (initialPose == null && autoCommand instanceof PathPlannerAuto ppAuto) {
            initialPose = ppAuto.getStartingPose();
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                initialPose = flipForRedAlliance(initialPose);
            }
        }

        // Priority 3: field origin fallback — warn the drive team loudly.
        if (initialPose == null) {
            initialPose = new Pose2d();
            DriverStation.reportWarning(
                "DriveTrain: pose could not be initialized from vision or auto starting pose. " +
                "Defaulting to field origin — heading will be incorrect if robot is not " +
                "facing the field-forward direction.", false);
            SmartDashboard.putBoolean("DriveTrain/HeadingInitialized", false);
        } else {
            SmartDashboard.putBoolean("DriveTrain/HeadingInitialized", true);
        }

        resetPose(initialPose);
    }

    /**
     * Mirrors a blue-alliance pose to its red-alliance equivalent.
     * Flips X across the field center; Y is unchanged; heading is reflected
     * across the vertical axis (180° - angle).
     */
    private static Pose2d flipForRedAlliance(Pose2d pose) {
        final double FIELD_LENGTH_METERS = 16.541;
        return new Pose2d(
            FIELD_LENGTH_METERS - pose.getX(),
            pose.getY(),
            Rotation2d.fromDegrees(180.0).minus(pose.getRotation())
        );
    }

    // ---- Teleop Drive ----

    /**
     * Teleop drive command. leftY = left wheel speed, rightY = right wheel speed.
     * boost (0.0–1.0) interpolates the speed scale from SPEED_SCALE to 1.0.
     */
    public Command teleopDriveCommand(DoubleSupplier leftYSupplier, DoubleSupplier rightYSupplier, DoubleSupplier boostSupplier) {
        return new RunCommand(
            () -> {
                double scale = SPEED_SCALE + (1.0 - SPEED_SCALE) * boostSupplier.getAsDouble();
                drive(scale * leftYSupplier.getAsDouble(), scale * rightYSupplier.getAsDouble());
            },
            this
        );
    }

    /** Applies deadband and drives in robot-relative tank mode. */
    public void drive(double leftY, double rightY) {
        driver.tankDrive(
            MathUtil.applyDeadband(leftY, JOYSTICK_DEADBAND),
            MathUtil.applyDeadband(rightY, JOYSTICK_DEADBAND)
        );
    }

    public void stop() {
        driver.stopMotor();
    }

    // ---- Testing ----

    public static List<Result> testDriveTrain() {
        return List.of(Result.pass("dummy test"));
    }

    // ---- PathPlanner Driving ----

    /**
     * Called by PathPlanner every loop tick during autonomous.
     * Converts desired ChassisSpeeds to left/right wheel percent outputs.
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(targetSpeeds);
        wheelSpeeds.desaturate(MAX_MODULE_SPEED);
        driver.tankDrive(
            wheelSpeeds.leftMetersPerSecond  / MAX_MODULE_SPEED,
            wheelSpeeds.rightMetersPerSecond / MAX_MODULE_SPEED,
            false
        );
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation())
        );
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    // ---- Odometry and Pose ----

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
            getHeading(), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle()); // Inverted for CCW positive
    }

    public void resetGyro() {
        gyro.reset();
    }

    /** Left wheel distance in meters. Conversion factor is set in motor config. */
    public double getLeftDistanceMeters() {
        return leftEncoder.getPosition();
    }

    /** Right wheel distance in meters. Conversion factor is set in motor config. */
    public double getRightDistanceMeters() {
        return rightEncoder.getPosition();
    }

    /** Wheel speeds in m/s. Conversion factor is set in motor config. */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity(),
            rightEncoder.getVelocity()
        );
    }

    // ---- Vision ----

    private void updateVisionMeasurements() {
        if (visionSubsystem == null) return;
        visionSubsystem.getBestVisionMeasurementIfFresh().ifPresent(measurement ->
            poseEstimator.addVisionMeasurement(
                measurement.estimatedPose(),
                measurement.timestampSeconds(),
                measurement.standardDeviations()
            )
        );
    }

    public Optional<Pose2d> getVisionSeededPose() {
        if (visionSubsystem == null) return Optional.empty();
        return visionSubsystem.getBestVisionMeasurementIfFresh()
            .map(VisionMeasurement::estimatedPose);
    }

    // ---- PathPlanner Auto ----

    public Command getAutoCommand(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    // ---- Telemetry ----

    private void updateTelemetry() {
        field.setRobotPose(getPose());
        SmartDashboard.putNumber("DriveTrain/LeftDistMeters",  getLeftDistanceMeters());
        SmartDashboard.putNumber("DriveTrain/RightDistMeters", getRightDistanceMeters());
        SmartDashboard.putNumber("DriveTrain/HeadingDeg",      getHeading().getDegrees());
    }
}
