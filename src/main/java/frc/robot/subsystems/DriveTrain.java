package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;


public class DriveTrain extends SubsystemBase {

    public enum DriveMode {
        ARCADE,
        TANK
    }

    public enum OrientationMode { 
        ROBOT_ORIENTED, 
        FIELD_ORIENTED 
    }

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

    // Odometry
    
    private final DifferentialDriveOdometry poseOdometry = new DifferentialDriveOdometry(
        gyro.getRotation2d(), 
        getLeftDistanceMeters(), 
        getRightDistanceMeters(), 
        new Pose2d()
    );

    // Pose estimator — same as poseOdometry but also accepts vision measurements.
    private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics, getHeading(), 
        getLeftDistanceMeters(), 
        getRightDistanceMeters(), 
        new Pose2d()
    );

    private final Field2d field = new Field2d();

    // PathPlanner
    private RobotConfig robotConfig;
    private final PPLTVController ltvController = new PPLTVController(0.02); //TODO do we need a different LTV constructor?

    // State
    private DriveMode driveMode = DriveMode.ARCADE;
    private OrientationMode orientationMode = OrientationMode.ROBOT_ORIENTED;
    private boolean visionEnabled = false;

    // Dependencies
    private final VisionSubsystem visionSubsystem;

    public DriveTrain(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        configureMotors();
        configurePathPlanner();
    }

    @Override
    public void periodic() {
        poseOdometry.update(gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
        poseEstimator.update(gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
        if (visionEnabled) {
            updateVisionMeasurements();
        }
        updateTelemetry();
    }

    // ---- Motor Configuration ----

    // TODO look in to deprecated modes
    private void configureMotors() {
        // Invert right side so positive output = forward on both sides
        SparkMaxConfig rightLeadConfig = new SparkMaxConfig();
        rightLeadConfig.inverted(true);
        rightMotorLead.configure(rightLeadConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follow motors mirror their respective leads
        SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
        leftFollowConfig.follow(leftMotorLead);
        leftMotorFollow.configure(leftFollowConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
        rightFollowConfig.follow(rightMotorLead);
        rightMotorFollow.configure(rightFollowConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(),
                e.getStackTrace());
        }
    }

    // ---- Pose Initialization ----

    /**
     * Set the robot's starting pose before autonomous.
     * Priority: (1) vision AprilTag fix, (2) PathPlanner auto starting pose, (3) field origin.
     */
    public void initializePose(Command autoCommand) {
        Pose2d initialPose = null;

        // Use vision if it can see AprilTags right now
        if (visionSubsystem != null) {
            Optional<VisionMeasurement> visionMeasurement = visionSubsystem.getBestVisionMeasurement();
            if (visionMeasurement.isPresent()) {
                initialPose = visionMeasurement.get().estimatedPose();
            }
        }

        // Fall back to the starting pose defined in the selected PathPlanner auto
        if (initialPose == null && autoCommand instanceof PathPlannerAuto ppAuto) {
            initialPose = ppAuto.getStartingPose();
        }

        // Last resort: field origin 
        if (initialPose == null) {
            initialPose = new Pose2d();
        }

        resetPose(initialPose);
    }

    // ---- Teleop Drive Commands ----

    public Command teleopArcadeCommand(DoubleSupplier fwd, DoubleSupplier rot) {
        return new RunCommand(() -> arcadeDrive(
            applyDeadband(fwd.getAsDouble(), JOYSTICK_DEADBAND),
            applyDeadband(rot.getAsDouble(), JOYSTICK_DEADBAND)
        ), this);
    }

    public Command teleopTankCommand(DoubleSupplier left, DoubleSupplier right) {
        return new RunCommand(() -> tankDrive(
            applyDeadband(left.getAsDouble(),  JOYSTICK_DEADBAND),
            applyDeadband(right.getAsDouble(), JOYSTICK_DEADBAND)
        ), this);
    }

    //TODO do we need to add a robotOriented command?
    public Command fieldOrientedArcadeCommand(DoubleSupplier fwd, DoubleSupplier rot) {
        return new RunCommand(() -> {
            double forward   = applyDeadband(fwd.getAsDouble(), JOYSTICK_DEADBAND);
            double rotation  = applyDeadband(rot.getAsDouble(), JOYSTICK_DEADBAND);
            double heading   = getPose().getRotation().getRadians();
            // Rotate driver inputs to field frame
            double fieldFwd  =  forward  * Math.cos(heading) + rotation * Math.sin(heading);
            double fieldRot  = -forward  * Math.sin(heading) + rotation * Math.cos(heading);
            arcadeDrive(fieldFwd, fieldRot);
        }, this);
    }

    //TODO are these drive commands needed if we have those above?

    /** Generic drive command for use from RobotContainer (x = fwd, y = turn). */
    public Command driveCommand(DoubleSupplier fnX, DoubleSupplier fnY) {
        return new RunCommand(() -> drive(fnX.getAsDouble(), fnY.getAsDouble()), this);
    }

    public void toggleDriveMode() {
        this.driveMode = switch (this.driveMode) {
            case ARCADE -> DriveMode.TANK;
            case TANK   -> DriveMode.ARCADE;
        };
    }

    public void setDriveMode(DriveMode mode) {
        this.driveMode = mode;
    }

    public DriveMode getDriveMode() {
        return this.driveMode;
    }

    public void drive(double x, double y) {
        switch (this.driveMode) {
            case ARCADE -> arcadeDrive(x, y);
            case TANK   -> tankDrive(x, y);
        }
    }

    private void arcadeDrive(double x, double y) {
        driver.arcadeDrive(-x, y);
    }

    private void tankDrive(double x, double y) {
        driver.tankDrive(-x, -y);
    }

    public void stop() {
        driver.stopMotor();
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }

    public void setOrientationMode(OrientationMode mode) {
        this.orientationMode = mode;
    }

    public OrientationMode getOrientationMode() {
        return this.orientationMode;
    }

    // Testing
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

    // odometery and pose

    public Pose2d getPose() {
        return poseOdometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        poseOdometry.resetPosition(
            gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
        poseEstimator.resetPosition(
            gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle()); // Inverted for CCW positive
    }

    public void resetGyro() {
        gyro.reset();
    }

    /** Left wheel distance in meters, accounting for gear ratio. */
    public double getLeftDistanceMeters() {
        return leftEncoder.getPosition() * WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
    }

    /** Right wheel distance in meters, accounting for gear ratio. */
    public double getRightDistanceMeters() {
        return rightEncoder.getPosition() * WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
    }

    /**
     * Wheel speeds in m/s.
     * SparkMax encoder velocity is RPM → convert: RPM * circumference / gearRatio / 60
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity()  * WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO / 60.0,
            rightEncoder.getVelocity() * WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO / 60.0
        );
    }

    // Vision 

    private void updateVisionMeasurements() {
        if (visionSubsystem == null) return;
        visionSubsystem.getBestVisionMeasurement().ifPresent(measurement ->
            poseEstimator.addVisionMeasurement(
                measurement.estimatedPose(),
                measurement.timestampSeconds(),
                measurement.standardDeviations()
            )
        );
    }

    public Optional<Pose2d> getVisionSeededPose() {
        if (visionSubsystem == null) return Optional.empty();
        return visionSubsystem.getBestVisionMeasurement()
            .map(VisionMeasurement::estimatedPose);
    }

    public boolean setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
        return enabled;
    }

    // PathPlanner Aut

    public Command getAutoCommand(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    // Telemetry

    private void updateTelemetry() {
        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("DriveTrain/LeftDistMeters",  getLeftDistanceMeters());
        SmartDashboard.putNumber("DriveTrain/RightDistMeters", getRightDistanceMeters());
        SmartDashboard.putNumber("DriveTrain/HeadingDeg",      getHeading().getDegrees());
        SmartDashboard.putString("DriveTrain/DriveMode",       driveMode.toString());

        //TODO do we need to add more telemetry stuff to dashboard?
    }
}
