package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Result;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.DriveTrain.*;
import static frc.robot.Constants.Auto.*;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class DriveTrain extends SubsystemBase{

    //enums
    public enum DriveMode {
        ARCADE,
        TANK,
    }
    public enum OrientationMode{ ROBOT_ORIENTED, FIELD_ORIENTED}

    //hardware
    private final SparkMax leftMotorLead = new SparkMax(LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax rightMotorLead = new SparkMax(RIGHT_MOTOR_PORT, MotorType.kBrushless);

    private final SparkMax leftMotorFollow = new SparkMax(LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax rightMotorFollow = new SparkMax(RIGHT_MOTOR_PORT, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftMotorLead.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotorLead.getEncoder();

    AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private final DifferentialDrive driver = new DifferentialDrive(leftMotorLead, rightMotorLead);

    //math
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.55245); //make constant 21 3/4
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
                gyro.getRotation2d(), 
                leftEncoder.getPosition(), 
                rightEncoder.getPosition(), 
                new Pose2d(5.0, 13.5, new Rotation2d()) //sample starting position
    );

    private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics, getHeading(), leftEncoder.getPosition()*WHEEL_CIRCUMFERENCE_METERS, rightEncoder.getPosition()*WHEEL_CIRCUMFERENCE_METERS, getPose());
    
    private Field2d field = new Field2d();

    //pathplanner
    RobotConfig robotConfig;
    LTVUnicycleController controller = new LTVUnicycleController(VecBuilder.fill(0.0625, 0.125, 2.0), VecBuilder.fill(1.0, 2.0), 0.02, 9);
    PPLTVController ltvController = new PPLTVController(0.02);
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    //STATE
    private DriveMode driveMode = DriveMode.ARCADE;
    private OrientationMode orientationMode = OrientationMode.ROBOT_ORIENTED;
    private Boolean visionEnabled = false;

    //DEPENDENCIES DONT DELETE
    //private VisionSubsystem visionSubsystem = null;

    private DifferentialDriveOdometry poseOdometry = new DifferentialDriveOdometry(gyroAngle, leftDistance, rightDistance, initialPoseMeters);

    //trajectory follower change with pathplanner values
    /*
     * The code example below initializes the LTV Unicycle Controller with qelems of 0.0625 m in X, 0.125 m in Y, and 2 radians in heading; 
     * relems of 1 m/s of linear velocity, and 2 rad/sec angular velocity; dt of 20 ms; and maxVelocity of 9 m/s.


     */

    public DriveTrain(){ //add vision subsystem here later
        
        rightMotorLead.setInverted(true); // common on FRC Robots


    }

    @Override
    public void periodic(){
  
    }
    
    public Command driveCommand(DoubleSupplier fnX,DoubleSupplier fnY){
        return new RunCommand(() -> this.drive(fnX.getAsDouble(),fnY.getAsDouble()));
    }

    public void toggleDriveMode(){
        this.driveMode = switch (this.driveMode){
            // if compiler complains about -> switch with :, -> is a newer syntax
            case ARCADE -> DriveMode.TANK;
            case TANK -> DriveMode.ARCADE;
        };
    }
    public void setDriveMode(DriveMode mode){
        this.driveMode = mode;
    }
    public DriveMode getDriveMode(){
        return this.driveMode;
    }

    

    // Drive Methods
    // x,y = controller joystick axes x,y
    // abstract drive method
    public void drive(double x,double y){
        switch (this.driveMode){
            case ARCADE -> arcadeDrive(x,y);
            case TANK -> tankDrive(x,y);
        }
    }
    private void arcadeDrive(double x,double y){
        // speed = x ; turn = y
        this.driver.arcadeDrive(-x,y);
    }

    private void tankDrive(double x, double y){
        // leftSpeed = x ; rightSpeed = y
        this.driver.tankDrive(-x,-y);
    }

    private void stop(){

    }

    private double applyDeadband(double value, double deadband){

    }

    private void setOrientationMode(OrientationMode mode){

    }

    // Testing
    public static List<Result> testDriveTrain(){
        return List.of(Result.pass("dummy test"));
    }

    //commands
    private Command teleopArcadeCommand(DoubleSupplier fwd, DoubleSupplier rot){

    }

    private Command teleopTankCommand(DoubleSupplier left, DoubleSupplier right){

    }

    private Command fieldOrientedArcadeCommand(DoubleSupplier fwd, DoubleSupplier rot){

    }


    //Odometry
    private Pose2d getPose(){
        //Pose2d pose = new Pose2d(0, 0, new Rotation2d(0.0));

        return odometry.getPoseMeters();
    }

    private void resetPose(Pose2d pose){
        System.out.println(pose);
        odometry.resetPosition(gyro.getRotation2d(), getPosition(), pose);
    }

    //reset pose test
    private static double getAnalogGyroAngle(int handle){
        handle = 0;
    }


    private Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle()); // Inverted for CCW positive
    }

    private void resetGyro() {
        gyro.reset();
    }

    private double getLeftDistanceMeters(){
        return leftEncoder.getPosition();
    }

   private double getRightDistanceMeters(){
        return rightEncoder.getPosition();
    }

    private DifferentialDriveWheelSpeeds getWheelSpeeds(){
        //find velocity of both encoders
    }

    private static double DifferentialDriveOdometry(Rotation2d gyroAngle, Distance leftDistance, Distance rightDistance){
        //gyroAngle = gyro.getAnalogGyroAngle();
        //gyroAngle = gyro.getAngle();
    }
    //poseOdometry = new DifferentialDriveOdometry();

    //chassis
    private ChassisSpeeds getRobotRelativeSpeeds(){
        return getSpeeds(); //idk are they the same thing??
    }

    private ChassisSpeeds getSpeeds(){
        return kinematics.toChassisSpeeds(getSpeeds());
    }

    private void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds){
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose.getRotation2d()));
    }

    private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        //????
    }

    //pathplanner
    //set up custom logging to add the current path to a field 2d widget
    private void addPath(){
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    

    //configuration for autobuilder for pathplanner
    try {
        robotConfig = RobotConfig.fromGUISettings();

        //configure autobuilder
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getSpeeds,
            this::driveRobotRelative,
            ltvController,
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    } catch (Exception e){
        DriverStation.reportError("failed to load pathplanner", e.getStackTrace());
    }
}

    private Command getAutoCommand (String autoName){
         //add return
    }

    private DifferentialDriveWheelSpeeds setSpeedsVoltage(DifferentialDriveWheelSpeeds speeds){
         //add return
    }

    private DifferentialDriveWheelSpeeds setSpeedsOpenLoop(DifferentialDriveWheelSpeeds speeds){
        //add return
    }


    //vision
    private void updateVisionMeasurements(){
        
    }

    private Optional<Pose2d> getVisionSeededPose(){
        
    }

    private Boolean setVisionEnabled(Boolean enabled){
        return enabled;
    }

    //telemetry
    private void updateTelemetry(){
    }

    //lifecycle
    private void configureMotors(){
        
    }

}

