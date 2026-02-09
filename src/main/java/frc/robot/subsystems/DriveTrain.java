package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Result;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.DriveTrain.*;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class DriveTrain extends SubsystemBase{
    public enum DriveMode {
        ARCADE,
        TANK,
    }
    private DriveMode driveMode = DriveMode.ARCADE;

    private final SparkMax leftMotor = new SparkMax(LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(RIGHT_MOTOR_PORT, MotorType.kBrushless);

    private final DifferentialDrive driver = new DifferentialDrive(leftMotor, rightMotor);

    //reset pose test CONFUSION
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    //private final Rotation2d gyroAngle = 
    private DifferentialDriveOdometry poseOdometry = new DifferentialDriveOdometry(Rotation2d gyroAngle, Distance leftDistance, Distance rightDistance, Pose2d initialPoseMeters);

    public DriveTrain(){
        
        rightMotor.setInverted(true); // common on FRC Robots
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

    // Testing
    public static List<Result> testDriveTrain(){
        return List.of(Result.pass("dummy test"));
    }

    //get pose test

    public Pose2d getPose(){
        Pose2d pose = new Pose2d(0, 0, new Rotation2d(0.0));

        return pose;
    }

    //reset pose test
    public static double getAnalogGyroAngle(int handle){
        handle = 0;
    }

    //get heading
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle()); // Inverted for CCW positive
    }

    public void resetGyro() {
        gyro.reset();
    }

    //odometry class
    public static double DifferentialDriveOdometry(Rotation2d gyroAngle, Distance leftDistance, Distance rightDistance){
        //gyroAngle = .getAnalogGyroAngle();
    }
    //poseOdometry = new DifferentialDriveOdometry();

    public void resetPose(){

    } 
}

