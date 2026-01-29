package frc.robot.subsystems;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Result;

import static frc.robot.Constants.DriveTrainConstants.*;

import java.util.List;
import java.util.function.DoubleSupplier;
public class DriveTrain extends SubsystemBase{
    public enum DriveMode {
        ARCADE,
        TANK,
    }
    private DriveMode driveMode = DriveMode.ARCADE;

    private final MotorController leftMotor = new PWMSparkMax(LEFT_MOTOR_PORT);
    private final MotorController rightMotor = new PWMSparkMax(RIGHT_MOTOR_PORT);

    private final DifferentialDrive driver = new DifferentialDrive(leftMotor, rightMotor);

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
}

