package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFx;
import com.ctre.phoenix6.hardware.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.DutyCycleOut;
public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax shooterMotor;
    private final SparkClosedLoopController shooterPID;
    private final RelativeEncoder shooterEncoder;

    
    // Motor
    private final TalonFx shooterMotor;

    public ShooterSubsystem() {
        // Instantiate TalonFX motor
        shooterMotor = new TalonFx(SHOOTER_MOTOR_ID);

        // Configure current limit
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.statorCurrentLimit.currentLimit = SHOOTER_CURRENT_LIMIT;
        shooterMotor.getConfigurator().apply(config);
    }

    // -------------------------------
    // Spin shooter at a fixed percentage of power
    // -------------------------------
    public void spinAtSpeed(double percentOutput) {
        // percentOutput: -1.0 (full reverse) to 1.0 (full forward)
        shooterMotor.setControl(DutyCycleOut.create().setDutyCycle(percentOutput));
    }

        //stops shooter
         public void stop() {
        shooterMotor.setControl(DutyCycleOut.create().setDutyCycle(0.0));
    }
}

     

       

    }
