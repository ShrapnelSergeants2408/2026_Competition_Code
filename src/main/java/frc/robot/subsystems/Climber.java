package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    private final SparkMax climberMotor;

    public Climber() {
        // Initialize Spark Max motor
        climberMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

        // Create config
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake); // brake when stopped
        config.smartCurrentLimit(ClimberConstants.CLIMBER_CURRENT_LIMIT);
        config.voltageCompensation(ClimberConstants.NOMINAL_VOLTAGE);

        // Apply config
        climberMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    /** Move climber up */
    public void climbUp() {
        climberMotor.set(ClimberConstants.CLIMB_SPEED);
    }

    /** Move climber down */
    public void climbDown() {
        climberMotor.set(-ClimberConstants.CLIMB_SPEED);
    }

    /** Stop the climber */
    public void stop() {
        climberMotor.set(0);
    }

    @Override
    public void periodic() {
        // Optional diagnostics/logging
    }
}
