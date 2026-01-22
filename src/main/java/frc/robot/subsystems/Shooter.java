package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // Motors
    private final SparkMax shooterMotor;
    private final SparkMax feederMotor;

    public Shooter(int nominalVoltage) {
        shooterMotor = new SparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

        // Optional safety limits
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(NOMINAL_VOLTAGE);
        config.smartCurrentLimit(STALL_LIMIT);

        config.idleMode(IdleMode.kBrake);
        shooterMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        feederMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    /** Spins the shooter wheel to launch the ball */
    public void startShooter() {
        shooterMotor.set(SHOOTER_SPEED);
    }

    /** Stops the shooter wheel */
    public void stopShooter() {
        shooterMotor.stopMotor();
    }

    /** Feeds the ball into the shooter */
    public void startFeeder() {
        feederMotor.set(FEEDER_SPEED);
    }

    /** Stops feeding */
    public void stopFeeder() {
        feederMotor.stopMotor();
    }

    /**
     * Shoots a ball:
     * - Shooter spins up
     * - Feeder pushes ball in
     *
     * NOTE: In a real robot, you usually want
     * a short delay before starting the feeder.
     */
    public void shoot() {
        startShooter();
        startFeeder();
    }

    /** Stops everything immediately */
    public void stopAll() {
        stopShooter();
        stopFeeder();
    }

    @Override
    public void periodic() {
        // Use for diagnostics if needed
    }
}
