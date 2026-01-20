package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // Motors
    private final CANSparkMax shooterMotor;
    private final CANSparkMax feederMotor;

    // CAN IDs (CHANGE THESE TO MATCH YOUR ROBOT)
    private static final int SHOOTER_MOTOR_ID = 1;
    private static final int FEEDER_MOTOR_ID = 2;

    // Motor speeds
    private static final double SHOOTER_SPEED = 0.9; // Shoots the ball
    private static final double FEEDER_SPEED = 0.6;  // Feeds ball into shooter

    public Shooter() {
        shooterMotor = new CANSparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        feederMotor = new CANSparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

        // Reset to safe defaults
        shooterMotor.restoreFactoryDefaults();
        feederMotor.restoreFactoryDefaults();

        // Brake helps stop faster when released
        shooterMotor.setIdleMode(IdleMode.kBrake);
        feederMotor.setIdleMode(IdleMode.kBrake);

        // Optional safety limits
        shooterMotor.setSmartCurrentLimit(40);
        feederMotor.setSmartCurrentLimit(30);
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
