package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // Motors
    private final SparkMax shooterMotor;
    private final SparkMax feederMotor;

    // 🔹 ADDED: PID + Encoder
    private final SparkClosedLoopController shooterPID;
    private final RelativeEncoder shooterEncoder;

    public Shooter(int nominalVoltage) {
        shooterMotor = new SparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

        // Optional safety limits
        SparkMaxConfig config = (SparkMaxConfig) new SparkMaxConfig()
        .voltageCompensation(NOMINAL_VOLTAGE)
        .smartCurrentLimit(STALL_LIMIT)
        .idleMode(IdleMode.kBrake);
        config.closedLoop.p(0).i(0).d(0);
        // config.voltageCompensation(NOMINAL_VOLTAGE);
        // config.smartCurrentLimit(STALL_LIMIT);

        // config.idleMode(IdleMode.kBrake);
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

        //  ADDED: PID setup
        // shooterPID = shooterMotor.getClosedLoopController();
        // shooterEncoder = shooterMotor.getEncoder();
        // shooterPID.setIAccum(nominalVoltage);
        // shooterPID.setSetpoint(nominalVoltage, null);
        // shooterPID.setP(SHOOTER_kP);
        // shooterPID.setI(SHOOTER_kI);
        // shooterPID.setD(SHOOTER_kD);
        // shooterPID.setFF(SHOOTER_kFF);
        // shooterPID.setOutputRange(-1.0, 1.0);
    }

    /** Spins the shooter wheel to launch the ball */
    public void startShooter() {
        shooterMotor.set(SHOOTER_SPEED);
    }

    /**  ADDED: Spins shooter using PID velocity control (RPM) */
    public void startShooterRPM(double rpm) {
        shooterPID.setReference(rpm, SparkMax.ControlType.kVelocity);
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

    // 🔹 ADDED: Shooter diagnostics
    public double getShooterRPM() {
        return shooterEncoder.getVelocity();
    }

    public boolean atSpeed(double targetRPM) {
        return Math.abs(targetRPM - getShooterRPM()) < SHOOTER_RPM_TOLERANCE;
    }

    @Override
    public void periodic() {
        // Use for diagnostics if needed
    }
}
