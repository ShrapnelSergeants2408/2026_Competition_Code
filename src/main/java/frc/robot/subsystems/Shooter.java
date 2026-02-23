package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private static final String TUNING_PREFIX = "Shooter/Tuning/";

    // Motors
    private final SparkMax shooterMotor;
    private final SparkMax feederMotor;

    private final RelativeEncoder shooterEncoder;
    private final PIDController shooterPid =
        new PIDController(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD);

    private boolean pidEnabled = false;
    private double targetRpm = SHOOTER_TARGET_RPM;
    private double lastKp = SHOOTER_KP;
    private double lastKi = SHOOTER_KI;
    private double lastKd = SHOOTER_KD;

    public Shooter(double nominalVoltage) {
        shooterMotor = new SparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();

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

        SmartDashboard.putNumber(TUNING_PREFIX + "kP", SHOOTER_KP);
        SmartDashboard.putNumber(TUNING_PREFIX + "kI", SHOOTER_KI);
        SmartDashboard.putNumber(TUNING_PREFIX + "kD", SHOOTER_KD);
        SmartDashboard.putNumber(TUNING_PREFIX + "TargetRPM", SHOOTER_TARGET_RPM);
        SmartDashboard.putBoolean(TUNING_PREFIX + "UsePID", SHOOTER_USE_PID);
    }

    /** Spins the shooter wheel to launch the ball */
    public void startShooter() {
        pidEnabled = DriverStation.isTest()
            ? SmartDashboard.getBoolean(TUNING_PREFIX + "UsePID", SHOOTER_USE_PID)
            : SHOOTER_USE_PID;
        if (pidEnabled) {
            shooterPid.reset();
            if (DriverStation.isTest()) {
                updatePidFromDashboard();
            }
        } else {
            shooterMotor.set(SHOOTER_SPEED);
        }
    }

    /** Stops the shooter wheel */
    public void stopShooter() {
        pidEnabled = false;
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
        if (pidEnabled) {
            if (DriverStation.isTest()) {
                updatePidFromDashboard();
            }
            double currentRpm = shooterEncoder.getVelocity();
            double output = shooterPid.calculate(currentRpm, targetRpm);
            shooterMotor.set(MathUtil.clamp(output, -1.0, 1.0));
            SmartDashboard.putNumber(TUNING_PREFIX + "CurrentRPM", currentRpm);
        }
    }

    private void updatePidFromDashboard() {
        double kp = SmartDashboard.getNumber(TUNING_PREFIX + "kP", SHOOTER_KP);
        double ki = SmartDashboard.getNumber(TUNING_PREFIX + "kI", SHOOTER_KI);
        double kd = SmartDashboard.getNumber(TUNING_PREFIX + "kD", SHOOTER_KD);
        targetRpm = SmartDashboard.getNumber(TUNING_PREFIX + "TargetRPM", SHOOTER_TARGET_RPM);

        if (kp != lastKp || ki != lastKi || kd != lastKd) {
            shooterPid.setPID(kp, ki, kd);
            lastKp = kp;
            lastKi = ki;
            lastKd = kd;
        }
    }
}
