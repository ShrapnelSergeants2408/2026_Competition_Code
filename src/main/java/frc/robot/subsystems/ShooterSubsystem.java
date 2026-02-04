package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.IdleMode; // Updated import for IdleMode
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    // Shooter motor
    private final TalonFX shooterMotor;

    // Feeder motor
    private final SparkMax feederMotor;

    // Optional: store the last target RPM
    private double lastTargetRPM = 0.0;

    public ShooterSubsystem() {

        // Shooter motor setup
        shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);

        // Configure current limits for shooter
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs shooterLimits = new CurrentLimitsConfigs();
        shooterLimits.StatorCurrentLimitEnable = true;
        shooterLimits.StatorCurrentLimit = SHOOTER_CURRENT_LIMIT;
        shooterConfig.CurrentLimits = shooterLimits;

        // Apply configuration to TalonFX
        shooterMotor.getConfigurator().apply(shooterConfig);

        // Feeder motor setup
        feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

        // Modern Spark MAX configuration
        // Feeder motor setup

// Create a default configuration object
SparkMaxConfig feederConfig = new SparkMaxConfig();

// Set current limit in the config

feederConfig.smartCurrentLimit(FEEDER_CURRENT_LIMIT, FEEDER_CURRENT_LIMIT);
feederConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
// Create a default configuration object

// Set current limit
feederConfig.smartCurrentLimit(FEEDER_CURRENT_LIMIT, FEEDER_CURRENT_LIMIT);

// Set idle mode
 feederConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

// Apply the configuration with reset and persist modes
feederMotor.configure(
    feederConfig,
    SparkMax.ResetMode.kResetSafeParameters,
    SparkMax.PersistMode.kPersistParameters
);
;}



    // Spin shooter at a fixed percent output
    public void spinAtSpeed(double percentOutput) {
        shooterMotor.setControl(new DutyCycleOut(percentOutput));
    }

    // Stop shooter
    public void stop() {
        shooterMotor.setControl(new DutyCycleOut(0.0));
    }

    // Target RPM methods (open-loop simulation)
    public void setTargetRPM(double rpm) {
        lastTargetRPM = rpm;

        double percentOutput = rpm / TARGET_RPM_10_FEET;
        percentOutput = Math.max(0.0, Math.min(percentOutput, 1.0));

        spinAtSpeed(percentOutput);
    }

    public double getCurrentRPM() {
        return lastTargetRPM;
    }

    public boolean isAtTargetSpeed(double tolerance) {
        double currentRPM = getCurrentRPM();
        return Math.abs(currentRPM - lastTargetRPM) <= tolerance;
    }

    // Feeder motor methods
    public void startFeeder() {
        feederMotor.set(FEEDER_SPEED);
    }

    public void stopFeeder() {
        feederMotor.set(0.0);
    }
}
