package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMaxConfig;
import com.revrobotics.spark.SparkMaxConfig.SparkMaxIdleMode;
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
        // Instantiate the TalonFX motor
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
        // Instantiate SparkMax for feeder
        feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

        // Create config object
        SparkMaxConfig feederConfig = new SparkMaxConfig();

        // Current limit
        feederConfig.CurrentLimits.SupplyCurrentLimit = SHOOTER_CURRENT_LIMIT;
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Idle mode
        feederConfig.MotorOutput.IdleMode = SparkMaxIdleMode.kBrake;

        // Apply configuration
        feederMotor.configure(feederConfig, SparkMax.kResetSafeParameters, SparkMax.kPersistParameters);
    }

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
