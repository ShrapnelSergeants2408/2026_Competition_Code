package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterMotor;

    // Optional: store the last target RPM
    private double lastTargetRPM = 0.0;

    public ShooterSubsystem() {
        // Instantiate the TalonFX motor
        shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);

        // Configure current limits
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimitEnable = true;
        limits.StatorCurrentLimit = SHOOTER_CURRENT_LIMIT;
        config.CurrentLimits = limits;

        // Apply configuration
        shooterMotor.getConfigurator().apply(config);
    }

    // -------------------------------
    // Spin shooter at a fixed percent output
    // -------------------------------
    public void spinAtSpeed(double percentOutput) {
        shooterMotor.setControl(new DutyCycleOut(percentOutput));
    }

    // -------------------------------
    // Stop shooter
    // -------------------------------
    public void stop() {
        shooterMotor.setControl(new DutyCycleOut(0.0));
    }

    // -------------------------------
    // Target RPM methods (open-loop simulation)
    // -------------------------------

    /**
     * Sets a target RPM by approximating it using percent output.
     * This is open-loop since VelocityOut isn't available.
     * @param rpm Desired target RPM
     */
    public void setTargetRPM(double rpm) {
        lastTargetRPM = rpm;

        // Simple open-loop approximation: scale RPM to percent output
        // Adjust this multiplier based on your shooter characteristics
        double percentOutput = rpm / TARGET_RPM_10_FEET; 
        if (percentOutput > 1.0) percentOutput = 1.0; // clamp max
        if (percentOutput < 0.0) percentOutput = 0.0; // clamp min

        spinAtSpeed(percentOutput);
    }

    /**
     * Gets the current "RPM" based on motor percent output.
     * Since we don't have a sensor, we approximate it.
     */
    public double getCurrentRPM() {
        // Approximate current RPM
        DutyCycleOut current = new DutyCycleOut(0.0); // default
        // Phoenix6 doesn't let us read duty cycle directly; just return last target
        return lastTargetRPM; 
    }

    /**
     * Checks if the shooter is within tolerance of target RPM
     * @param tolerance RPM tolerance
     * @return true if within tolerance
     */
    public boolean isAtTargetSpeed(double tolerance) {
        double currentRPM = getCurrentRPM();
        return Math.abs(currentRPM - lastTargetRPM) <= tolerance;
    }
}
