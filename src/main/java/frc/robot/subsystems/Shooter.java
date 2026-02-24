package frc.robot.subsystems;

import static frc.robot.Constants.SensorConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX shooterMotor;
    private final SparkMax feederMotor;
    private final DigitalInput photoSensor; // null when PHOTO_SENSOR_ENABLED = false

    // ── Reusable control requests (avoid per-loop allocation) ─────────────────
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    // ── State ─────────────────────────────────────────────────────────────────
    public enum ShooterState { IDLE, SPIN_UP, FEED, INTAKE, EJECT, JAM_CLEAR }
    private ShooterState currentState = ShooterState.IDLE;

    private double targetRPM = TARGET_RPM_10_FEET;
    private double currentTargetDistance = 10.0;

    // Jam detection
    private final Timer spikeDebounceTimer = new Timer();
    private boolean spikeDebounceRunning = false;
    private final Timer jamReverseTimer = new Timer();
    private ShooterState stateBeforeJam = ShooterState.IDLE;

    // ── Constructor ───────────────────────────────────────────────────────────
    public Shooter() {
        // TalonFX shooter — Phoenix 6 on-controller velocity PID, slot 0
        shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(SHOOTER_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(SHOOTER_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
                .withKP(SHOOTER_KP)
                .withKI(SHOOTER_KI)
                .withKD(SHOOTER_KD)
                .withKV(SHOOTER_KV));
        shooterMotor.getConfigurator().apply(shooterConfig);

        // SparkMAX feeder/intake — open-loop duty cycle, brake at idle
        feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(FEEDER_CURRENT_LIMIT)
            .inverted(FEEDER_INVERTED)
            .openLoopRampRate(0.0);
        feederMotor.configure(feederConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Photo sensor — skip DIO allocation until the sensor is installed
        photoSensor = PHOTO_SENSOR_ENABLED ? new DigitalInput(PHOTO_SENSOR_DIO_PORT) : null;

        // Seed SmartDashboard tuning entries with constant defaults
        SmartDashboard.putNumber("Shooter/Tuning/kP", SHOOTER_KP);
        SmartDashboard.putNumber("Shooter/Tuning/kI", SHOOTER_KI);
        SmartDashboard.putNumber("Shooter/Tuning/kD", SHOOTER_KD);
        SmartDashboard.putNumber("Shooter/Tuning/kV", SHOOTER_KV);
    }

    // ── Shooter control ───────────────────────────────────────────────────────

    /** Send a velocity target (RPM) to the TalonFX via Phoenix 6 velocity PID. */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        shooterMotor.setControl(velocityRequest.withVelocity(rpm / 60.0));
    }

    /** Coast the shooter to a stop. */
    public void stopShooter() {
        shooterMotor.setControl(neutralRequest);
    }

    /** Read actual shooter velocity from the TalonFX encoder in RPM. */
    public double getCurrentRPM() {
        return shooterMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    /** True when the shooter is spinning and within RPM_TOLERANCE of the target. */
    public boolean isAtTargetSpeed() {
        return targetRPM > 0 && Math.abs(getCurrentRPM() - targetRPM) <= RPM_TOLERANCE;
    }

    // ── Distance → RPM table ─────────────────────────────────────────────────

    /** Interpolate target RPM from the distance-to-RPM map. */
    public double getRPMFromDistance(double distanceFeet) {
        if (distanceFeet <= DISTANCES_FEET[0])
            return DISTANCE_RPM_MAP[0];
        if (distanceFeet >= DISTANCES_FEET[DISTANCES_FEET.length - 1])
            return DISTANCE_RPM_MAP[DISTANCE_RPM_MAP.length - 1];
        for (int i = 0; i < DISTANCES_FEET.length - 1; i++) {
            if (distanceFeet >= DISTANCES_FEET[i] && distanceFeet <= DISTANCES_FEET[i + 1]) {
                double t = (distanceFeet - DISTANCES_FEET[i])
                         / (DISTANCES_FEET[i + 1] - DISTANCES_FEET[i]);
                return DISTANCE_RPM_MAP[i] + t * (DISTANCE_RPM_MAP[i + 1] - DISTANCE_RPM_MAP[i]);
            }
        }
        return DISTANCE_RPM_MAP[DISTANCE_RPM_MAP.length - 1];
    }

    /**
     * Set the target distance (feet) — looks up and applies the corresponding RPM immediately.
     * Use during an active shoot to re-aim on the fly.
     */
    public void setTargetDistance(double distanceFeet) {
        currentTargetDistance = distanceFeet;
        setTargetRPM(getRPMFromDistance(distanceFeet));
    }

    /**
     * Pre-set the distance/RPM target without spinning the motor.
     * Use with POV buttons so the operator can stage the shot before pressing shoot.
     */
    public void setDistancePreset(double distanceFeet) {
        currentTargetDistance = distanceFeet;
        targetRPM = getRPMFromDistance(distanceFeet);
    }

    public double getTargetDistance() {
        return currentTargetDistance;
    }

    // ── Feeder/intake control (private — exposed via commands) ────────────────

    private void runFeeder(double speed) {
        feederMotor.set(speed);
    }

    private void stopFeeder() {
        feederMotor.set(0.0);
    }

    // ── Ball detection ────────────────────────────────────────────────────────

    /**
     * Returns true if the photo sensor detects a ball.
     * Always returns false when PHOTO_SENSOR_ENABLED = false (sensor not installed).
     */
    public boolean hasBall() {
        if (photoSensor == null) return false;
        boolean raw = photoSensor.get();
        return PHOTO_SENSOR_INVERTED ? !raw : raw;
    }

    /**
     * True when it is safe to feed: shooter is at target speed AND either the
     * sensor is disabled (assume ball present) or the sensor detects a ball.
     */
    public boolean canShoot() {
        return isAtTargetSpeed() && (!PHOTO_SENSOR_ENABLED || hasBall());
    }

    // ── Jam detection ─────────────────────────────────────────────────────────

    private void updateJamDetection() {
        if (currentState == ShooterState.JAM_CLEAR) return;
        if (currentState != ShooterState.INTAKE && currentState != ShooterState.FEED) {
            if (spikeDebounceRunning) {
                spikeDebounceTimer.stop();
                spikeDebounceRunning = false;
            }
            return;
        }

        boolean spiking = feederMotor.getOutputCurrent() > FEEDER_SPIKE_THRESHOLD_AMPS;
        if (spiking) {
            if (!spikeDebounceRunning) {
                spikeDebounceTimer.reset();
                spikeDebounceTimer.start();
                spikeDebounceRunning = true;
            } else if (spikeDebounceTimer.hasElapsed(0.1)) { // 100 ms debounce
                triggerJamClear();
            }
        } else {
            spikeDebounceTimer.stop();
            spikeDebounceRunning = false;
        }
    }

    private void triggerJamClear() {
        stateBeforeJam = currentState;
        currentState = ShooterState.JAM_CLEAR;
        spikeDebounceRunning = false;
        spikeDebounceTimer.stop();
        jamReverseTimer.reset();
        jamReverseTimer.start();
        runFeeder(JAM_REVERSE_SPEED);
    }

    private void updateJamClear() {
        if (currentState != ShooterState.JAM_CLEAR) return;
        if (jamReverseTimer.hasElapsed(JAM_REVERSE_TIME_SEC)) {
            jamReverseTimer.stop();
            currentState = stateBeforeJam;
            switch (currentState) {
                case INTAKE -> runFeeder(INTAKE_SPEED);
                case FEED   -> runFeeder(FEEDER_SPEED);
                default     -> stopFeeder();
            }
        }
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    /**
     * Spin up to target RPM, then feed automatically when ready.
     * Shooter coasts and feeder stops when the command ends (button released).
     */
    public Command shootCommand() {
        return Commands.run(() -> {
            if (currentState == ShooterState.IDLE) {
                currentState = ShooterState.SPIN_UP;
                setTargetRPM(targetRPM);
            }
            if (currentState == ShooterState.SPIN_UP && canShoot()) {
                currentState = ShooterState.FEED;
                runFeeder(FEEDER_SPEED);
            }
        }, this).finallyDo(interrupted -> {
            stopShooter();
            stopFeeder();
            currentState = ShooterState.IDLE;
        });
    }

    /**
     * Feed only — runs feeder while shooter is already at target speed.
     * Stops feeder (but not shooter) when command ends.
     */
    public Command feedCommand() {
        return Commands.run(() -> {
            if (canShoot()) {
                currentState = ShooterState.FEED;
                runFeeder(FEEDER_SPEED);
            } else {
                stopFeeder();
                if (currentState == ShooterState.FEED) currentState = ShooterState.SPIN_UP;
            }
        }, this).finallyDo(interrupted -> {
            stopFeeder();
            if (currentState == ShooterState.FEED) currentState = ShooterState.IDLE;
        });
    }

    /** Manual feed — runs feeder regardless of shooter speed. Use with caution. */
    public Command manualFeedCommand() {
        return Commands.startEnd(
            () -> { currentState = ShooterState.FEED; runFeeder(FEEDER_SPEED); },
            () -> { stopFeeder(); currentState = ShooterState.IDLE; },
            this
        );
    }

    /** Draw ball in — runs feeder forward while held. */
    public Command intakeCommand() {
        return Commands.startEnd(
            () -> { currentState = ShooterState.INTAKE; runFeeder(INTAKE_SPEED); },
            () -> { stopFeeder(); currentState = ShooterState.IDLE; },
            this
        );
    }

    /** Eject ball — runs feeder in reverse while held. */
    public Command ejectCommand() {
        return Commands.startEnd(
            () -> { currentState = ShooterState.EJECT; runFeeder(EJECT_SPEED); },
            () -> { stopFeeder(); currentState = ShooterState.IDLE; },
            this
        );
    }

    /** Immediately stop shooter and feeder and return to idle. */
    public Command stopAllCommand() {
        return Commands.runOnce(() -> {
            stopShooter();
            stopFeeder();
            currentState = ShooterState.IDLE;
        }, this);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        updateJamDetection();
        updateJamClear();
        updateSmartDashboardTuning();
        logTelemetry();
    }

    /** Read SmartDashboard PID values and push to TalonFX slot 0. Test mode only. */
    private void updateSmartDashboardTuning() {
        if (!DriverStation.isTest()) return;
        Slot0Configs slot0 = new Slot0Configs()
            .withKP(SmartDashboard.getNumber("Shooter/Tuning/kP", SHOOTER_KP))
            .withKI(SmartDashboard.getNumber("Shooter/Tuning/kI", SHOOTER_KI))
            .withKD(SmartDashboard.getNumber("Shooter/Tuning/kD", SHOOTER_KD))
            .withKV(SmartDashboard.getNumber("Shooter/Tuning/kV", SHOOTER_KV));
        shooterMotor.getConfigurator().apply(slot0);
    }

    private void logTelemetry() {
        SmartDashboard.putNumber("Shooter/Current RPM", getCurrentRPM());
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Target Distance (ft)", currentTargetDistance);
        SmartDashboard.putNumber("Shooter/Motor Current (A)",
            shooterMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Motor Current (A)", feederMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Shooter/At Target Speed", isAtTargetSpeed());
        SmartDashboard.putBoolean("Shooter/Ball Detected", hasBall());
        SmartDashboard.putBoolean("Shooter/Can Shoot", canShoot());
        SmartDashboard.putBoolean("Feeder/Jam Clearing", currentState == ShooterState.JAM_CLEAR);
        SmartDashboard.putString("Shooter/State", currentState.name());
    }
}
