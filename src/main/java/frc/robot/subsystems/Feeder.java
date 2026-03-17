package frc.robot.subsystems;

import static frc.robot.Constants.SensorConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Feeder subsystem — owns the intake roller (CAN 31) and trigger/hopper (CAN 32).
 * Separated from the Shooter subsystem so intake/eject commands can run concurrently
 * with the shooter wheel spinning (they require different subsystems).
 */
public class Feeder extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    // CAN 31: SparkMAX intake roller (primary + secondary mechanically linked) — CCW only
    private final SparkMax intakeMotor;
    // CAN 32: SparkMAX trigger/hopper — bidirectional
    private final SparkMax triggerMotor;
    // DIO 1: ball-presence sensor — null when PHOTO_SENSOR_ENABLED = false
    private final DigitalInput photoSensor;

    // ── State ─────────────────────────────────────────────────────────────────
    public enum FeederState { IDLE, INTAKE, FEED, EJECT, JAM_CLEAR }
    private FeederState currentState = FeederState.IDLE;

    // Jam detection (monitored on trigger motor — most likely jam point)
    private final Timer spikeDebounceTimer = new Timer();
    private boolean spikeDebounceRunning = false;
    private final Timer jamReverseTimer = new Timer();
    private FeederState stateBeforeJam = FeederState.IDLE;

    private int telemetryLoopCounter = 0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public Feeder() {
        // SparkMAX intake roller — CAN 31, open-loop, CCW = into robot
        intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT)
            .inverted(INTAKE_MOTOR_INVERTED)
            .voltageCompensation(NOMINAL_VOLTAGE)
            .openLoopRampRate(0.0);
        intakeMotor.configure(intakeConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // SparkMAX trigger/hopper — CAN 32, open-loop, bidirectional
        triggerMotor = new SparkMax(TRIGGER_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig triggerConfig = new SparkMaxConfig();
        triggerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(TRIGGER_MOTOR_CURRENT_LIMIT)
            .inverted(TRIGGER_MOTOR_INVERTED)
            .voltageCompensation(NOMINAL_VOLTAGE)
            .openLoopRampRate(0.0);
        triggerMotor.configure(triggerConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Photo sensor — skip DIO allocation until the sensor is installed
        photoSensor = PHOTO_SENSOR_ENABLED ? new DigitalInput(PHOTO_SENSOR_DIO_PORT) : null;
    }

    // ── Motor control ─────────────────────────────────────────────────────────

    /**
     * Run intake roller and trigger in feed direction (hopper → shooter).
     * No-op during JAM_CLEAR to avoid overriding the jam reversal.
     */
    public void startFeed() {
        //if (currentState == FeederState.JAM_CLEAR) return;
        currentState = FeederState.FEED;
        intakeMotor.set(INTAKE_SPEED);
        triggerMotor.set(TRIGGER_FEED_SPEED); //previously had -
    }

    /** Stop both motors and return to idle. Always stops even during JAM_CLEAR. */
    public void stopAll() {
        intakeMotor.set(0.0);
        triggerMotor.set(0.0);
        currentState = FeederState.IDLE;
        //spikeDebounceTimer.stop();
        //spikeDebounceRunning = false;
        //jamReverseTimer.stop();
    }

    // ── Sensor ────────────────────────────────────────────────────────────────

    public boolean hasBall() {
        if (photoSensor == null) return false;
        boolean raw = photoSensor.get();
        return PHOTO_SENSOR_INVERTED ? !raw : raw;
    }

    // ── State access ──────────────────────────────────────────────────────────

    public FeederState getState() { return currentState; }

    // ── Jam detection ─────────────────────────────────────────────────────────

    private void updateJamDetection() {
        if (currentState == FeederState.JAM_CLEAR) return;
        if (currentState != FeederState.INTAKE && currentState != FeederState.FEED) {
            if (spikeDebounceRunning) {
                spikeDebounceTimer.stop();
                spikeDebounceRunning = false;
            }
            return;
        }

        boolean spiking = triggerMotor.getOutputCurrent() > TRIGGER_SPIKE_THRESHOLD_AMPS;
        if (spiking) {
            if (!spikeDebounceRunning) {
                spikeDebounceTimer.reset();
                spikeDebounceTimer.start();
                spikeDebounceRunning = true;
            } else if (spikeDebounceTimer.hasElapsed(0.1)) {
                triggerJamClear();
            }
        } else {
            spikeDebounceTimer.stop();
            spikeDebounceRunning = false;
        }
    }

    private void triggerJamClear() {
        stateBeforeJam = currentState;
        currentState = FeederState.JAM_CLEAR;
        spikeDebounceRunning = false;
        spikeDebounceTimer.stop();
        jamReverseTimer.reset();
        jamReverseTimer.start();
        intakeMotor.set(0.0);
        triggerMotor.set(JAM_REVERSE_SPEED);
    }

    private void updateJamClear() {
        if (currentState != FeederState.JAM_CLEAR) return;
        if (jamReverseTimer.hasElapsed(JAM_REVERSE_TIME_SEC)) {
            jamReverseTimer.stop();
            currentState = stateBeforeJam;
            switch (currentState) {
                case INTAKE -> {
                    intakeMotor.set(INTAKE_SPEED);
                    triggerMotor.set(TRIGGER_INTAKE_SPEED);
                }
                case FEED -> {
                    intakeMotor.set(INTAKE_SPEED);
                    triggerMotor.set(TRIGGER_FEED_SPEED);
                }
                default -> stopAll();
            }
        }
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    /**
     * Intake — draw ball in from ground. Toggle with toggleOnTrue().
     * Requires only Feeder, so it runs concurrently with the shooter wheel spinning.
     * Uses run() so motors are re-commanded every 20 ms, preventing stalls from
     * a single missed or current-limited startup command.
     */
    public Command intakeCommand() {
        return Commands.run(() -> {
            currentState = FeederState.INTAKE;
            intakeMotor.set(INTAKE_SPEED);
            triggerMotor.set(TRIGGER_INTAKE_SPEED);
        }, this)
        .finallyDo(() -> stopAll());
    }

    /**
     * Removal — reverse both motors to expel ball. Toggle with toggleOnTrue().
     * Requires only Feeder, so it runs concurrently with the shooter wheel spinning.
     */
    public Command ejectCommand() {
        return Commands.run(() -> {
            currentState = FeederState.EJECT;
            intakeMotor.set(INTAKE_EJECT_SPEED);
            triggerMotor.set(TRIGGER_EJECT_SPEED);
        }, this)
        .finallyDo(() -> stopAll());
    }

    /**
     * Feed ball to shooter — intake holds ball while trigger drives toward shooter wheel.
     * Used by X (whileTrue) and A (toggleOnTrue).
     */
    public Command shootCommand() {
        return Commands.run(() -> {
            currentState = FeederState.FEED;
            intakeMotor.set(INTAKE_SPEED);
            triggerMotor.set(TRIGGER_EJECT_SPEED);  //was TRIGGER_FEED_SPEED
        }, this)
        .finallyDo(() -> stopAll());
    }

    public Command manualTriggerOverride() {
        return Commands.run(() -> {
            currentState = FeederState.FEED;
            triggerMotor.set(TRIGGER_EJECT_SPEED);
        }, this)
        .finallyDo(() -> stopAll());
    }

    public Command manualIntakeOverride() {
        return Commands.run(() -> {
            
            intakeMotor.set(INTAKE_SPEED);
        }, this)
        .finallyDo(() -> stopAll());
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Jam clear temporarily disabled
        // updateJamDetection();
        // updateJamClear();
        if (++telemetryLoopCounter >= SHOOTER_TELEMETRY_PERIOD_LOOPS) {
            telemetryLoopCounter = 0;
            logTelemetry();
        }
    }

    private void logTelemetry() {
        SmartDashboard.putBoolean("Feeder/Intake Active",      currentState == FeederState.INTAKE);
        SmartDashboard.putBoolean("Feeder/Eject Active",       currentState == FeederState.EJECT);
        //SmartDashboard.putBoolean("Feeder/Jam Clearing",       currentState == FeederState.JAM_CLEAR);
        SmartDashboard.putString("Feeder/State",               currentState.name());
        //SmartDashboard.putBoolean("Feeder/Ball Detected",      hasBall());
        SmartDashboard.putNumber("Feeder/Intake Current (A)",  intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Feeder/Trigger Current (A)", triggerMotor.getOutputCurrent());
    }
}
