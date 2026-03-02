package frc.robot.subsystems;

import static frc.robot.Constants.SensorConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.VisionMeasurement;

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
    // CAN 30: TalonFX shooter wheel — CW only, velocity PID via Phoenix 6
    private final TalonFX shooterMotor;
    // CAN 31: SparkMAX intake roller (primary + secondary mechanically linked) — CCW only
    //   positive set() = CCW (into robot) when INTAKE_MOTOR_INVERTED is configured correctly
    //   runs CCW for both intake mode AND shooting assist
    private final SparkMax intakeMotor;
    // CAN 32: SparkMAX trigger/hopper — bidirectional
    //   positive set() = CW (hopper → shooter); negative set() = CCW (intake → hopper)
    private final SparkMax triggerMotor;
    // DIO 1: ball-presence sensor — null when PHOTO_SENSOR_ENABLED = false
    private final DigitalInput photoSensor;

    // ── Reusable control requests (avoid per-loop allocation) ─────────────────
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    // ── State ─────────────────────────────────────────────────────────────────
    public enum ShooterState { IDLE, SPIN_UP, FEED, INTAKE, EJECT, JAM_CLEAR }
    private ShooterState currentState = ShooterState.IDLE;

    private double targetRPM = TARGET_RPM_10_FEET;
    private double currentTargetDistance = 10.0;

    // Jam detection (monitored on trigger motor — most likely jam point)
    private final Timer spikeDebounceTimer = new Timer();
    private boolean spikeDebounceRunning = false;
    private final Timer jamReverseTimer = new Timer();
    private ShooterState stateBeforeJam = ShooterState.IDLE;
    private int telemetryLoopCounter = 0;

    // ── Distance resolution ───────────────────────────────────────────────────
    private final Vision vision;
    private final DriveTrain drivetrain;
    private boolean povPresetSet = false;
    private double povPresetDistanceFt = 10.0;
    private String distanceSource = "Default";
    private double visionDistanceFt = -1.0;
    private double odometryDistanceFt = -1.0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public Shooter(Vision vision, DriveTrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;

        // TalonFX shooter wheel — CAN 30, Phoenix 6 on-controller velocity PID, slot 0
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

        // SparkMAX intake roller — CAN 31, open-loop, CCW = into robot
        intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT)
            .inverted(INTAKE_MOTOR_INVERTED)
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
            .openLoopRampRate(0.0);
        triggerMotor.configure(triggerConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Photo sensor — skip DIO allocation until the sensor is installed
        photoSensor = PHOTO_SENSOR_ENABLED ? new DigitalInput(PHOTO_SENSOR_DIO_PORT) : null;

        // Seed SmartDashboard tuning entries with constant defaults
        SmartDashboard.putNumber("Shooter/Tuning/kP", SHOOTER_KP);
        SmartDashboard.putNumber("Shooter/Tuning/kI", SHOOTER_KI);
        SmartDashboard.putNumber("Shooter/Tuning/kD", SHOOTER_KD);
        SmartDashboard.putNumber("Shooter/Tuning/kV", SHOOTER_KV);
    }

    // ── Shooter wheel control ─────────────────────────────────────────────────

    /** Send a velocity target (RPM) to the TalonFX via Phoenix 6 velocity PID. */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        shooterMotor.setControl(velocityRequest.withVelocity(rpm / 60.0));
    }

    /** Coast the shooter wheel to a stop. */
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

    // ── Intake roller and trigger motor control (private) ─────────────────────

    /**
     * Run intake roller (CAN 31).
     * Positive = CCW (into robot) when INTAKE_MOTOR_INVERTED is correctly set.
     */
    private void runIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * Run trigger/hopper motor (CAN 32).
     * Positive = CW (hopper → shooter); negative = CCW (intake → hopper or eject).
     */
    private void runTriggerMotor(double speed) {
        triggerMotor.set(speed);
    }

    /** Stop both the intake roller and trigger motor. */
    private void stopIntakeSystem() {
        intakeMotor.set(0.0);
        triggerMotor.set(0.0);
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
     * Set the target distance (feet) and apply the corresponding RPM immediately.
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
        povPresetDistanceFt = distanceFeet;
        currentTargetDistance = distanceFeet;
        targetRPM = getRPMFromDistance(distanceFeet);
        distanceSource = "POV Preset";
        povPresetSet = true;
    }

    public void clearDistancePreset() {
        povPresetSet = false;
        if ("POV Preset".equals(distanceSource)) {
            distanceSource = "Default";
        }
    }

    public double getTargetDistance() {
        return currentTargetDistance;
    }

    // ── Zone logic and distance resolution ───────────────────────────────────

    private Pose2d getAllianceHubPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_HUB_POSE;
        }
        return BLUE_HUB_POSE;
    }

    private boolean isInOffensiveZone(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        return alliance.get() == DriverStation.Alliance.Blue
            ? pose.getX() <= BLUE_OFFENSIVE_MAX_X
            : pose.getX() >= RED_OFFENSIVE_MIN_X;
    }

    private boolean isVisionMeasurementUsable(VisionMeasurement measurement) {
        return measurement.numTagsUsed() != 1
            || measurement.bestTargetAmbiguity() <= MAX_AMBIGUITY;
    }

    /**
     * Returns true when the shooter is permitted to spin up.
     * In test mode this is always true (bench/pit testing from any field position).
     * Otherwise the robot must be in the offensive zone.
     */
    private boolean canSpinShooter() {
        if (DriverStation.isTest()) return true;
        if (vision != null) {
            var freshMeasurement = vision.getBestVisionMeasurementIfFresh();
            if (freshMeasurement.isPresent()) {
                VisionMeasurement measurement = freshMeasurement.get();
                if (isVisionMeasurementUsable(measurement)) {
                    return isInOffensiveZone(measurement.estimatedPose());
                }
            }
        }
        if (drivetrain != null) {
            return isInOffensiveZone(drivetrain.getPose());
        }
        return false;
    }

    /**
     * Resolve the best available shooting distance (feet) using a four-tier priority:
     *   1. Vision pose (fresh AprilTag measurement)
     *   2. Odometry pose (drivetrain encoder + gyro)
     *   3. POV preset — explicitly set by operator via D-pad
     *   4. Default 10 ft
     */
    private double resolveShooterDistance() {
        boolean testMode = DriverStation.isTest();
        Pose2d hubPose = getAllianceHubPose();
        visionDistanceFt = -1.0;
        odometryDistanceFt = -1.0;

        // ── Priority 1: Vision ──────────────────────────────────────────────
        if (vision != null) {
            var freshMeasurement = vision.getBestVisionMeasurementIfFresh();
            if (freshMeasurement.isPresent()) {
                VisionMeasurement measurement = freshMeasurement.get();
                if (isVisionMeasurementUsable(measurement)) {
                    Pose2d visionPose = measurement.estimatedPose();
                    double meters = visionPose.getTranslation().getDistance(hubPose.getTranslation());
                    visionDistanceFt = meters / 0.3048;
                    if (testMode || isInOffensiveZone(visionPose)) {
                        currentTargetDistance = visionDistanceFt;
                        targetRPM = getRPMFromDistance(visionDistanceFt);
                        distanceSource = "Vision";
                        return visionDistanceFt;
                    }
                }
            }
        }

        // ── Priority 2: Odometry ────────────────────────────────────────────
        if (drivetrain != null) {
            Pose2d robotPose = drivetrain.getPose();
            double meters = robotPose.getTranslation().getDistance(hubPose.getTranslation());
            odometryDistanceFt = meters / 0.3048;
            if (testMode || isInOffensiveZone(robotPose)) {
                currentTargetDistance = odometryDistanceFt;
                targetRPM = getRPMFromDistance(odometryDistanceFt);
                distanceSource = "Odometry";
                return odometryDistanceFt;
            }
        }

        // ── Priority 3: POV preset ──────────────────────────────────────────
        if (povPresetSet) {
            currentTargetDistance = povPresetDistanceFt;
            targetRPM = getRPMFromDistance(povPresetDistanceFt);
            distanceSource = "POV Preset";
            return povPresetDistanceFt;
        }

        // ── Priority 4: Default 10 ft ───────────────────────────────────────
        currentTargetDistance = 10.0;
        targetRPM = getRPMFromDistance(10.0);
        distanceSource = "Default";
        return 10.0;
    }

    // ── Ball detection ────────────────────────────────────────────────────────

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

    // ── Jam detection (trigger motor) ─────────────────────────────────────────

    private void updateJamDetection() {
        if (currentState == ShooterState.JAM_CLEAR) return;
        if (currentState != ShooterState.INTAKE && currentState != ShooterState.FEED) {
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
        stopIntakeSystem();
        runTriggerMotor(JAM_REVERSE_SPEED); // CCW — reverse trigger to clear jam
    }

    private void updateJamClear() {
        if (currentState != ShooterState.JAM_CLEAR) return;
        if (jamReverseTimer.hasElapsed(JAM_REVERSE_TIME_SEC)) {
            jamReverseTimer.stop();
            currentState = stateBeforeJam;
            switch (currentState) {
                case INTAKE -> {
                    runIntakeMotor(INTAKE_SPEED);
                    runTriggerMotor(TRIGGER_INTAKE_SPEED);
                }
                case FEED -> {
                    runIntakeMotor(INTAKE_SPEED);
                    runTriggerMotor(TRIGGER_FEED_SPEED);
                }
                default -> stopIntakeSystem();
            }
        }
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    /**
     * Full shoot sequence: spin up shooter wheel, then auto-feed when at speed.
     * FEED phase: intake motor CCW (assists ball through system) + trigger CW (into shooter).
     * Zone-locked in teleop/auto; bypassed in test mode.
     */
    public Command shootCommand() {
        return Commands.run(() -> {
            if (!canSpinShooter()) {
                stopShooter();
                stopIntakeSystem();
                currentState = ShooterState.IDLE;
                return;
            }
            if (currentState == ShooterState.IDLE) {
                currentState = ShooterState.SPIN_UP;
            }
            resolveShooterDistance();
            shooterMotor.setControl(velocityRequest.withVelocity(targetRPM / 60.0));

            if (currentState == ShooterState.SPIN_UP && canShoot()) {
                currentState = ShooterState.FEED;
                runIntakeMotor(INTAKE_SPEED);        // CCW — assists ball into system
                runTriggerMotor(TRIGGER_FEED_SPEED); // CW  — pushes ball from hopper into shooter
            }
        }, this).finallyDo(interrupted -> {
            stopShooter();
            stopIntakeSystem();
            currentState = ShooterState.IDLE;
        });
    }

    /**
     * Feed only — runs intake and trigger in feed direction while shooter is already at speed.
     * Stops both feed motors (not shooter wheel) when command ends.
     */
    public Command feedCommand() {
        return Commands.run(() -> {
            if (canShoot()) {
                currentState = ShooterState.FEED;
                runIntakeMotor(INTAKE_SPEED);
                runTriggerMotor(TRIGGER_FEED_SPEED);
            } else {
                stopIntakeSystem();
                if (currentState == ShooterState.FEED) currentState = ShooterState.SPIN_UP;
            }
        }, this).finallyDo(interrupted -> {
            stopIntakeSystem();
            if (currentState == ShooterState.FEED) currentState = ShooterState.IDLE;
        });
    }

    /** Manual feed — runs intake and trigger in feed direction regardless of shooter speed. */
    public Command manualFeedCommand() {
        return Commands.startEnd(
            () -> {
                currentState = ShooterState.FEED;
                runIntakeMotor(INTAKE_SPEED);
                runTriggerMotor(TRIGGER_FEED_SPEED);
            },
            () -> {
                stopIntakeSystem();
                currentState = ShooterState.IDLE;
            },
            this
        );
    }

    /**
     * Intake — intake roller CCW (pulls ball from ground into robot) and
     * trigger CCW (pulls ball from intake into hopper). Both motors run together.
     * Toggle with toggleOnTrue(): first press starts, second press stops.
     */
    public Command intakeCommand() {
        return Commands.startEnd(
            () -> {
                currentState = ShooterState.INTAKE;
                runIntakeMotor(INTAKE_SPEED);           // CCW — ground into robot
                runTriggerMotor(TRIGGER_INTAKE_SPEED);  // CCW — intake into hopper
            },
            () -> {
                stopIntakeSystem();
                currentState = ShooterState.IDLE;
            },
            this
        );
    }

    /**
     * Eject — reverses both intake roller (CW) and trigger (CCW) to push ball
     * back out through the intake opening. Toggle with toggleOnTrue().
     */
    public Command ejectCommand() {
        return Commands.startEnd(
            () -> {
                currentState = ShooterState.EJECT;
                runIntakeMotor(INTAKE_EJECT_SPEED);    // CW  — pushes ball back out of robot
                runTriggerMotor(TRIGGER_EJECT_SPEED);  // CCW — moves ball from hopper toward intake path
            },
            () -> {
                stopIntakeSystem();
                currentState = ShooterState.IDLE;
            },
            this
        );
    }

    /**
     * Spin up shooter wheel to distance-resolved RPM (toggle — no feed).
     * Zone-locked in teleop/auto; bypassed in test mode.
     */
    public Command spinUpCommand() {
        return Commands.run(() -> {
            if (!canSpinShooter()) {
                stopShooter();
                currentState = ShooterState.IDLE;
                return;
            }
            currentState = ShooterState.SPIN_UP;
            resolveShooterDistance();
            shooterMotor.setControl(velocityRequest.withVelocity(targetRPM / 60.0));
        }, this).finallyDo(interrupted -> {
            stopShooter();
            currentState = ShooterState.IDLE;
        });
    }

    /** Immediately stop shooter wheel, intake roller, and trigger motor; return to idle. */
    public Command stopAllCommand() {
        return Commands.runOnce(() -> {
            stopShooter();
            stopIntakeSystem();
            currentState = ShooterState.IDLE;
        }, this);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        updateJamDetection();
        updateJamClear();
        updateSmartDashboardTuning();
        if (++telemetryLoopCounter >= SHOOTER_TELEMETRY_PERIOD_LOOPS) {
            telemetryLoopCounter = 0;
            logTelemetry();
        }
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
        double currentRPM = getCurrentRPM();
        boolean atSpeed    = isAtTargetSpeed();
        boolean inZone     = canSpinShooter();

        // ── Shooter wheel (CAN 30) ─────────────────────────────────────────
        SmartDashboard.putNumber("Shooter/Current RPM",         currentRPM);
        SmartDashboard.putNumber("Shooter/Target RPM",          targetRPM);
        SmartDashboard.putBoolean("Shooter/RPM At Speed",       atSpeed);
        SmartDashboard.putBoolean("Shooter/Can Shoot",          canShoot());
        SmartDashboard.putNumber("Shooter/Motor Current (A)",
            shooterMotor.getSupplyCurrent().getValueAsDouble());

        // ── Distance resolution ────────────────────────────────────────────
        SmartDashboard.putNumber("Shooter/Active Distance (ft)",     currentTargetDistance);
        SmartDashboard.putNumber("Shooter/POV Preset Distance (ft)", povPresetDistanceFt);
        SmartDashboard.putNumber("Shooter/Vision Distance (ft)",     visionDistanceFt);
        SmartDashboard.putNumber("Shooter/Odometry Distance (ft)",   odometryDistanceFt);
        SmartDashboard.putString("Shooter/Distance Source",          distanceSource);

        // ── Zone status ────────────────────────────────────────────────────
        SmartDashboard.putBoolean("Shooter/In Offensive Zone",  inZone);

        // ── Intake roller (CAN 31) ─────────────────────────────────────────
        SmartDashboard.putBoolean("Shooter/Intake Active",      currentState == ShooterState.INTAKE);
        SmartDashboard.putNumber("Intake/Motor Current (A)",    intakeMotor.getOutputCurrent());

        // ── Trigger motor (CAN 32) ─────────────────────────────────────────
        SmartDashboard.putBoolean("Shooter/Eject Active",       currentState == ShooterState.EJECT);
        SmartDashboard.putNumber("Trigger/Motor Current (A)",   triggerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Trigger/Jam Clearing",       currentState == ShooterState.JAM_CLEAR);

        // ── General state ──────────────────────────────────────────────────
        SmartDashboard.putBoolean("Shooter/Ball Detected",      hasBall());
        SmartDashboard.putString("Shooter/State",               currentState.name());
    }
}
