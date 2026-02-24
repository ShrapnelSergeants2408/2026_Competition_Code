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

    // ── Distance resolution ───────────────────────────────────────────────────
    private final Vision vision;
    private final DriveTrain drivetrain;
    private boolean povPresetSet = false;
    private double povPresetDistanceFt = 10.0;     // distance explicitly staged by POV buttons
    private String distanceSource = "Default";     // which priority tier is active
    private double visionDistanceFt = -1.0;        // last vision-computed distance (-1 = not available)
    private double odometryDistanceFt = -1.0;      // last odometry-computed distance (-1 = not available)

    // ── Constructor ───────────────────────────────────────────────────────────
    public Shooter(Vision vision, DriveTrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
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
     * Marks povPresetSet = true so resolveShooterDistance() uses this value as priority 3.
     */
    public void setDistancePreset(double distanceFeet) {
        povPresetDistanceFt = distanceFeet;
        currentTargetDistance = distanceFeet;
        targetRPM = getRPMFromDistance(distanceFeet);
        povPresetSet = true;
    }

    public double getTargetDistance() {
        return currentTargetDistance;
    }

    // ── Zone logic and distance resolution ───────────────────────────────────

    /**
     * Returns the hub pose for the current alliance (defaults to blue if alliance unknown).
     * Coordinates are derived from the 2026-rebuilt-welded AprilTag layout JSON.
     */
    private Pose2d getAllianceHubPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_HUB_POSE;
        }
        return BLUE_HUB_POSE;
    }

    /**
     * Returns true if the given robot pose is within the offensive zone for the current alliance.
     * Blue offensive zone: X ≤ BLUE_OFFENSIVE_MAX_X (between blue DS and blue hub).
     * Red  offensive zone: X ≥ RED_OFFENSIVE_MIN_X  (between red  DS and red  hub).
     * Returns false when alliance is unknown (prevents accidental spin-up during setup).
     */
    private boolean isInOffensiveZone(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        return alliance.get() == DriverStation.Alliance.Blue
            ? pose.getX() <= BLUE_OFFENSIVE_MAX_X
            : pose.getX() >= RED_OFFENSIVE_MIN_X;
    }

    /**
     * Returns true when the shooter is permitted to spin up.
     * In test mode this is always true (manual bench/pit testing from any field position).
     * Otherwise the robot must be in the offensive zone.
     */
    private boolean canSpinShooter() {
        if (DriverStation.isTest()) return true;

        // Prefer the freshest available pose for zone determination.
        if (vision != null) {
            var freshMeasurement = vision.getBestVisionMeasurementIfFresh();
            if (freshMeasurement.isPresent()) {
                return isInOffensiveZone(freshMeasurement.get().estimatedPose());
            }
        }
        if (drivetrain != null) {
            return isInOffensiveZone(drivetrain.getPose());
        }
        return false; // no pose → don't spin
    }

    /**
     * Resolve the best available shooting distance (feet) using a four-tier priority:
     *   1. Vision pose (fresh AprilTag measurement) — only when in offensive zone or test mode.
     *   2. Odometry pose (drivetrain encoder + gyro) — only when in offensive zone or test mode.
     *   3. POV preset — explicitly set by the operator via D-pad.
     *   4. Default 10 ft.
     *
     * Updates currentTargetDistance, targetRPM, distanceSource, and the diagnostic fields
     * visionDistanceFt / odometryDistanceFt as a side effect.
     *
     * @return resolved distance in feet
     */
    private double resolveShooterDistance() {
        boolean testMode = DriverStation.isTest();
        Pose2d hubPose = getAllianceHubPose();

        // ── Priority 1: Vision ─────────────────────────────────────────────
        if (vision != null) {
            var freshMeasurement = vision.getBestVisionMeasurementIfFresh();
            if (freshMeasurement.isPresent()) {
                Pose2d visionPose = freshMeasurement.get().estimatedPose();
                if (testMode || isInOffensiveZone(visionPose)) {
                    double meters = visionPose.getTranslation().getDistance(hubPose.getTranslation());
                    visionDistanceFt = meters / 0.3048;
                    currentTargetDistance = visionDistanceFt;
                    targetRPM = getRPMFromDistance(visionDistanceFt);
                    distanceSource = "Vision";
                    return visionDistanceFt;
                }
            }
        }

        // ── Priority 2: Odometry ───────────────────────────────────────────
        if (drivetrain != null) {
            Pose2d robotPose = drivetrain.getPose();
            if (testMode || isInOffensiveZone(robotPose)) {
                double meters = robotPose.getTranslation().getDistance(hubPose.getTranslation());
                odometryDistanceFt = meters / 0.3048;
                currentTargetDistance = odometryDistanceFt;
                targetRPM = getRPMFromDistance(odometryDistanceFt);
                distanceSource = "Odometry";
                return odometryDistanceFt;
            }
        }

        // ── Priority 3: POV preset ─────────────────────────────────────────
        if (povPresetSet) {
            currentTargetDistance = povPresetDistanceFt;
            targetRPM = getRPMFromDistance(povPresetDistanceFt);
            distanceSource = "POV Preset";
            return povPresetDistanceFt;
        }

        // ── Priority 4: Default 10 ft ──────────────────────────────────────
        currentTargetDistance = 10.0;
        targetRPM = getRPMFromDistance(10.0);
        distanceSource = "Default";
        return 10.0;
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
     * Spin up to distance-resolved RPM, then feed automatically when at speed.
     * Shooter and feeder stop when the command ends (button released).
     * Zone-locked: does nothing outside the offensive zone unless in test mode.
     */
    public Command shootCommand() {
        return Commands.run(() -> {
            if (!canSpinShooter()) {
                stopShooter();
                stopFeeder();
                currentState = ShooterState.IDLE;
                return;
            }
            if (currentState == ShooterState.IDLE) {
                currentState = ShooterState.SPIN_UP;
            }
            // Continuously re-resolve distance so RPM tracks robot position.
            resolveShooterDistance();
            shooterMotor.setControl(velocityRequest.withVelocity(targetRPM / 60.0));

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

    /**
     * Spin the shooter wheel to the distance-resolved RPM (toggle — no feed).
     * Continuously updates RPM as robot position changes.
     * Zone-locked: motor stays off outside the offensive zone unless in test mode.
     * Bind with toggleOnTrue() so first press starts, second press stops.
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
        double currentRPM = getCurrentRPM();
        boolean atSpeed    = isAtTargetSpeed();
        boolean inZone     = canSpinShooter();

        // ── Shooter wheel ──────────────────────────────────────────────────
        SmartDashboard.putNumber("Shooter/Current RPM",          currentRPM);
        SmartDashboard.putNumber("Shooter/Target RPM",           targetRPM);
        SmartDashboard.putBoolean("Shooter/RPM At Speed",        atSpeed);   // green = ready, red = not
        SmartDashboard.putBoolean("Shooter/Can Shoot",           canShoot());
        SmartDashboard.putNumber("Shooter/Motor Current (A)",
            shooterMotor.getSupplyCurrent().getValueAsDouble());

        // ── Distance resolution ────────────────────────────────────────────
        SmartDashboard.putNumber("Shooter/Active Distance (ft)",  currentTargetDistance);
        SmartDashboard.putNumber("Shooter/POV Preset Distance (ft)", povPresetDistanceFt);
        SmartDashboard.putNumber("Shooter/Vision Distance (ft)",  visionDistanceFt);       // -1 = not available
        SmartDashboard.putNumber("Shooter/Odometry Distance (ft)", odometryDistanceFt);    // -1 = not available
        SmartDashboard.putString("Shooter/Distance Source",       distanceSource);

        // ── Zone status ────────────────────────────────────────────────────
        SmartDashboard.putBoolean("Shooter/In Offensive Zone",   inZone);

        // ── Feeder / intake ────────────────────────────────────────────────
        SmartDashboard.putBoolean("Shooter/Intake Active",       currentState == ShooterState.INTAKE);
        SmartDashboard.putBoolean("Shooter/Eject Active",        currentState == ShooterState.EJECT);
        SmartDashboard.putNumber("Feeder/Motor Current (A)",     feederMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Feeder/Jam Clearing",         currentState == ShooterState.JAM_CLEAR);

        // ── General state ──────────────────────────────────────────────────
        SmartDashboard.putBoolean("Shooter/Ball Detected",       hasBall());
        SmartDashboard.putString("Shooter/State",                currentState.name());
    }
}
