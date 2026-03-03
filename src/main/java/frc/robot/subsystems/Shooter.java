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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter subsystem — owns only the flywheel (CAN 30 TalonFX) and distance/zone logic.
 * Intake and trigger motors now live in the Feeder subsystem so that intake/eject
 * commands can run concurrently with the shooter wheel spinning.
 */
public class Shooter extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    // CAN 30: TalonFX shooter wheel — CW only, velocity PID via Phoenix 6
    private final TalonFX shooterMotor;

    // ── Reusable control requests (avoid per-loop allocation) ─────────────────
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    // ── State ─────────────────────────────────────────────────────────────────
    private double targetRPM = TARGET_RPM_10_FEET;
    private double currentTargetDistance = 10.0;

    // ── Distance resolution ───────────────────────────────────────────────────
    private final Vision vision;
    private final DriveTrain drivetrain;
    private boolean povPresetSet = false;
    private double povPresetDistanceFt = 10.0;
    private String distanceSource = "Default";
    private double visionDistanceFt = -1.0;
    private double odometryDistanceFt = -1.0;

    private int telemetryLoopCounter = 0;

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
        shooterMotor.setControl(velocityRequest.withVelocity(rpm * SHOOTER_GEAR_RATIO / 60.0));
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

    /**
     * True when it is safe to feed: shooter is at target speed AND either the
     * sensor is disabled (assume ball present) or the sensor detects a ball.
     * Pass feeder.hasBall() as the hasBall argument.
     */
    public boolean canShoot(boolean hasBall) {
        return isAtTargetSpeed() && (!PHOTO_SENSOR_ENABLED || hasBall);
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
    public boolean canSpinShooter() {
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
    private void resolveShooterDistance() {
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
                        return;
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
                return;
            }
        }

        // ── Priority 3: POV preset ──────────────────────────────────────────
        if (povPresetSet) {
            currentTargetDistance = povPresetDistanceFt;
            targetRPM = getRPMFromDistance(povPresetDistanceFt);
            distanceSource = "POV Preset";
            return;
        }

        // ── Priority 4: Default 10 ft ───────────────────────────────────────
        currentTargetDistance = 10.0;
        targetRPM = getRPMFromDistance(10.0);
        distanceSource = "Default";
    }

    /**
     * Resolve best distance and apply velocity command to the shooter wheel.
     * Called each loop by shootCommand and spinUpCommand in RobotContainer.
     */
    public void resolveDistanceAndSpin() {
        resolveShooterDistance();
        shooterMotor.setControl(velocityRequest.withVelocity(targetRPM / 60.0));
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    /**
     * Spin up shooter wheel to distance-resolved RPM (toggle — no feed).
     * Requires only Shooter, so LB/LT/RB (Feeder commands) run concurrently.
     * Zone-locked in teleop/auto; bypassed in test mode.
     */
    public Command spinUpCommand() {
        return Commands.run(() -> {
            if (!canSpinShooter()) {
                stopShooter();
                return;
            }
            resolveDistanceAndSpin();
        }, this).finallyDo(interrupted -> stopShooter());
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
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

        // ── Shooter wheel (CAN 30) ─────────────────────────────────────────
        SmartDashboard.putNumber("Shooter/Current RPM",        currentRPM);
        SmartDashboard.putNumber("Shooter/Target RPM",         targetRPM);
        SmartDashboard.putBoolean("Shooter/RPM At Speed",      isAtTargetSpeed());
        SmartDashboard.putNumber("Shooter/Motor Current (A)",
            shooterMotor.getSupplyCurrent().getValueAsDouble());

        // ── Distance resolution ────────────────────────────────────────────
        SmartDashboard.putNumber("Shooter/Active Distance (ft)",     currentTargetDistance);
        SmartDashboard.putNumber("Shooter/POV Preset Distance (ft)", povPresetDistanceFt);
        SmartDashboard.putNumber("Shooter/Vision Distance (ft)",     visionDistanceFt);
        SmartDashboard.putNumber("Shooter/Odometry Distance (ft)",   odometryDistanceFt);
        SmartDashboard.putString("Shooter/Distance Source",          distanceSource);

        // ── Zone status ────────────────────────────────────────────────────
        SmartDashboard.putBoolean("Shooter/In Offensive Zone", canSpinShooter());
    }
}
