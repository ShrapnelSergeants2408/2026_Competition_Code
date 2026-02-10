package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.SensorConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Velocity;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotor;
    private final SparkMax feederMotor;
    private final DigitalInput lightSensor;
    private double lastTargetRPM = 0.0;

    // Distance-to-RPM system
    private double currentTargetDistance = 10.0;

    public ShooterSubsystem() {
        // Shooter motor setup
        shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);

        OpenLoopRampsConfigs openLoopRamps = new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(0.5);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                )
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(SHOOTER_CURRENT_LIMIT)
                                .withStatorCurrentLimitEnable(true)
                )
                .withOpenLoopRamps(openLoopRamps);

        shooterMotor.getConfigurator().apply(shooterConfig);

        // Feeder motor setup
        feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(FEEDER_CURRENT_LIMIT)
                .inverted(false)
                .openLoopRampRate(0.0)
                .closedLoopRampRate(0.0);

        feederMotor.configure(
                feederConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        // Light sensor setup
        lightSensor = new DigitalInput(LIGHT_SENSOR_DIO_PORT);
    }

    // --- Shooter control ---
    public void spinAtSpeed(double rpm) {
        shooterMotor.setControl(new VelocityVoltage(rpm/60));
    }

    public void stop() {
        shooterMotor.setControl(new VelocityVoltage(0.0));
    }

    private double targetRpm = 0;
    public void enableShooter(Boolean enable){
        lastTargetRPM = enable? targetRpm : 0;
        spinAtSpeed(lastTargetRPM);
    }
    public void setTargetRPM(double rpm) {
        targetRpm = rpm;
    }

    public double getCurrentRPM() {
        return shooterMotor.getVelocity().getValueAsDouble()*60;
    }

    public boolean isAtTargetSpeed(double tolerance) {
        return Math.abs(getCurrentRPM() - lastTargetRPM) <= tolerance;
    }

    // --- Distance-to-RPM system ---
    public double getRPMFromDistance(double distanceFeet) {
        double[] distances = DISTANCES_FEET;
        double[] rpms = DISTANCE_RPM_MAP;

        if (distanceFeet <= distances[0]) return rpms[0];
        if (distanceFeet >= distances[distances.length - 1]) return rpms[rpms.length - 1];

        for (int i = 0; i < distances.length - 1; i++) {
            if (distanceFeet >= distances[i] && distanceFeet <= distances[i + 1]) {
                double ratio = (distanceFeet - distances[i]) / (distances[i + 1] - distances[i]);
                return rpms[i] + ratio * (rpms[i + 1] - rpms[i]);
            }
        }

        return rpms[rpms.length - 1];
    }

    public void setTargetDistance(double distanceFeet) {
        currentTargetDistance = distanceFeet;
        double rpm = getRPMFromDistance(distanceFeet);
        setTargetRPM(rpm);
    }

    public double getTargetDistance() {
        return currentTargetDistance;
    }

    // --- Feeder motor ---
    public void startFeeder() {
        if (!hasBall()) {
            DriverStation.reportWarning(
                    "Feeder blocked: attempted to feed with NO ball detected",
                    false
            );
            return;
        }
        feederMotor.set(FEEDER_SPEED);
    }

    public void stopFeeder() {
        feederMotor.set(0.0);
    }

    // --- Ball detection ---
    public boolean hasBall() {
        return !lightSensor.get();
    }

    // --- Shooter commands ---
    public boolean canShoot() {
        return isAtTargetSpeed(RPM_TOLERANCE) && hasBall();
    }

    public Command shooterCommand() {
        return new RunCommand(
                () -> {
                    if (canShoot()) startFeeder();
                    else stopFeeder();
                },
                this
        );
    }

    @Override
    public void periodic() {
        logTelemetry();
    }

    public void logTelemetry() {
        SmartDashboard.putNumber("Shooter Current RPM", getCurrentRPM());
        SmartDashboard.putNumber("Shooter Target RPM", lastTargetRPM);
        SmartDashboard.putNumber("Target Distance (ft)", currentTargetDistance);

        var currentSignal = shooterMotor.getSupplyCurrent();
        double currentAmps = currentSignal.getValueAsDouble();
        SmartDashboard.putNumber("Shooter Motor Current (A)", currentAmps);

        boolean feederOn = feederMotor.get() != 0.0;
        SmartDashboard.putBoolean("Feeder On", feederOn);

        SmartDashboard.putBoolean("At Target Speed", isAtTargetSpeed(RPM_TOLERANCE));
        SmartDashboard.putBoolean("Ball Detected", hasBall());
        SmartDashboard.putBoolean("Ready to Shoot", canShoot());
    }
}
