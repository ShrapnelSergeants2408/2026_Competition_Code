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

    public ShooterSubsystem() {
        // Shooter motor setup
        shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);
    
    OpenLoopRampsConfigs openLoopRamps = new OpenLoopRampsConfigs()
    .withDutyCycleOpenLoopRampPeriod(0.5);

TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
    .withMotorOutput(
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
    )
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true)
    )
    .withOpenLoopRamps(openLoopRamps);

            
           
        CurrentLimitsConfigs shooterLimits = new CurrentLimitsConfigs();
        shooterLimits.StatorCurrentLimitEnable = true;
        shooterLimits.StatorCurrentLimit = SHOOTER_CURRENT_LIMIT;
        shooterConfig.CurrentLimits = shooterLimits;
        shooterMotor.getConfigurator().apply(shooterConfig);
        //shooterMotor.setNeutralMode(NeutralModeValue.Coast);// coast mode
        


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

    // Spin shooter at a fixed percent output
    public void spinAtSpeed(double percentOutput) {
        shooterMotor.setControl(new DutyCycleOut(percentOutput));
    }

    public void stop() {
        shooterMotor.setControl(new DutyCycleOut(0.0));
    }

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

    // Ball detection (beam break inverted)
    public boolean hasBall() {
        return !lightSensor.get();
    }

    // Returns true if shooter is at target RPM AND a ball is present
    public boolean canShoot() {
        return isAtTargetSpeed(RPM_TOLERANCE) && hasBall();
    }

    // Command factory: run feeder while button held, only if canShoot() is true
    public Command shooterCommand() {
        return new RunCommand(
            () -> {
                if (canShoot()) {
                    startFeeder();
                } else {
                    stopFeeder();
                }
            },
            this // requires this subsystem
        );
    }

    @Override
    public void periodic() {
        logTelemetry();
    }

    public void logTelemetry() {
        SmartDashboard.putNumber("Shooter Current RPM", getCurrentRPM());
        SmartDashboard.putNumber("Shooter Target RPM", lastTargetRPM);

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
