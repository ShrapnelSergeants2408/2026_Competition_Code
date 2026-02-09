package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.SensorConstants.*;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase {

    public final SparkMax intakeMotor;
    private final DigitalInput ballSensor;

    private int currentSpikeCounter = 0;
    private boolean currentSpiking = false;

    private static final int SPIKE_DEBOUNCE_CYCLES = 5;

    // Jam handling variables (needed for your runIntake logic)
    private boolean reversing = false;
    private double lastForwardSpeed = 0.0;
    private final Timer jamTimer = new Timer();
        public double EJECT_SPEED;
        private static final double JAM_REVERSE_SPEED = -0.5;
        private static final double JAM_REVERSE_TIME_SEC = 0.25;
    
        public Intake() {
            intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    
            SparkMaxConfig config = new SparkMaxConfig();
            config
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(INTAKE_CURRENT_LIMIT)
                .inverted(false)
                .openLoopRampRate(0.0)
                .closedLoopRampRate(0.0);
    
            intakeMotor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            );
    
            DigitalInput sensorTemp;
            try {
                sensorTemp = new DigitalInput(BALL_SENSOR_DIO_PORT);
            } catch (Exception e) {
                sensorTemp = null;
            }
            ballSensor = sensorTemp;
        }
    
        public void runIntake(double speed) {
            if (!reversing) {
                lastForwardSpeed = speed;
                intakeMotor.set(speed);
            }
                    
        }
    
        public void stopIntake() {
            intakeMotor.stopMotor();
        }
    
        // Current monitoring
        public double getCurrentDraw() {
            return intakeMotor.getOutputCurrent();
        }
    
        public boolean isCurrentSpiking() {
            return currentSpiking;
        }
    
        private void updateCurrentSpikeState() {
            boolean runningForward = intakeMotor.get() > 0.0;
    
            if (runningForward && getCurrentDraw() > INTAKE_CURRENT_SPIKE_THRESHOLD) {
                currentSpikeCounter++;
            } else {
                currentSpikeCounter = Math.max(currentSpikeCounter - 1, 0);
            }
    
            if (currentSpikeCounter >= SPIKE_DEBOUNCE_CYCLES && runningForward && !reversing) {
                currentSpiking = true;
                startJamReverse();
            }
    
            if (currentSpikeCounter == 0) {
                currentSpiking = false;
            }
    
            updateJamReverse();
        }
    
        private void startJamReverse() {
            reversing = true;
            jamTimer.reset();
            jamTimer.start();
            intakeMotor.set(JAM_REVERSE_SPEED);
        }
    
        private void updateJamReverse() {
            if (reversing && jamTimer.hasElapsed(JAM_REVERSE_TIME_SEC)) {
                reversing = false;
                jamTimer.stop();
                intakeMotor.set(lastForwardSpeed);
                currentSpikeCounter = 0;
                currentSpiking = false;
            }
        }
    
        // Ball detection
        public boolean hasBall() {
            if (ballSensor == null) {
                return false;
            }
    
            boolean rawState = ballSensor.get();
            return BALL_SENSOR_INVERTED ? !rawState : rawState;
        }
    
        public void logIntakeTelemetry() {
            SmartDashboard.putNumber("Intake Motor Output", intakeMotor.get());
            SmartDashboard.putNumber("Intake Motor Current (A)", getCurrentDraw());
            SmartDashboard.putBoolean("Intake Current Spike", isCurrentSpiking());
            SmartDashboard.putBoolean("Intake Ball Detected", hasBall());
        }
        public Command intakeCommand() {
            return Commands.run(() -> runIntake(INTAKE_SPEED), this)
                           .finallyDo(interrupted -> stopIntake());
        }
    
        public Command ejectCommand() {
            return Commands.run(() -> runIntake(EJECT_SPEED), this)
                       .finallyDo(interrupted -> stopIntake());
    }

    @Override
    public void periodic() {
        updateCurrentSpikeState();
        logIntakeTelemetry();
    }

    @Override
    public void simulationPeriodic() {}
}

   