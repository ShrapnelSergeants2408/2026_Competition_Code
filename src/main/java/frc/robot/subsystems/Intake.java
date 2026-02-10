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

    public final SparkMax intakeMotor; // Intake motor
    private final DigitalInput ballSensor; // Beam break sensor

    private int currentSpikeCounter = 0; // Spike debounce counter
    private boolean currentSpiking = false; // Spike state
    private static final int SPIKE_DEBOUNCE_CYCLES = 5;

    private boolean reversing = false; // Jam reverse active
    private double lastForwardSpeed = 0.0; // Last forward speed before jam
    private final Timer jamTimer = new Timer();
    private static final double JAM_REVERSE_SPEED = -0.5; // Jam reverse speed
    private static final double JAM_REVERSE_TIME_SEC = 0.25; // Jam reverse duration
    
        // Intake states
        public enum IntakeState {
            STOPPED,    // Motor stopped
            RUNNING,    // Running forward
            EJECTING,   // Running backward
            JAM_CLEAR   // Temporarily reversing to clear jam
        }
        private IntakeState currentState = IntakeState.STOPPED;
    
        //  Constructor 
        public Intake() {
            // Motor setup
            intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            config
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(INTAKE_CURRENT_LIMIT)
                .inverted(false)
                .openLoopRampRate(0.0)
                .closedLoopRampRate(0.0);
            intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
            // Sensor setup with safe null
            DigitalInput sensorTemp;
            try {
                sensorTemp = new DigitalInput(BALL_SENSOR_DIO_PORT);
            } catch (Exception e) {
                sensorTemp = null;
            }
            ballSensor = sensorTemp;
        }
    
        //  Intake Control 
        public void runIntake(double speed) {
            if (!reversing) {
                lastForwardSpeed = speed;
                intakeMotor.set(speed);
                currentState = speed > 0 ? IntakeState.RUNNING :
                               speed < 0 ? IntakeState.EJECTING :
                               IntakeState.STOPPED;
            }
        }
    
        public void stopIntake() {
            intakeMotor.stopMotor();
            currentState = IntakeState.STOPPED;
        }
    
        //  Current Monitoring 
        public double getCurrentDraw() {
            return intakeMotor.getOutputCurrent(); // Motor current
        }
    
        public boolean isCurrentSpiking() {
            return currentSpiking; // Spike flag
        }
    
        private void updateCurrentSpikeState() {
            boolean runningForward = currentState == IntakeState.RUNNING;
    
            if (runningForward && getCurrentDraw() > INTAKE_CURRENT_SPIKE_THRESHOLD) {
                currentSpikeCounter++; // Spike detected
            } else {
                currentSpikeCounter = Math.max(currentSpikeCounter - 1, 0); // Debounce
            }
    
            if (currentSpikeCounter >= SPIKE_DEBOUNCE_CYCLES && runningForward && !reversing) {
                currentSpiking = true;
                startJamReverse(); // Auto-reverse for jam
            }
    
            if (currentSpikeCounter == 0) {
                currentSpiking = false; // Reset spike
            }
    
            updateJamReverse(); // Handle ongoing jam reverse
        }
    
        //  Jam Handling 
        private void startJamReverse() {
            reversing = true;
            jamTimer.reset();
            jamTimer.start();
            intakeMotor.set(JAM_REVERSE_SPEED);
            currentState = IntakeState.JAM_CLEAR;
        }
    
        private void updateJamReverse() {
            if (reversing && jamTimer.hasElapsed(JAM_REVERSE_TIME_SEC)) {
                reversing = false;
                jamTimer.stop();
                intakeMotor.set(lastForwardSpeed);
                currentSpikeCounter = 0;
                currentSpiking = false;
                currentState = lastForwardSpeed > 0 ? IntakeState.RUNNING :
                               lastForwardSpeed < 0 ? IntakeState.EJECTING :
                               IntakeState.STOPPED;
            }
        }
    
        //  Ball Detection 
        public boolean hasBall() {
            if (ballSensor == null) {
                return false; // Sensor not installed
            }
            boolean rawState = ballSensor.get();
            return BALL_SENSOR_INVERTED ? !rawState : rawState; // Invert if needed
        }
    
        //  Telemetry 
        public void logIntakeTelemetry() {
            SmartDashboard.putNumber("Intake Motor Output", intakeMotor.get());
            SmartDashboard.putNumber("Intake Motor Current (A)", getCurrentDraw());
            SmartDashboard.putBoolean("Intake Current Spike", isCurrentSpiking());
            SmartDashboard.putBoolean("Intake Ball Detected", hasBall());
            SmartDashboard.putBoolean("Intake Jam Clearing", reversing);
            SmartDashboard.putString("Intake State", currentState.name());
        }
    
        //  Commands 
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
