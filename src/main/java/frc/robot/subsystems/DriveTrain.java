package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrain.*;
// this uses swerve drive. we may switch to Differential Drive.
public class DriveTrain extends SubsystemBase{
    public DriveTrain(){
    }
    
    void driveArcade(double x, double y){}

    void driveTank(double x,double y){}

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        
    }
}
