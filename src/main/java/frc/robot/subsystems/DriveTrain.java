package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrain.*;
// this uses swerve drive. we may switch to Differential Drive.
public class DriveTrain extends SubsystemBase
{
    public DriveTrain(){}
    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
    // SwerveDriveKinematics kinematics;
    // public DriveTrain() {
    //     double wheelDistanceMeters = Constants.DriveTrain.WHEEL_DISTANCE / 1000;
    //     kinematics = new SwerveDriveKinematics(
    //         new Translation2d(wheelDistanceMeters,wheelDistanceMeters),

    //     );
    // }
}
