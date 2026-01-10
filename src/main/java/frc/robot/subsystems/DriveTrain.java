import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class DriveTrain extends SubsystemBase{
    SwerveDriveKinematics kinematics;

    public SwerveDrive() {
        // move to constants later, this defines center of robot to center of wheel in cm
        double wheelDistanceMeters = DriveTrain.WHEEL_DISTANCE / 1000;
        kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelDistanceMeters,wheelDistanceMeters),

        );
    }
}