package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

class RaspberryPi {
    // record TargetPair(PhotonTrackedTarget target,double ambiguity){}
    
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private double lastTimestamp;
    private double lastCheck;
    RaspberryPi(String cameraName, Transform3d cam /*what is this*/){
        this.camera = new PhotonCamera(cameraName);
        this.estimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cam
        );
        this.lastCheck = getNowSeconds();
        this.lastTimestamp = -1.0;
    }
    double getNowSeconds(){
        return (double) System.currentTimeMillis()/1000;
    }
    void updateFreshness(double timestampSeconds){
        double nowSeconds = getNowSeconds();
        if (this.lastTimestamp < timestampSeconds){
            this.lastTimestamp = timestampSeconds;
            this.lastCheck = nowSeconds;
        }
    }
    Optional<PhotonTrackedTarget> getBestTarget(){
        // if (this.camera.getAllUnreadResults().hasTargets()){
        //     var target = this.camera.getAllUnreadResults().getBestTarget();
        //     return Optional.of(
        //         target
        //         );
        // } else return Optional.empty();
        return this.camera.getAllUnreadResults().stream().map(r -> r.getBestTarget()).reduce((acc,t) -> t.getPoseAmbiguity() < acc.getPoseAmbiguity() ? t : acc);        
    }
    List<Integer> getVisibleTags(){
        return this.camera.getAllUnreadResults().stream().flatMap(r -> r.getTargets().stream().map(t -> t.getFiducialId())).toList();
    }

    

}