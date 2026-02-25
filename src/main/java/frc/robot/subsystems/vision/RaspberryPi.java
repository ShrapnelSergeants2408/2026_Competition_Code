package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;

class RaspberryPi {
    // record TargetPair(PhotonTrackedTarget target,double ambiguity){}
    private record Pair<A,B>(A a,B b){};
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private double lastTimestamp;
    private double lastCheck;
    private AprilTagFieldLayout field;
    RaspberryPi(String cameraName, Transform3d cam /*what is this*/){
        this.camera = new PhotonCamera(cameraName);
        this.field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        this.estimator = new PhotonPoseEstimator(
            this.field, 
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
        return this.camera.getAllUnreadResults()
        .stream()
        .map(r -> r.getBestTarget())
        .reduce( // probably doesnt work. 
            (acc,t) -> t.getPoseAmbiguity() < acc.getPoseAmbiguity() ? t : acc
            );
    }
    List<Integer> getVisibleTags(){
        return this.camera.getAllUnreadResults().stream().flatMap(r -> r.getTargets().stream().map(t -> t.getFiducialId())).toList();
    }

    // Optional<VisionMeasurement> getBestVisionMeasurement(){
    //     var measurement = 
    // }
    // private PhotonPipelineResult getLatestResult(){
        
    // }
    private boolean shouldUseMeasurement(Pair<PhotonPipelineResult,EstimatedRobotPose> pair){
        var result = pair.a();
        var pose = pair.b();
        var pose2d = pose.estimatedPose.toPose2d();
        double fieldLength = this.field.getFieldLength();
        double fieldWidth = this.field.getFieldWidth();
        if (pose2d.getX() < 0 || pose2d.getX() > fieldLength || pose2d.getY() < 0 || pose2d.getY() > fieldWidth ){
            return false;
        } 
        if (result.getTargets().size() == 1){
            double ambiguity = result.getBestTarget().getPoseAmbiguity();
            if (VisionConstants.MAX_AMBIGUITY < ambiguity){
                return false;
            }
        }
    }
    Optional<VisionMeasurement> processCameraResult(){
        
        var poses = this.camera.getAllUnreadResults().stream()
                .filter(r -> r.hasTargets())
                .map(r -> 
                    new Pair<PhotonPipelineResult,Optional<EstimatedRobotPose>>(
                        r,
                        this.estimator.estimateCoprocMultiTagPose(r)
                    )
                ).filter(p -> p.b().isPresent()).map(p -> 
                    new Pair<PhotonPipelineResult,EstimatedRobotPose>(p.a(),p.b().get())
                ).filter(p -> shouldUseMeasurement(p))
        for (var pose : poses){
            
        }
    }
    

}