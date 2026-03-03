package frc.robot.subsystems.vision;

import java.util.ArrayList;
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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

class RaspberryPi {
    // record TargetPair(PhotonTrackedTarget target,double ambiguity){}
    private record Pair<A,B>(A a,B b){};
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private double lastTimestamp;
    private double lastCheck;
    private String cameraName;
    private AprilTagFieldLayout field;
    RaspberryPi(String cameraName, Transform3d cam /*what is this*/){
        this.camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
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
    // private boolean shouldUseMeasurement(PhotonPipelineResult result , EstimatedRobotPose pose){
    //     // var result = pair.a();
    //     // var pose = pair.b();
 
    // }
    Optional<VisionMeasurement> processCameraResult(){
        
        var poses = new ArrayList<Double>();
        for (PhotonPipelineResult r : this.camera.getAllUnreadResults()){
            var optResult = newResultPose(r);
            if (!optResult.isPresent()){
                continue;
            }
            var result = optResult.get();
            if (result.shouldUseMeasurement()){
                SmartDashboard.putString(
                    String.format("Vision/%s/CamStatus", this.cameraName),
                    "Result Rejected (out of bounds or too high ambiguity)");
                    continue;
                }else{
                    Matrix<N3,N1> stdDevs = result.calculateStandardDeviations();
                };
        }
        
    }

    Optional<ResultPose> newResultPose(PhotonPipelineResult result){
            
            if (!result.hasTargets()){
                return Optional.empty();
            }
            var optPose = estimator.estimateCoprocMultiTagPose(result);
            if (!optPose.isPresent()){
                 return Optional.empty();   
            }
            return Optional.of(new ResultPose(result, optPose.get()));
        }
    
    private class ResultPose{
        EstimatedRobotPose pose;
        PhotonPipelineResult result;

        ResultPose(PhotonPipelineResult result, EstimatedRobotPose pose){
            this.result = result;
            this.pose = pose;
            
        }
        boolean shouldUseMeasurement(){
            var pose2d = pose.estimatedPose.toPose2d();
            double fieldLength = field.getFieldLength();
            double fieldWidth = field.getFieldWidth();
            if (pose2d.getX() < 0 || pose2d.getX() > fieldLength || pose2d.getY() < 0 || pose2d.getY() > fieldWidth ){
                return false;
            } 
            if (result.getTargets().size() == 1){
                double ambiguity = result.getBestTarget().getPoseAmbiguity();
                if (VisionConstants.MAX_AMBIGUITY < ambiguity){
                    return false;
                }
            }
            return true;
        }

        private Matrix<N3,N1> calculateStandardDeviations(
            double averageDistance
        ){
            int numTags = pose.targetsUsed.size();
            if (numTags >= VisionConstants.MIN_TAGS_FOR_MULTI_TAG){
                return VisionConstants.MULTI_TAG_STDDEVS;
            }
            if (averageDistance <2.0){
                return VisionConstants.SINGLE_TAG_CLOSE_STDDEVS;
            }else{
                return VisionConstants.SINGLE_TAG_FAR_STDDEVS;
            }
        }
   }
}