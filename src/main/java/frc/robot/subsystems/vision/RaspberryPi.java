package frc.robot.subsystems.vision;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

class RaspberryPi {
    // record TargetPair(PhotonTrackedTarget target,double ambiguity){}
    private PhotonCamera camera;
    private double lastTimestamp;
    private double lastCheck;
    RaspberryPi(){

    }
    void updateFreshness(double timestampSeconds){
        double nowSeconds = (double) System.currentTimeMillis()/1000;
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
        var results = this.camera.getAllUnreadResults();
        List<Double> ambiguity = results.stream().map(r -> r.getPoseAmbiguity()).mapToDouble(Double::valueOf).toList();
    }
    List<Integer> getVisibleTags(){
        var results = this.camera.getAllUnreadResults();
        // for (var result:results){

        // }
    }
}