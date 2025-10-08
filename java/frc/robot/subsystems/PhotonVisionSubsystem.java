package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhotonVisionConstants;


public class PhotonVisionSubsystem extends SubsystemBase{

    private final PhotonCamera leftCamera; //The left camera
    private final PhotonCamera rightCamera; //The right camera

    public PhotonVisionSubsystem(){
        leftCamera = PhotonVisionConstants.leftCamera;
        rightCamera = PhotonVisionConstants.rightCamera;
    }

    /**
     * Returns the latest result from the camera
     * @param camera The {@link PhotonCamera} that we want the latest readings from
     * @return The latest {@link PhotonPipelineResult} from the camera
    */
    public PhotonPipelineResult getLatestResult(PhotonCamera camera){
        return camera.getLatestResult();
    }

    /**
     * Returns readings from an april tag
     * @param isLeftCamera Whether to use the left camera as opposed to the right
     * @param id What id to look for
     * @return A {@link PhotonTrackedTarget}
     */
    public Optional<PhotonTrackedTarget> getTarget(boolean isLeftCamera, int id) {
        List<PhotonTrackedTarget> targets = isLeftCamera ? leftCamera.getLatestResult().getTargets() : rightCamera.getLatestResult().getTargets();
        return targets.stream()
            .filter(target -> target.getFiducialId() == id)
            .findFirst();
    }

    public int getClosestTargetID(boolean isLeftCamera) {
        double closestDist = Double.MAX_VALUE;
        int closestTagID = 0;
        double dist;
        List<PhotonTrackedTarget> targets = isLeftCamera ? leftCamera.getLatestResult().getTargets() : rightCamera.getLatestResult().getTargets();
        for(PhotonTrackedTarget target : targets){
            //Pythagorean Theorum
            dist = Math.sqrt(Math.pow(target.getBestCameraToTarget().getX(), 2) + Math.pow(target.getBestCameraToTarget().getY(), 2));
            if(dist < closestDist){
                closestDist = dist;
                closestTagID = target.getFiducialId();
            }
        }
        return closestTagID;
    }

    public int findClosestTagIDFromBothCameras(){
        //Get closest tag ID's from each camera
        int leftCamTagID = getClosestTargetID(true);
        int rightCamTagID = getClosestTargetID(false);

        //If neither of the cameras see a tag
        if(leftCamTagID == 0 && rightCamTagID == 0) return 0;
        else if(leftCamTagID == 0 && rightCamTagID != 0) return rightCamTagID;
        else if(leftCamTagID != 0 && rightCamTagID == 0) return leftCamTagID;

        //If each camera has a different closest tag (very rare)
        else if(leftCamTagID != rightCamTagID){
            double leftCamTagDist;
            double rightCamTagDist;
            if(getTarget(true, leftCamTagID).isPresent()){ //To avoid crashing
                leftCamTagDist = 
                    Math.sqrt(
                        Math.pow(getTarget(true, leftCamTagID).get().getBestCameraToTarget().getX(), 2) + 
                        Math.pow(getTarget(true, leftCamTagID).get().getBestCameraToTarget().getY(), 2));
            }else leftCamTagDist = Double.MAX_VALUE;
            if(getTarget(false, rightCamTagID).isPresent()){ //To avoid crashing
                rightCamTagDist = 
                    Math.sqrt(
                        Math.pow(getTarget(false, rightCamTagID).get().getBestCameraToTarget().getX(), 2) + 
                        Math.pow(getTarget(false, rightCamTagID).get().getBestCameraToTarget().getY(), 2));
            }else rightCamTagDist = Double.MAX_VALUE;
            return leftCamTagDist < rightCamTagDist ? leftCamTagID : rightCamTagID;
        }

        //If both tags see the same closest target
        else if(leftCamTagID == rightCamTagID) return leftCamTagID;

        //Code should never reach this point, this line is just to avoid crashing
        else return 0;
    }

    public Optional<Pose3d> getFieldRelativeTagPosition(int tagID){
        return FieldConstants.aprilTagFieldLayout.getTagPose(tagID);
    }
}