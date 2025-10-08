/*
This file gets the targets that the camera sees, and uses them to calculate where it is on the field. The way this works is it
figures out the id of the tag, and retrieves the field relative position of that tag. It then calculates where the robot is in
relation to the tag, and uses those numbers plus the tag's field relative position to calculate the robot's field relative
position is. It then combines those reading with odometry reading to create an accurate estimate of the robot's position
*/

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
    private final PhotonCamera leftCamera, rightCamera;
    private final PhotonPoseEstimator leftEstimator, rightEstimator;
    private final EstimateConsumer estConsumer;
    private final Transform3d leftTransform, rightTransform;
    private final Matrix<N3, N1> singleTagStdDevs, multiTagStdDevs;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    private final SwerveSubsystem swerveSubsystem;

    // Simulation
    private PhotonCameraSim leftSim, rightSim;
    private VisionSystemSim visionSim;

    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
     *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public Vision(EstimateConsumer estConsumer, SwerveSubsystem swerveSubsystem) {
        this.estConsumer = estConsumer;

        this.swerveSubsystem = swerveSubsystem;

        //Default standard deviations if the camera only sees one tag
        singleTagStdDevs = VecBuilder.fill(4, 4, 8);

        //Default standard deviations if the camera sees multiple tags
        multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        //Creates an object to represent each camera
        leftCamera = PhotonVisionConstants.leftCamera;
        rightCamera = PhotonVisionConstants.rightCamera;
        
        //Creats an object to represent the positions of the april tags on the field
        aprilTagFieldLayout = FieldConstants.aprilTagFieldLayout;

        //Creats an object to represent the transformation from the cameras' position to the center of the robot
        leftTransform = PhotonVisionConstants.kLeftRobotToCam;
        rightTransform = PhotonVisionConstants.kRightRobotToCam;

        /*        
        In the photon vision docs, they also pass the camera into the PhotonPoseEstimator class. However, this
        is outdated, the more recent pose estimator doesn't take in the camera in the constructor

        Creates the estimator, passes in the april tag positions, the strategy to use when calculating the position of the robot, and
        the transformation from the left camera to the center of the robot
        */
        leftEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftTransform);
        leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


        //Same thing but for the right camera
        rightEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightTransform);
        rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation, turned off for now because i don't really understand what it's for
        if (Robot.isSimulation() && PhotonVisionConstants.kEnableSimulation) {
            //Create the vision system simulation which handles cameras and targets on the field
            visionSim = new VisionSystemSim("main");
            //Add all the AprilTags inside the tag layout as visible targets to this simulated field
            visionSim.addAprilTags(aprilTagFieldLayout);
            //Create simulated camera properties. These can be set to mimic your actual camera
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            //Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible targets
            leftSim = new PhotonCameraSim(leftCamera, cameraProp);
            rightSim = new PhotonCameraSim(rightCamera, cameraProp);
            //Add the simulated camera to view the targets on this simulated field
            visionSim.addCamera(leftSim, leftTransform);
            visionSim.addCamera(rightSim, rightTransform);
        }
            
    }

    public void periodic(){
        //Set reference poses
        leftEstimator.setReferencePose(swerveSubsystem.getPose());
        rightEstimator.setReferencePose(swerveSubsystem.getPose());

        //Retrieve the images from the cameras
        PhotonPipelineResult leftResult = leftCamera.getLatestResult();
        PhotonPipelineResult rightResult = rightCamera.getLatestResult();

        //Adds the estimation to the robot's position if it is deemed to be reliable
        useIfReliable(leftEstimator.update(leftResult), leftResult.getTargets());
        useIfReliable(rightEstimator.update(rightResult), rightResult.getTargets());
    }

    /**
     * Adds the vision estimation to the robot's position estimator if the estimation is deemed to be reliable
     * @param poseOpt the vision estimate
     * @param targets the tags used in the estimate
     */
    private void useIfReliable(Optional<EstimatedRobotPose> poseOpt, List<PhotonTrackedTarget> targets) {
        poseOpt.ifPresent(pose -> {
            Pose2d visionPose = pose.estimatedPose.toPose2d(); //The vision estimation pose
            Pose2d visionPoseAdjusted;

            int numUsable = computeStats(targets).numUsable; //The number of usable targets in the vision estimate
            double avgDist = computeStats(targets).avgDist; //The average distance from the cameras to the seen targets

            double adjustedRotation = 0; //A variable to store the adjusted pose rotation

            //If the camera findings are deemed to be reliable
            if (numUsable >= PhotonVisionConstants.kMinimumTagsRequiredForAccuratePoseEstimate && !Double.isNaN(avgDist) && avgDist < PhotonVisionConstants.kMaximumAcceptableDistanceForAccuratePoseEstimateMeters) {
                Matrix<N3, N1> stdDevs = calculateStdDevs(poseOpt, targets, numUsable, avgDist); //Calculates the standard deviations
                adjustedRotation = visionPose.getRotation().getDegrees();
                SmartDashboard.putNumber("Vision/X", visionPose.getX());
                SmartDashboard.putNumber("Vision/Y", visionPose.getY());
                SmartDashboard.putNumber("Vision/Deg", visionPose.getRotation().getDegrees());
                visionPoseAdjusted = new Pose2d(visionPose.getTranslation(), Rotation2d.fromDegrees(adjustedRotation)); //Create a new estimated pose that includes the adjusted rotation
                estConsumer.accept(visionPoseAdjusted, pose.timestampSeconds, stdDevs); //Send estimate to pose estimator
            }
        });
    }

    /**
     * Calculates the standard deviations of a vision pose estimation
     * @param poseOpt the vision pose estimation
     * @param targets the tags used in the estimate
     * @param numUsable the number of usable targets in the estimate
     * @param avgDist the average distance from the camera to the targets
     * @return the standard deviations
     */
    private Matrix<N3, N1> calculateStdDevs(Optional<EstimatedRobotPose> poseOpt, List<PhotonTrackedTarget> targets, int numUsable, double avgDist) {
        if (poseOpt.isEmpty()) return PhotonVisionConstants.defaultStandardDeviations; //To avoid crashing
    
        //If there are no usable tags, or if the average distance couldn't be calculated
        if (numUsable == 0 || Double.isNaN(avgDist)) return PhotonVisionConstants.defaultStandardDeviations; //return conservative default
    
        //If there aren't enough tags seen, or if the average distance is too high
        if (numUsable < PhotonVisionConstants.kMinimumTagsRequiredForAccuratePoseEstimate || avgDist >= PhotonVisionConstants.kMaximumAcceptableDistanceForAccuratePoseEstimateMeters) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE); //Reject
        }

        Matrix<N3, N1> base = (numUsable > 1) ? multiTagStdDevs : singleTagStdDevs; //Fetch base standard deviations
        double scale = 1.0 + (avgDist*avgDist / 30.0); //Scale based on average distance to target
        return base.times(scale / Math.max(1.0, Math.sqrt(numUsable))); //Increase scale based on number of tags seen and return
    }

    private static final class TargetStats{
        final double avgDist;
        final int numUsable;
        TargetStats(double avgDist, int numUsable){ 
            this.avgDist = avgDist;
            this.numUsable = numUsable;
        }
    }

    private TargetStats computeStats(List<PhotonTrackedTarget> targets){
        if (targets == null || targets.isEmpty()) return new TargetStats(Double.NaN, 0);
        double avgDist = 0;
        int numUsable = 0;
        for (var target : targets){
            if(target.getBestCameraToTarget() != null){
                avgDist += Math.sqrt(Math.pow(target.getBestCameraToTarget().getX(), 2) + Math.pow(target.getBestCameraToTarget().getY(), 2));
                numUsable++;
            }
        }
        return new TargetStats(numUsable > 0 ? avgDist / numUsable : Double.NaN, numUsable);
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        if (visionSim != null) visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation() && PhotonVisionConstants.kEnableSimulation && visionSim != null) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation() || !PhotonVisionConstants.kEnableSimulation || visionSim == null) return null;
        return visionSim.getDebugField();
        
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}