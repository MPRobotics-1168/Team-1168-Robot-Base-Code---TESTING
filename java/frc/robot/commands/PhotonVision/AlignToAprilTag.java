package frc.robot.commands.PhotonVision;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToAprilTag extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final PhotonVisionSubsystem photonVisionSubsystem;

    private final int targetId; //ID of the desired tag

    private final PIDController xController; //Vertical PID controller
    private final PIDController yController; //Horizontal PID controller
    private final PIDController yawController; //Rotational PID controller

    private final double xTarget; //Target position vertically
    private final double yTarget; //Target position horizontally
    private final double rotTarget; //Target rotation

    private final double toleranceMeters; //Maximum acceptable horizontal/vertical error
    private final double toleranceDegrees; //Maximum acceptable rotational error

    private Optional<PhotonTrackedTarget> leftTargetOpt; //The reading from the left camera
    private Optional<PhotonTrackedTarget> rightTargetOpt; //The reading from the right camera

    private boolean aligned = false; //Whether the robot is aligned
    private boolean xAligned = false; //Whether the robot is vertically aligned
    private boolean yAligned = false; //Whether the robot is horizontally aligned
    private boolean rotAligned = false; //Whether the robot is rotationally aligned

    private double x; //Vertical offset
    private double y; //Horizontal offset
    private double rot; //Rotational offset

    private double xSpeed; //Vertical speed
    private double ySpeed; //Horizontal speed
    private double rotSpeed; //Rotational speed

    private int numberOfTagsSeen; //Number of cameras that see the desired april tag

    //POSSIBLE PROBLEM: robot is moving robot relative, maybe i should translate speeds from just horizontal/vertical movement to
    //field relative movement. (didn't think i needed to but i might)

    public AlignToAprilTag(SwerveSubsystem swerveSubsystem, PhotonVisionSubsystem photonVisionSubsystem, int targetId, boolean isLeftPost) {
        this.swerveSubsystem = swerveSubsystem;
        this.photonVisionSubsystem = photonVisionSubsystem;
        this.targetId = targetId;

        xTarget = PhotonVisionConstants.kIdealXDistanceToTagMeters;
        yTarget = PhotonVisionConstants.kIdealYDistanceToTagMeters;
        rotTarget = PhotonVisionConstants.kIdealRotationalOffsetToTagDegrees;

        toleranceMeters = PhotonVisionConstants.kAprilTagAlignToleranceMeters;
        toleranceDegrees = PhotonVisionConstants.kAprilTagAlignToleranceDegrees;

        xController = new PIDController(0.9, 0, 0);
        yController = new PIDController(0.9, 0, 0);
        yawController = new PIDController(1, 0, 0);

        yawController.enableContinuousInput(-Math.PI, Math.PI); //POSSIBLE PROBLEM: degrees or radians??

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute() {
        //Retrieve values from the left and right cameras
        leftTargetOpt = photonVisionSubsystem.getTarget(true, targetId);
        rightTargetOpt = photonVisionSubsystem.getTarget(false, targetId);

        //If neither of the cameras see the desired april tag
        if(leftTargetOpt.isEmpty() && rightTargetOpt.isEmpty()){
            //Stop the wheels
            swerveSubsystem.stopModules();
            //Return to the beginning
            return;
        }

        //Reset values
        x = 0;
        y = 0;
        rot = 0;
        numberOfTagsSeen = 0;

        //If the left camera sees the desired tag
        if (leftTargetOpt.isPresent()) {
            //Create an object to hold the transformation from the left camera to the tag
            Transform3d leftCamToTarget = leftTargetOpt.get().getBestCameraToTarget();
            //Create an object to hold the transformation from the robot to the tag
            Transform3d robotToTarget = PhotonVisionConstants.kLeftRobotToCam.plus(leftCamToTarget);
            //Add readings from the tag to the offset values
            x += robotToTarget.getX();
            y += robotToTarget.getY();
            rot += robotToTarget.getRotation().getZ() + Math.PI;
            //Set rot to a number betweeen -pi and pi
            rot = MathUtil.angleModulus(rot);
            //Add 1 to the variable that holds the number of cameras that see the desired tag
            numberOfTagsSeen++;
        }

        //If the right camera sees the desired tag
        if (rightTargetOpt.isPresent()) {
            //Create an object to hold the transformation from the right camera to the tag
            Transform3d rightCamToTarget = rightTargetOpt.get().getBestCameraToTarget();
            //Create an object to hold the transformation from the robot to the tag
            Transform3d robotToTarget = PhotonVisionConstants.kRightRobotToCam.plus(rightCamToTarget);
            //Add readings from the tag to the offset values
            x += robotToTarget.getX();
            y += robotToTarget.getY();
            rot += robotToTarget.getRotation().getZ() + Math.PI;
            //Set rot to a number betweeen -pi and pi
            rot = MathUtil.angleModulus(rot);
            //Add 1 to the variable that holds the number of cameras that see the desired tag
            numberOfTagsSeen++;
        }

        //Calculate the average of the offset values
        x /= numberOfTagsSeen;
        y /= numberOfTagsSeen;
        rot /= numberOfTagsSeen;

        //rot = Math.toDegrees(rot);

        //Generate speeds using PID controllers (outputs values from 0 to 1)
        xSpeed = xController.calculate(x, -xTarget);
        ySpeed = yController.calculate(y, -yTarget);
        rotSpeed = yawController.calculate(rot, rotTarget);

        //Set speeds to a range from 0 to the max speed
        xSpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        rotSpeed *= DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        
        //Calculate whether any offsets are at 0 (aligned)
        xAligned = Math.abs(x + xTarget) < toleranceMeters;
        yAligned = Math.abs(y - yTarget) < toleranceMeters + 0.02;
        rotAligned = Math.abs(rot - rotTarget) < toleranceDegrees;
        aligned = xAligned && yAligned && rotAligned;

        //Set minimum speeds
        xSpeed = Math.abs(xSpeed) < 0.1 && !xAligned ? Math.signum(xSpeed) * 0.1 : xSpeed;
        ySpeed = Math.abs(ySpeed) < 0.2 && !yAligned ? Math.signum(ySpeed) * 0.2 : ySpeed;

        //Set maximum speeds (only needed for vertical speed)
        xSpeed = Math.abs(xSpeed) > 2 ? Math.signum(xSpeed) * 2 : xSpeed;

        //Construct chassis speeds from individual speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds/*.fromFieldRelativeSpeeds*/(-xSpeed, -ySpeed, -rotSpeed);

        //Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //Set swerves to the desired module states
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("Aligned", aligned);
        return aligned;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
} 
