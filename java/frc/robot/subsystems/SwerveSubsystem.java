package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhotonVisionConstants;

public class SwerveSubsystem extends SubsystemBase {

    //Creates and defines the front left swerve module
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    //Creates and defines the front right swerve module
    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    //Creates and defines the back left swerve module
    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    //Creates and defines the back right swerve module
    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    

    //Sets up the gyro which reads the robot's current rotation
    private final static AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
     
    //Sets up the field that will be put on the smart dashboard
    private final Field2d field = new Field2d();

    //Sets up the pose estimator, which fuses pose estimates from the gyro with pose estimates from april tags
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getModulePositions(),
        new Pose2d()
    );

    private RobotConfig config;
    
    public SwerveSubsystem() {
        //Set robot's current direction to forward (when the robot is turned on)
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }).start();

        //Fetch the robot config from the pathplanner GUI
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        //Configure AutoBuilder
        AutoBuilder.configure(
            this::getPose, 
            this::resetPoseEstimator, 
            this::getSpeeds, 
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(4.5, 0, 0), //Translation constants
                new PIDConstants(4.5, 0, 0) //Rotation constants
            ),
            config,
            () -> isRedAlliance(),
            this
        );

        //Puts the field image on the smart dashboard
        SmartDashboard.putData("Field", field);
    }

    /**
     * Returns what alliance we are currently on (red or blue). Defaults to blue if no alliance is found.
     * @return true if on red, false if on blue
     */
    public boolean isRedAlliance(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    /**
     * Resets the gyro
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the robot's current heading
     * @return the robot's heading
     */
    public double getHeading() {
       return Math.IEEEremainder(-gyro.getAngle(), 360); //-gyro is because CCW is positive in calculations
    }

    /**
     * Returns the robot's current rotation
     * @return the robot's current rotation
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Returns the position's of all four swerve modules with front left as the first in the list, front
     * right as the second, back left as the third, and back right as the fourth
     * @return a {@link SwerveModulePosition} array with each swerve module's position
     */
    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[] {
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            backLeft.getModulePosition(),
            backRight.getModulePosition()};
    }
    
    /**
     * Returns the robot's current pose from the pose estimator
     * @return the robot's current pose
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current speeds of the swerve modules
     * @return the current {@link ChassisSpeeds}
     */
    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Drives the robot robot-relative (whichever direction the robot is facing is considered forward)
     * @param robotRelativeSpeeds the desired speeds
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    /**
     * Resets the pose estimator
     * @param pose the robot's current pose
     */
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        poseEstimator.update(getRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Yaw", gyro.getYaw());
        SmartDashboard.putString("Estimator Location", getPose().getTranslation().toString());

        //Uncomment this when calculating encoder offsets
        /*
        SmartDashboard.putNumber("FL Rad", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FR Rad", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BL Rad", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BR Rad", backRight.getAbsoluteEncoderRad());
        */
    }

    /**
     * Adds a vision estimate to the pose estimator
     * @param visionPose the position calculated by the cameras
     * @param timestamp when the position was calculated
     * @param stdDevs standard deviations of the calculated position
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }

    /**
     * Stops the swerve modules
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Sets the swerve modules to the desired states
     * @param desiredStates the desired states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Average Josh method
     */
    public void XWingAttackFormationEngageNoMovementAutomaticAntiDefenseMeasuresForOptimalImmovability() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        moduleStates[0] = new SwerveModuleState(.01, Rotation2d.fromDegrees(45));
        moduleStates[1] = new SwerveModuleState(.01, Rotation2d.fromDegrees(135));
        moduleStates[2] = new SwerveModuleState(.01, Rotation2d.fromDegrees(315));
        moduleStates[3] = new SwerveModuleState(.01, Rotation2d.fromDegrees(225));
        setModuleStates(moduleStates);
    }

    /**
     * Returns the current state of the swerve modules, with front left as the first in the list, front
     * right as the second, back left as the third, and back right as the fourth
     * @return the swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        
        states[0] = frontLeft.getState();
        states[1] = frontRight.getState();
        states[2] = backLeft.getState();
        states[3] = backRight.getState();

        return states;
    }

    public Pose2d getIdealRobotPose(Pose2d tagPose){
        double tagAngle = tagPose.getRotation().getRadians();
        double yOffset = Math.sin(tagAngle) * PhotonVisionConstants.kIdealXDistanceToTagMeters;
        double xOffset = Math.cos(tagAngle) * PhotonVisionConstants.kIdealXDistanceToTagMeters;
        return new Pose2d(
            tagPose.getX() - xOffset,
            tagPose.getY() - yOffset,
            Rotation2d.fromRadians(MathUtil.angleModulus(tagAngle + Math.PI))
        );
    }

    /**
     * Command to set the robot's current rotation to zero degrees
     */
    public Command orientBot() {
        return this.runOnce(() -> this.zeroHeading());
    }

    /**
     * Command to reset the odometry of the robot
     */
    public Command resetOdometry(){
        return this.runOnce(() -> this.resetPoseEstimator(getPose()));
    }

    /**
     * Command to initiate XWingAttackFormationEngageNoMovementAutomaticAntiDefenseMeasuresForOptimalImmovability
     */
    public Command XGamesMode() {
        return this.runOnce(() -> this.XWingAttackFormationEngageNoMovementAutomaticAntiDefenseMeasuresForOptimalImmovability());
    }


    //The following methods are getters for the autonomous generator
    
    /**
     * Returns the 'center position' that corresponds to a given id. This is a method for the example
     * autonomous generator.
     * @param id the id of the position
     * @return the position
     */
    public Pose2d getCenterPosition(int id) {
        if(isRedAlliance()) return FieldConstants.kRedCenterPositions[id-1];
        else return FieldConstants.kBlueCenterPositions[id-1];
    }
}