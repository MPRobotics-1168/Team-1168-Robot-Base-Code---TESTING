package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

    /**
     * These class holds information about the individual swerve modules, like the gearing ratios, the
     * wheel diameter, conversion constants, etc.
     */
    public static final class ModuleConstants {
        //**REMINDER: Verify gearing ratios, they could be a bit off**
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //The diameter of the swerve wheels
        public static final double kDriveMotorGearRatio = 1 / 6.75; //The gearing ratio of the drive motor
        public static final double kTurningMotorGearRatio = 1 / 21.4286; //The gearing ratio of the turning motor
        public static final double kDriveEncoderMetersPerRotation = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; //How many meters the robot travels per 1 rotation of the drive motor shaft
        public static final double kTurningEncoderRadiansPerRotation = kTurningMotorGearRatio * 2 * Math.PI; //How many radians the wheels turns per 1 rotation of the turning motor shaft
        public static final double kDriveEncoderRPMToMetersPerSecond = kDriveEncoderMetersPerRotation / 60; //How many meters per second the wheel travels per 1 RPM of the drive motor shaft
        public static final double kTurningEncoderRPMToRadiansPerSecond = kTurningEncoderRadiansPerRotation / 60; //How many radians per second the wheel rotates per 1 RPM of the turning motor shaft
        public static final double kPTurning = 0.5;
    }

    /**
     * These class holds information about the position of the swerve modules relative to the center of
     * the robot, as well as motor/encoder port numbers, whether the encoders are reversed, physical max
     * speeds of the robot, and speed limits in teleoperated mode
     */
    public static final class DriveConstants {
        //Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(25);
        //Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(25);

        //Object to hold the translations from the center of the robot to each wheel position
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackWidth / 2, kWheelBase / 2),
            new Translation2d(kTrackWidth / 2, -kWheelBase / 2),
            new Translation2d(-kTrackWidth / 2, kWheelBase / 2),
            new Translation2d(-kTrackWidth / 2, -kWheelBase / 2));

        //The port that each drive motor is plugged into on the roborio
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 6;

        //The port that each turning motor is plugged into on the roborio
        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 5;

        //Whether each turning motor encoder is reversed
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        //Whether each drive motor encoder is reversed
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        //The port that each absolute encoder is plugged into on the roborio
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        //Whether each absolute encoder is reversed
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //Absolute encoder Offsets
        /*
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;
        */

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.044 + 0.007 - 0.02 + Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.129 + 0.028 + 0.005 + 0.005;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -0.52 - 0.084 + 0.023 + Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -1.02 - 0.083 - 0.02 - 0.021 + Math.PI + 0.445 + Math.PI/2;

        /*
        Physical max speeds are used to tell the computer what is the theoretical maximum speed that the
        robot can travel at. These speeds are very rarely reached, as they can damage the gears
         */
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5; //Actual maximum movement speed
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; //Actual maximum rotational speed

        /*
        These are the top speeds in teleoperated mode. The robot can physically go faster, but will be
        told not to.

        **NOTE: Do no increase to more than half of the physical max speed unless you edit the turbo speed
        fuction, because full turbo speed doubles the speed so if the speed is set to more than half of
        the physical max speed, and turbo speed is enabled, then the robot will try to go faster than it
        physically can, which could severely damage the motors.**
         */
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;

        
        //How much the robot can accelerate it's current speed in one second.
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3; //Translation
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3; //Rotation
    }

    /**
     * This class holds information about the driving aspect of the controllers, like which joystick axis
     * makes the robot move forward, which trigger enables turbo speed, etc. It also containes the set
     * joystick deadband
     */
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1; //Y Axis on left joystick
        public static final int kDriverXAxis = 0; //X Axis on left joystick, Change this to something like 7 or above to only let robot go straight
        public static final int kDriverRotAxis = 4; //X Axis on right joystick
        public static final int kSlowAxis = 2; //Left Trigger
        public static final int kTurboAxis = 3; //Right Trigger

        public static final double kDeadband = 0.09; //If a joystick value is below this, value is set to 0 (avoids unintentional movement, as well as controller drift)
    }

    /**
     * This class holds information about the cameras position relative to the center of the robot, as
     * well as april tag information that is used in pose estimation, and tolerances and offsets for the
     * align to april tag command
     */
    public static final class PhotonVisionConstants{
        public static final PhotonCamera leftCamera = new PhotonCamera("Left Camera");
        public static final PhotonCamera rightCamera = new PhotonCamera("Right Camera");

        public static final boolean kEnableSimulation = true;

        public static final Matrix<N3, N1> defaultStandardDeviations = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> defaultStdDevsSingleTag = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> defaultStdDevsMultiTag = VecBuilder.fill(0.5, 0.5, 1);

        /*
        public static final double kLeftCamToCenterVerticalMeters = 0; //Vertical offset from the center of the robot to the left camera
        public static final double kLeftCamToCenterHorizontalMeters = 0; //Horizontal offset from the center of the robot to the left camera
        public static final double kLeftCamHeightMeters = 0; //Height of the left camera, measured from the ground
        public static final double kLeftCamRollRadians = Math.toRadians(0); //Roll of the left camera in radians
        public static final double kLeftCamPitchRadians = Math.toRadians(0); //Pitch of the left camera in radians
        public static final double kLeftCamYawRadians = Math.toRadians(0); //Yaw of the left camera in radians

        //Transform3d that stores the full transformation from the center of the robot to the left camera
        public static final Transform3d kLeftRobotToCam = new Transform3d(
            new Translation3d(
                kLeftCamToCenterVerticalMeters,
                kLeftCamToCenterHorizontalMeters,
                kLeftCamHeightMeters),
            new Rotation3d(
                kLeftCamRollRadians,
                kLeftCamPitchRadians,
                kLeftCamYawRadians
                )
            );

        public static final double kRightCamToCenterVerticalMeters = 0; //Vertical offset from the center of the robot to the right camera
        public static final double kRightCamToCenterHorizontalMeters = 0; //Horizontal offset from the center of the robot to the right camera
        public static final double kRightCameraHeightMeters = 0; //Height of the right camera, measured from the ground
        public static final double kRightCamRollRadians = Math.toRadians(0); //Roll of the right camera in radians
        public static final double kRightCamPitchRadians = Math.toRadians(0); //Pitch of the right camera in radians
        public static final double kRightCamYawRadians = Math.toRadians(0); //Yaw of the right camera in radians

        //Transform3d that stores the full transformation from the center of the robot to the right camera
        public static final Transform3d kRightRobotToCam = new Transform3d(
            new Translation3d(
                kRightCamToCenterVerticalMeters,
                kRightCamToCenterHorizontalMeters,
                kRightCameraHeightMeters),
            new Rotation3d(
                kRightCamRollRadians,
                kRightCamPitchRadians,
                kRightCamYawRadians
                )
            );
            */
            public static final double kLeftCamToCenterXMeters = 0.263;
            public static final double kLeftCamToCenterYMeters = 0.226;
            public static final double kLeftCameraZMeters = 0.33;
            public static final Transform3d kLeftRobotToCam = new Transform3d(
                new Translation3d(
                    kLeftCamToCenterXMeters,
                    kLeftCamToCenterYMeters,
                    kLeftCameraZMeters),
                new Rotation3d(0, 0, Math.toRadians(-30)));
    
            public static final double kRightCamToCenterXMeters = 0.263;
            public static final double kRightCamToCenterYMeters = -0.226;
            public static final double kRightCameraZMeters = 0.33;
            public static final Transform3d kRightRobotToCam = new Transform3d(
                new Translation3d(
                    kRightCamToCenterXMeters,
                    kRightCamToCenterYMeters,
                    kRightCameraZMeters),
                new Rotation3d(0, 0, Math.toRadians(30)));

        public static final double kMinimumTagsRequiredForAccuratePoseEstimate = 1; //If the number of tags seen is less that this number, the pose estimate is scrapped
        public static final double kMaximumAcceptableDistanceForAccuratePoseEstimateMeters = 4; //If the average distance of read april tags is above this, the pose estimate is scrapped

        /*
        These values represent how much error the robot's position can have that is still considered to be
        correctly aligned. Tolerances are used because if the robot had to align with an exact point, it 
        would try infinitely because it would never be able to get it's position and the target position
        to be the exact same. By adding tolerances, the robot's position just needs to be within a certain
        window of the target position, not exactly on it. Increasing the tolerance too much can make the
        align command stop before it is correctly aligned, but decreasing them too much will cause the
        command to run for a much longer time, if not infinitely, as the robot tries to align with a
        window that is too small. The best way to find the best tolerances is simply test with difference
        ones and find the ones that most consistently achieve good results
         */
        public static final double kAprilTagAlignToleranceMeters = 0.02; //Translation
        public static final double kAprilTagAlignToleranceDegrees = 1; //Rotation

        /*
        These values represent the ideal offsets from the tag's position to the robot's position. Change
        these values to make the robot align with a spot to the side of a tag, or farther back from a tag,
        or in front of the tag but rotated, etc.

        **NOTE: Remember to factor in the robot's size into the vertical distance calculation, as setting
        that value to 0 will tell the robot to move so that the tag is directly above the center of the
        robot, which will cause the robot to slam into whatever structure the tag is on (unless the tag's
        height is above the robot, then it would be fine)
         */
        public static final double kIdealXDistanceToTagMeters = -0.5; //Calculated from the center of the robot to the tag
        public static final double kIdealYDistanceToTagMeters = 0; //Calculated from the center of the robot to the tag
        public static final double kIdealRotationalOffsetToTagDegrees = 0; //Facing the tag is 0 degrees
    }

    /**
     * This class holds target positions on the field that the robot will be told to move to during
     * autonomous
     */
    public class FieldConstants{
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        //Examples
        public static final Pose2d kBlueLeftCenterPosition = new Pose2d(7, 4.53, Rotation2d.fromDegrees(180));
        public static final Pose2d kBlueRightCenterPosition = new Pose2d(7, 3.53, Rotation2d.fromDegrees(180));
        public static final Pose2d[] kBlueCenterPositions = {kBlueLeftCenterPosition, kBlueRightCenterPosition};

        public static final Pose2d kRedLeftCenterPosition = new Pose2d(10.55, 3.53, Rotation2d.fromDegrees(0));
        public static final Pose2d kRedRightCenterPosition = new Pose2d(10.55, 4.53, Rotation2d.fromDegrees(0));
        public static final Pose2d[] kRedCenterPositions = {kRedLeftCenterPosition, kRedRightCenterPosition};
    }

    /**
     * This class holds max speeds and accelerations during a pathfinding command, as well as tolerances
     */
    public class PathfindingConstants{
        /*
        Tolerances are used because the pathfinding command will never get the robot exactly to the goal
        position. To prevent the robot from continuing to try, we end the command if the robot's current
        position is within the target area, not just equal to an exact point
        */
        public static final double kPositionToleranceMeters = 0.1; //Acceptable position offset from goal position
        public static final double kRotationToleranceDegrees = 2; //Acceptable rotational offset from goal position

        /*
        Pathfinding constraints are used to set maximum speeds during a pathfinding command, so that even
        if the robot is traveling a far distance it does not exceed speeds that could damage the robot or
        make the path less accurate
        */
        public static final double kMaxVelocityMetersPerSecond = 3; //Max movement velocity during a pathfinding command
        public static final double kMaxAccelerationMetersPerSecond = 3; //Max movement acceleration during a pathfinding command
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI*2; //Max rotational velocity during a pathfinding command
        public static final double kMaxAngularAccelerationRadiansPerSecond = Math.PI*2; //Max rotational acceleration during a pathfinding command
        public static final PathConstraints kPathConstraints = new PathConstraints(kMaxVelocityMetersPerSecond, kMaxAccelerationMetersPerSecond, kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecond); //Object to store the pathfinding limits
    }

    /**
     * This class holds information about each button id on the custom dashboard. This information will be
     * used to translate each button id into a full command that will be used in autonomous
     */
    public class AutonomousGeneratorConstants{
        /*
         * A category list is a list that holds a group of buttons that all share the same command in the
         * commands/autonomous subfolder, they just pass in different parameters to that command. We will
         * call this command the 'category command'. EACH BUTTON ID MUST BE IN EXACTLY ONE CATEGORY
         * LIST. A property list is a list of button IDs that will all pass in the same parameter to their
         * respective category command. **2025 reference**: To demonstrate, let's say the category command
         * moves to a position on the reef, alignes with one of the posts on that side, raises the
         * elevator to a certain level on that post, and shoots. Let's say the parameters that that
         * command takes in are: which side of the reef, which post on that side, and which level of that
         * post. All of the button IDs that use that command would be in the same category; however, they
         * would all pass in different parameters. The property lists define which button IDs pass in which
         * parameters. One property list could include all of the button IDs that would pass in the same side
         * of the reef. Another could include all of the button IDs that would tell the elevator to move up
         * to the same level. Another could include all of the button IDs that would tell the robot to align
         * with the same post. When it comes to generating the autonomous, the command that is added is
         * decided by which category the button ID is in, and the parameters that are passed in to that
         * command are decided by the property lists that the button ID is a part of. If the button ID is a
         * part of a property list that holds all of the button IDs for level 1 of the reef, then level 1 
         * will be passed into the category command. If the button ID is also a part of a property list that
         * holds all of the button IDs for the right post on a side, then right post will be passed in. Using 
         * this method, every button ID will be translated into a command with a list of parameters specific 
         * to that button ID.
         */

        public static final int kNumberOfButtons = 2;

        //Category, uses command 'MoveToPosition,' parameters: position
        public static final List<Integer> kButtonsInCategory1 = List.of(1, 2);
        //Properties for parameter 'position'
        public static final List<Integer> kButtonsInLeftCenterPosition = List.of(1); //Passes in ID 1 for parameter 'position'
        public static final List<Integer> kButtonsInRightCenterPosition = List.of(2); //Passes in ID 2 for parameter 'position'

    }
}