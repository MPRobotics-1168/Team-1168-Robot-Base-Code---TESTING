package frc.robot.commands.PhotonVision;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToAprilTagOdemeter extends SequentialCommandGroup{
public AlignToAprilTagOdemeter(SwerveSubsystem swerveSubsystem, PhotonVisionSubsystem photonVisionSubsystem){
        addCommands(
            new ProxyCommand(() -> {
                Optional<Pose3d> tagPose = photonVisionSubsystem.getFieldRelativeTagPosition(photonVisionSubsystem.findClosestTagIDFromBothCameras());
                if(tagPose.isPresent()){
                    return AutoBuilder.pathfindToPose(
                        swerveSubsystem.getIdealRobotPose(tagPose.get().toPose2d()),
                        PathfindingConstants.kPathConstraints,
                        0
                    );
                }else return new ProxyCommand(() -> new InstantCommand());
            })
        );
    }
}
