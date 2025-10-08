package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToPosition extends SequentialCommandGroup {
    /**
     * This command will pathfind to any spot while avoiding obstacles on the field (not moving ones like
     * other robot's, but fixed structures that are always in the same place)
     * @param swerveSubsystem the swerve subsystem
     * @param position the position to pathfind to
     */
    public MoveToPosition(SwerveSubsystem swerveSubsystem, int position){
        addCommands(
            new ProxyCommand(() -> //Set up pathfinding command as a proxy command, not it's own command. That way it is much easier to control
                AutoBuilder.pathfindToPose(
                    swerveSubsystem.getCenterPosition(position), //Get target position
                    PathfindingConstants.kPathConstraints, //Pass in max speeds
                    0 //Set a goal end velocity (the speed that the robot will be moving at the end of the path)
            ))
        );
    }
}