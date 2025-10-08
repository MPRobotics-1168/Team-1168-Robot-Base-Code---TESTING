/*
The way this works is it takes in a list of integers, each one corresponding to a position during an autonomous routine (Ex: one
of the intakes, a post on the reef, a ground coral, etc.). The program then calculates certain aspect of that position, like what
level to set the elevator to, what position on the field to drive to. The program then creates a list of command to get to the
desired positions, and puts all of the commands into a sequential command group. This sequential command group is then sent to the
robot container, where it is added to the list of autonomous routines
*/

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonomousGeneratorConstants;
import frc.robot.commands.autonomous.MoveToPosition;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonomousGenerator {

    public AutonomousGenerator(){}

    /**
     * Returns the id of the position that corresponds to the given button
     * @param number the button id
     * @return the id of the corresponding position
     */
    public int getWhichPosition(int number){
        if(AutonomousGeneratorConstants.kButtonsInLeftCenterPosition.contains(number)) return 1;
        else if(AutonomousGeneratorConstants.kButtonsInRightCenterPosition.contains(number)) return 2;
        else return 0; //To avoid crashing
    }

    /**
     * This command takes in a list of button id's and returns a fully generated autonomous command that
     * is ready to be scheduled
     * @param swerveSubsystem the swerve subsystem
     * @param photonVisionSubsystem the PhotonVision subsystem
     * @param numbers the list of button id's
     * @return the autonomous routine
     */
    public SequentialCommandGroup generateAutonomous(SwerveSubsystem swerveSubsystem, PhotonVisionSubsystem photonVisionSubsystem, List<Integer> numbers){
        SequentialCommandGroup autonomous = new SequentialCommandGroup();
        //To avoid crashing
        if(numbers != null){
            //Loops through all of the button ids
            for(int i = 1; i<=numbers.size(); i++){
                //If the button is in a certain category (in this case, 'category 1')
                if(AutonomousGeneratorConstants.kButtonsInCategory1.contains(numbers.get(i-1))){ //i-1 because first object in a list is index 0
                    //Add the corresponding category command, passes in the parameters that correspond to that button id
                    if(getWhichPosition(numbers.get(i-1)) != 0) autonomous.addCommands(new MoveToPosition(swerveSubsystem, getWhichPosition(numbers.get(i-1)))); //i-1 because first object in a list is index 0
                }
            }
        }
        return autonomous;
    }
}