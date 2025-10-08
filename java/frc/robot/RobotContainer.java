package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonomousGeneratorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PhotonVision.AlignToAprilTag;
import frc.robot.commands.PhotonVision.AlignToAprilTagOdemeter;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {

        private final NetworkTable controlBoardTable; //The network table for key presses
        private final NetworkTableEntry lastKeyPressedEntry; //A variable to hold the most recent key that was pressed
        
        private final NetworkTable customDashboardTable; //The network table for the custom autonomous dashboard

        private final BooleanSupplier cKeyPressed; //Variable to hold whether the 'c' key was pressed (create one of these for each key that you want to detect)

        private final SendableChooser<Command> autoChooser; //The autonomous chooser on the smart dashboard

        private final List<Integer> numbers = new ArrayList<>(); //The list of button id's for generating custom autonomous

        private final AutonomousGenerator autonomousGenerator = new AutonomousGenerator(); //The autonomous generator

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(); //The swerve subsystem
        private final PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem(); //The PhotonVision subsystem

        private final XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort); //An object to hold the suppliers (joysticks, triggers, etc.) from the driving joystick
        private final CommandXboxController driverJoystickCommand = new CommandXboxController(OIConstants.kDriverControllerPort); //An object to hold the booleans (buttons, etc.) from the drivinig joystick

        //The vision aspect of pose estimation
        private final Vision vision = new Vision(
            (pose, time, devs) -> swerveSubsystem.addVisionMeasurement(pose, time, devs), 
            swerveSubsystem
        );

    
    /**
     * This is where commands can be tied to certain variables/triggers, where default commands are set,
     * where network tables are set up, where the autonomous selector is created, where named command are
     * created, and where the autonomous generator is used
     */
    public RobotContainer() {
        //Fetch the the custom autonomous dashboard network table
        customDashboardTable = NetworkTableInstance.getDefault().getTable("CustomDashboard");

        //Fetch the keyboard listener network table
        controlBoardTable = NetworkTableInstance.getDefault().getTable("ControlBoard");
        lastKeyPressedEntry = controlBoardTable.getEntry("LastKeyPressed");

        //Duplicate this line for any key that you want to listen for
        cKeyPressed = () -> lastKeyPressedEntry.getString("None").equalsIgnoreCase("C");

        //Set the SwerveJoystickCmd to be the swerve subsystem's default command. This means that unless the swerve subsystem is being used by a different command, this command will be running
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kTurboAxis), //Right trigger, if pressed the robot speed is doubled **only use in open space, very fast, avoid rotation when using**
                () -> driverJoystick.getRawAxis(OIConstants.kSlowAxis))); //Left trigger, if pressed the robot speed is cut in half, use to align with a small target

        configureButtonBindings();
        
        //Create the autonomous chooser
        autoChooser = AutoBuilder.buildAutoChooser();

        //Add options to the AutoChooser below by using the 'autoChooser.addOption(...)' method

        SmartDashboard.putString("Generated Auto Name", "Default"); //Where you input the name of the autonomous that you generated

        SmartDashboard.putNumber("Target ID", 21);

        SmartDashboard.putData("Auto Mode", autoChooser); //Puts the autonomous chooser on the Smart Dashboard
    }

    /*
     * This is where different buttons/variables are tied to commands
     */
    private void configureButtonBindings() {
        //Resets the robots forward direction
        driverJoystickCommand.b().onTrue(swerveSubsystem.orientBot());

        new Trigger(cKeyPressed).whileTrue(new InstantCommand(() -> SmartDashboard.putBoolean("C Key Pressed", true))).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("C Key Pressed", false)));

        driverJoystickCommand.y().whileTrue(new AlignToAprilTag(swerveSubsystem, photonVisionSubsystem, 21, true));

        driverJoystickCommand.x().whileTrue(new AlignToAprilTagOdemeter(swerveSubsystem, photonVisionSubsystem));
    }

    /**
     * Returns the selected autonomous
     * @return the selected autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void periodic() {
        vision.periodic();

        /*
        SmartDashboard.putNumber("Closest Tag ID", photonVisionSubsystem.findClosestTagIDFromBothCameras());
        SmartDashboard.putString("Closest Tag Position", photonVisionSubsystem.getFieldRelativeTagPosition(photonVisionSubsystem.findClosestTagIDFromBothCameras()).toString());
        SmartDashboard.putString("Ideal Robot Position", swerveSubsystem.getIdealRobotPose(photonVisionSubsystem.getFieldRelativeTagPosition(photonVisionSubsystem.findClosestTagIDFromBothCameras()).get().toPose2d()).toString());
        */

        //Detects button clicks on the custom dashboard, if one is clicked adds a setpoint to the list for autonomous
        for (int i = 1; i <= AutonomousGeneratorConstants.kNumberOfButtons; i++) { //Loops through all of the button ids
            if (customDashboardTable.getEntry(String.valueOf(i)).getBoolean(false)) { //If that button is pressed
                numbers.add(i); //Add that number to the list of setpoints
                customDashboardTable.getEntry(String.valueOf(i)).setBoolean(false); //reset
            }
        }

        //Generates the autonomous, adds it to the list
        if(customDashboardTable.getEntry(String.valueOf(100)).getBoolean(false)){
            autoChooser.addOption(SmartDashboard.getString("Auto Name", "Generated Autonomous"), autonomousGenerator.generateAutonomous(swerveSubsystem, photonVisionSubsystem, numbers));
            customDashboardTable.getEntry(String.valueOf(100)).setBoolean(false); //reset
        }

        //Clears the list of setpoints for the autonomous
        if(customDashboardTable.getEntry(String.valueOf(101)).getBoolean(false)){
            numbers.clear();
            customDashboardTable.getEntry(String.valueOf(101)).setBoolean(false); //reset
        }

        //Logs the current generated autonomous to the smart dashboard
        SmartDashboard.putString("Autonomous Numbers", numbers.toString());
    }
}