package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final SparkMax driveMotor; //The drive motor
    private final SparkMax turningMotor; //The turning motor

    private final SparkMaxConfig driveConfig; //The configuration of the drive motor
    private final SparkMaxConfig turningConfig; //The configuration of the turning motor

    private final RelativeEncoder driveEncoder; //The drive motor encoder
    private final RelativeEncoder turningEncoder; //The turning motor encoder

    private final PIDController turningPidController; //The PID controller for the turning motor

    private final AnalogInput absoluteEncoder; //The encoder for the swerve module, reads the absolute rotation of the turning motor
    private final boolean absoluteEncoderReversed; //Whether the encoder is reversed
    private final double absoluteEncoderOffsetRad; //The encoder offset

    /**
     * One individual swerve module
     * @param driveMotorId The ID of the drive motor
     * @param turningMotorId The ID of the turning motor
     * @param driveMotorReversed Whether the drive motor is reversed
     * @param turningMotorReversed Whether the turning motor is reversed
     * @param absoluteEncoderId The ID of the encoder
     * @param absoluteEncoderOffset The encoder offset
     * @param absoluteEncoderReversed Whether the encoder is reversed
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        //Set up the encoder as an analog input
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        //Set up the driving and turning motors as their own individual spark max motor
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveConfig = new SparkMaxConfig();
        turningConfig = new SparkMaxConfig();
        
        //Get the encoders for each individual motor
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        //Set up the driving motor configuration
        driveConfig.inverted(driveMotorReversed);
        driveConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderMetersPerRotation);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPMToMetersPerSecond);
        
        //Set up the turning motor configuration
        turningConfig.inverted(turningMotorReversed);
        turningConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRadiansPerRotation);
        turningConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPMToRadiansPerSecond);

        //Pass in the configuration
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Set up the turning motor PID controller
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-180, 180); //POSSIBLE PROBLEM: degrees or radians??? Could this be the cause of the autonomous swirles??
        /*
         * I THINK I MIGHT KNOW THE PROBLEM WITH AUTONOMOUS!!!!!!!!!!
         * 
         * Before, -Math.PI and Math.PI were passed in to 'enableContinuousInput.' However, enable
         * continuous input is looking for degrees, not radians. That means that the wheel sees -3.14159
         * and 3.14159 as the same points (like -180 and 180 are the same points). In autonomous,
         * Pathplanner uses the same PID controller for error correction. This means that if the error 
         * ever reaches a value greater than 3.14159 (pi) degrees, then the wheel would think that it is
         * faster to go the opposite direction. This would make the wheel spin very fast in the wrong
         * direction, causing the robot to veer off course. This wouldn't happen ever time, which would
         * explain why sometimes the autonomous would work perfectly.
         * 
         * Josh that's your fault you wrote this code. Screw you Josh.
         */

        resetEncoders();
    }

    /**
     * Returns the drive motor's position in radians.
     * @return the drive motor's position
     */
    public final double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns the turning motor's position in radians.
     * @return the turning motor's position
     */
    public final double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /**
     * Returns the drive motor's current velocity.
     * @return the drive motor's current velocity
     */
    public final double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the turning motor's current velocity.
     * @return the turning motor's current velocity
     */
    public final double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * Returns the current rotation of both motors.
     * @return the current {@link SwerveModulePosition}
     */
    public final SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Returns the absolute global rotation of the turning motor in radians. This value ignores resets
     * or any other position changes.
     * @return the absolute rotation of the turning motor
     */
    public final double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * Resets both drive and turning encoders.
     */
    public final void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * Returns the current state of the module (the drive motor's current speed, and the turning motor's
     * current rotation).
     * @return the current {@link SwerveModuleState}
     */
    public final SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Sets the module to the desired state (driving motor speed and turning motor position).
     * @param state the desired {@link SwerveModuleState}
     */
    public final void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state.optimize(getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(Math.toRadians(turningPidController.calculate(Math.toDegrees(getTurningPosition()), state.angle.getDegrees())));
    }
    
    /**
     * Stops both drive and turning motors
     */
    public final void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
