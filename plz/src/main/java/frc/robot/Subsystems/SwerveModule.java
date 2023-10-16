package frc.robot.Subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleConstants.ModuleSpecificConstants;
import frc.robot.Util.LowPassFilter;

public class SwerveModule {


    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final ProfiledPIDController turningProfiledPIDController;
    private final CANcoder absoluteEnCoder;
    private final double absoluteEnCoderOffset;
    private final boolean absoluteEnCoderReversed;

    private final ModuleSpecificConstants constants;

    /**
     * Creates a new SwerveModule with the given moduleID and moduleLocation
     * {@link ModuleSpecificConstants} is used to get the constants for the module
     * @param moduleID The ID of the module. Each module has a unique ID located on the module
     * @param moduleLocation Where the module is located on the robot (FL, FR, BL, BR)
     */
    public SwerveModule(int moduleID, String moduleLocation)
    {
        constants = new ModuleSpecificConstants(moduleID, moduleLocation);

        absoluteEnCoder = new CANcoder(constants.kAbsoluteEncoderID);
        absoluteEnCoderOffset = constants.kAbsoluteEncoderOffset;
        absoluteEnCoderReversed = constants.kAbsoluteEncoderReversed;

        driveMotor = new CANSparkMax(constants.kdriveMotorID, CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(constants.kturningMotorID, CANSparkMax.MotorType.kBrushless);

        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveMotor.setInverted(constants.kdriveEncoderReversed);
        turnMotor.setInverted(constants.kturningEncoderReversed);

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningProfiledPIDController = new ProfiledPIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning, ModuleConstants.kTurningConstraints);

        turningProfiledPIDController.setTolerance(ModuleConstants.kTurningTolerance);
        turningProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        new LowPassFilter(0.2);
        new LowPassFilter(0.1);

        resetEncoders();
    }

    // ------- Gets ------- //
    
    /**
     * gets the drive position from the encoder based on the conversion factor
     * @return the drive position in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the drive velocity from the encoder based on the conversion factor
     * @return the drive velocity in meters per second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Gets the turn position from the encoder based on the conversion factor
     * @return the turn position in radians
     */
    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    /**
     * Gets the turn velocity from the encoder based on the conversion factor
     * @return the turn velocity in radians per second
     */
    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    /**
     * Gets the position of the absolute encoder
     * @return the absolute position in radians
     */
    public double getAbsolutePositionRadians() {
        return Math.toRadians(getAbsolutePositionDegrees());
    }

    /**
     * Gets the position of the absolute encoder
     * @return the absolute position in degrees
     */
    public double getAbsolutePositionDegrees() {
        double rawPosition = absoluteEnCoder.getAbsolutePosition().getValue() + absoluteEnCoderOffset;
        double angle = rawPosition * 360.0;
        return (angle * (absoluteEnCoderReversed ? -1 : 1));
    }

    /**
     * Gets the position of the module
     * @return the position of the module as a SwerveModulePosition
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurnPosition()));
    }
    
    /**
     * Gets the state of the module as a with the drive velocity and turn position
     * @return the state of the module as a SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    // ------- Commands ------- //

    /**
     * Resets the Drive Encoder to 0
     * Resets the Turn Encoder to the absolute encoder position in radians
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsolutePositionRadians());
    }

    /**
     * Sets the desired state of the module
     * @param state the desired state of the module as a SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState state) {

        if (Math.abs(state.speedMetersPerSecond) < 0.1) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / constants.kMaxModuleSpeed);
        turnMotor.set(turningProfiledPIDController.calculate(getTurnPosition(), state.angle.getRadians()));
    
        SmartDashboard.putNumber("Current Speed", getDriveVelocity());
        SmartDashboard.putNumber("Desired speed", state.speedMetersPerSecond);
        SmartDashboard.putNumber("Drive motor set", driveMotor.get());
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public double getSpeed() {
        return constants.kMaxModuleSpeed;
    }
}
