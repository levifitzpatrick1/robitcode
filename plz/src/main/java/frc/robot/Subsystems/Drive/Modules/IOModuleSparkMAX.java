package frc.robot.Subsystems.Drive.Modules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.Constants;
import frc.robot.Constants.ModuleConstants.ModuleSpecificConstants;
import frc.robot.Util.SparkMaxDerivedVelocityController;


/**
 * IOModuleSparkMAX is an implementation of the IOModule interface,
 * specifically designed for SparkMax-based swerve modules.
 *
 * <p>This class handles the low-level control and data acquisition
 * for both the drive and turn motors of a swerve module.</p>
 *
 * <p>It also includes specific configurations and constants for
 * different robots and module locations.</p>
 */
public class IOModuleSparkMAX implements IOModule{
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final double driveAfterEncoderReduction =
    (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

    private final double turnAfterEncoderReduction = 150.0 / 7.0;

    private final SparkMaxDerivedVelocityController driveDerivedVelocityController;
    private final SparkMaxDerivedVelocityController turnDerivedVelocityController;
    private final RelativeEncoder driveEncoder;
    private final CANcoder absoluteEnCoder;
    private final double absoluteEnCoderOffset;
    private final boolean absoluteEnCoderReversed;

    private final ModuleSpecificConstants constants;


    /**
     * Constructor for initializing the IOModuleSparkMAX.
     *
     * @param moduleID The ID of the swerve module.
     * @param moduleLocation The location of the swerve module on the robot.
     */
    public IOModuleSparkMAX(int moduleID, String moduleLocation){
        switch (Constants.getRobot()) {
            case ROBOT_PHYSICAL:
            constants = new ModuleSpecificConstants(moduleID, moduleLocation);

            absoluteEnCoder = new CANcoder(constants.kAbsoluteEncoderID);
            absoluteEnCoderOffset = constants.kAbsoluteEncoderOffset;
            absoluteEnCoderReversed = constants.kAbsoluteEncoderReversed;
    
            driveMotor = new CANSparkMax(constants.kdriveMotorID, CANSparkMax.MotorType.kBrushless);
            turnMotor = new CANSparkMax(constants.kturningMotorID, CANSparkMax.MotorType.kBrushless);
    
            turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    
            driveEncoder = driveMotor.getEncoder();
                turnMotor.getEncoder();
    
            driveMotor.setInverted(constants.kdriveEncoderReversed);
            turnMotor.setInverted(constants.kturningEncoderReversed);

            driveMotor.setCANTimeout(0);
            turnMotor.setCANTimeout(0);
            
            driveMotor.setSmartCurrentLimit(30);
            turnMotor.setSmartCurrentLimit(30);
    
            // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
            // turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
            // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
            // turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

            driveDerivedVelocityController = new SparkMaxDerivedVelocityController(driveMotor);
            turnDerivedVelocityController = new SparkMaxDerivedVelocityController(turnMotor);

            break;

            default:
            throw new RuntimeException("Invalid Robot For IOModuleSparkMAX");
        }
        
    }

        /**
     * Updates the set of loggable inputs for the swerve module.
     *
     * @param inputs An instance of IOModuleInputs containing the latest sensor data.
     */
    @Override
    public void updateInputs(IOModuleInputs inputs){
        inputs.drivePositionRad =
            Units.rotationsToRadians(driveDerivedVelocityController.getPosition())
                / driveAfterEncoderReduction;
        inputs.driveVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(driveDerivedVelocityController.getVelocity())
                / driveAfterEncoderReduction;
        inputs.driveVelocityFilteredRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
                / driveAfterEncoderReduction;
        inputs.driveAppliedVolts =
            driveMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};
        inputs.driveTempC = new double[] {driveMotor.getMotorTemperature()};


        double rawPosition = absoluteEnCoder.getAbsolutePosition().getValue() + absoluteEnCoderOffset;
        double angle = rawPosition * 360.0 * (absoluteEnCoderReversed ? -1 : 1);
        inputs.turnAbsolutePositionRad = Units.degreesToRadians(angle);

        inputs.turnPositionRad =
            Units.rotationsToRadians(turnDerivedVelocityController.getPosition())
                / turnAfterEncoderReduction;
        inputs.turnVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(turnDerivedVelocityController.getVelocity())
                / turnAfterEncoderReduction;
        inputs.turnAppliedVolts =
            turnMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.turnCurrentAmps = new double[] {turnMotor.getOutputCurrent()};
        inputs.turnTempC = new double[] {turnMotor.getMotorTemperature()};
    }

    /**
     * Sets the voltage for the drive motor. This is used for open-loop control.
     *
     * @param voltage The voltage to apply to the drive motor.
     */
    public void setDriveVoltage(double voltage){
        driveMotor.setVoltage(voltage);
    }


    /**
     * Sets the voltage for the turn motor. This is used for open-loop control.
     *
     * @param voltage The voltage to apply to the turn motor.
     */
    public void setTurnVoltage(double voltage){
        turnMotor.setVoltage(voltage);
    }
    
}
