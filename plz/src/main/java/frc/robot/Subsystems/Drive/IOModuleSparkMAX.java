package frc.robot.Subsystems.Drive;

import javax.management.RuntimeErrorException;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleConstants.ModuleSpecificConstants;
import frc.robot.Util.SparkMaxDerivedVelocityController;

public class IOModuleSparkMAX implements IOModule{
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final double driveAfterEncoderReduction =
    (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

    private final double turnAfterEncoderReduction = 150.0 / 7.0;

    private final SparkMaxDerivedVelocityController driveDerivedVelocityController;
    private final SparkMaxDerivedVelocityController turnDerivedVelocityController;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final CANcoder absoluteEnCoder;
    private final double absoluteEnCoderOffset;
    private final boolean absoluteEnCoderReversed;

    private final ModuleSpecificConstants constants;

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
            turnEncoder = turnMotor.getEncoder();
    
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

    public void setDriveVoltage(double voltage){
        driveMotor.setVoltage(voltage);
    }

    public void setTurnVoltage(double voltage){
        turnMotor.setVoltage(voltage);
    }
    
}
