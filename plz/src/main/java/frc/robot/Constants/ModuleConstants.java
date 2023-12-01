package frc.robot.Constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ModuleConstants {

public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
public static final double kDriveMotorGearRatio = 1.0 / 6.75;
public static final double kTurningMotorGearRatio = 7.0 / 150.0;
public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;


public static final double kPTurning = 1;
public static final double kITurning = 0;
public static final double kDTurning = .01;
public static final double kTurningTolerance = 0.1;
public static final double kTurningMaxVelocity = 5676 * 60 * kTurningEncoderRot2Rad;
public static final double kTurningMaxAcceleration = kTurningMaxVelocity / 60;
public static final Constraints kTurningConstraints = new Constraints(kTurningMaxVelocity, kTurningMaxAcceleration);

public static final double kPDrive = 1;
public static final double kIDrive = 0;
public static final double kDDrive = 0;

public static final double kDriveMaxVelocity = Units.feetToMeters(16.5);
public static final double kDriveMaxAcceleration = kDriveMaxVelocity / 60;


public static class ModuleSpecificConstants {
    //Absolute Encoder Offset and ID
    public double kAbsoluteEncoderOffset;
    public boolean kAbsoluteEncoderReversed;
    public Integer kAbsoluteEncoderID;

    //Module Speeds
    public double kMaxModuleSpeed;
    public double kMaxModuleAngularSpeed;

     //Encoder Direction
     public boolean kdriveEncoderReversed;
     public boolean kturningEncoderReversed;
 
     //Motor IDs
     public Integer kdriveMotorID;
     public Integer kturningMotorID;

    public ModuleSpecificConstants(Integer moduleID, String moduleLoc)
    {
        switch (moduleID) {
            // Module 1
            case 1:
                kAbsoluteEncoderOffset = 0.17724609375;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 1;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 2
            case 2:
                kAbsoluteEncoderOffset = -0.886474609375;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 2;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 3
            case 3:                
                kAbsoluteEncoderOffset = -0.18505859375;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 3;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 4
            case 4:
                kAbsoluteEncoderOffset = -0.7998046875;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 4;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 5 "BL" SCHOOLBOT
            case 5:
                kAbsoluteEncoderOffset = 0.4;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 5;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 6 "FL" SCHOOLBOT
            case 6:
                kAbsoluteEncoderOffset = -0.05;                
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 6;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 7 "BR" SCHOOLBOT
            case 7:
                kAbsoluteEncoderOffset = 0;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 7;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;
            
            // Module 8 "FR" SCHOOLBOT
            case 8:
                kAbsoluteEncoderOffset = -0.10;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 8;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

        }

        switch (moduleLoc) {
            // Front Left
            case "FL":
            switch (Constants.robotID) {
                case ROBOT_OTHER:
                    kdriveEncoderReversed = false;
                    kturningEncoderReversed = true;

                    kdriveMotorID = 14;
                    kturningMotorID = 13;
                    break;
                case ROBOT_SCHOOL:
                    kdriveEncoderReversed = false;
                    kturningEncoderReversed = true;

                    kdriveMotorID = 16;
                    kturningMotorID = 15;
                    break;
            }
            break;
            
            // Front Right
            case "FR":
            switch (Constants.robotID) {
                case ROBOT_OTHER:
                    kdriveEncoderReversed = false;
                    kturningEncoderReversed = true;

                    kdriveMotorID = 18;
                    kturningMotorID = 17;
                    break;
                case ROBOT_SCHOOL:
                    kdriveEncoderReversed = false;
                    kturningEncoderReversed = true;

                    kdriveMotorID = 12;
                    kturningMotorID = 11;
                    break;
            }
            break;

            // Back Left
            case "BL":
            switch (Constants.robotID) {
                case ROBOT_OTHER:
                    kdriveEncoderReversed = true;
                    kturningEncoderReversed = true;

                    kdriveMotorID = 16;
                    kturningMotorID = 15;
                    break;
                case ROBOT_SCHOOL:
                    kdriveEncoderReversed = true;
                    kturningEncoderReversed = true;

                    kdriveMotorID = 14;
                    kturningMotorID = 13;
                    break;
            }
            break;

            // Back Right
            case "BR":
            switch (Constants.robotID) {
                case ROBOT_OTHER:
                    kdriveEncoderReversed = true;
                    kturningEncoderReversed = true;

                    kdriveMotorID = 12;
                    kturningMotorID = 11;
                    break;
                case ROBOT_SCHOOL:
                    kdriveEncoderReversed = true;
                    kturningEncoderReversed = true;

                    kdriveMotorID = 18;
                    kturningMotorID = 17;
                    break;
            }
            break;
            
        }
}
}
}
