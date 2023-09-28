package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public class ModuleConstants {

public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
public static final double kDriveMotorGearRatio = 1 / 6.75;
public static final double kTurningMotorGearRatio = 1 / (150 / 7);
public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
public static final double kPTurning = 0.5;

public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(14.5);
public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;

public static class ModuleSpecificConstants {
    //Absolute Encoder Offset and ID
    public double kAbsoluteEncoderOffset;
    public boolean kAbsoluteEncoderReversed;
    public int kAbsoluteEncoderID;

    //Module Speeds
    public double kMaxModuleSpeed;
    public double kMaxModuleAngularSpeed;

     //Encoder Direction
     public boolean kdriveEncoderReversed;
     public boolean kturningEncoderReversed;
 
     //Motor IDs
     public int kdriveMotorID;
     public int kturningMotorID;

    public ModuleSpecificConstants(int moduleID, String moduleLoc)
    {
        switch (moduleID) {
            // Module 1
            case 1:
                kAbsoluteEncoderOffset = 0;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 1;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;

            // Module 2
            case 2:
                kAbsoluteEncoderOffset = 0;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 2;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;

            // Module 3
            case 3:                
                kAbsoluteEncoderOffset = 0;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 3;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;

            // Module 4
            case 4:
                kAbsoluteEncoderOffset = 0;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 4;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;

            // Module 5
            case 5:
                kAbsoluteEncoderOffset = 0;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 5;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;

            // Module 6
            case 6:
                kAbsoluteEncoderOffset = 0;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 6;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;

            // Module 7
            case 7:
                kAbsoluteEncoderOffset = 0;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 7;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;

            // Module 8
            case 8:
                kAbsoluteEncoderOffset = 0;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 8;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;

            default:
                kAbsoluteEncoderOffset = 0;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 0;

                kMaxModuleSpeed = 0;
                kMaxModuleAngularSpeed = 0;
        }

    

        switch (moduleLoc) {
            // Front Left
            case "FL":
                kdriveEncoderReversed = false;
                kturningEncoderReversed = false;

                kdriveMotorID = 18;
                kturningMotorID = 17;
            
            // Front Right
            case "FR":
                kdriveEncoderReversed = true;
                kturningEncoderReversed = false;

                kdriveMotorID = 12;
                kturningMotorID = 11;
            
            // Back Left
            case "BL":
                kdriveEncoderReversed = false;
                kturningEncoderReversed = false;

                kdriveMotorID = 16;
                kturningMotorID = 15;
            
            // Back Right
            case "BR":
                kdriveEncoderReversed = true;
                kturningEncoderReversed = false;

                kdriveMotorID = 14;
                kturningMotorID = 13;
            
            default:
                kdriveEncoderReversed = false;
                kturningEncoderReversed = false;

                kdriveMotorID = 0;
                kturningMotorID = 0;
        }
}
    

}


}
