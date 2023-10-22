package frc.robot.Constants;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Util.Alert;
import frc.robot.Util.Alert.AlertType;

public class Constants{

    // Logging Stuff
    private static final RobotType robot = RobotType.ROBOT_PHYSICAL;
    public static final RobotID robotID = RobotID.ROBOT_OTHER;
    public static final double loopPeriod = 0.02;
    public static final boolean tuningMode = false;

    private static final Alert invalidRobotAlert = 
        new Alert("Invalid Robot", "The robot type is invalid. Please check the robot type in Constants.java", AlertType.ERROR);

    
  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_PHYSICAL;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_PHYSICAL:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

    public static class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final double kDeadband = 0.1;

        public static final int kXboxAButton = 1;
        public static final int kXboxBButton = 2;
        public static final int kXboxXButton = 3;
        public static final int kXboxYButton = 4;
        public static final int kXboxLeftBumper = 5;
        public static final int kXboxRightBumper = 6;
        public static final int kXboxBackButton = 7;
        public static final int kXboxStartButton = 8;
        public static final int kXboxLeftStickButton = 9;
        public static final int kXboxRightStickButton = 10;
        public static final int kXboxLeftTriggerAxis = 2;
        public static final int kXboxRightTriggerAxis = 3;
        public static final int kXboxLeftXAxis = 0;
        public static final int kXboxLeftYAxis = 1;
        public static final int kXboxRightXAxis = 4;
        public static final int kXboxRightYAxis = 5;

    }

    public static class DriveConstants{

        public static final boolean kPidgeonGyro = true;
        public static final boolean kPidgeonInverse = true;

        public static final double kTrackWidth = Units.inchesToMeters(30);
        public static final double kWheelBase = Units.inchesToMeters(30);

        public static final double kMaxAcceleration = 3;
        public static final double kMaxAngularAcceleration = 3;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static double kVisionRotationP = 1.5;
        public static double kVisionRotationI = 0.2;
        public static double kVisionRotationD = 0.075;

        public static final double kTurnMaxVelocity = 9.33;
        public static final double kTurnMaxAcceleration = 773.44;

        public static final double kDriveMaxVelocity = Units.feetToMeters(16.5);
        public static final double kDriveMaxAcceleration = 78.63;
        public static Constraints kDriveConstraints = new Constraints(kDriveMaxVelocity, kDriveMaxAcceleration);


        public static double kVisionTranslationxP = 0.50;
        public static double kVisionTranslationxI = 0.00;
        public static double kVisionTranslationxD = 0.00;

        public static double kVisionTranslationyP = 0.50;
        public static double kVisionTranslationyI = 0.00;
        public static double kVisionTranslationyD = 0.00;

    }

    public static class VisionConstants {

        public static final double kFrontCameraPitch = 0;
        public static final Transform3d kFrontRobotToCam = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));

        public static final String kFrontCamName = "Cam1";
    }




// Robot Type Info
    public static final Map<RobotType, String> logFolders =
    Map.of(RobotType.ROBOT_PHYSICAL, "/media/sda2");

public static enum RobotType {
  ROBOT_PHYSICAL, ROBOT_SIMBOT
}

public static enum Mode {
  REAL, REPLAY, SIM
}

public static enum RobotID {
  ROBOT_SCHOOL, ROBOT_OTHER
}

}
