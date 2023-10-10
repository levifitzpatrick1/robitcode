package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Constants implements Loggable {

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

    public static class DriveConstants implements Loggable {

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

        public static double kVisionRotationP = 0.03;
        public static double kVisionRotationI = 0.00;
        public static double kVisionRotationD = 0.00;

        public static final double kTurnMaxVelocity = 9.33;
        public static final double kTurnMaxAcceleration = 773.44;

        public static final double kDriveMaxVelocity = Units.feetToMeters(16.5);
        public static final double kDriveMaxAcceleration = 78.63;
        public static Constraints kDriveConstraints = new Constraints(kDriveMaxVelocity, kDriveMaxAcceleration);

        @Config(width = 4, height = 4)
        void setVisionPID(double p, double i, double d) {
            kVisionRotationP = p;
            kVisionRotationI = i;
            kVisionRotationD = d;
        }

        @Log
        public static double kVisionTranslationxP = 0.50;
        @Log
        public static double kVisionTranslationxI = 0.00;
        @Log
        public static double kVisionTranslationxD = 0.00;

        @Config(width = 4, height = 4)
        void setTranslationxPID(double p, double i, double d) {
            kVisionTranslationxP = p;
            kVisionTranslationxI = i;
            kVisionTranslationxD = d;
        }

        @Log
        public static double kVisionTranslationyP = 0.50;
        @Log
        public static double kVisionTranslationyI = 0.00;
        @Log
        public static double kVisionTranslationyD = 0.00;

        @Config(width = 4, height = 4)
        void setTranslationyPID(double p, double i, double d) {
            kVisionTranslationyP = p;
            kVisionTranslationyI = i;
            kVisionTranslationyD = d;
        }
    }

    public static class VisionConstants {

        public static final double kFrontCameraPitch = 0;
        public static final Transform3d kFrontRobotToCam = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));

        public static final String kFrontCamName = "Cam1";
    }

}
