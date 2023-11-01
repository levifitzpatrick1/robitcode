package frc.robot.Util;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.Constants.DriveConstants;

/**
 * Singleton class for SwerveDrivePoseEstimator that allows it to be shared by
 * subsystems
 * (drivetrain and vision)
 */
public class RobotOdometry {
  private static final RobotOdometry robotOdometry = new RobotOdometry();
  private SwerveDrivePoseEstimator estimator;
  private SwerveModulePosition[] defaultPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  private RobotOdometry() {
    estimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        new Rotation2d(),
        defaultPositions,
        new Pose2d());
  }

  public static RobotOdometry getInstance() {
    return robotOdometry;
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return estimator;
  }
}