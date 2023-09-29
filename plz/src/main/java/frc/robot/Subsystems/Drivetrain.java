package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.VisionConstants;

public class Drivetrain extends SubsystemBase {

    private final SwerveModule frontLeftModule = new SwerveModule( 6, "FL");
    private final SwerveModule frontRightModule = new SwerveModule(8, "FR");
    private final SwerveModule backLeftModule = new SwerveModule(5, "BL");
    private final SwerveModule backRightModule = new SwerveModule(7, "BR");

    private Pigeon2 gyro;
    private AHRS navx = new AHRS(SPI.Port.kMXP);




    public PhotonCameraWrapper FrontCam;

 private final SwerveDrivePoseEstimator positionEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    getRotation2d(),
    getModulePositions(),
    new Pose2d()
        );
    

    public Drivetrain() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
            } catch (Exception e) {}
        }).start();

        if (DriveConstants.kPidgeonGyro) {
            gyro = new Pigeon2(62);
        }

        zeroHeading();

        FrontCam = new PhotonCameraWrapper(VisionConstants.kFrontCamName, VisionConstants.kFrontRobotToCam);
    }

    public void zeroHeading() {
        if (DriveConstants.kPidgeonGyro) {
            gyro.zeroGyroBiasNow();
        } else {
            navx.reset();
        }
    }

    public void resetOdometry(Pose2d pose) {
        positionEstimator.update(getRotation2d(), getModulePositions());
        positionEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public double getHeading() {
        if (DriveConstants.kPidgeonGyro) {
            return gyro.getYaw();
        } else {
            return navx.getAngle();
        }
    }


    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    

    

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getModulePosition(),
            frontRightModule.getModulePosition(),
            backLeftModule.getModulePosition(),
            backRightModule.getModulePosition()
        };
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("FL Cancoder", frontLeftModule.getAbsolutePosition());
        SmartDashboard.putNumber("FR Cancoder", frontRightModule.getAbsolutePosition());
        SmartDashboard.putNumber("BL Cancoder", backLeftModule.getAbsolutePosition());
        SmartDashboard.putNumber("BR Cancoder", backRightModule.getAbsolutePosition());

        positionEstimator.update(getRotation2d(), getModulePositions());

    }
    
    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    public double getLowestSpeed() {
        double[] speeds = new double[4];
        speeds[0] = frontLeftModule.getSpeed();
        speeds[1] = frontRightModule.getSpeed();
        speeds[2] = backLeftModule.getSpeed();
        speeds[3] = backRightModule.getSpeed();
        double lowest = speeds[0];
        for (int i = 1; i < speeds.length; i++) {
            if (speeds[i] < lowest) {
                lowest = speeds[i];
            }
        }
        return lowest;
    }

    public void setModuleStates (SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, getLowestSpeed());
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }


    public void updateOdometry() {
        positionEstimator.update(getRotation2d(), getModulePositions());

        Optional<EstimatedRobotPose> result = FrontCam.getEstimatedGlobalPose(positionEstimator.getEstimatedPosition());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            positionEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        }
    }

    public void trackTargetIDRotation(Integer targetID) {
        PhotonPipelineResult result = FrontCam.photonCamera.getLatestResult();

        if (result.hasTargets() && (result.getBestTarget().getFiducialId() == targetID || targetID == 99)) {
            double yaw = result.getBestTarget().getYaw();
            double yawRate = result.getBestTarget().getYaw();
            double yawSetpoint = yaw + yawRate * 0.1;
            double yawError = yawSetpoint - getHeading();
            double yawOutput = yawError * DriveConstants.kVisionRotationP;
            setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(0, 0, yawOutput)
            ));
        } else {
            setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(0, 0, 0)
            ));
        }
    }

}
