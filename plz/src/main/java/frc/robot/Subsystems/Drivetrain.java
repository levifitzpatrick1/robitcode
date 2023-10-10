package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.VisionConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable {

    // Other Bot
    private final SwerveModule frontLeftModule = new SwerveModule( 3, "FL");
    private final SwerveModule frontRightModule = new SwerveModule(4, "FR");
    private final SwerveModule backLeftModule = new SwerveModule(2, "BL");
    private final SwerveModule backRightModule = new SwerveModule(1, "BR");

    // School Bot
    // private final SwerveModule frontLeftModule = new SwerveModule( 6, "FL");
    // private final SwerveModule frontRightModule = new SwerveModule(8, "FR");
    // private final SwerveModule backLeftModule = new SwerveModule(5, "BL");
    // private final SwerveModule backRightModule = new SwerveModule(7, "BR");

    private Pigeon2 gyro = new Pigeon2(62);
    private AHRS navx = new AHRS(SPI.Port.kMXP);

    public PIDController angularPID = 
        new PIDController(
        DriveConstants.kVisionRotationP,
        DriveConstants.kVisionRotationI,
        DriveConstants.kVisionRotationD
        );

    public ProfiledPIDController xtranslationPID = 
        new ProfiledPIDController(
        DriveConstants.kVisionTranslationxP,
        DriveConstants.kVisionTranslationxI,
        DriveConstants.kVisionTranslationxD,
        DriveConstants.kDriveConstraints
        );

    public ProfiledPIDController ytranslationPID = 
        new ProfiledPIDController(
        DriveConstants.kVisionTranslationyP,
        DriveConstants.kVisionTranslationyI,
        DriveConstants.kVisionTranslationyD,
        DriveConstants.kDriveConstraints
        );

    public PhotonCameraWrapper FrontCam;

    private final SwerveDrivePoseEstimator positionEstimator = 
        new SwerveDrivePoseEstimator(
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

        zeroHeading();
        FrontCam = new PhotonCameraWrapper(VisionConstants.kFrontCamName, VisionConstants.kFrontRobotToCam);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("FL Cancoder", frontLeftModule.getAbsolutePositionDegrees());
        SmartDashboard.putNumber("FR Cancoder", frontRightModule.getAbsolutePositionDegrees());
        SmartDashboard.putNumber("BL Cancoder", backLeftModule.getAbsolutePositionDegrees());
        SmartDashboard.putNumber("BR Cancoder", backRightModule.getAbsolutePositionDegrees());

        updateOdometry();

        angularPID.setP(DriveConstants.kVisionRotationP);
        angularPID.setI(DriveConstants.kVisionRotationI);
        angularPID.setD(DriveConstants.kVisionRotationD);


        xtranslationPID.setP(DriveConstants.kVisionTranslationxP);
        xtranslationPID.setI(DriveConstants.kVisionTranslationxI);
        xtranslationPID.setD(DriveConstants.kVisionTranslationxD);


        ytranslationPID.setP(DriveConstants.kVisionTranslationyP);
        ytranslationPID.setI(DriveConstants.kVisionTranslationyI);
        ytranslationPID.setD(DriveConstants.kVisionTranslationyD);
    }
    

    // ------ Gets ------ //
    
    /**
     * Normalizes wheel speeds
     * @return Lowest kMaxModuleSpeed
     */
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

    /**
     * Gets the current state of the drivetrain
     * @return ChassisSpeeds from ModuleStates
     */
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        );
    }

    /**
     * Gets the current module positions
     * @return List of SwerveModulePositions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getModulePosition(),
            frontRightModule.getModulePosition(),
            backLeftModule.getModulePosition(),
            backRightModule.getModulePosition()
        };
    }

    /**
     * Gets the current robot heading from the gyro
     * @return Double representing gyro degrees
     */
    public double getHeading() {
        if (DriveConstants.kPidgeonGyro) {
            if (DriveConstants.kPidgeonInverse) {
                return -gyro.getAngle();
            } else {
                return gyro.getAngle();
            }
        } else {
            return navx.getAngle();
        }
    }

    /**
     * Gets the current robot rotation from the gyro
     * @return Rotation2d from gyro degrees
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // ------ Odometry ------ //

    /**
     * Resets the current odometry to the given pose
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        positionEstimator.update(getRotation2d(), getModulePositions());
        positionEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Updates the robot's position on the field using the encoders gyro and vision
     */
    public void updateOdometry() {
        positionEstimator.update(getRotation2d(), getModulePositions());

        Optional<EstimatedRobotPose> result = FrontCam.getEstimatedGlobalPose(positionEstimator.getEstimatedPosition());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            positionEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        }
    }

    // ------ Commands ------ //

    /**
     * Stops all modules
     */
    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    /**
     * Resets the gyro to 0
     */
    public void zeroHeading() {
        if (DriveConstants.kPidgeonGyro) {
            gyro.reset();
        } else {
            navx.reset();
        }
    }

    /**
     * Sets the desired states for the swerve modules
     * @param desiredStates List of desired states for the swerve modules
     */

    public void setModuleStates (SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, getLowestSpeed());
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);

    }

    @Log.Graph(name="April Tag Yaw")
    public double getRotationYawDiff() {
        PhotonPipelineResult result = FrontCam.photonCamera.getLatestResult();
        double yaw = 0;
        if (result.hasTargets()) {
            yaw = result.getBestTarget().getYaw();
        }
        return yaw;
    }

    @Log
    public String getRotationPIDValues() {
        return angularPID.getP() + ", " + angularPID.getI() + ", " + angularPID.getD();
    }

    @Log
    public double getHasTarget() {
        PhotonPipelineResult result = FrontCam.photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return 1;
        } else {
            return 0;
        }
    }



/**
 * Tracks the rotation of a target with a given ID
 * @param targetID ID of the target to track. Set to 99 to track any target
 * @return Rotation speed to track the target
 */
    public double trackTargetIDRotation(Integer targetID) {
        PhotonPipelineResult result = FrontCam.photonCamera.getLatestResult();
        double rotspeed;

        if (result.hasTargets() && (result.getBestTarget().getFiducialId() == targetID || targetID == 99)) {
            int foundTargetID = result.getBestTarget().getFiducialId();
            double yaw = Math.toRadians(result.getBestTarget().getYaw());
            SmartDashboard.putNumber("vision target id", foundTargetID);
            SmartDashboard.putNumber("vision target yaw", yaw);

            rotspeed = angularPID.calculate(yaw, 0);
            
            SmartDashboard.putNumber("speed", rotspeed);
            
        } else {
            rotspeed = 0;
        }
        return rotspeed;
    }


    /**
     * Follows the target while maintaing an X distance of 1 meter
     * @param targetID ID of the target to track. Set to 99 to track any target
     * @return Translation speed to track the target
     */
    public ChassisSpeeds trackTargetIDPosition(Integer targetID) {
        PhotonPipelineResult result = FrontCam.photonCamera.getLatestResult();
        double rotspeed;
        double x;
        double y;
        double rot;
        double vx;
        double vy;

        Transform3d cameraToTarget;

        if (result.hasTargets() && (result.getBestTarget().getFiducialId() == targetID || targetID == 99)){
            cameraToTarget = result.getBestTarget().getBestCameraToTarget();

            x = cameraToTarget.getX();
            y = cameraToTarget.getY();

            rot = cameraToTarget.getRotation().getZ();

            rotspeed = angularPID.calculate(result.getBestTarget().getYaw(), 0);

            double angularSetpoint;

            if (rot > 0){
                angularSetpoint = 180;
            } else {
                angularSetpoint = -180;
            }

            vx = xtranslationPID.calculate(-x, 1) * DriveConstants.kDriveMaxVelocity;
            vy = ytranslationPID.calculate(Math.toDegrees(rot), angularSetpoint) * DriveConstants.kDriveMaxVelocity;

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, rotspeed, getRotation2d());

            SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
            setModuleStates(states);

            return speeds;
        } else {
            return new ChassisSpeeds(0, 0, 0);
        }
    }

}
