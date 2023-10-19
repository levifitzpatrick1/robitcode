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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.Util.LowPassFilter;
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

    private Timer timer = new Timer();
    private double initTime = 0;
    private Double finalTime = null;
    private boolean isTiming = false;

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

    private final LowPassFilter visionFilter = new LowPassFilter(0.2);

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

        SmartDashboard.putNumber("x pos", positionEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("y pos", positionEstimator.getEstimatedPosition().getY());

        updateOdometry();

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
     * If the robot can see an april tag, set the odometry based off of that
     * <p> This should be run on the start of autonomous so we know where we are on the field to start, no matter our position
     */
    public void odometryToVision(){


        Optional<EstimatedRobotPose> result = FrontCam.getEstimatedGlobalPose(positionEstimator.getEstimatedPosition());

        if (result.isPresent()){
        EstimatedRobotPose camPose = result.get();

        positionEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        }
    }

    /**
     * Updates the robot's position on the field using the encoders gyro and vision
     * only update with vision if the reading is within 1 meter of the current estimate
     */
    public void updateOdometry() {
        positionEstimator.update(getRotation2d(), getModulePositions());

        Optional<EstimatedRobotPose> result = FrontCam.getEstimatedGlobalPose(positionEstimator.getEstimatedPosition());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            Pose2d currEstimate = positionEstimator.getEstimatedPosition();
            double distance = currEstimate.getTranslation().getDistance(camPose.estimatedPose.toPose2d().getTranslation());
            if (distance <= 1.0){
                positionEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            }
            
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
            yaw = visionFilter.filter(yaw);
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
        double rot;
        double vx;
        double vy;

        Transform3d cameraToTarget;

        if (result.hasTargets() && (result.getBestTarget().getFiducialId() == targetID || targetID == 99)){
            cameraToTarget = result.getBestTarget().getBestCameraToTarget();

            x = cameraToTarget.getX();
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
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kDriveMaxVelocity);
            setModuleStates(states);

            return speeds;
        } else {
            return new ChassisSpeeds(0, 0, 0);
        }
    }

    /**
     * Drives to a given position
     * @param targetPose Pose to drive to
     * @return Chassis speeds to drive to the pose
     */
    public ChassisSpeeds driveToPosition2D(Pose2d targetPose){

        Pose2d currentPose = positionEstimator.getEstimatedPosition();

        double xError = targetPose.getTranslation().getX() - currentPose.getTranslation().getX();
        double yError = targetPose.getTranslation().getY() - currentPose.getTranslation().getY();
        double rotError = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

        double vx = xtranslationPID.calculate(xError, 0) * DriveConstants.kDriveMaxVelocity;
        double vy = ytranslationPID.calculate(yError, 0) * DriveConstants.kDriveMaxVelocity;
        double rot = angularPID.calculate(rotError, 0) * DriveConstants.kTurnMaxVelocity;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, rot, getRotation2d());

        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kDriveMaxVelocity);
        setModuleStates(states);

        return speeds;

    }

    public void getAcceleration(){
        // Initialize the timer and flags if not already timing
        if (!isTiming) {
            timer.reset();
            timer.start();
            initTime = timer.get();
            isTiming = true;
        }

        // Set the module states to full forward
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState(ModuleConstants.kDriveMaxVelocity, new Rotation2d(0));
        states[1] = new SwerveModuleState(ModuleConstants.kDriveMaxVelocity, new Rotation2d(0));
        states[2] = new SwerveModuleState(ModuleConstants.kDriveMaxVelocity, new Rotation2d(0));
        states[3] = new SwerveModuleState(ModuleConstants.kDriveMaxVelocity, new Rotation2d(0));
        setModuleStates(states);  // Assume setModuleStates() is implemented elsewhere in your code

        // Check if the modules have reached max velocity
        if (frontLeftModule.getDriveVelocity() >= ModuleConstants.kDriveMaxVelocity) {  // Assume this function exists and returns true when max velocity is reached
            finalTime = timer.get();
            double deltaTime = finalTime - initTime;
            double acceleration = ModuleConstants.kDriveMaxVelocity / deltaTime;

            SmartDashboard.putNumber("Acceleration",  acceleration);

            // Reset timing flag
            isTiming = false;
        }
    }
    }

