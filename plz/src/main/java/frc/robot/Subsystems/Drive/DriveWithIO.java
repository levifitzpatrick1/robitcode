package frc.robot.Subsystems.Drive;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;
//import org.photonvision.PhotonVersion;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.Subsystems.Drive.Gyros.IOGyro;
import frc.robot.Subsystems.Drive.Gyros.IOGyroInputsAutoLogged;
import frc.robot.Subsystems.Drive.Modules.IOModule;
import frc.robot.Subsystems.Drive.Modules.IOModuleInputsAutoLogged;
import frc.robot.Util.GeomUtil;
import frc.robot.Util.LoggedTunableNumber;
import frc.robot.Util.PhotonCameraWrapper;

/**
 * This class handles the drive functionality of the robot using IO components.
 * It extends the SubsystemBase class from WPILib and integrates various sensors and actuators.
 */
public class DriveWithIO extends SubsystemBase{

    private final IOGyro gyro;
    private final IOGyroInputsAutoLogged gyroInputs = new IOGyroInputsAutoLogged();
    private final IOModule[] modules = new IOModule[4];
    private final IOModuleInputsAutoLogged[] moduleInputs =
    new IOModuleInputsAutoLogged[] {
        new IOModuleInputsAutoLogged(),
        new IOModuleInputsAutoLogged(), 
        new IOModuleInputsAutoLogged(),
        new IOModuleInputsAutoLogged()
    };

    private final double maxLinearSpeed;
    private final double maxAngularSpeed;
    private final double wheelRadius;
    private final double trackWidthX;
    private final double trackWidthY;

    private final LoggedTunableNumber driveP = new LoggedTunableNumber("DriveP");
    private final LoggedTunableNumber driveI = new LoggedTunableNumber("DriveI");
    private final LoggedTunableNumber driveD = new LoggedTunableNumber("DriveD");
    private final LoggedTunableNumber driveS = new LoggedTunableNumber("DriveS");
    private final LoggedTunableNumber driveV = new LoggedTunableNumber("DriveV");

    private final LoggedTunableNumber turnP = new LoggedTunableNumber("TurnP");
    private final LoggedTunableNumber turnI = new LoggedTunableNumber("TurnI");
    private final LoggedTunableNumber turnD = new LoggedTunableNumber("TurnD");

    private final SwerveDriveKinematics kinematics;
    private SimpleMotorFeedforward driveFeedForward;

    private final PIDController[] driveFeedback = new PIDController[4];
    private final PIDController[] turnFeedback = new PIDController[4];

    private Pose2d odometryPose = new Pose2d();
    private Translation2d fieldVelocity = new Translation2d();
    private double[] lastModulePositionRad = new double[] {0.0, 0.0, 0.0, 0.0};
    private double lastGyroPositionRad = 0.0;

    private DriveMode driveMode = DriveMode.NORMAL;
    private ChassisSpeeds closedLoopSetpoint = new ChassisSpeeds();
    private double characterizationVoltage = 0.0;


    public PhotonCameraWrapper frontCamera;
    private PIDController frontCameraPID;

    /**
     * Constructor for DriveWithIO class.
     * Initializes gyro, IO modules, and other parameters based on the robot type.
     *
     * @param gyro       The IOGyro object for orientation data.
     * @param frontLeft  The front-left IO module.
     * @param frontRight The front-right IO module.
     * @param backLeft   The back-left IO module.
     * @param backRight  The back-right IO module.
     */
    public DriveWithIO(IOGyro gyro, IOModule frontLeft, IOModule frontRight, IOModule backLeft, IOModule backRight) {
        this.gyro = gyro;
        modules[0] = frontLeft;
        modules[1] = frontRight;
        modules[2] = backLeft;
        modules[3] = backRight;

        switch (Constants.getRobot()) {
            case ROBOT_PHYSICAL:
                maxLinearSpeed = DriveConstants.kDriveMaxVelocity;
                wheelRadius = ModuleConstants.kWheelDiameterMeters / 2.0;
                trackWidthX = DriveConstants.kTrackWidth;
                trackWidthY = DriveConstants.kWheelBase;

                driveP.initdDefault(0.1);
                driveI.initdDefault(0.0);
                driveD.initdDefault(0.0);
                driveS.initdDefault(0.12349);
                driveV.initdDefault(0.13477);
                
                turnP.initdDefault(10.0);
                turnI.initdDefault(0.0);
                turnD.initdDefault(0.0);
                break;
            case ROBOT_SIMBOT:
            maxLinearSpeed = Units.feetToMeters(14.5);
            wheelRadius = Units.inchesToMeters(1.0);
            trackWidthX = 0.65;
            trackWidthY = 0.65;

            driveP.initdDefault(0.9);
            driveI.initdDefault(0.0);
            driveD.initdDefault(0.0);
            driveS.initdDefault(0.116970);
            driveV.initdDefault(0.133240);

            turnP.initdDefault(23.0);
            turnI.initdDefault(0.0);
            turnD.initdDefault(0.0);
            break;
        default:
            maxLinearSpeed = 0.0;
            wheelRadius = 0.0;
            trackWidthX = 0.0;
            trackWidthY = 0.0;

            driveP.initdDefault(0.0);
            driveI.initdDefault(0.0);
            driveD.initdDefault(0.0);
            driveS.initdDefault(0.0);
            driveV.initdDefault(0.0);

            turnP.initdDefault(0.0);
            turnI.initdDefault(0.0);
            turnD.initdDefault(0.0);
            break;
        }

        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        driveFeedForward = new SimpleMotorFeedforward(driveS.get(), driveV.get());
        for (int i = 0; i < 4; i++) {
            driveFeedback[i] = new PIDController(driveP.get(), driveI.get(), driveD.get(), Constants.loopPeriod);
            turnFeedback[i] = new PIDController(turnP.get(), turnI.get(), turnD.get(), Constants.loopPeriod);
            turnFeedback[i].enableContinuousInput(-Math.PI, Math.PI);
        }

        maxAngularSpeed = maxLinearSpeed / Arrays.stream(getModuleTranslations()).map
        (translation -> translation.getNorm()).max(Double::compare).get();


        frontCamera = new PhotonCameraWrapper(VisionConstants.kFrontCamName, VisionConstants.kFrontRobotToCam);
        frontCameraPID = new PIDController(0.5, 0.0, 0.0, Constants.loopPeriod);

    }

    @Override
    public void periodic() {
        gyro.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
        for (int i = 0; i < 4; i++) {
            modules[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i), moduleInputs[i]);
        }

        if (driveP.hasChanged() || driveI.hasChanged() || driveD.hasChanged()
         || driveS.hasChanged() || driveV.hasChanged() || turnP.hasChanged()
         || turnI.hasChanged() || turnD.hasChanged()) {
            driveFeedForward = new SimpleMotorFeedforward(driveS.get(), driveV.get());
            for (int i = 0; i < 4; i++) {
                driveFeedback[i].setP(driveP.get());
                driveFeedback[i].setI(driveI.get());
                driveFeedback[i].setD(driveD.get());
                turnFeedback[i].setP(turnP.get());
                turnFeedback[i].setI(turnI.get());
                turnFeedback[i].setD(turnD.get());
            }
        }

        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            turnPositions[i] = new Rotation2d(moduleInputs[i].turnAbsolutePositionRad);
        }

        if (DriverStation.isDisabled()){
            for (int i = 0; i <4; i++){
                modules[i].setDriveVoltage(0.0);
                modules[i].setTurnVoltage(0.0);
            }
        } else {
            switch (driveMode) {
                case NORMAL:
                SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(closedLoopSetpoint);
                SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxLinearSpeed);

                boolean isStationary = 
                Math.abs(closedLoopSetpoint.vxMetersPerSecond) < 1e-3
                && Math.abs(closedLoopSetpoint.vyMetersPerSecond) < 1e-3
                && Math.abs(closedLoopSetpoint.omegaRadiansPerSecond) < 1e-3;

                SwerveModuleState[] optimizedModuleStates = 
                new SwerveModuleState[] {null, null, null, null};
                for (int i = 0; i < 4; i++) {
                    optimizedModuleStates[i] = SwerveModuleState.optimize(
                        moduleStates[i], turnPositions[i]);
                    if (isStationary) {
                        modules[i].setTurnVoltage(0.0);
                    } else {
                        modules[i].setTurnVoltage(
                            turnFeedback[i].calculate(turnPositions[i].getRadians(),
                            optimizedModuleStates[i].angle.getRadians()));
                    }

                    optimizedModuleStates[i].speedMetersPerSecond *=
                    Math.cos(turnFeedback[i].getPositionError());

                    double velocityRadPerSec =
                    optimizedModuleStates[i].speedMetersPerSecond / wheelRadius;
                    modules[i].setDriveVoltage(
                        driveFeedForward.calculate(velocityRadPerSec) +
                        driveFeedback[i].calculate(
                            moduleInputs[i].driveVelocityRadPerSec, velocityRadPerSec));
            // Log individual setpoints
            Logger.getInstance().recordOutput(
                "SwerveModuleStatesSetpoints/Drive/" + Integer.toString(i),
                velocityRadPerSec);
            Logger.getInstance().recordOutput(
                "SwerveModuleStatesSetpoints/Turn/" + Integer.toString(i),
                optimizedModuleStates[i].angle.getRadians());

            Logger.getInstance().recordOutput(
                "NeedToKnow/Drive/ModuleRot" + Integer.toString(i),
                moduleInputs[i].turnAbsolutePositionRad);

                SmartDashboard.putNumber("encoder" + i, moduleInputs[i].turnAbsolutePositionRad);
          
                }

                Logger.getInstance().recordOutput("SwerveModuleStates/Setpoints", moduleStates);
                Logger.getInstance().recordOutput("SwerveModuleStates/OptimizedSetpoints", optimizedModuleStates);
                break;

                case CHARACTERIZATION:

                for (int i = 0; i < 4; i++) {
                    modules[i].setTurnVoltage(
                        turnFeedback[i].calculate(turnPositions[i].getRadians(), 0.0));
                    modules[i].setDriveVoltage(characterizationVoltage);
                }
                break;

                case X:
                for (int i = 0; i < 4; i++) {
                    Rotation2d targetRotation =
                        GeomUtil.direction(getModuleTranslations()[i]);
                    Rotation2d currentRotation = turnPositions[i];
                    if (Math.abs(
                        targetRotation.minus(currentRotation).getDegrees()) > 90.0) {
                        
                        targetRotation = 
                            targetRotation.minus(Rotation2d.fromDegrees(180.0));
                }

                modules[i].setTurnVoltage(turnFeedback[i].calculate(
                    currentRotation.getRadians(), targetRotation.getRadians()));
                modules[i].setDriveVoltage(0.0);
                
            }
            break;
        }
    }


    SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
    
        measuredStatesDiff[i] = new SwerveModuleState(
            (moduleInputs[i].drivePositionRad - lastModulePositionRad[i])
            * wheelRadius,
            turnPositions[i]);
        
        lastModulePositionRad[i] = moduleInputs[i].drivePositionRad;
    }

    ChassisSpeeds chassisStateDiff = 
        kinematics.toChassisSpeeds(measuredStatesDiff);
    
    if (gyroInputs.connected) {
        odometryPose =
            odometryPose.exp(new Twist2d(chassisStateDiff.vxMetersPerSecond,
            chassisStateDiff.vyMetersPerSecond,
            gyroInputs.positionRad - lastGyroPositionRad));
    } else {
        odometryPose = odometryPose.exp(
            new Twist2d(chassisStateDiff.vxMetersPerSecond,
            chassisStateDiff.vyMetersPerSecond,
            chassisStateDiff.omegaRadiansPerSecond));
    }

    lastGyroPositionRad = gyroInputs.positionRad;

    SwerveModuleState[] measuredStates = 
        new SwerveModuleState[] {null, null, null, null};
    for (int i = 0; i < 4; i++) {
        measuredStates[i] = new SwerveModuleState(
            moduleInputs[i].driveVelocityRadPerSec * wheelRadius,
            turnPositions[i]);
    }

    ChassisSpeeds chassisState = kinematics.toChassisSpeeds(measuredStates);
    fieldVelocity = new Translation2d(chassisState.vxMetersPerSecond,
        chassisState.vyMetersPerSecond).rotateBy(getRotation());

    Logger.getInstance().recordOutput("SwerveModuleStates/Measured", measuredStates);
    Logger.getInstance().recordOutput("Odometry/Robot", odometryPose);

}


    /**
     * Sets the velocity of the robot.
     *
     * @param speeds The desired chassis speeds.
     */
    public void runVelocity(ChassisSpeeds speeds) {
        driveMode = DriveMode.NORMAL;
        closedLoopSetpoint = speeds;
    }

    /**
     * Stops the robot by setting all velocities to zero.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void goToX() {
        driveMode = DriveMode.X;
    }

/**
 * Gets the max linear speed of the robot.
 * @return The max linear speed of the robot in meters per second.
 */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxLinearSpeed;
    }

    /**
     * Gets the max angular speed of the robot.
     * @return The max angular speed of the robot in radians per second.
     */
    public double getMaxAngularSpeedRadiansPerSec() {
        return maxAngularSpeed;
    }

    /**
     * Gets the current pose of the robot.
     * @return The current pose of the robot.
     */
    public Pose2d getPose() {
        return odometryPose;
    }

    /**
     * Gets the current rotation of the robot.
     * @return The current rotation of the robot.
     */
    public Rotation2d getRotation() {
        return odometryPose.getRotation();
    }

    /**
     * Sets the current pose of the robot.
     * @param pose The current pose of the robot.
     */
    public void setPose(Pose2d pose) {
        odometryPose = pose;
    }

    /**
     * Gets the current field velocity of the robot.
     * @return The current field velocity of the robot.
     */
    public Translation2d getFieldVelocity() {
        return fieldVelocity;
    }

    /**
     * gets the module positions relative to the center of the robot.
     * @return The module positions relative to the center of the robot.
     */
    public Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
            new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
            new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
            new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)};
      }

      /**
       * Runs the characterization routine.
       * @param voltage
       */
      public void runCharacterization(double voltage) {
        driveMode = DriveMode.CHARACTERIZATION;
        characterizationVoltage = voltage;
      }

        /**
         * Gets the characterization velocity.
         * @return The characterization velocity.
         */
      public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (int i = 0; i < 4; i++) {
          driveVelocityAverage += moduleInputs[i].driveVelocityRadPerSec;
        }
        return driveVelocityAverage / 4.0;
      }

      public double getVisionRot(int targetID) {
        PhotonPipelineResult result = frontCamera.photonCamera.getLatestResult();
        double rotSpeed;

        Logger.getInstance().recordOutput("Vision/Result", result.getBestTarget().getYaw());
        Logger.getInstance().recordOutput("Vision/TargetID", result.getBestTarget().getFiducialId());

        if (result.hasTargets() && (result.getBestTarget().getFiducialId() == targetID || targetID == 99)) {
            double yaw = Math.toRadians(result.getBestTarget().getYaw());
            rotSpeed = frontCameraPID.calculate(yaw, 0);
        } else {
            rotSpeed = 0;
        }
        return rotSpeed;
    }

    
      /**
       * Sets the drive mode.
       */
    private static enum DriveMode {
        NORMAL, X, CHARACTERIZATION
      }
    
}
