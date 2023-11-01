package frc.robot.Subsystems.Vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.ejml.equation.ManagerFunctions.Input1;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.cameraserver.CameraServerShared;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.Subsystems.Drive.DriveWithIO;
import frc.robot.Subsystems.Vision.IOVision.IOVisionInputs;
import frc.robot.Util.Alert;
import frc.robot.Util.LoggedTunableNumber;
import frc.robot.Util.RobotOdometry;
import frc.robot.Util.Alert.AlertType;

public class Vision extends SubsystemBase {
  private IOVision[] cameras;
  private Transform3d[] robotToCams;
  private final IOVisionInputs[] visionInputs;
  private double[] lastTimestamps;

  private AprilTagFieldLayout layout;
  private Alert noAprilTagLayout = new Alert("No AprilTag Layout found", AlertType.WARNING);

  private boolean isEnabled = true;
  private boolean isVisionUdpdating = false;

  private SwerveDrivePoseEstimator poseEstimator;
  private final LoggedTunableNumber poseCheckerDistance = new LoggedTunableNumber("Vision/VisionPoseThreshold",
      VisionConstants.poseCheckerDistance);

  private final LoggedTunableNumber stdDevSlope = new LoggedTunableNumber("Vision/stdDevSlope", 0.10);
  private final LoggedTunableNumber stdDevPower = new LoggedTunableNumber("Vision/stdDevPower", 2.0);

  private static class RobotPoseFromAprilTag {
    public final Pose3d robotPose;
    public final double distanceToAprilTag;

    public RobotPoseFromAprilTag(Pose3d robotPose, double distance) {
      this.robotPose = robotPose;
      this.distanceToAprilTag = distance;
    }
  }

  public Vision(IOVision[] cameras, Transform3d[] robotToCams) {
    this.cameras = cameras;
    this.robotToCams = robotToCams;
    this.lastTimestamps = new double[cameras.length];
    this.visionInputs = new IOVisionInputs[cameras.length];

    for (int i = 0; i < cameras.length; i++) {
      this.visionInputs[i] = new IOVisionInputs();
    }

    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    try {
      layout = new AprilTagFieldLayout(VisionConstants.fieldLayoutPath);
      noAprilTagLayout.set(false);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16, 8);
      noAprilTagLayout.set(true);
    }

    for (AprilTag tag : layout.getTags()) {
      Logger.recordOutput("Vision/AprilTags/" + tag.ID, tag.pose);
    }
  }

  public void updateAlliance(DriverStation.Alliance newAlliance) {

    if (newAlliance == DriverStation.Alliance.Red) {
      layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
      for (IOVision camera : cameras) {
        camera.setLayoutOrigin(OriginPosition.kRedAllianceWallRightSide);
      }
    } else {
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      for (IOVision camera : cameras) {
        camera.setLayoutOrigin(OriginPosition.kBlueAllianceWallRightSide);
      }
    }

    for (AprilTag tag : layout.getTags()) {
      layout
          .getTagPose(tag.ID)
          .ifPresent(pose -> Logger.recordOutput("Vision/AprilTags/" + tag.ID, pose));
    }
  }

  @Override
  public void periodic() {
    isVisionUdpdating = false;

    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(visionInputs[i]);
      Logger.processInputs("Vision" + i, visionInputs[i]);

      if (lastTimestamps[i] < visionInputs[i].lastTimestamp) {
        lastTimestamps[i] = visionInputs[i].lastTimestamp;
        RobotPoseFromAprilTag poseAndDistance = getRobotPose(i);
        Pose3d robotPose = poseAndDistance.robotPose;

        if (robotPose == null)
          return;

        if (poseEstimator
            .getEstimatedPosition()
            .minus(robotPose.toPose2d())
            .getTranslation()
            .getNorm() < VisionConstants.poseCheckerDistance) {
          if (isEnabled) {
            poseEstimator.addVisionMeasurement(robotPose.toPose2d(),
                visionInputs[i].lastTimestamp,
                getStandardDeviations(poseAndDistance.distanceToAprilTag));
            isVisionUdpdating = true;
          }

          Logger.recordOutput("Vision/RobotPose" + i, robotPose.toPose2d());
          Logger.recordOutput("Vision/IsEnabled", isEnabled);
        }
      }
    }
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  public Pose3d getBestRobotPose() {
    Pose3d robotPoseFromClosestTarget = null;
    Double closestTargetDistance = Double.MAX_VALUE;
    for (int i = 0; i < cameras.length; i++) {
      RobotPoseFromAprilTag poseAndDistance = getRobotPose(i);
      Pose3d robotPose = poseAndDistance.robotPose;
      double distanceToAprilTag = poseAndDistance.distanceToAprilTag;
      if (robotPose != null && distanceToAprilTag < closestTargetDistance) {
        robotPoseFromClosestTarget = robotPose;
        closestTargetDistance = distanceToAprilTag;
      }
    }

    return robotPoseFromClosestTarget;
  }

  public void enable(boolean enable) {
    isEnabled = enable;
  }

  public boolean posesHaveConverged() {
    for (int i = 0; i < cameras.length; i++) {
      Pose3d robotPose = getRobotPose(i).robotPose;
      if (robotPose != null
          && poseEstimator
              .getEstimatedPosition()
              .minus(robotPose.toPose2d())
              .getTranslation()
              .getNorm() < VisionConstants.poseCheckerDistance) {
        Logger.recordOutput("Vision/PosesInLine", true);
        return true;
      }
    }
    Logger.recordOutput("Vision/PosesInLine", false);
    return false;
  }

  private Matrix<N3, N1> getStandardDeviations(double distanceToAprilTag) {
    double stdDev = stdDevSlope.get() * (Math.pow(distanceToAprilTag, stdDevPower.get()));
    return VecBuilder.fill(stdDev, stdDev, stdDev);
  }

  private RobotPoseFromAprilTag getRobotPose(int cameraIndex) {
    int targetCount = 0;
    Pose3d robotPoseFromClosestTarget = null;
    Double closestTargetDistance = Double.MAX_VALUE;

    for (int i = 0; i < 2; i++) {
      Logger.recordOutput("Vision/TagPose" + cameraIndex + "_" + i, new Pose2d());
      Logger.recordOutput("Vision/TagDistance" + cameraIndex + "_" + i, new Pose2d());
    }

    for (PhotonTrackedTarget target : visionInputs[cameraIndex].lastResult.getTargets()) {
      if (isValidTarget(target)) {
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Optional<Pose3d> targetPose = layout.getTagPose(target.getFiducialId());
        if (targetPose.isPresent()) {
          Pose3d tagPose = targetPose.get();
          Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());
          Pose3d robotPose = cameraPose.transformBy(robotToCams[cameraIndex].inverse());

          Logger.recordOutput("Vision/TagPose" + cameraIndex + "_" + targetCount, tagPose.toPose2d());
          Logger.recordOutput("Vision/camRobotPose" + cameraIndex + "_" + targetCount,
              robotPose.toPose2d());

          double targetDistance = target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();

          if (targetDistance < closestTargetDistance
              && targetDistance < VisionConstants.poseCheckerDistance) {
            robotPoseFromClosestTarget = robotPose;
            closestTargetDistance = targetDistance;
          }
        }
      }
      targetCount++;
    }

    return new RobotPoseFromAprilTag(robotPoseFromClosestTarget, closestTargetDistance);
  }

  private boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < VisionConstants.maxAmbiguity
        && layout.getTagPose(target.getFiducialId()).isPresent();
  }
}
