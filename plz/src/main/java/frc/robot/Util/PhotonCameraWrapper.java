package frc.robot.Util;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    PhotonPoseEstimator photonPoseEstimator;

    public PhotonCameraWrapper(String name, Transform3d robotToCam) throws IOException {
        photonCamera = new PhotonCamera(name);

        // Attempt to load the AprilTagFieldLayout that will tell us where the tags are
        // on the field.
        AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        // Create pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, photonCamera, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and
     *         targets used to create
     *         the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
