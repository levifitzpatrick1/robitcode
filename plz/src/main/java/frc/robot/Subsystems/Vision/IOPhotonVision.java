package frc.robot.Subsystems.Vision;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;

public class IOPhotonVision implements IOVision {
    private final PhotonCamera[] cameras;
    private final Transform3d[] cameraPoses;
    
    private final AprilTagFieldLayout fieldLayout;
    private final Path fieldJsonPath = Paths.get(Filesystem.getDeployDirectory().toString(), "field.json");
    private final ArrayList<Pair<PhotonCamera, Transform3d>> cameraList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

    private final ArrayList<PhotonPoseEstimator> poseEstimators = new ArrayList<PhotonPoseEstimator>();

    private Optional<EstimatedRobotPose> estimatedPose;

    public IOPhotonVision(PhotonCamera[] cameras, Transform3d[] cameraPoses) {
        this.cameras = new PhotonCamera[cameras.length];
        this.cameraPoses = new Transform3d[cameraPoses.length];

        for (int i = 0; i < cameras.length; i++) {
            this.cameras[i] = cameras[i];
            this.cameraPoses[i] = cameraPoses[i];
            cameraList.add(new Pair<PhotonCamera, Transform3d>(cameras[i], cameraPoses[i]));
        }

        try {
            fieldLayout = new AprilTagFieldLayout(fieldJsonPath);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load field layout", e);
        }

        for (int i = 0; i < cameras.length; i++){
            PhotonPoseEstimator pe = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, cameras[i], cameraPoses[i]);
            pe.setLastPose(new Pose2d());
            pe.setReferencePose(new Pose2d());
            poseEstimators.add(pe);
        }
        
    }

    @Override
    public void setReferencePose(Pose2d pose) {
        for (PhotonPoseEstimator pe : poseEstimators) {
            pe.setReferencePose(pose);
        }
    }

    @Override
    public Optional<EstimatedRobotPose> getRobotPose() {
        if (estimatedPose == null) {
            return Optional.empty();
        } else {
            return estimatedPose;
        }
    }


    
}
