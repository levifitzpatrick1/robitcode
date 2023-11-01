package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.EnumSet;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimVisionTarget;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * PhotonVision-compatible simulated implementation of the VisionIO interface.
 * Only a single
 * IOSimVision object may be instantiated. It uses the PhotonVision
 * SimVisionSystem to simulates the
 * AprilTag targets that would be seen by a camera based on the robot's pose,
 * which is determined
 * based on its odometry.
 */
public class IOSimVision implements IOVision {
  private static final String CAMERA_NAME = "simCamera";
  private static final double DIAGONAL_FOV = 160; // FOV in degrees
  private static final int IMG_WIDTH = 1280; // image width in px
  private static final int IMG_HEIGHT = 720; // image heigh in px
  private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);

  private double lastTimestamp = 0;
  private PhotonPipelineResult lastResult = new PhotonPipelineResult();

  private Supplier<Pose2d> poseSupplier;
  private VisionSystemSim simVision;
  private AprilTagFieldLayout layout;

  /**
   * Creates a new IOSimVision object.
   *
   * @param layout        the AprilTag field layout
   * @param poseSupplier  a Pose2d supplier that returns the robot's pose based on
   *                      its odometry
   * @param robotToCamera the transform from the robot's center to the simulated
   *                      camera
   */
  public IOSimVision(
      AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier, Transform3d robotToCamera) {
    this.layout = layout;
    this.poseSupplier = poseSupplier;

    this.simVision = new VisionSystemSim(
        CAMERA_NAME, DIAGONAL_FOV, robotToCamera, 9000, IMG_WIDTH, IMG_HEIGHT, 0);

    // default to the blue alliance; can be changed by invoking the setLayoutOrigin
    // method
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    for (AprilTag tag : layout.getTags()) {
      this.simVision.addSimVisionTarget(
          new SimVisionTarget(tag.pose, Units.inchesToMeters(6), Units.inchesToMeters(6), tag.ID));
    }

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /*
     * based on
     * https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-
     * change.html#listening-for-changes
     * and
     * https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java
     * /frc/robot/subsystems/vision/VisionIOPhotonVision.java
     */
    DoubleArraySubscriber targetPoseSub = inst.getTable("/photonvision/" + CAMERA_NAME)
        .getDoubleArrayTopic("targetPose")
        .subscribe(new double[0]);

    inst.addListener(
        targetPoseSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          PhotonPipelineResult result = camera.getLatestResult();
          double timestamp = result.getTimestampSeconds();
          synchronized (IOSimVision.this) {
            lastTimestamp = timestamp;
            lastResult = result;
          }
        });
  }

  /**
   * Updates the specified IOVisionInputs object with the latest data from the
   * camera.
   *
   * @param inputs the IOVisionInputs object to update with the latest data from
   *               the camera
   */
  @Override
  public synchronized void updateInputs(IOVisionInputs inputs) {
    this.simVision.processFrame(poseSupplier.get());
    inputs.lastTimestamp = this.lastTimestamp;
    inputs.lastResult = this.lastResult;
  }

  /**
   * Sets the origin position of the AprilTag field layout and updates the
   * simulated vision targets
   * based on the new origin.
   *
   * @param origin the origin position of the AprilTag field layout
   */
  @Override
  public void setLayoutOrigin(OriginPosition origin) {
    layout.setOrigin(origin);

    this.simVision.clearVisionTargets();
    for (AprilTag tag : layout.getTags()) {
      layout
          .getTagPose(tag.ID)
          .ifPresent(
              pose -> this.simVision.addSimVisionTarget(
                  new SimVisionTarget(
                      pose, Units.inchesToMeters(6), Units.inchesToMeters(6), tag.ID)));
    }
  }
}