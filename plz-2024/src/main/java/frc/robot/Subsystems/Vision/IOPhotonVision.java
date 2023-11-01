package frc.robot.Subsystems.Vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Util.Alert;
import frc.robot.Util.Alert.AlertType;

import java.util.EnumSet;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** PhotonVision-based implementation of the VisionIO interface. */
public class IOPhotonVision implements IOVision {
  private Alert noCameraConnectedAlert = new Alert("specified camera not connected", AlertType.WARNING);
  private final PhotonCamera camera;

  private double lastTimestamp = 0;
  private PhotonPipelineResult lastResult = new PhotonPipelineResult();

  /**
   * Creates a new IOPhotonVision object.
   *
   * @param cameraName the name of the PhotonVision camera to use; the name must
   *                   be unique
   */
  public IOPhotonVision(String cameraName) {
    camera = new PhotonCamera(cameraName);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /*
     * based on
     * https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-
     * change.html#listening-for-changes
     * and
     * https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java
     * /frc/robot/subsystems/vision/IOPhotonVision.java
     */
    DoubleArraySubscriber targetPoseSub = inst.getTable("/photonvision/" + cameraName)
        .getDoubleArrayTopic("targetPose")
        .subscribe(new double[0]);

    inst.addListener(
        targetPoseSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          PhotonPipelineResult result = camera.getLatestResult();

          double timestamp = result.getTimestampSeconds();
          synchronized (IOPhotonVision.this) {
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
    inputs.lastTimestamp = this.lastTimestamp;
    inputs.lastResult = this.lastResult;

    noCameraConnectedAlert.set(!camera.isConnected());
  }
}