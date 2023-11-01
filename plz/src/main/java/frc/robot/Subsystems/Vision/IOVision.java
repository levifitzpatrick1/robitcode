package frc.robot.Subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;



public interface IOVision  {

    default public void update() {}
    default public void setReferencePose(Pose2d pose) {}
    default public Optional<EstimatedRobotPose> getRobotPose() {return Optional.empty();}

}
