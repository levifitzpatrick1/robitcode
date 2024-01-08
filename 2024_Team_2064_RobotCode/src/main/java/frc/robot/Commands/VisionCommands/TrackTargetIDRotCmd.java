package frc.robot.Commands.VisionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drivetrain;

public class TrackTargetIDRotCmd extends CommandBase {

    private Drivetrain drivetrain;
    private Integer targetID;

    public TrackTargetIDRotCmd(Drivetrain drivetrain, Integer targetID) {
        this.drivetrain = drivetrain;
        this.targetID = targetID;
    }

    @Override
    public void execute() {
        drivetrain.trackTargetIDRotation(targetID);
    }
    
}
