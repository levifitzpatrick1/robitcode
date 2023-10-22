package frc.robot.Commands.VisionCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drive.Drivetrain;

public class Move1MeterFromIdCmd extends CommandBase {

    private Drivetrain drivetrain;


    public Move1MeterFromIdCmd(Drivetrain drivetrain){
        this.drivetrain = drivetrain;

        drivetrain.odometryToVision();
    }
    
    @Override
    public void execute() {
        drivetrain.driveToPosition2D(new Pose2d(-1, 0, null));
    }

}
