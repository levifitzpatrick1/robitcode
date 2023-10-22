package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drive.Drivetrain;

public class ResetOdometry extends CommandBase {
    private Drivetrain drivetrain;

    public ResetOdometry(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        drivetrain.resetOdometry(new Pose2d());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
