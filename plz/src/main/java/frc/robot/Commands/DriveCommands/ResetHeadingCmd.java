package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drive.Drivetrain;

public class ResetHeadingCmd extends CommandBase {
    private final Drivetrain drivetrain;

    public ResetHeadingCmd(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        drivetrain.zeroHeading();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
