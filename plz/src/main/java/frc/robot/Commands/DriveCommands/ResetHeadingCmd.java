package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Subsystems.Drivetrain;

public class ResetHeadingCmd extends CommandBase {
    private final Drivetrain drivetrain;

    public ResetHeadingCmd(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        if (DriveConstants.kPidgeonGyro) {
            drivetrain.zeroHeading();
        } else {
            drivetrain.zeroNavx();
        }
    }
}
