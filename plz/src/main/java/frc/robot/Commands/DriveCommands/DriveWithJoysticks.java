package frc.robot.Commands.DriveCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drive.DriveWithIO;
import frc.robot.Util.GeomUtil;


/**
 * DriveWithJoysticks is a command that allows the robot to be driven using joystick inputs.
 *
 * <p>This command uses the DriveWithIO subsystem and joystick suppliers to control the robot.</p>
 */
public class DriveWithJoysticks extends CommandBase {

    private final DriveWithIO drive;
    private final Supplier<Double> leftXSupplier;
    private final Supplier<Double> leftYSupplier;
    private final Supplier<Double> rightYSupplier;
    private final Supplier<Boolean> robotRelativeOverride;
    private final Supplier<Boolean> targetingOverride;
    
    private static final double deadband = 0.1;


    /**
     * Constructor for DriveWithJoysticks.
     *
     * @param drive The DriveWithIO subsystem.
     * @param leftXSupplier Supplier for the left joystick X-axis.
     * @param leftYSupplier Supplier for the left joystick Y-axis.
     * @param rightYSupplier Supplier for the right joystick Y-axis.
     * @param robotRelativeOverride Supplier for the field vs robot oriented drive.
     * @param targetingOverride Supplier for targeting.
     */
    public DriveWithJoysticks(DriveWithIO drive,
    Supplier<Double> leftXSupplier, 
    Supplier<Double> leftYSupplier, 
    Supplier<Double> rightYSupplier, 
    Supplier<Boolean> robotRelativeOverride,
    Supplier<Boolean> targetingOverride) {
        this.drive = drive;
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.rightYSupplier = rightYSupplier;
        this.robotRelativeOverride = robotRelativeOverride;
        this.targetingOverride = targetingOverride;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double leftX = leftXSupplier.get();
        double leftY = leftYSupplier.get();
        double rightY = rightYSupplier.get();

        double linearMagnitude = Math.hypot(leftX, leftY);
        Rotation2d linearDirection = new Rotation2d(leftX, leftY);


        linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);
        rightY = MathUtil.applyDeadband(rightY, deadband);

        linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
        rightY = Math.copySign(rightY * rightY, rightY);


        Translation2d linearVelocity = new Pose2d(
            new Translation2d(), linearDirection).transformBy(
                GeomUtil.transformFromTranslation(linearMagnitude, 0.0)).getTranslation();

        ChassisSpeeds speeds = new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            rightY * drive.getMaxAngularSpeedRadiansPerSec());

        if (!robotRelativeOverride.get()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                drive.getRotation());
        }

        if (targetingOverride.get()) {
            double newRot = drive.getVisionRot(99);
            if (newRot != 0) {
                speeds = new ChassisSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    newRot * drive.getMaxAngularSpeedRadiansPerSec());
            }
        }

        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
