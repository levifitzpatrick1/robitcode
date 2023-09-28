package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private final SwerveModule frontLeftModule = new SwerveModule(1, "FL");
    private final SwerveModule frontRightModule = new SwerveModule(2, "FR");
    private final SwerveModule backLeftModule = new SwerveModule(3, "BL");
    private final SwerveModule backRightModule = new SwerveModule(4, "BR");

    private Pigeon2 gyro = new Pigeon2(0);

    public Drivetrain() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
            } catch (Exception e) {}
        }).start();
    }

    public boolean zeroHeading() {
        gyro.zeroGyroBiasNow();
        return true;
    }

    public double getHeading() {
        return gyro.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }
    
    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    public double getLowestSpeed() {
        double[] speeds = new double[4];
        speeds[0] = frontLeftModule.getSpeed();
        speeds[1] = frontRightModule.getSpeed();
        speeds[2] = backLeftModule.getSpeed();
        speeds[3] = backRightModule.getSpeed();
        double lowest = speeds[0];
        for (int i = 1; i < speeds.length; i++) {
            if (speeds[i] < lowest) {
                lowest = speeds[i];
            }
        }
        return lowest;
    }

    public void setModuleStates (SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, getLowestSpeed());
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }
}
