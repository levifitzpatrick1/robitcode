package frc.robot.Subsystems.Drive.Gyros;

import org.littletonrobotics.junction.AutoLog;

/**
 * IOGyro serves as a base interface for gyros.
 * 
 * <p>
 * This interface outlines the essential methods for obtaining data from a gyro,
 * including
 * whether or not the gyro is connected, its current position, and its current
 * velocity.
 * </p>
 */
public interface IOGyro {
    @AutoLog
    public static class IOGyroInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
    }

    public default void updateInputs(IOGyroInputsAutoLogged gyroInputs) {
    }

}
