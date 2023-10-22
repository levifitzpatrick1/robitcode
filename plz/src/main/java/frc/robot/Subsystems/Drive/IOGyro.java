package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

public interface IOGyro {
    @AutoLog
    public static class IOGyroInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
    }

    public default void updateInputs(IOGyroInputs inputs) {}
    
}
