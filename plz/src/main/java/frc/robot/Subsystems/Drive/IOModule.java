package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;


public interface IOModule {
    @AutoLog
    public static class IOModuleInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveVelocityFilteredRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempC = new double[] {};

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};
        public double[] turnTempC = new double[] {};
    }

    public default void updateInputs(IOModuleInputs inputs) {}

    public default void setDriveVoltage(double volts) {}

    public default void setTurnVoltage(double volts) {}

    public default void setDriveBrakeMode(boolean enable) {}

    public default void setTurnBrakeMode(boolean enable) {}
    
}
