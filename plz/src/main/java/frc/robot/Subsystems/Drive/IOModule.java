package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;


/**
 * IOModule serves as a base interface for swerve modules.
 *
 * <p>This interface outlines the essential methods for controlling
 * and obtaining data from a swerve module's drive and turn motors.</p>
 *
 * <p>Derived classes such as <code>IOModuleSim</code> and <code>IOModuleSparkMax</code>
 * provide specific implementations for simulation and SparkMax-based swerve modules,
 * respectively.</p>
 */
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

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IOModuleInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}
}
