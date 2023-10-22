package frc.robot.Subsystems.Drive;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.Constants;
import frc.robot.Constants.ModuleConstants;



/**
 * IOModuleSim is a simulation class for the IOModule interface.
 *
 * <p>This class uses FlywheelSim to simulate the behavior of drive and turn motors.</p>
 */
public class IOModuleSim implements IOModule {
    private FlywheelSim driveSim =
        new FlywheelSim(DCMotor.getNEO(1), ModuleConstants.kDriveMotorGearRatio, 0.025);
    private FlywheelSim turnSim =
        new FlywheelSim(DCMotor.getNEO(1), ModuleConstants.kTurningMotorGearRatio, 0.025);

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    /**
     * Updates the simulation inputs based on the current state.
     *
     * @param inputs The IOModuleInputs object to update.
     */
    public void updateInputs(IOModuleInputs inputs){
        driveSim.update(Constants.loopPeriod);
        turnSim.update(Constants.loopPeriod);

        double angledDiffRad = 
            turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriod;
        turnRelativePositionRad += angledDiffRad;
        turnAbsolutePositionRad += angledDiffRad;
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.drivePositionRad = inputs.drivePositionRad
            + (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriod);
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = 
            new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
        inputs.driveTempC = new double[] {};

        inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.turnPositionRad = turnRelativePositionRad;
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = 
            new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
        inputs.turnTempC = new double[] {};    
    }

    /**
     * Sets the applied voltage for the drive motor in the simulation.
     *
     * @param volts The voltage to apply.
     */
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(volts);
    }
    
    /**
     * Sets the applied voltage for the turn motor in the simulation.
     *
     * @param volts The voltage to apply.
     */
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(volts);
    }
}
