package frc.robot.Subsystems.Drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants.Constants;

/**
 * This class extends the IOGyro interface, specifically for the NavX gyro.
 */
public class IONavX implements IOGyro {
    private AHRS navx;
    private final double[] xyzDps = new double[3];

    public IONavX() {
        switch (Constants.getRobot()) {
            case ROBOT_PHYSICAL:
                navx = new AHRS(SPI.Port.kMXP);
                break;
            default:
                throw new RuntimeException("Invalid robot type");
        }
    }

    public void updateInputs(IOGyroInputs inputs) {

        inputs.connected = navx.isConnected();
        inputs.positionRad = Math.toRadians(navx.getAngle());
        inputs.velocityRadPerSec = Math.toRadians(navx.getRate());

        xyzDps[0] = navx.getRawGyroX();
        xyzDps[1] = navx.getRawGyroY();
        xyzDps[2] = navx.getRawGyroZ();

    }
    
}
