package frc.robot.Subsystems.Drive;


import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Constants;


/**
 * This class extends the IOGyro interface, specifically for the Pigeon2 gyro.
 */
public class IOPigeon2 implements IOGyro {
    private final Pigeon2 gyro;
    private final double[] xyzDps = new double[3];

    public IOPigeon2() {
        switch (Constants.getRobot()) {
            case ROBOT_PHYSICAL:
                gyro = new Pigeon2(62);
                break;
            default:
                throw new RuntimeException("Invalid robot type");
        }
    }
    
    public void updateInputs(IOGyroInputs inputs) {

        inputs.connected = gyro.getStickyFaultField().getValue() == 0;
        inputs.positionRad = Units.degreesToRadians(gyro.getAngle());
        inputs.velocityRadPerSec = Units.degreesToRadians(gyro.getRate());

        xyzDps[0] = gyro.getAngularVelocityX().getValue();
        xyzDps[1] = gyro.getAngularVelocityY().getValue();
        xyzDps[2] = gyro.getAngularVelocityZ().getValue();

    }
    
}
