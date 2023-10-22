package frc.robot.Util;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.ctre.phoenix6.sim.DeviceType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

public class SparkMaxDerivedVelocityController {
    private final int deviceManufacturer = 5;
    private final int deviceID = 2;
    private final int apiID = 98;

    private final CANSparkMax sparkMax;
    private final CAN canInterface;
    private final LinearFilter velocityFilter;
    private final PIDController velocityController;
    private final Notifier notifier;

    private boolean firstCycle = true;
    private boolean enabled = false;
    private double ffVolts = 0.0;
    private double timestamp = 0.0;
    private double position = 0.0;
    private double velocity = 0.0;

    public SparkMaxDerivedVelocityController(CANSparkMax sparkMax){
        this(sparkMax, 0.02, 5);
    }

    public SparkMaxDerivedVelocityController(CANSparkMax sparkMax, double periodSeconds, int averageTaps){
        this.sparkMax = sparkMax;
        sparkMax.getEncoder().setPositionConversionFactor(1.0);
        int periodMs = (int)(periodSeconds * 1000);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, periodMs);

        canInterface = 
            new CAN(sparkMax.getDeviceId(), deviceManufacturer, deviceID);
        velocityFilter = LinearFilter.movingAverage(averageTaps);
        velocityController = new PIDController(0.0, 0.0, 0.0, periodSeconds);
        notifier = new Notifier(this::update);
        notifier.startPeriodic(periodSeconds);
    }

    private void update(){
        CANData canData = new CANData();
        boolean isFresh = canInterface.readPacketNew(apiID, canData);
        double newTimestamp = canData.timestamp / 1000.0;
        double newPosition = ByteBuffer.wrap(canData.data).order(ByteOrder.LITTLE_ENDIAN).asFloatBuffer().get(0);

        if (isFresh){
            synchronized (this){
                if (!firstCycle){
                    velocity = velocityFilter.calculate(
                        (newPosition - position) / (newTimestamp - timestamp) * 60);
                }
                firstCycle = false;
                timestamp = newTimestamp;
                position = newPosition;

                if (DriverStation.isDisabled()){
                    enabled = false;
                    sparkMax.stopMotor();
                }
                if (enabled){
                    sparkMax.setVoltage(ffVolts + velocityController.calculate(velocity));
                }
            }
        }
    }

    public synchronized void setReference(double velocityRpm, double ffVolts){
        velocityController.setSetpoint(velocityRpm);
        this.ffVolts = ffVolts;
        if (!enabled){
            velocityController.reset();
        }
        enabled = true;
    }

    public synchronized void disable() {
        if (enabled){
            sparkMax.stopMotor();
        }
        enabled = false;
    }

    public synchronized void setPID(double kP, double kI, double kD){
        velocityController.setPID(kP, kI, kD);
    }

    public synchronized double getPosition(){
        return position;
    }

    public synchronized double getVelocity(){
        return velocity;
    }
}
