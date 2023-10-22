package frc.robot.Util;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import frc.robot.Constants.Constants;

public class LoggedTunableNumber {
    private final String tableKey = "TunableNumbers";

    private final String key;
    private boolean hasDefault = false;
    private double defaultValue;
    private LoggedDashboardNumber dashboardNumber;
    private double lastHasChangedValue;


    public LoggedTunableNumber(String dashboardKey) {
        this.key = tableKey + "/" + dashboardKey;
    }

    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initdDefault(defaultValue);
    }

    public void initdDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (Constants.tuningMode) {
                dashboardNumber = new LoggedDashboardNumber(key, defaultValue);
            }
        }
    }

    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return Constants.tuningMode ? dashboardNumber.get() : defaultValue;
        }
    }

    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        } else {
            return false;
        }
    }

}