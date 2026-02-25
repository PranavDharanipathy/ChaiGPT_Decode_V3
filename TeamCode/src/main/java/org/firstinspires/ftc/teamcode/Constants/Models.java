package org.firstinspires.ftc.teamcode.Constants;

public class Models {

    public static double getScaledFlywheelKv(double unscaledKv, double currentVoltage) {
        return (12.5 / currentVoltage) * unscaledKv;
    }
}
