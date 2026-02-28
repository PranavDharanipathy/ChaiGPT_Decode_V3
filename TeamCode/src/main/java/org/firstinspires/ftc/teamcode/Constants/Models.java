package org.firstinspires.ftc.teamcode.Constants;

public class Models {

    public static double getScaledFlywheelKv(double unscaledKv, double currentVoltage) {
        return (13 / currentVoltage) * unscaledKv;
    }


    public static double getCloseHoodPositionFromRegression(double distanceToGoal) {
        return 0;
    }
}
