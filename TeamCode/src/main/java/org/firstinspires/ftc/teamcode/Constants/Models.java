package org.firstinspires.ftc.teamcode.Constants;

import org.apache.commons.math3.util.FastMath;

public class Models {

    public static double getScaledFlywheelKv(double unscaledKv, double currentVoltage) {

        final double SCALE_WEIGHT = 0.5;

        double scaledKv = (12.7 / currentVoltage) * unscaledKv;

        return SCALE_WEIGHT * scaledKv + (1 - SCALE_WEIGHT) * unscaledKv;
    }

    public static double getCloseHoodPositionFromRegression(double distanceToGoal) {

        final double PIECEWISE_SWITCH = 80.3;

        double hoodPosition =
                distanceToGoal <= PIECEWISE_SWITCH ?
                -0.00000917712 * FastMath.pow(distanceToGoal, 3) + 0.00218461 * FastMath.pow(distanceToGoal, 2) - 0.175314 * distanceToGoal + 4.88663 : // below/equal PIECEWISE_SWITCH
                -0.0000009500073 * FastMath.pow(distanceToGoal, 4) + 0.0003606123 * FastMath.pow(distanceToGoal, 3) - 0.05110807 * FastMath.pow(distanceToGoal, 2) + 3.210404 * distanceToGoal - 75.32115; // above PIECEWISE_SWITCH

        return hoodPosition;
    }
}
