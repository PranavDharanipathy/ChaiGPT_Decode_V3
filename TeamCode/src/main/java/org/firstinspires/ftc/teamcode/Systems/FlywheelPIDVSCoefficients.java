package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.TURRET_KPS;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.TURRET_KDS;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.TURRET_KFS;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.TURRET_PD_POSITIONS;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.TURRET_FEEDFORWARD_POSITIONS;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.Models;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.InterpolationData;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/// Easier usage of the coefficients for the flywheel
public class FlywheelPIDVSCoefficients {

    public double kp;
    public double kiFar, kiClose;
    public double kd;
    public double unscaledKv;
    public double ks;

    public double kPIDFUnitsPerVolt;

    public double iSwitch;

    public double kISmash;

    public double voltageFilterAlpha;

    public double minP, maxP;
    public double minI, maxI;
    public double minD, maxD;

    private boolean tuning = false;

    public void setTuning(boolean tuning) {
        this.tuning = tuning;
    }

    public FlywheelPIDVSCoefficients(
            double kp,
            double kiFar,
            double kiClose,
            double kd,
            double unscaledKv,
            double ks,
            double kPIDFUnitsPerVolt,
            double iSwitch,
            double kISmash,
            double minP,
            double maxP,
            double minI,
            double maxI,
            double minD,
            double maxD
    ) {

        this.kp = kp;

        this.kiFar = kiFar;
        this.kiClose = kiClose;

        this.kd = kd;

        this.unscaledKv = unscaledKv;

        this.ks = ks;
        this.kPIDFUnitsPerVolt = kPIDFUnitsPerVolt;

        this.iSwitch = iSwitch;

        this.kISmash = kISmash;

        this.minP = minP;
        this.maxP = maxP;

        this.minI = minI;
        this.maxI = maxI;

        this.minD = minD;
        this.maxD = maxD;
    }

    private double kISwitchTargetVelocity;

    public double ki(double targetVelocity, double currentVelocity) {

        if (kISwitchTargetVelocity == targetVelocity || (targetVelocity - currentVelocity) < iSwitch) {

            kISwitchTargetVelocity = targetVelocity;
            return kiClose;
        }
        else return kiFar;
    }

    private double filteredVoltage = 0;

    /// Run every loop
    public double kv(VoltageSensor batteryVoltageSensor) {

        filteredVoltage = LowPassFilter.getFilteredValue(filteredVoltage, batteryVoltageSensor.getVoltage(), voltageFilterAlpha);

        return Models.getScaledFlywheelKv(unscaledKv, filteredVoltage);
    }
}
