package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class HoodAngler {

    private Servo leftHoodAngler;
    private Servo rightHoodAngler;

    /// Index 0 is left
    /// <p>
    /// Index 1 is right
    public Servo[] getServos() {
        return new Servo[] { leftHoodAngler, rightHoodAngler };
    }

    public HoodAngler(HardwareMap hardwareMap, String leftHoodAnglerServoName, String rightHoodAnglerServoName) {

        leftHoodAngler = hardwareMap.get(Servo.class, leftHoodAnglerServoName);
        rightHoodAngler = hardwareMap.get(Servo.class, rightHoodAnglerServoName);

        setServoDirections(ConfigurationConstants.HOOD_ANGLER_SERVO_DIRECTIONS);
    }

    /// The first item is for the left angler servo and the seconds item is for the right angler servo
    public void setServoDirections(Servo.Direction[] directions) {

        leftHoodAngler.setDirection(directions[0]); //1st item
        rightHoodAngler.setDirection(directions[1]); //2nd item
    }

    public Servo.Direction[] getServoDirections() {

        return new Servo.Direction[] {
                leftHoodAngler.getDirection(), //1st item
                rightHoodAngler.getDirection() //2nd item
        };
    }

    public void setPosition(double position) {

        this.position = position;

        leftHoodAngler.setPosition(this.position + ConfigurationConstants.LEFT_HOOD_ALIGNMENT_OFFSET);
        rightHoodAngler.setPosition(this.position + ConfigurationConstants.RIGHT_HOOD_ALIGNMENT_OFFSET);
    }

    public void setSafePosition(double position) {

        this.position = MathUtil.clamp(position, ShooterConstants.HOOD_ANGLER_MAX_POSITION, ShooterConstants.HOOD_ANGLER_MIN_POSITION);

        leftHoodAngler.setPosition(this.position + ConfigurationConstants.LEFT_HOOD_ALIGNMENT_OFFSET);
        rightHoodAngler.setPosition(this.position + ConfigurationConstants.RIGHT_HOOD_ALIGNMENT_OFFSET);
    }

    /// Index 0 is left
    /// <p>
    /// Index 1 is right
    public double[] getServoPositions() {
        return new double[] { leftHoodAngler.getPosition(), rightHoodAngler.getPosition() };
    }

    private double position;

    public double getPosition() {
        return position;
    }
}