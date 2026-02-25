package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;

public class HoodAngler {

    private Servo leftHoodAngler;
    private Servo rightHoodAngler;

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

        leftHoodAngler.setPosition(position);
        rightHoodAngler.setPosition(position);
    }

    public double getPosition() {
        return leftHoodAngler.getPosition();
    }
}