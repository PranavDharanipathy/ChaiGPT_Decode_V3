package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private DcMotorEx motor;

    private Direction direction;
    public Encoder(DcMotorEx motor) {

        this.motor = motor;

        this.direction = Direction.FORWARD;
    }

    public Direction getDirection() {
        return direction;
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public int getCurrentPosition() {
        int multiplier = direction.getMultiplier();
        return motor.getCurrentPosition() * multiplier;
    }

    public double getVelocity() {
        int multiplier = direction.getMultiplier();
        return motor.getVelocity() * multiplier;
    }

}