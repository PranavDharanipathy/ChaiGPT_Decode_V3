package org.firstinspires.ftc.teamcode.TeleOp.drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class PedroDrive extends Subsystem {

    public enum DriveState {
        MANUAL, DRIVE_TO_BASE
    }

    private DriveState state = DriveState.MANUAL;

    public DriveState getState() {
        return state;
    }

    private Follower follower;
    public Follower getFollower() {
        return follower;
    }

    private BetterGamepad controller1, controller2;

    public void provideComponents(Follower follower, BetterGamepad controller1, BetterGamepad controller2) {

        this.follower = follower;

        this.controller1 = controller1;
        this.controller2 = controller2;
    }

    @Override
    public void update() {

        stateTransitions();

        if (state == DriveState.MANUAL) {

            if (!follower.getTeleopDrive()) {
                follower.breakFollowing();
                follower.startTeleOpDrive(true);
            }
            follower.setTeleOpDrive(
                    -controller1.left_stick_y(),
                    -controller1.left_stick_x(),
                    -controller1.right_stick_x()
            );
        }
        else if (state == DriveState.DRIVE_TO_BASE) {

            if (!follower.isBusy()) {
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                follower.getPose(),
                                                FieldConstants.BLUE_BASE_POSE
                                        )
                                )
                                .setHeadingInterpolation(
                                        HeadingInterpolator.piecewise(
                                                new HeadingInterpolator.PiecewiseNode(
                                                        0,
                                                        0.7,
                                                        HeadingInterpolator.tangent
                                                ),
                                                new HeadingInterpolator.PiecewiseNode(
                                                        0.7,
                                                        1,
                                                        HeadingInterpolator.constant(Math.toRadians(270))
                                                )
                                        )
                                )
                                .build()
                );
            }
        }
    }

    private void stateTransitions() {

        if (Math.abs(controller1.left_stick_x()) > GeneralConstants.JOYSTICK_MINIMUM || Math.abs(controller1.left_stick_y()) > GeneralConstants.JOYSTICK_MINIMUM || Math.abs(controller1.right_stick_x()) > GeneralConstants.JOYSTICK_MINIMUM || controller2.right_stick_buttonHasJustBeenPressed) {
            state = DriveState.MANUAL;
        }
        else if (controller2.aHasJustBeenPressed) {
            state = DriveState.DRIVE_TO_BASE;
        }
    }
}
