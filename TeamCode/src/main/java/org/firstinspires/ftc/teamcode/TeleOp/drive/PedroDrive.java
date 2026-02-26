package org.firstinspires.ftc.teamcode.TeleOp.drive;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class PedroDrive extends Subsystem {

    public enum DriveState {
        MANUAL
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

        follower.update();

        if (state == DriveState.MANUAL) {

            follower.startTeleOpDrive(true);
            follower.setTeleOpDrive(
                    -controller1.left_stick_y(),
                    -controller1.left_stick_x(),
                    -controller1.right_stick_x()
            );
        }
    }
}
