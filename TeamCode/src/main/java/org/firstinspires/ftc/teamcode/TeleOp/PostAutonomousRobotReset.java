package org.firstinspires.ftc.teamcode.TeleOp;

public class PostAutonomousRobotReset {

    public PostAutonomousRobotReset(TeleOpBaseOpMode opMode) {

        opMode.turret.setPosition(opMode.turret.startPosition);
    }
}
