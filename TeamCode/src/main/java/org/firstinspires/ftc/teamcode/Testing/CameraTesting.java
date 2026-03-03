package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Systems.Camera;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.BooleanTrigger;

@Config
@TeleOp(group = "testing")
public class CameraTesting extends TeleOpBaseOpMode {

    public static CurrentAlliance.ALLIANCE ALLIANCE = CurrentAlliance.ALLIANCE.BLUE_ALLIANCE;

    private RobotCentricDrive robotCentricDrive = new RobotCentricDrive();

    private Telemetry telemetry;

    @Override
    public void init() {

        initializeDevices();
        applyComponentTraits();

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(30);

        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);
        
        camera.pipelineSwitch(CameraConstants.PIPELINES.getPipelineFromAlliance(ALLIANCE).getPipelineIndex());
    }

    @Override
    public void start() {

        camera.start();
    }

    @Override
    public void loop() {

        controller1.getInformation();
        controller2.getInformation();

        follower.update();
        camera.update(new BooleanTrigger(controller2.main_buttonHasJustBeenPressed));

        robotCentricDrive.update();

        telemetry.addData("Localize Pose", camera.getLocalizePose());
        telemetry.addData("LL Pose", camera.getBotPoseMT2());
        telemetry.addData("Follower Pose", follower.getPose());
        telemetry.addLine();

        Pose finalPose = camera.isEligibleForMT2() ? camera.getBotPoseMT2() : follower.getPose();

        telemetry.addData("Final Pose", finalPose);

        telemetry.update();

    }
}
