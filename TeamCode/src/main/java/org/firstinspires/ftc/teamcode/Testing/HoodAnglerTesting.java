package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.HoodAngler;

@Config
@TeleOp(group = "testing")
public class HoodAnglerTesting extends LinearOpMode {

    private HoodAngler hoodAngler;

    public static double POSITION;

    public static double LEFT_OFFSET;
    public static double RIGHT_OFFSET;

    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        hoodAngler = new HoodAngler(hardwareMap, MapSetterConstants.hoodAnglerLeftServoDeviceName, MapSetterConstants.hoodAnglerRightServoDeviceName);

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {

            ConfigurationConstants.LEFT_HOOD_ALIGNMENT_OFFSET = LEFT_OFFSET;
            ConfigurationConstants.RIGHT_HOOD_ALIGNMENT_OFFSET = RIGHT_OFFSET;

            hoodAngler.setPosition(POSITION);

            telemetry.addData("position", hoodAngler.getPosition());
            telemetry.addData("internal left position", hoodAngler.getServos()[0].getPosition());
            telemetry.addData("internal right position", hoodAngler.getServos()[1].getPosition());
            telemetry.update();
        }

    }
}
