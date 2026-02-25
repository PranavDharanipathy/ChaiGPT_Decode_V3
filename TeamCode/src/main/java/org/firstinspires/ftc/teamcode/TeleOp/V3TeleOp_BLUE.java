package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;

@TeleOp(name = "V3TeleOp BLUE", group = "Match")
public class V3TeleOp_BLUE extends TeleOpBaseOpMode {

    private final CurrentAlliance alliance = new CurrentAlliance(CurrentAlliance.ALLIANCE.BLUE_ALLIANCE);

    private final Intake intake = new Intake();
    private final Blocker blocker = new Blocker().asSubsystem();
    private final PedroDrive pedroDrive = new PedroDrive();

    @Override
    public void init() {

        useEOALocalizationData();

        initializeDevices();
        applyComponentTraits();

        pedroDrive.provideComponents(follower, controller1, controller2);
        setUpLynxModule();
    }

    @Override
    public void start() {

        new PostAutonomousRobotReset(this);
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();
        controller2.getInformation();

        intake.update();
        blocker.update();
        pedroDrive.update();
    }

    @Override
    public void stop() {

        closeLynxModule();
    }
}
